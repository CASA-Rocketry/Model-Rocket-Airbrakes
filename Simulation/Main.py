from rocketpy import Environment, Rocket, Flight, Barometer
from rocketpy.motors import GenericMotor
import math
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
from filterpy.kalman import KalmanFilter
from dataclasses import dataclass
import logging

logging.basicConfig(level=logging.WARNING)


@dataclass
class ControllerConfig:
    max_deployment_cd: float = 1.0
    rocket_radius: float = 0.028
    airbrake_area: float = 0.00246300864
    air_density: float = 1.2
    target_apogee: float = 228.6
    env_elevation: float = 200
    burnout_mass: float = 0.5
    sampling_rate: int = 20
    sensor_std: float = 0.062
    model_y_std: float = 0.02
    model_v_std: float = 0.5
    model_a_std: float = 0.1
    kp_base: float = 7000.0
    max_deployment_rate: float = 0.5
    apogee_prediction_cd: float = 1.0
    acceleration_decay_burn: float = 0.85
    acceleration_decay_coast: float = 0.85


@dataclass
class RocketConfig:
    # Rocket values
    radius: float = 0.028
    mass: float = 0.45
    inertia: tuple = (0.001, 0.001, 0.0001)
    power_off_drag: str = "RocketDragCurve.csv"
    power_on_drag: str = "RocketDragCurve.csv"
    center_of_mass_without_motor: float = 0.35
    coordinate_system_orientation: str = "nose_to_tail"

    # Motor values
    motor_eng_file: str = "AeroTech_F42T_L.eng"
    motor_chamber_radius: float = 0.0145
    motor_chamber_height: float = 0.083
    motor_position: float = 0.77

    # Nose values
    nose_length: float = 0.13
    nose_kind: str = "von karman"
    nose_position: float = 0

    # Fin values
    fin_n: int = 4
    fin_root_chord: float = 0.075
    fin_tip_chord: float = 0.050
    fin_span: float = 0.03
    fin_position: float = 0.695
    fin_cant_angle: float = 0.5

    # Parachute values
    parachute_name: str = "Chute"
    parachute_cd_s: float = 0.8
    parachute_trigger: float = 210
    parachute_sampling_rate: int = 10
    parachute_lag: float = 0
    parachute_noise: tuple = (0, 8.3, 0.5)

    # Barometer values
    barometer_sampling_rate: int = 50
    barometer_measurement_range: float = 100000
    barometer_resolution: float = 1.0 / 4096
    barometer_noise_density: float = 0
    barometer_noise_variance: float = 0.5625
    barometer_random_walk_density: float = 0.0
    barometer_constant_bias: float = 0.0
    barometer_operating_temperature: float = 0
    barometer_temperature_bias: float = 0.0
    barometer_temperature_scale_factor: float = 0.0
    barometer_name: str = "Barometer"
    barometer_position: tuple = (0.265, 0, 0)

    # Airbrake values
    airbrake_drag_coefficient_curve: str = "BrakeDragCurve.csv"
    airbrake_clamp: bool = True
    airbrake_initial_observed_variables: list = None
    airbrake_override_rocket_drag: bool = False
    airbrake_name: str = "Airbrakes"

    def __post_init__(self):
        if self.airbrake_initial_observed_variables is None:
            self.airbrake_initial_observed_variables = [0]


class KalmanAltitudeFilter:
    """Kalman filter for altitude and velocity estimation matching Arduino implementation"""

    def __init__(self, config: ControllerConfig):
        self.config = config
        self.K = KalmanFilter(dim_x=3, dim_z=1)  # (y, v, a)
        self.initialized = False
        self.previous_time = None
        self.obs = np.zeros((1, 1))  # observation vector

    def initialize(self, initial_altitude_agl: float, sampling_rate: int):
        """Initialize filter matching Arduino structure"""
        dt = 1.0 / sampling_rate

        # State evolution matrix (updated with DT)
        self.K.F = np.array([[1.0, dt, dt * dt / 2],
                             [0.0, 1.0, dt],
                             [0.0, 0.0, 1.0]])

        # State to measurement
        self.K.H = np.array([[1, 0, 0]])

        # Measurement covariance
        self.K.R = np.array([[self.config.sensor_std * self.config.sensor_std]])

        # Model covariance matrix
        self.K.Q = np.array([[self.config.model_y_std * self.config.model_y_std, 0, 0],
                             [0, self.config.model_v_std * self.config.model_v_std, 0],
                             [0, 0, self.config.model_a_std * self.config.model_a_std]])

        self.K.x = np.array([initial_altitude_agl, 0.0, 0.0])
        self.initialized = True

    def update(self, measurement_agl: float, time: float, motor_burn_time: float):
        """Update filter with barometer measurement matching Arduino structure"""
        if not self.initialized:
            raise RuntimeError("Filter not initialized")

        dt = time - self.previous_time if self.previous_time is not None else 1.0 / self.config.sampling_rate
        dt = max(dt, 1e-6)

        # Update state transition matrix with new dt
        decay = self.config.acceleration_decay_burn if time < motor_burn_time else self.config.acceleration_decay_coast
        self.K.F = np.array([[1.0, dt, dt * dt / 2],
                             [0.0, 1.0, dt],
                             [0.0, 0.0, decay]])

        # Create measurement matrix
        self.obs[0] = measurement_agl

        # Kalman update
        self.K.predict()
        self.K.update(self.obs)
        self.previous_time = time

        return float(self.K.x[0]), float(self.K.x[1])


class AirbrakeController:
    """Airbrake controller"""

    def __init__(self, config: ControllerConfig, motor_burn_time: float):
        self.config = config
        self.motor_burn_time = motor_burn_time
        self.filter = KalmanAltitudeFilter(config)
        self.sea_level_pressure = 101325.0
        self.ground_pressure = None  # Will be calibrated from first reading
        self.ground_pressure_calibrated = False
        self.calibration_readings = []  # Store multiple readings for better calibration
        self.calibration_samples = 5  # Number of samples for calibration

        # Data storage
        self.data = {
            'time': [], 'sim_altitude_agl': [], 'raw_altitude_agl': [], 'filtered_altitude_agl': [],
            'sim_velocity': [], 'filtered_velocity': [], 'deployment': [], 'predicted_apogee_agl': [],
            'motor_burning': [], 'control_active': []
        }

    def _calibrate_barometer(self, pressure_pa: float):
        """Calibrate barometer using multiple readings."""
        if not self.ground_pressure_calibrated:
            self.calibration_readings.append(pressure_pa)

            if len(self.calibration_readings) >= self.calibration_samples:
                # Use average of calibration readings
                self.ground_pressure = np.mean(self.calibration_readings)
                self.ground_pressure_calibrated = True
                std_dev = np.std(self.calibration_readings)
                print(f"Barometer calibrated: Ground pressure = {self.ground_pressure:.1f} ± {std_dev:.1f} Pa")
                return True
            else:
                print(f"Calibrating barometer... {len(self.calibration_readings)}/{self.calibration_samples}")
                return False
        return True

    def _read_barometer(self, sensors):
        """Read barometer and return AGL altitude."""
        for sensor in sensors:
            if (hasattr(sensor, 'measured_data') and sensor.name == "Barometer" and
                    hasattr(sensor, 'measurement') and sensor.measurement is not None):
                try:
                    pressure_pa = float(sensor.measurement)
                    if pressure_pa <= 0:
                        return None, False

                    # Calibrate barometer with multiple readings
                    if not self._calibrate_barometer(pressure_pa):
                        return None, False

                    # Convert pressure to altitude AGL using calibrated ground pressure
                    if self.ground_pressure and self.ground_pressure > 0:
                        # Use the barometric formula relative to ground pressure
                        altitude_agl = 44330 * (1 - (pressure_pa / self.ground_pressure) ** 0.1903)

                        # Sanity check for reasonable altitude values
                        if altitude_agl < -10 or altitude_agl > 1000:
                            print(f"Warning: Suspicious barometer reading: {altitude_agl:.1f}m AGL")

                        return altitude_agl, True
                    else:
                        return None, False

                except (ValueError, TypeError, AttributeError) as e:
                    print(f"Barometer read error: {e}")
                    return None, False
        return None, False

    def _predict_apogee(self, altitude_agl: float, velocity: float):
        """Apogee prediction considering drag and gravity in 1D"""
        if velocity <= 0:
            return altitude_agl

        # Use configurable drag coefficient
        k = 0.5 * self.config.air_density * self.config.apogee_prediction_cd * self.config.airbrake_area
        if k <= 0:
            return 0

        log_arg = (k * velocity ** 2) / (self.config.burnout_mass * 9.81) + 1
        if log_arg <= 0:
            return 0

        delta_altitude = (self.config.burnout_mass / (2 * k)) * math.log(log_arg)
        predicted_apogee_agl = altitude_agl + delta_altitude

        # Sanity check
        if predicted_apogee_agl < altitude_agl:
            print(
                f"Warning: Predicted apogee ({predicted_apogee_agl:.1f}m) below current altitude ({altitude_agl:.1f}m)")
            return altitude_agl

        return predicted_apogee_agl

    def _calculate_deployment(self, predicted_apogee_agl: float, velocity: float, current_deployment: float,
                              sampling_rate: int):
        """Calculate airbrake deployment based on AGL apogee prediction."""
        error = predicted_apogee_agl - self.config.target_apogee
        if error <= 0:
            return 0.0

        velocity_factor = max(velocity ** 2 + 15.0, 1.0)
        kp = self.config.kp_base / velocity_factor
        new_deployment = kp * error

        # Rate limiting
        max_change = self.config.max_deployment_rate / sampling_rate
        lower_bound = max(0, current_deployment - max_change)
        upper_bound = min(1, current_deployment + max_change)

        new_deployment = np.clip(new_deployment, 0, 1)
        new_deployment = np.clip(new_deployment, lower_bound, upper_bound)

        return new_deployment

    def control(self, time: float, sampling_rate: int, state: np.ndarray,
                state_history, observed_variables, air_brakes, sensors):
        """Main control function"""

        # Convert ASL to AGL for all calculations
        true_altitude_asl = state[2]
        true_altitude_agl = max(0, true_altitude_asl - self.config.env_elevation)
        true_velocity = state[5]

        # Read barometer (returns AGL altitude)
        barometer_altitude_agl, barometer_available = self._read_barometer(sensors)

        # Initialize defaults
        filtered_altitude_agl = barometer_altitude_agl if barometer_available else true_altitude_agl
        filtered_velocity = 0.0
        predicted_apogee_agl = 0.0
        deployment_level = 0.0
        motor_burning = time < self.motor_burn_time
        control_active = False

        # Process barometer data if available
        if barometer_available:
            if not self.filter.initialized:
                # Initialize filter with AGL altitude
                self.filter.initialize(barometer_altitude_agl, sampling_rate)
                filtered_altitude_agl = barometer_altitude_agl
                filtered_velocity = 0.0
            else:
                # Update filter with AGL altitude, get AGL results
                filtered_altitude_agl, filtered_velocity = self.filter.update(
                    barometer_altitude_agl, time, self.motor_burn_time)
            # Predict apogee using original method
            predicted_apogee_agl = self._predict_apogee(filtered_altitude_agl, filtered_velocity)

            # Control logic. Only active after motor burnout and with positive velocity
            if not motor_burning and filtered_velocity > 0:
                deployment_level = self._calculate_deployment(
                    predicted_apogee_agl, filtered_velocity, air_brakes.deployment_level, sampling_rate)
                air_brakes.deployment_level = deployment_level
                control_active = True

        # Store data
        self.data['time'].append(time)
        self.data['sim_altitude_agl'].append(true_altitude_agl)
        self.data['raw_altitude_agl'].append(barometer_altitude_agl if barometer_available else None)
        self.data['filtered_altitude_agl'].append(filtered_altitude_agl)
        self.data['sim_velocity'].append(true_velocity)
        self.data['filtered_velocity'].append(filtered_velocity)
        self.data['deployment'].append(deployment_level)
        self.data['predicted_apogee_agl'].append(predicted_apogee_agl)
        self.data['motor_burning'].append(motor_burning)
        self.data['control_active'].append(control_active)

        return time, deployment_level, predicted_apogee_agl

class SimulationRunner:
    def __init__(self, config: ControllerConfig, rocket_config: RocketConfig):
        self.config = config
        self.rocket_config = rocket_config
        self.controller = None

    def setup_environment(self):
        return Environment(
            latitude=32.990254,
            longitude=-106.974998,
            elevation=self.config.env_elevation,
        )

    def setup_rocket(self):
        rocket = Rocket(
            radius=self.rocket_config.radius,
            mass=self.rocket_config.mass,
            inertia=self.rocket_config.inertia,
            power_off_drag=self.rocket_config.power_off_drag,
            power_on_drag=self.rocket_config.power_on_drag,
            center_of_mass_without_motor=self.rocket_config.center_of_mass_without_motor,
            coordinate_system_orientation=self.rocket_config.coordinate_system_orientation,
        )

        motor = GenericMotor.load_from_eng_file(
            file_name=self.rocket_config.motor_eng_file,
            chamber_radius=self.rocket_config.motor_chamber_radius,
            chamber_height=self.rocket_config.motor_chamber_height
        )
        rocket.add_motor(motor, position=self.rocket_config.motor_position)

        rocket.add_nose(
            length=self.rocket_config.nose_length,
            kind=self.rocket_config.nose_kind,
            position=self.rocket_config.nose_position
        )
        rocket.add_trapezoidal_fins(
            n=self.rocket_config.fin_n,
            root_chord=self.rocket_config.fin_root_chord,
            tip_chord=self.rocket_config.fin_tip_chord,
            span=self.rocket_config.fin_span,
            position=self.rocket_config.fin_position,
            cant_angle=self.rocket_config.fin_cant_angle
        )
        rocket.add_parachute(
            name=self.rocket_config.parachute_name,
            cd_s=self.rocket_config.parachute_cd_s,
            trigger=self.rocket_config.parachute_trigger,
            sampling_rate=self.rocket_config.parachute_sampling_rate,
            lag=self.rocket_config.parachute_lag,
            noise=self.rocket_config.parachute_noise
        )

        barometer = Barometer(
            sampling_rate=self.rocket_config.barometer_sampling_rate,
            measurement_range=self.rocket_config.barometer_measurement_range,
            resolution=self.rocket_config.barometer_resolution,
            noise_density=self.rocket_config.barometer_noise_density,
            noise_variance=self.rocket_config.barometer_noise_variance,
            random_walk_density=self.rocket_config.barometer_random_walk_density,
            constant_bias=self.rocket_config.barometer_constant_bias,
            operating_temperature=self.rocket_config.barometer_operating_temperature,
            temperature_bias=self.rocket_config.barometer_temperature_bias,
            temperature_scale_factor=self.rocket_config.barometer_temperature_scale_factor,
            name=self.rocket_config.barometer_name
        )
        rocket.add_sensor(barometer, position=self.rocket_config.barometer_position)

        self.controller = AirbrakeController(self.config, motor.burn_out_time)

        rocket.add_air_brakes(
            drag_coefficient_curve=self.rocket_config.airbrake_drag_coefficient_curve,
            controller_function=self.controller.control,
            sampling_rate=self.config.sampling_rate,
            reference_area=self.config.airbrake_area,
            clamp=self.rocket_config.airbrake_clamp,
            initial_observed_variables=self.rocket_config.airbrake_initial_observed_variables,
            override_rocket_drag=self.rocket_config.airbrake_override_rocket_drag,
            name=self.rocket_config.airbrake_name,
        )

        return rocket, motor

    def run_simulation(self, rocket, environment):
        return Flight(
            rocket=rocket,
            environment=environment,
            rail_length=3,
            inclination=85,
            heading=0,
            time_overshoot=False,
            terminate_on_apogee=True
        )

    def export_to_csv(self, flight):
        """Export flight data to CSV."""
        if not self.controller.data['time']:
            print("No controller data available for export.")
            return

        try:
            # Create DataFrame from controller data
            df = pd.DataFrame({
                'Time_s': self.controller.data['time'],
                'Simulated_Altitude_AGL_m': self.controller.data['sim_altitude_agl'],
                'Raw_Barometer_Altitude_AGL_m': self.controller.data['raw_altitude_agl'],
                'Filtered_Altitude_AGL_m': self.controller.data['filtered_altitude_agl'],
                'Simulated_Velocity_ms': self.controller.data['sim_velocity'],
                'Filtered_Velocity_ms': self.controller.data['filtered_velocity'],
                'Deployment_Level': self.controller.data['deployment'],
                'Predicted_Apogee_AGL_m': self.controller.data['predicted_apogee_agl'],
                'Motor_Burning': self.controller.data['motor_burning'],
                'Control_Active': self.controller.data['control_active'],
                'Target_Apogee_AGL_m': self.config.target_apogee,
                'Environment_Elevation_m': self.config.env_elevation,
                'Apogee_Prediction_Cd': self.config.apogee_prediction_cd,
            })

            # Calculate errors
            df['Altitude_Error_m'] = np.abs(df['Simulated_Altitude_AGL_m'] - df['Filtered_Altitude_AGL_m'])
            df['Velocity_Error_ms'] = np.abs(df['Simulated_Velocity_ms'] - df['Filtered_Velocity_ms'])
            df['Apogee_Error_m'] = df['Predicted_Apogee_AGL_m'] - self.config.target_apogee

            # Add flight data if available
            try:
                times = np.array(self.controller.data['time'])
                flight_alt_asl = flight.z(times)
                df['Flight_Altitude_AGL_m'] = flight_alt_asl - self.config.env_elevation
                df['Flight_Velocity_Total_ms'] = flight.speed(times)
                df['Flight_Acceleration_ms2'] = flight.acceleration(times)
                df['Flight_Mach_Number'] = flight.mach_number(times)
                df['Flight_Dynamic_Pressure_Pa'] = flight.dynamic_pressure(times)
            except Exception as e:
                print(f"Warning: Could not add flight data: {e}")

            # Export to CSV
            filename = 'Rocket_Flight_Data.csv'
            df.to_csv(filename, index=False)
            print(f"\nData exported to {filename}")
            print(f"Rows: {len(df)}, Columns: {len(df.columns)}")

        except Exception as e:
            print(f"Error exporting CSV: {e}")
            import traceback
            traceback.print_exc()

    def plot_results(self):
        """Create plots"""
        if not self.controller.data['time']:
            return

        times = np.array(self.controller.data['time'])

        fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(15, 10))

        # Altitude tracking
        ax1.plot(times, self.controller.data['sim_altitude_agl'], 'blue', label='True AGL', linewidth=2)
        ax1.plot(times, self.controller.data['filtered_altitude_agl'], 'red', label='Filtered AGL', linewidth=2)

        # Add raw barometer data
        raw_times = []
        raw_altitudes = []
        for i, raw_alt in enumerate(self.controller.data['raw_altitude_agl']):
            if raw_alt is not None:
                raw_times.append(times[i])
                raw_altitudes.append(raw_alt)

        if raw_times:
            ax1.scatter(raw_times, raw_altitudes, c='orange', s=10, alpha=0.6, label='Raw Barometer', zorder=3)

        ax1.set_ylabel("Altitude AGL (m)")
        ax1.set_title("Altitude Tracking (Above Ground Level)")
        ax1.legend()
        ax1.grid(True, alpha=0.3)

        # Velocity tracking
        ax2.plot(times, self.controller.data['sim_velocity'], 'darkgreen', label='True', linewidth=2)
        ax2.plot(times, self.controller.data['filtered_velocity'], 'orange', label='Filtered', linewidth=2)
        ax2.set_ylabel("Velocity (m/s)")
        ax2.set_title("Velocity Tracking")
        ax2.legend()
        ax2.grid(True, alpha=0.3)

        # Deployment
        ax3.plot(times, self.controller.data['deployment'], 'purple', linewidth=2)
        ax3.set_ylabel("Deployment Level")
        ax3.set_xlabel("Time (s)")
        ax3.set_title("Air Brake Deployment")
        ax3.grid(True, alpha=0.3)

        # Apogee prediction
        ax4.plot(times, self.controller.data['predicted_apogee_agl'], 'darkred', linewidth=2, label='Predicted AGL')
        ax4.axhline(y=self.config.target_apogee, color='forestgreen', linestyle='--', linewidth=2, label='Target AGL')
        ax4.set_ylabel("Apogee AGL (m)")
        ax4.set_xlabel("Time (s)")
        ax4.set_title("Apogee Prediction (Above Ground Level)")
        ax4.legend()
        ax4.grid(True, alpha=0.3)

        plt.tight_layout()
        plt.show()

    def print_summary(self, flight):
        """Print flight summary."""
        flight_apogee_agl = flight.apogee - self.config.env_elevation
        error = flight_apogee_agl - self.config.target_apogee
        error_pct = 100 * error / self.config.target_apogee

        print(f"\n{'=' * 50}")
        print(f"FLIGHT SUMMARY")
        print(f"{'=' * 50}")
        print(f"Apogee AGL: {flight_apogee_agl:.1f}m (Target: {self.config.target_apogee:.1f}m)")
        print(f"Error: {error:.1f}m ({error_pct:.1f}%)")
        print(f"Flight Time: {max(self.controller.data['time']):.2f}s")
        print(f"Data Points: {len(self.controller.data['time'])}")
        print(f"{'=' * 50}")

    def run_full_simulation(self):
        """Run complete simulation with analysis."""
        try:
            print("Setting up simulation...")
            print(f"Target Apogee: {self.config.target_apogee:.1f}m AGL")
            print(f"Environment Elevation: {self.config.env_elevation}m ASL")
            print(f"Apogee Prediction Cd: {self.config.apogee_prediction_cd}")

            environment = self.setup_environment()
            rocket, motor = self.setup_rocket()

            print("Running simulation...")
            flight = self.run_simulation(rocket, environment)

            print("Exporting data...")
            self.export_to_csv(flight)

            print("Creating plots...")
            self.plot_results()

            self.print_summary(flight)

        except Exception as e:
            print(f"Simulation error: {e}")
            import traceback
            traceback.print_exc()


def main():
    config = ControllerConfig()
    rocket_config = RocketConfig()
    runner = SimulationRunner(config, rocket_config)
    runner.run_full_simulation()


if __name__ == "__main__":
    main()