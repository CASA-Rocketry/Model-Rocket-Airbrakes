import math
import numpy as np
from kalman_filter import KalmanAltitudeFilter
from config import ControllerConfig

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
        """Apogee predictor for control algorithm"""
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

        new_deployment = np.clip(new_deployment, lower_bound, upper_bound)

        return new_deployment

    def control(self, time: float, sampling_rate: int, state: np.ndarray,
                state_history, observed_variables, air_brakes, sensors):
        """
        Main control function - consistently uses AGL coordinates.

        Args:
            state: RocketPy state vector where state[2] is altitude ASL
        """

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

            # Predict apogee in AGL coordinates
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