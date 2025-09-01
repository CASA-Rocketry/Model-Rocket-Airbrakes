from rocketpy import Environment, Rocket, Flight, Barometer
from rocketpy.motors import GenericMotor
import pandas as pd
import numpy as np
from controller import AirbrakeController
from config import ControllerConfig

class SimulationRunner:
    def __init__(self, config: ControllerConfig):
        self.config = config
        self.controller = None

    def setup_environment(self):
        return Environment(
            latitude=self.config.latitude,
            longitude=self.config.longitude,
            elevation=self.config.env_elevation,
        )

    def setup_rocket(self):
        rocket = Rocket(
            radius=self.config.rocket_radius,
            mass=self.config.dry_mass,
            inertia=(
                self.config.I_xx,
                self.config.I_yy,
                self.config.I_zz
            ),
            power_off_drag="rocket_drag_curve.csv",
            power_on_drag="rocket_drag_curve.csv",
            center_of_mass_without_motor=self.config.com_no_motor,
            coordinate_system_orientation="nose_to_tail",
        )

        motor = GenericMotor.load_from_eng_file(
            file_name="AeroTech_F42T_L.eng",
            chamber_radius=0.0145,
            chamber_height=0.083
        )
        rocket.add_motor(motor, position=0.77)

        rocket.add_nose(length=0.13, kind="von karman", position=0)
        rocket.add_trapezoidal_fins(
            n=self.config.n_fins, root_chord=self.config.root_chord, tip_chord=self.config.tip_chord, span=self.config.span,
            position=self.config.fin_position, cant_angle=self.config.cant_angle)
        rocket.add_parachute(
            name="Chute", cd_s=self.config.chute_cd, trigger=self.config.deployment_alt, sampling_rate=10,
            lag=0, noise=(0, 8.3, 0.5))

        barometer = Barometer(
            sampling_rate=self.config.sampling_rate,
            measurement_range=100000,
            resolution=0.2,  # 0.2 Pa resolution as per datasheet
            noise_density=0.0,
            noise_variance=(0.75) ** 2,
            random_walk_density=0.0,
            constant_bias=0.0,
            operating_temperature=25,
            temperature_bias=0.0,
            temperature_scale_factor=0.0,
            name="Barometer"
        )
        rocket.add_sensor(barometer, position=(0, 0, 0.35))

        self.controller = AirbrakeController(self.config, motor.burn_out_time)

        rocket.add_air_brakes(
            drag_coefficient_curve="airbrake_drag_curve.csv",
            controller_function=self.controller.control,
            sampling_rate=self.config.sampling_rate,
            reference_area=self.config.airbrake_area,
            clamp=True,
            initial_observed_variables=[0],
            override_rocket_drag=False,
            name="Air Brakes",
        )

        rocket.draw()

        return rocket, motor

    def run_simulation(self, rocket, environment):
        return Flight(
            rocket=rocket,
            environment=environment,
            rail_length=self.config.rail_length,
            inclination=85,
            heading=0,
            time_overshoot=False,
            terminate_on_apogee=self.config.terminate_on_apogee
        )

    def export_to_csv(self, flight):
        """Export flight data to CSV"""
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

            # Add filtered acceleration if available
            if 'filtered_acceleration' in self.controller.data:
                df['Filtered_Acceleration_ms2'] = self.controller.data['filtered_acceleration']

            # Calculate errors
            df['Altitude_Error_m'] = np.abs(df['Simulated_Altitude_AGL_m'] - df['Filtered_Altitude_AGL_m'])
            df['Velocity_Error_ms'] = np.abs(df['Simulated_Velocity_ms'] - df['Filtered_Velocity_ms'])
            df['Apogee_Error_m'] = df['Predicted_Apogee_AGL_m'] - self.config.target_apogee

            # Add flight data if available
            if flight is not None:
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
            filename = 'rocket_flight_data.csv'
            df.to_csv(filename, index=False)
            print(f"\nData exported to {filename}")
            print(f"Rows: {len(df)}, Columns: {len(df.columns)}")

        except Exception as e:
            print(f"Error exporting CSV: {e}")
            import traceback
            traceback.print_exc()

    def print_summary(self, flight):
        """Print flight summary"""
        if flight is None:
            print(f"\n{'=' * 50}")
            print(f"FLIGHT SUMMARY")
            print(f"{'=' * 50}")
            print("ERROR: Flight simulation failed")
            print(f"Data Points: {len(self.controller.data['time']) if self.controller and self.controller.data['time'] else 0}")
            print(f"{'=' * 50}")
            return

        try:
            flight_apogee_agl = flight.apogee - self.config.env_elevation
            error = flight_apogee_agl - self.config.target_apogee
            error_pct = 100 * error / self.config.target_apogee

            print(f"\n{'=' * 50}")
            print(f"FLIGHT SUMMARY")
            print(f"{'=' * 50}")
            print(f"Apogee AGL: {flight_apogee_agl:.1f}m (Target: {self.config.target_apogee:.1f}m)")
            print(f"Error: {error:.1f}m ({error_pct:.1f}%)")
            print(f"Flight Time: {max(self.controller.data['time']) if self.controller and self.controller.data['time'] else 'N/A'}s")
            print(f"Data Points: {len(self.controller.data['time']) if self.controller and self.controller.data['time'] else 0}")
            print(f"{'=' * 50}")
        except Exception as e:
            print(f"Error printing summary: {e}")

    def run_full_simulation(self):
        """Run complete simulation with analysis"""
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

            self.print_summary(flight)

            return flight

        except Exception as e:
            print(f"Simulation error: {e}")
            import traceback
            traceback.print_exc()
            return None  # Return None if simulation fails