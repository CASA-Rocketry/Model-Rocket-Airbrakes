import pandas as pd
import numpy as np
from scipy.interpolate import interp1d
from .controller_base import ControllerBase
from config import Config


class ControllerFile(ControllerBase):

    def __init__(self, config: Config, deployment_file: str,
                 time_col: str = None, deployment_col: str = None,
                 time_unit: str = None):
        super().__init__(config)

        self.deployment_file = deployment_file
        # Use defaults if None provided (match real flight CSV format)
        self.time_col = time_col if time_col is not None else 'time'
        self.deployment_col = deployment_col if deployment_col is not None else 'Servo deployment_rate_limited'
        self.time_unit = time_unit if time_unit is not None else 's'

        # Load deployment data
        self.deployment_interpolator = None
        self.deployment_time_data = None
        self.deployment_value_data = None
        self.max_time = 0.0

        self._load_deployment_data()

        print(f"\nFile-based controller initialized:")
        print(f"  Deployment file: {deployment_file}")
        print(f"  Time range: 0.0 to {self.max_time:.3f} s")
        print(f"  Deployment range: {np.min(self.deployment_value_data):.3f} to {np.max(self.deployment_value_data):.3f}")
        print(f"  Data points: {len(self.deployment_time_data)}")

    def _load_deployment_data(self):
        """Load deployment data from CSV file"""
        import os

        # Check if file exists
        if not os.path.exists(self.deployment_file):
            raise FileNotFoundError(
                f"\nDeployment file not found: {self.deployment_file}\n\n"
                f"To use FILE controller, you need a CSV with deployment data.\n"
                f"Options:\n"
                f"  1. Run real flight processing first to create the file\n"
                f"  2. Create a custom deployment CSV with columns:\n"
                f"     - '{self.time_col}': time values\n"
                f"     - '{self.deployment_col}': deployment values (0-1)\n"
                f"  3. Change 'Control Algorithm' in config.csv to PID/BANGBANG/etc.\n"
            )

        # Load deployment data from CSV file
        df = pd.read_csv(self.deployment_file)

        # Validate columns exist
        if self.time_col not in df.columns:
            available_cols = ', '.join(df.columns[:10])  # Show first 10 columns
            raise ValueError(
                f"Time column '{self.time_col}' not found in {self.deployment_file}\n"
                f"Available columns: {available_cols}..."
            )
        if self.deployment_col not in df.columns:
            available_cols = ', '.join(df.columns[:10])
            raise ValueError(
                f"Deployment column '{self.deployment_col}' not found in {self.deployment_file}\n"
                f"Available columns: {available_cols}..."
            )

        # Extract time and deployment data
        time_data = df[self.time_col].values
        deployment_data = df[self.deployment_col].values

        # Convert time to seconds if needed
        if self.time_unit == 'us':
            time_data = (time_data - time_data[0]) / 1e6
        elif self.time_unit == 'ms':
            time_data = (time_data - time_data[0]) / 1e3
        elif self.time_unit == 's':
            time_data = time_data - time_data[0]
        else:
            raise ValueError(f"Invalid time_unit '{self.time_unit}'. Must be 'us', 'ms', or 's'")

        # Store data
        self.deployment_time_data = time_data
        self.deployment_value_data = deployment_data
        self.max_time = time_data[-1]

        # Verify data was loaded correctly
        if self.max_time <= 0:
            print(f"  WARNING: Max time is {self.max_time:.6f}s - time data may not be loaded correctly!")
            print(f"  First few time values: {time_data[:5]}")
            print(f"  Time unit conversion: {self.time_unit}")
        if np.max(deployment_data) == 0 and np.min(deployment_data) == 0:
            print(f"  WARNING: All deployment values are 0!")
            print(f"  Check that '{self.deployment_col}' column has non-zero values")

        # Create interpolator
        self.deployment_interpolator = interp1d(
            time_data,
            deployment_data,
            kind='linear',
            bounds_error=False,
            fill_value=(deployment_data[0], deployment_data[-1])  # Hold first/last values outside range
        )

    def get_deployment(self, time: float) -> float:
        """
        Get interpolated deployment at given time

        Args:
            time: Time in seconds

        Returns:
            Deployment value (0-1)
        """
        if self.deployment_interpolator is None:
            return 0.0

        return float(self.deployment_interpolator(time))

    def compute_control(self, filtered_altitude, filtered_velocity, filtered_acceleration,
                       predicted_apogee_w_brake, predicted_apogee_no_brake,
                       error_w_brake, error_no_brake, dt):
        """
        Compute control output (not used for file-based controller)

        This is required by ControllerBase abstract class but not used.
        Deployment comes from file interpolation instead.
        """
        return 0.0  # Not used

    def controller(self, time, sampling_rate, state, state_history, observed_variables, air_brakes, sensors):
        """
        Controller function called by RocketPy simulation

        Uses interpolated deployment from file but still calculates apogee prediction and error.

        Args:
            time: Current simulation time
            sampling_rate: Sampling rate
            state: Rocket state vector
            state_history: State history
            observed_variables: Observed variables
            air_brakes: Airbrake object
            sensors: Sensor readings from simulation

        Returns:
            Deployment value (0-1)
        """
        from .controller_functions.predict_apogee import predict_apogee
        from .controller_functions.convert_p_2_alt import find_altitude

        # Extract state
        altitude = state[2]  # MSL altitude
        velocity = state[5]  # Vertical velocity
        e0, e1, e2, e3 = state[6], state[7], state[8], state[9]
        altitude_agl = altitude - self.config.env_elevation

        # Control activation logic (same as base controller)
        if velocity <= 0 or time < self.config.burn_time or not self.config.use_airbrake:
            self.control_active = False
        else:
            self.control_active = True

        # Only update state if this is a new timestep
        if time - self.last_time >= 1.0 / sampling_rate * 0.5:
            # Create sensor objects
            barometer = sensors[0]
            accelerometer = sensors[1]

            # Filtering (same as base controller)
            if not self.filter_init:
                self.p_0 = barometer.measurement
                measurement_agl = find_altitude(barometer.measurement, self.p_0)
                self.kalman_filter.initialize(measurement_agl, sampling_rate)
                self.filter_init = True

                if self.config.use_orientation_correction:
                    from .controller_base import correct_accelerometer_orientation
                    measurement_accel = correct_accelerometer_orientation(
                        accelerometer.measurement, e0, e1, e2, e3
                    )
                else:
                    measurement_accel = accelerometer.measurement[2]

                filtered_y = measurement_agl
                filtered_v = 0
                filtered_a = measurement_accel
            else:
                measurement_agl = find_altitude(barometer.measurement, self.p_0)

                if self.config.use_orientation_correction:
                    from .controller_base import correct_accelerometer_orientation
                    measurement_accel = correct_accelerometer_orientation(
                        accelerometer.measurement, e0, e1, e2, e3
                    )
                else:
                    measurement_accel = accelerometer.measurement[2]

                self.kalman_filter.update(measurement_agl, measurement_accel, time, self.config.burn_time)
                filtered_y = self.kalman_filter.getYEstimate()
                filtered_v = self.kalman_filter.getVEstimate()
                filtered_a = self.kalman_filter.getAEstimate()

            # Predict apogee with current deployment (use last_deployment for prediction)
            predicted_apogee_w_brake = predict_apogee(filtered_y, filtered_v, self.last_deployment, self.config)
            predicted_apogee_no_brake = predict_apogee(filtered_y, filtered_v, 0.0, self.config, combined_cd=False)

            # Calculate error
            error_w_brake = predicted_apogee_w_brake - self.config.target_apogee
            error = error_w_brake

            # Get interpolated deployment from file
            desired_deployment = self.get_deployment(time)
            desired_deployment = np.clip(desired_deployment, 0.0, 1.0)

            # Apply rate limiting and control activation logic
            dt = 1 / sampling_rate
            if self.control_active:
                max_change = self.config.max_deployment_rate * dt
                actual_change = desired_deployment - self.last_deployment
                limited_change = np.clip(actual_change, -max_change, max_change)
                deployment = self.last_deployment + limited_change
                air_brakes.deployment_level = np.clip(deployment, 0.0, 1.0)
                self.last_deployment = air_brakes.deployment_level
            else:
                air_brakes.deployment_level = self.last_deployment
                desired_deployment = 0
                deployment = self.last_deployment

            # Compute sim acceleration
            if dt > 0:
                sim_accel = (velocity - self.last_velocity) / dt
            else:
                sim_accel = 0
            self.last_velocity = velocity

            # Log data
            self.data['time'].append(round(time, 3))
            self.data['sim_altitude_agl'].append(round(altitude_agl, 3))
            self.data['raw_altitude_agl'].append(round(measurement_agl, 3))
            self.data['filtered_altitude_agl'].append(round(filtered_y, 3))
            self.data['sim_velocity'].append(round(velocity, 3))
            self.data['filtered_velocity'].append(round(filtered_v, 3))
            self.data['deployment'].append(round(air_brakes.deployment_level, 3))
            self.data['desired_deployment'].append(round(desired_deployment, 3))
            self.data['predicted_apogee'].append(round(predicted_apogee_w_brake, 3))
            self.data['predicted_apogee_no_brake'].append(round(predicted_apogee_no_brake, 3))
            self.data['control_active'].append(round(self.control_active, 3))
            self.data['filtered_acceleration'].append(round(filtered_a, 3))
            self.data['error'].append(round(error, 3))
            self.data['raw_acceleration'].append(round(measurement_accel, 3))
            self.data['sim_acceleration'].append(round(sim_accel, 3))

            self.last_time = time
        else:
            # Same timestep
            air_brakes.deployment_level = self.last_deployment

        return
