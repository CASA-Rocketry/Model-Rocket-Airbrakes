import math
import numpy as np
from kalman_filter import KalmanAltitudeFilter
from config import ControllerConfig

import math
import numpy as np


class AirbrakeController:
    """Airbrake controller with enhanced temperature-aware barometer"""

    def __init__(self, config: ControllerConfig, motor_burn_time: float):
        self.config = config
        self.motor_burn_time = motor_burn_time
        self.filter = KalmanAltitudeFilter(config)

        self.sea_level_pressure = config.sealevel_pressure_kpa * 1000
        self.ground_pressure = None
        self.ground_temperature = None  # Ground reference temperature

        # Temperature model parameters
        self.standard_lapse_rate = 0.0065  # K/m
        self.standard_temp_sea_level = 288.15  # K (15°C)

        # Rate limiter state
        self.rate_limiter_initialized = False
        self.previous_deployment = 0.0
        self.previous_time = None

        # Data storage
        self.data = {
            'time': [], 'sim_altitude_agl': [], 'raw_altitude_agl': [], 'filtered_altitude_agl': [],
            'sim_velocity': [], 'filtered_velocity': [], 'deployment': [], 'predicted_apogee_agl': [],
            'motor_burning': [], 'control_active': [], 'filtered_acceleration': [], 'temperature': []
        }

    def _get_atmospheric_temperature(self, altitude_asl, flight_time=None, environment=None):
        """
        Get atmospheric temperature using multiple approaches

        Args:
            altitude_asl: Altitude above sea level (m)
            flight_time: Current flight time (optional, for environment lookup)
            environment: RocketPy environment object (optional)

        Returns:
            Temperature in Kelvin
        """

        # Approach 1: Try to get from RocketPy environment if available
        if environment is not None and flight_time is not None:
            try:
                # RocketPy environment has temperature vs altitude data
                temp_celsius = environment.temperature(altitude_asl)
                return temp_celsius + 273.15  # Convert to Kelvin
            except:
                pass

        # Approach 2: Use International Standard Atmosphere model
        if self.ground_temperature is None:
            # Initialize ground temperature from environment or use standard
            if environment is not None:
                try:
                    ground_temp_celsius = environment.temperature(self.config.env_elevation)
                    self.ground_temperature = ground_temp_celsius + 273.15
                except:
                    self.ground_temperature = self.standard_temp_sea_level - (
                                self.config.env_elevation * self.standard_lapse_rate)
            else:
                # Use standard atmosphere
                self.ground_temperature = self.standard_temp_sea_level - (
                            self.config.env_elevation * self.standard_lapse_rate)

        # Calculate temperature at altitude using lapse rate
        height_above_ground = altitude_asl - self.config.env_elevation
        temperature = self.ground_temperature - (self.standard_lapse_rate * height_above_ground)

        # Ensure temperature doesn't go below reasonable limits
        return max(temperature, 200.0)  # Don't go below -73°C

    def _read_barometer(self, sensors, flight_time=None, environment=None):
        """Read barometer sensor and return altitude above ground level (AGL)."""
        # Physical constants
        R = 8.314462618  # J/(mol*K)
        M = 0.0289644  # kg/mol (air)
        g = 9.80665  # m/s^2

        for sensor in sensors:
            if (
                    hasattr(sensor, "measured_data")
                    and sensor.name == "Barometer"
                    and hasattr(sensor, "measurement")
                    and sensor.measurement is not None
            ):
                try:
                    pressure_pa = float(sensor.measurement)
                    if pressure_pa <= 0:
                        return None, False, None

                    # Initialize ground reference
                    if self.ground_pressure is None:
                        self.ground_pressure = pressure_pa
                        # Get initial temperature
                        self.ground_temperature = self._get_atmospheric_temperature(
                            self.config.env_elevation, flight_time, environment)

                    if self.ground_pressure and self.ground_pressure > 0:
                        # First, get rough altitude estimate using ground temperature
                        T_ground = self.ground_temperature
                        exponent = (R * self.standard_lapse_rate) / (g * M)
                        ratio = pressure_pa / self.ground_pressure
                        rough_altitude = self.config.env_elevation + (T_ground / self.standard_lapse_rate) * (
                                    1.0 - ratio ** exponent)

                        # Get temperature at estimated altitude
                        T_altitude = self._get_atmospheric_temperature(rough_altitude, flight_time, environment)

                        # Use average temperature for more accurate calculation
                        T_avg = (T_ground + T_altitude) / 2.0

                        # Recalculate altitude with average temperature
                        altitude_asl = self.config.env_elevation + (T_avg / self.standard_lapse_rate) * (
                                    1.0 - ratio ** exponent)
                        altitude_agl = altitude_asl - self.config.env_elevation

                        return altitude_agl, True, T_altitude
                    else:
                        return None, False, None

                except (ValueError, TypeError, AttributeError) as e:
                    print(f"Barometer read error: {e}")
                    return None, False, None

        return None, False, None

    def _predict_apogee(self, altitude_agl: float, velocity: float, current_deployment: float = 0.0):
        """Apogee predictor for control algorithm with combined rocket and airbrake drag"""
        if velocity <= 0:
            return altitude_agl

        # Calculate combined drag coefficient
        # Rocket base drag coefficient
        rocket_cd = self.config.apogee_prediction_cd

        # Airbrake contribution based on current deployment level
        airbrake_cd = current_deployment * self.config.airbrake_drag

        # Combined drag coefficient (rocket + airbrake)
        combined_cd = rocket_cd + airbrake_cd

        # Use rocket's reference area for drag calculation
        # (airbrake area is already accounted for in the airbrake_drag coefficient)
        rocket_reference_area = 3.14159 * (self.config.rocket_radius ** 2)

        # Calculate drag parameter k with combined drag
        k = 0.5 * self.config.air_density * combined_cd * rocket_reference_area
        if k <= 0:
            return altitude_agl

        log_arg = (k * velocity ** 2) / (self.config.burnout_mass * 9.81) + 1
        if log_arg <= 0:
            return altitude_agl

        delta_altitude = (self.config.burnout_mass / (2 * k)) * math.log(log_arg)
        predicted_apogee_agl = altitude_agl + delta_altitude

        # Sanity check
        if predicted_apogee_agl < altitude_agl:
            print(
                f"Warning: Predicted apogee ({predicted_apogee_agl:.1f}m) below current altitude ({altitude_agl:.1f}m)")
            print(f"  Combined Cd: {combined_cd:.3f} (rocket: {rocket_cd:.3f}, airbrake: {airbrake_cd:.3f})")
            return altitude_agl

        return predicted_apogee_agl

    def _calculate_deployment(self, predicted_apogee_agl: float, velocity: float, current_time: float):
        """Calculate airbrake deployment (simplified version)"""
        # Calculate desired deployment based on error
        error = predicted_apogee_agl - self.config.target_apogee
        if error <= 0:
            desired_deployment = 0.0
        else:
            velocity_factor = velocity ** 3 + 15.0
            kp = self.config.kp_base #/ velocity_factor
            desired_deployment = kp * error

        # Clamp desired deployment [0, 1]
        desired_deployment = float(np.clip(desired_deployment, 0.0, 1.0))

        # Rate limiting (existing code)
        if not self.rate_limiter_initialized:
            self.previous_deployment = 0.0
            self.previous_time = current_time
            self.rate_limiter_initialized = True
            return 0.0

        # Calculate actual time interval
        dt = current_time - self.previous_time
        if dt <= 1e-6:  # Handle zero or negative time intervals
            return self.previous_deployment

        # Calculate maximum allowed change this timestep
        max_change = self.config.max_deployment_rate * dt

        # Calculate actual change needed
        change_needed = desired_deployment - self.previous_deployment

        # Apply rate limiting
        if change_needed > max_change:
            new_deployment = self.previous_deployment + max_change
        elif change_needed < -max_change:
            new_deployment = self.previous_deployment - max_change
        else:
            new_deployment = desired_deployment

        # Final clamp to [0, 1]
        new_deployment = float(np.clip(new_deployment, 0.0, 1.0))

        # Update state for next iteration
        self.previous_deployment = new_deployment
        self.previous_time = current_time

        return new_deployment

    def control(self, time: float, sampling_rate: int, state: np.ndarray,
                state_history, observed_variables, air_brakes, sensors, environment=None):
        """Main control function with environment parameter for temperature"""

        # Convert ASL to AGL for all calculations
        true_altitude_asl = state[2]
        true_altitude_agl = max(0, true_altitude_asl - self.config.env_elevation)
        true_velocity = state[5]

        # Read barometer (returns AGL altitude and temperature)
        barometer_altitude_agl, barometer_available, current_temp = self._read_barometer(
            sensors, time, environment)

        # Initialize defaults
        filtered_altitude_agl = barometer_altitude_agl
        filtered_velocity = 0.0
        filtered_acceleration = 0.0
        predicted_apogee_agl = 0.0
        deployment_level = 0.0
        motor_burning = time < self.motor_burn_time
        control_active = False

        # Get current deployment level from airbrakes
        current_deployment = getattr(air_brakes, 'deployment_level', 0.0)

        # Process barometer data if available
        if barometer_available:
            if not self.filter.initialized:
                # Initialize filter with AGL altitude
                self.filter.initialize(barometer_altitude_agl, sampling_rate)
                filtered_altitude_agl = barometer_altitude_agl
                filtered_velocity = 0.0
                filtered_acceleration = 0.0
            else:
                # Update filter with AGL altitude, get AGL results
                filtered_altitude_agl, filtered_velocity = self.filter.update(
                    barometer_altitude_agl, time, self.motor_burn_time)

                filtered_acceleration = self.filter.getAEstimate()

            # Predict apogee in AGL coordinates with current deployment
            predicted_apogee_agl = self._predict_apogee(
                filtered_altitude_agl, filtered_velocity, current_deployment)

            # Control logic
            # Only active after motor burnout and with positive velocity
            if not motor_burning and filtered_velocity > 0:
                deployment_level = self._calculate_deployment(
                    predicted_apogee_agl, filtered_velocity, time)

                air_brakes.deployment_level = deployment_level
                control_active = True

        # Store data for analysis (including temperature)
        self.data['time'].append(time)
        self.data['sim_altitude_agl'].append(true_altitude_agl)
        self.data['raw_altitude_agl'].append(barometer_altitude_agl)
        self.data['filtered_altitude_agl'].append(filtered_altitude_agl)
        self.data['sim_velocity'].append(true_velocity)
        self.data['filtered_velocity'].append(filtered_velocity)
        self.data['deployment'].append(deployment_level)
        self.data['predicted_apogee_agl'].append(predicted_apogee_agl)
        self.data['motor_burning'].append(motor_burning)
        self.data['control_active'].append(control_active)
        self.data['filtered_acceleration'].append(filtered_acceleration)
        self.data['temperature'].append(current_temp - 273.15 if current_temp else None)  # Store in Celsius

        return time, deployment_level, predicted_apogee_agl