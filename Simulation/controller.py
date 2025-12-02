import numpy as np

from controller_functions.kalman_filter import KalmanAltitudeFilter
from config import Config
from controller_functions.predict_apogee import predict_apogee
from controller_functions.convert_p_2_alt import find_altitude


def quaternion_to_rotation_matrix(e0, e1, e2, e3):
    """Convert quaternion to rotation matrix"""
    # Normalize quaternion
    norm = np.sqrt(e0**2 + e1**2 + e2**2 + e3**2)
    if norm > 0:
        e0, e1, e2, e3 = e0/norm, e1/norm, e2/norm, e3/norm

    # Convert to rotation matrix (body to world)
    R = np.array([
        [1 - 2*(e2**2 + e3**2), 2*(e1*e2 - e0*e3), 2*(e1*e3 + e0*e2)],
        [2*(e1*e2 + e0*e3), 1 - 2*(e1**2 + e3**2), 2*(e2*e3 - e0*e1)],
        [2*(e1*e3 - e0*e2), 2*(e2*e3 + e0*e1), 1 - 2*(e1**2 + e2**2)]
    ])

    return R


def correct_accelerometer_orientation(accel_body, e0, e1, e2, e3):
    """
    Transform accelerometer reading from body frame to world frame

    Args:
        accel_body: 3D acceleration vector in body frame [ax, ay, az]
        e0, e1, e2, e3: Quaternion components representing rocket orientation

    Returns:
        Vertical acceleration component in world frame (positive = up)
    """
    R = quaternion_to_rotation_matrix(e0, e1, e2, e3)
    accel_world = R @ accel_body
    return accel_world[2]  # Return vertical component


class Control:

    def __init__(self, config: Config):

        self.config = config

        self.last_deployment = 0
        self.last_time = 0
        self.last_velocity = 0
        self.filter_init = False
        self.control_active = False
        self.kalman_filter = KalmanAltitudeFilter(self.config)

        # Data storage
        self.data = {
            'time': [], 'sim_altitude_agl': [], 'raw_altitude_agl': [], 'filtered_altitude_agl': [],
            'sim_velocity': [], 'filtered_velocity': [], 'deployment': [], 'desired_deployment': [],
            'predicted_apogee': [], 'predicted_apogee_no_brake': [],'control_active': [],
            'filtered_acceleration': [], 'error': [], 'raw_acceleration': [], 'sim_acceleration': []
        }

        self.p_0 = 0
        self.i_error = 0
        self.i_error_list = []
        self.last_desired_deployment = 0
        self.last_error = 0
        self.leak_f = config.leak_f

    def controller(self, time, sampling_rate, state, state_history, observed_variables, air_brakes, sensors):
        # Extract state
        altitude = state[2]  # MSL altitude
        velocity = state[5]  # Vertical velocity (positive = up)
        e0 = state[6]
        e1 = state[7]
        e2 = state[8]
        e3 = state[9]

        # Convert to AGL
        altitude_agl = altitude - self.config.env_elevation

        # Don't deploy on descent
        if velocity <= 0:
            self.last_desired_deployment = 0.0
            self.control_active = False
        else:
            self.control_active = True

        # Don't deploy during motor burn
        if time < self.config.burn_time:
            self.last_desired_deployment = 0.0
            self.control_active = False
        else:
            self.control_active = True

        # Don't deploy if airbrake is disabled
        if self.config.use_airbrake == False:
            self.control_active = False

        # Only update state if this is a new timestep (to handle multiple calls per timestep)
        if time - self.last_time >= 1.0 / sampling_rate * 0.5:  # More than half a sampling period and control active

            # Create sensor objects
            barometer = sensors[0]
            accelerometer = sensors[1]

            # Filtering
            if self.filter_init == False:
                # Initialize with AGL altitude
                self.p_0 = barometer.measurement
                measurement_agl = find_altitude(barometer.measurement, self.p_0)
                self.kalman_filter.initialize(measurement_agl, sampling_rate)
                self.filter_init = True

                # Use initial measurements for first data point
                if self.config.use_orientation_correction:
                    # Transform accelerometer from body frame to world frame
                    measurement_accel = correct_accelerometer_orientation(
                        accelerometer.measurement, e0, e1, e2, e3
                    )
                else:
                    # Use raw Z-axis measurement (assumes rocket is vertical)
                    measurement_accel = accelerometer.measurement[2]

                filtered_y = measurement_agl
                filtered_v = 0
                filtered_a = measurement_accel

            else:
                measurement_agl = find_altitude(barometer.measurement, self.p_0)

                if self.config.use_orientation_correction:
                    # Transform accelerometer from body frame to world frame
                    measurement_accel = correct_accelerometer_orientation(
                        accelerometer.measurement, e0, e1, e2, e3
                    )
                else:
                    # Use raw Z-axis measurement (assumes rocket is vertical)
                    measurement_accel = accelerometer.measurement[2]

                self.kalman_filter.update(measurement_agl, measurement_accel, time, self.config.burn_time)
                filtered_y = self.kalman_filter.getYEstimate()
                filtered_v = self.kalman_filter.getVEstimate()
                filtered_a = self.kalman_filter.getAEstimate()

            predicted_apogee_w_brake = predict_apogee(filtered_y, filtered_v, self.last_desired_deployment, self.config)
            predicted_apogee_no_brake = predict_apogee(filtered_y, filtered_v, self.last_desired_deployment, self.config,
                                                       combined_cd=False)

            # Calculate error
            error = predicted_apogee_w_brake - self.config.target_apogee

            # Get dt
            dt = 1 / self.config.sampling_rate
            self.last_time = time

            # in controller
            alpha = np.exp(-dt / self.leak_f)  # decay factor per step
            if self.control_active and time > self.config.burn_time + 1.5:
                #self.i_error = alpha * self.i_error + (1 - alpha) * (error * self.leak_f)
                self.i_error += error * dt

            # Calculate error derivative
            d_error = (error - self.last_error) / dt
            self.last_error = error

            kp = self.config.kp
            ki = self.config.ki
            kd = self.config.kd
            deadband = self.config.deadband

            # Find deployment change
            deployment_change = kp * error + kd * d_error + ki * self.i_error

            # Apply deadbanding
            if abs(error) < deadband:
                deployment_change *= (abs(error) / deadband)

            # Desired deployment acts fully independent of rate-limited deployment
            desired_deployment = self.last_desired_deployment + deployment_change
            desired_deployment = np.clip(desired_deployment, 0.0, 1.0)
            self.last_desired_deployment = desired_deployment

            #print(kp * error,
            #      ki * self.i_error,
            #      kd * d_error)

            if self.control_active == True:
                max_change = self.config.max_deployment_rate * dt
                actual_change = desired_deployment - self.last_deployment
                limited_change = np.clip(actual_change, -max_change, max_change)
                deployment = self.last_deployment + limited_change
                air_brakes.deployment_level = np.clip(deployment, 0.0, 1.0)

                # Update state for next iteration
                self.last_deployment = air_brakes.deployment_level
            else:
                air_brakes.deployment_level = self.last_deployment
                self.last_desired_deployment = 0
                desired_deployment = 0

            # Store data for analysis (including temperature and error)
            error = predicted_apogee_w_brake - self.config.target_apogee if predicted_apogee_w_brake > 0 else 0

            # Compute sim acceleration from velocity derivative
            if dt > 0:
                sim_accel = (velocity - self.last_velocity) / dt
            else:
                sim_accel = 0
            self.last_velocity = velocity

            self.data['time'].append(round(time, 3))
            self.data['sim_altitude_agl'].append(round(altitude_agl, 3))
            self.data['raw_altitude_agl'].append(round(measurement_agl, 3))
            self.data['filtered_altitude_agl'].append(round(filtered_y, 3))
            self.data['sim_velocity'].append(round(state[5], 3))
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

        else:
            # Same timestep
            air_brakes.deployment_level = self.last_deployment

        return