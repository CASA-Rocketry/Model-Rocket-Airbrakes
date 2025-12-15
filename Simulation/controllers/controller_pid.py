import numpy as np

from .controller_base import ControllerBase
from config import Config


class ControllerPID(ControllerBase):

    def __init__(self, config: Config):
        super().__init__(config)

        # PID-specific state
        self.i_error = 0
        self.i_error_list = []
        self.last_desired_deployment = 0
        self.last_error = 0
        self.frequency = config.sampling_rate
        self.i_window = config.i_window

    def compute_control(self, filtered_altitude, filtered_velocity, filtered_acceleration,
                       predicted_apogee_w_brake, predicted_apogee_no_brake,
                       error_w_brake, error_no_brake, dt):
        # Use error from predicted apogee with brake
        error = error_w_brake
        # Accumulate integral error with sliding window
        self.i_error_list.append(error * dt)
        if self.control_active and self.last_time > self.config.burn_time + 1.5:
            # Only integrate when control is active and after motor burn + settling time
            self.i_error = sum(self.i_error_list[-(self.frequency * self.i_window):])

        # Calculate error derivative
        d_error = (error - self.last_error) / dt
        self.last_error = error

        # Get PID gains from config
        kp = self.config.kp
        ki = self.config.ki
        kd = self.config.kd
        deadband = self.config.deadband

        # Calculate PID output (deployment change)
        deployment_change = kp * error * dt + kd * d_error + ki * self.i_error

        # Apply deadband scaling
        # When error is small (< deadband), reduce control effort proportionally
        if abs(error) < deadband:
            deployment_change *= (abs(error) / deadband)

        # Update desired deployment (integrating the change)
        desired_deployment = self.last_desired_deployment + deployment_change
        desired_deployment = np.clip(desired_deployment, 0.0, 1.0)
        self.last_desired_deployment = desired_deployment

        return desired_deployment