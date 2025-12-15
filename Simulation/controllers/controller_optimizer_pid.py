import numpy as np
from scipy.optimize import minimize_scalar

from .controller_base import ControllerBase
from .controller_functions.predict_apogee import predict_apogee
from config import Config


class ControllerOptimizerPID(ControllerBase):

    def __init__(self, config: Config):
        super().__init__(config)

        self.last_deployment = 0
        self.i_error = 0
        self.i_error_list = []
        self.last_error = 0
        self.frequency = config.sampling_rate
        self.i_window = config.i_window

    def compute_control(self, filtered_altitude, filtered_velocity, filtered_acceleration,
                       predicted_apogee_w_brake, predicted_apogee_no_brake,
                       error_w_brake, error_no_brake, dt):
        # Optimizer calculates its own error internally

        def objective(deployment):
            # Clip deployment to valid range
            deployment = np.clip(deployment, 0.0, 1.0)

            # Predict apogee with this deployment value
            pred_apogee = predict_apogee(
                filtered_altitude,
                filtered_velocity,
                deployment,
                self.config
            )

            # Return error
            error = abs(pred_apogee - self.config.target_apogee)
            return error

        # Solve for optimal deployment using scipy
        result = minimize_scalar(
            objective,
            bounds=(0.0, 1.0),
            method='bounded'
        )

        deployment_opt = result.x

        error = deployment_opt - self.last_deployment
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

        # Update desired deployment
        deployment_change = (kp * error + ki * self.i_error + kd * d_error) * dt
        deployment = self.last_deployment + deployment_change
        deployment = np.clip(deployment, 0.0, 1.0)
        self.last_desired_deployment = deployment

        # Clip to valid range
        deployment = np.clip(deployment, 0.0, 1.0)

        return deployment