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

        desired_deployment = result.x

        # Deadbanding
        if abs(error_w_brake) < self.config.deadband:
            desired_deployment = self.last_deployment

        # Clip to valid range
        desired_deployment = np.clip(desired_deployment, 0.0, 1.0)

        return desired_deployment