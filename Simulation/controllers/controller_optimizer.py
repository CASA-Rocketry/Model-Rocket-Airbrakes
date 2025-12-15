import numpy as np
from scipy.optimize import minimize_scalar

from .controller_base import ControllerBase
from .controller_functions.predict_apogee import predict_apogee
from config import Config


class ControllerOptimizer(ControllerBase):

    def __init__(self, config: Config):
        super().__init__(config)

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

        # Extract optimal deployment
        optimal_deployment = result.x

        # Clip to valid range (should already be in range, but just in case)
        optimal_deployment = np.clip(optimal_deployment, 0.0, 1.0)

        return optimal_deployment