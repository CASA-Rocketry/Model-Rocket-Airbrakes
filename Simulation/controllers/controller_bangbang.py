import numpy as np

from .controller_base import ControllerBase
from config import Config


class ControllerBangBang(ControllerBase):

    def __init__(self, config: Config):
        super().__init__(config)

    def compute_control(self, filtered_altitude, filtered_velocity, filtered_acceleration,
                       predicted_apogee_w_brake, predicted_apogee_no_brake,
                       error_w_brake, error_no_brake, dt):

        # Use error from predicted apogee without brake
        if error_no_brake > 0:
            desired_deployment = 1
        else:
            desired_deployment = 0

        return desired_deployment