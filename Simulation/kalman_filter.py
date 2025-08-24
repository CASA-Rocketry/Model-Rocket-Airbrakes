import numpy as np
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
from config import ControllerConfig

class KalmanAltitudeFilter:
    """Kalman filter for altitude and velocity estimation"""
    def __init__(self, config: ControllerConfig):
        self.config = config
        self.kf = None
        self.initialized = False
        self.previous_time = None

    def initialize(self, initial_altitude_agl: float, sampling_rate: int):
        """Initialize filter"""
        self.kf = KalmanFilter(dim_x=3, dim_z=1)
        self.kf.x = np.array([initial_altitude_agl, 0.0, 0.0])
        dt = 1.0 / sampling_rate
        self._update_matrices(dt, is_burning=True)
        self.kf.P = np.diag([1.0, 5.0, 20.0])
        self.initialized = True

    def _update_matrices(self, dt: float, is_burning: bool):
        """Update filter matrices"""
        decay = self.config.acceleration_decay_burn if is_burning else self.config.acceleration_decay_coast
        self.kf.F = np.array([[1, dt, 0.5 * dt ** 2], [0, 1, dt], [0, 0, decay]])
        self.kf.H = np.array([[1, 0, 0]])
        self.kf.R = np.array([[self.config.sensor_noise_variance]])
        q_var = self.config.process_noise_burn if is_burning else self.config.process_noise_coast
        self.kf.Q = Q_discrete_white_noise(dim=3, dt=dt, var=q_var)
        self.kf.Q[1, 1] *= self.config.velocity_process_boost

    def update(self, measurement_agl: float, time: float, motor_burn_time: float):
        """Update filter with barometer measurement and return altitude and velocity."""
        if not self.initialized:
            raise RuntimeError("Filter not initialized")

        dt = time - self.previous_time if self.previous_time is not None else 1.0 / self.config.sampling_rate
        dt = max(dt, 1e-6)
        is_burning = time < motor_burn_time

        self._update_matrices(dt, is_burning)
        self.kf.predict()
        self.kf.update(np.array([[measurement_agl]]))
        self.previous_time = time

        return float(self.kf.x[0]), float(self.kf.x[1])