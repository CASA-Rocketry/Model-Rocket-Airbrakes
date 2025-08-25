import numpy as np
from config import ControllerConfig


class KalmanAltitudeFilter:
    """Kalman filter for altitude and velocity estimation - matches Arduino StateEstimation.ino"""

    def __init__(self, config: ControllerConfig):
        self.config = config
        self.initialized = False
        self.previous_time = None

        # State estimate vector [position, velocity, acceleration] - matches Arduino x
        self.x = np.array([[0.0], [0.0], [0.0]])

        # Measurement vector - matches Arduino z
        self.z = np.array([[0.0]])

        # 3x3 identity matrix - matches Arduino I
        self.I = np.eye(3)

        # State transition matrix - matches Arduino phi
        self.phi = np.eye(3)

        # State to measurement matrix - matches Arduino H
        self.H = np.array([[1.0, 0.0, 0.0]])

        # Measurement covariance - matches Arduino R
        self.R = np.array([[config.alt_std * config.alt_std]])

        # Process covariance - matches Arduino Q
        self.Q = np.array([
            [config.model_y_std * config.model_y_std, 0.0, 0.0],
            [0.0, config.model_v_std * config.model_v_std, 0.0],
            [0.0, 0.0, config.model_a_std * config.model_a_std]
        ])

        # Error covariance - matches Arduino P
        self.P = np.eye(3)

        # Kalman gain - matches Arduino K
        self.K = np.zeros((3, 1))

    def initialize(self, initial_altitude_agl: float, sampling_rate: int):
        """Initialize filter with initial altitude"""
        self.x = np.array([[initial_altitude_agl], [0.0], [0.0]])
        self.P = np.eye(3)  # Reset to identity matrix
        self.initialized = True

    def updateKalmanFilter(self, measurement_agl: float, dt: float):
        """Update Kalman filter - matches Arduino updateKalmanFilter() function"""
        if not self.initialized:
            raise RuntimeError("Filter not initialized")

        # Add data - matches Arduino code
        self.z[0, 0] = measurement_agl

        # Update phi - matches Arduino phi calculation
        self.phi = np.array([
            [1.0, dt, 0.5 * dt * dt],
            [0.0, 1.0, dt],
            [0.0, 0.0, 1.0]
        ])

        # Update Kalman Gain - matches Arduino K = P * ~H * Inverse((H * P * ~H + R))
        HPH_R = self.H @ self.P @ self.H.T + self.R
        self.K = self.P @ self.H.T @ np.linalg.inv(HPH_R)

        # Update Estimate - matches Arduino x = x + K * (z - H * x)
        innovation = self.z - self.H @ self.x
        self.x = self.x + self.K @ innovation

        # Update Covariance - matches Arduino P = (I - K * H) * P
        self.P = (self.I - self.K @ self.H) @ self.P

        # Project to next time stamp - matches Arduino projection
        self.x = self.phi @ self.x
        self.P = self.phi @ self.P @ self.phi.T + self.Q

    def update(self, measurement_agl: float, time: float, motor_burn_time: float):
        """Update filter and return estimates"""
        # Calculate dt
        if self.previous_time is not None:
            dt = time - self.previous_time
        else:
            dt = 1.0 / self.config.sampling_rate

        dt = max(dt, 1e-6)  # Prevent zero dt

        # Update the Kalman filter
        self.updateKalmanFilter(measurement_agl, dt)

        self.previous_time = time

        return self.getYEstimate(), self.getVEstimate()

    def getYEstimate(self):
        """Get position estimate - matches Arduino getYEstimate()"""
        return float(self.x[0, 0])

    def getVEstimate(self):
        """Get velocity estimate - matches Arduino getVEstimate()"""
        return float(self.x[1, 0])

    def getAEstimate(self):
        """Get acceleration estimate - matches Arduino getAEstimate()"""
        return float(self.x[2, 0])