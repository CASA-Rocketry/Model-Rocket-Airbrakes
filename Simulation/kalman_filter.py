import numpy as np
from config import ControllerConfig


class KalmanAltitudeFilter:
    """Kalman filter for altitude and velocity estimation"""

    def __init__(self, config: ControllerConfig):
        self.config = config
        self.initialized = False
        self.previous_time = None

        # State estimate vector [position, velocity, acceleration]
        self.x = np.array([[0.0], [0.0], [0.0]])

        # Measurement vector
        self.z = np.array([[0.0]])

        # 3x3 identity matrix
        self.I = np.eye(3)

        # State transition matrix
        self.phi = np.eye(3)

        # State to measurement matrix
        self.H = np.array([[1.0, 0.0, 0.0]])

        # Measurement covariance
        self.R = np.array([[config.alt_std * config.alt_std]])

        # Process covariance
        self.Q = np.array([
            [config.model_y_std * config.model_y_std, 0.0, 0.0],
            [0.0, config.model_v_std * config.model_v_std, 0.0],
            [0.0, 0.0, config.model_a_std * config.model_a_std]
        ])

        # Error covariance
        self.P = np.eye(3)

        # Kalman gain
        self.K = np.zeros((3, 1))

    def initialize(self, initial_altitude_agl: float, sampling_rate: int):
        """Initialize filter with initial altitude"""
        self.x = np.array([[initial_altitude_agl], [0.0], [0.0]])
        self.P = np.eye(3)  # Reset to identity matrix
        self.initialized = True

    def updateKalmanFilter(self, measurement_agl: float, dt: float):
        """Standard Kalman Filter - Predict then Update"""
        # 1. PREDICT STEP
        # Update phi matrix
        self.phi = np.array([
            [1.0, dt, 0.5 * dt * dt],
            [0.0, 1.0, dt],
            [0.0, 0.0, 1.0]
        ])

        # Predict state and covariance forward
        self.x = self.phi @ self.x
        self.P = self.phi @ self.P @ self.phi.T + self.Q

        # 2. update step
        self.z[0, 0] = measurement_agl

        # Calculate Kalman gain
        HPH_R = self.H @ self.P @ self.H.T + self.R
        self.K = self.P @ self.H.T @ np.linalg.inv(HPH_R)

        # Update state estimate
        innovation = self.z - self.H @ self.x
        self.x = self.x + self.K @ innovation

        # Update covariance
        self.P = (self.I - self.K @ self.H) @ self.P

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
        """Get position estimate"""
        return float(self.x[0, 0])

    def getVEstimate(self):
        """Get velocity estimate"""
        return float(self.x[1, 0])

    def getAEstimate(self):
        """Get acceleration estimate"""
        return float(self.x[2, 0])