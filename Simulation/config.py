from dataclasses import dataclass


@dataclass
class ControllerConfig:
    # Physical parameters
    max_deployment_cd: float = 1.0
    rocket_radius: float = 0.028
    airbrake_area: float = 0.00246300864
    air_density: float = 1.2

    # Target and environment
    target_apogee: float = 229
    env_elevation: float = 200
    burnout_mass: float = 0.5

    # Sampling
    sampling_rate: int = 20

    # Kalman filter parameters
    sensor_noise_variance: float = 0.0302523
    acceleration_decay_burn: float = 1.0
    acceleration_decay_coast: float = 1.0

    # Control parameters
    kp_base: float = 7000.0
    max_deployment_rate: float = 0.5
    apogee_prediction_cd: float = 1.0

    # Calibration parameters
    calibration_sample_size: int = 100
    calibration_sample_rate: float = 0.020
    calibration_point: float = 0.0

    # Timing
    burn_time: float = 1.5

    # Target duration for time control
    target_duration: float = 37.5