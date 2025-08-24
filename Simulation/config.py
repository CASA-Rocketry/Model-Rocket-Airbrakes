from dataclasses import dataclass

@dataclass
class ControllerConfig:
    max_deployment_cd: float = 1.0
    rocket_radius: float = 0.028
    airbrake_area: float = 0.00246300864
    air_density: float = 1.2
    target_apogee: float = 228.6
    env_elevation: float = 200
    burnout_mass: float = 0.5
    sampling_rate: int = 20
    sensor_noise_variance: float = 0.5
    process_noise_burn: float = 200.0
    process_noise_coast: float = 25.0
    acceleration_decay_burn: float = 0.99
    acceleration_decay_coast: float = 0.95
    velocity_process_boost: float = 4.0
    kp_base: float = 7000.0
    max_deployment_rate: float = 0.5
    apogee_prediction_cd: float = 1.0