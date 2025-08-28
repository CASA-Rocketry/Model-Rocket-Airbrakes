from dataclasses import dataclass


@dataclass
class ControllerConfig:
    # Physical parameters
    rocket_radius = 0.028
    airbrake_area = 0.00246300864
    air_density = 1.2
    burnout_mass = 0.561
    dry_mass = 0.534
    I_xx = 0.031
    I_yy = 0.031
    I_zz = 0.0001
    rail_length = 3

    # Target and environment
    target_apogee = 229
    env_elevation = 200
    latitude = 30
    longitude = 30
    terminate_on_apogee = True

    # Sampling
    sampling_rate: int = 20

    # Kalman filter params
    alt_std = 0.173932
    model_y_std = 0.01
    model_v_std = 10
    model_a_std = 40

    # Control parameters
    kp_base = 20000.0
    max_deployment_rate = 2   # Percent deployed / time
    apogee_prediction_cd = 0.8   # Should match the rocket drag curve

    # Calibration parameters
    calibration_sample_size = 1
    calibration_sample_rate = 0.020
    calibration_point = 0.0

    # Timing parameters
    burn_time = 1.5
    target_duration = 37.5

    # Constants
    sealevel_pressure_kpa = 101.325