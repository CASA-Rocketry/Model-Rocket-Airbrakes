from dataclasses import dataclass


@dataclass
class ControllerConfig:
    # Physical parameters
    rocket_radius = 0.028
    airbrake_area = 0.00246300864
    air_density = 1.2
    burnout_mass = 0.571
    dry_mass = 0.544
    I_xx = 0.031
    I_yy = 0.031
    I_zz = 0.0001
    rail_length = 3
    com_no_motor = 0.3

    # Fins
    n_fins = 4
    root_chord = 0.055
    tip_chord = 0.03
    span = 0.032
    fin_position = 0.66
    cant_angle = 0

    # Chute
    chute_cd = 0.8
    deployment_alt = 210


    # Target and environment
    target_apogee = 228
    env_elevation = 260
    latitude = 38
    longitude = 92
    terminate_on_apogee = True

    # Sampling
    sampling_rate: int = 20

    # Kalman filter params
    alt_std = 1.73932
    model_y_std = 0.001
    model_v_std = 1
    model_a_std = 1000

    # Control parameters
    kp_base = 20000.0
    max_deployment_rate = 3   # deployment / time
    apogee_prediction_cd = 0.8   # Should match the rocket drag curve
    airbrake_drag = 0.5    # Max Cd from airbrake. Needs to match airbrake drag curve

    # Calibration parameters
    calibration_sample_size = 1
    calibration_sample_rate = 0.020
    calibration_point = 0.0

    # Timing parameters
    burn_time = 1.5
    target_duration = 37.5

    # Constants
    sealevel_pressure_kpa = 101.325