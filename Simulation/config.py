from dataclasses import dataclass


@dataclass
class ControllerConfig:
    # Physical parameters
    rocket_radius = 0.028
    airbrake_area = 3.14159 * rocket_radius ** 2 # This is just the rocket cross-section
    air_density = 1.2 # kg/m3
    dry_mass = 0.53
    burnout_mass = dry_mass + 0.027
    I_xx = 0.031 # Get from CAD file (kg-m3)
    I_yy = 0.031
    I_zz = 0.0001
    rail_length = 2 # meters
    com_no_motor = 0.3 # meters

    # Fins
    n_fins = 4
    root_chord = 0.055 # all meters
    tip_chord = 0.03
    span = 0.032
    fin_position = 0.66
    cant_angle = 0

    # Chute - NOT CONFIGURED YET
    chute_cd = 0.8
    deployment_alt = 210

    # Target and environment
    target_apogee = 228.6 # 228.6 meters = 750 feet
    env_elevation = 260
    latitude = 38
    longitude = 92
    terminate_on_apogee = True

    # Sampling
    sampling_rate: int = 20

    # Accelerometer parameters
    accel_range = 160  # m/s2
    accel_resolution = 0.1  # m/s2
    accel_noise_density = 0.1
    accel_noise_variance = 0.1
    accel_random_walk_density = 0.0
    accel_constant_bias = 0.0
    accel_temperature_bias = 0.0
    accel_temperature_scale_factor = 0.0
    accel_cross_axis_sensitivity = 0.0

    # Kalman filter params
    alt_std = 1.73932
    accel_std = 0.2  # Standard deviation for accelerometer measurements

    # Params for first phase of flight (before motor burnout)
    model_y_std = 0.001
    model_v_std = 1
    model_a_std = 1000

    # Params for second phase of flight (after motor burnout)
    time_factor = 0.2 # used for increasing accelerometer variance as the rocket becomes less vertical
    model_y_std_2 = 0.05
    model_v_std_2 = 1
    model_a_std_2 = 20

    # Control parameters
    kp_base = 400.0
    kd_base = 0.2
    control_deadband = 0.1
    hysteresis_factor = 0.8 # used to stop oscillations around deadband edge

    max_deployment_rate = 1.5   # deployment / time
    apogee_prediction_cd = 0.8   #Should match the rocket drag curve
    airbrake_drag = 0.5    # Max Cd from airbrake. Needs to match airbrake drag curve

    # Calibration parameters
    calibration_sample_size = 1
    calibration_sample_rate = 0.020
    calibration_point = 0.0

    # Timing parameters
    burn_time = 1.3
    target_duration = 37.5 # Not implemented yet

    # Constants
    sealevel_pressure_kpa = 101.325