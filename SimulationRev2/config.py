from dataclasses import dataclass


@dataclass
class Config:
    # Physical parameters
    rocket_radius = 0.028
    dry_mass = 0.573
    burnout_mass = dry_mass + 0.027
    I_xx = 0.031  # Get from CAD file (kg-m3)
    I_yy = 0.031
    I_zz = 0.0001
    com_no_motor = 0.3  # meters from nose

    # Motor parameters
    chamber_radius = 0.0145
    chamber_height = 0.083
    motor_position = 0.77

    # Nosecone parameters
    nosecone_length = 0.13
    nosecone_type = "von karman"

    air_density = 1.18 # kg/m3
    rail_length = 2 # meters

    # Fins
    n_fins = 4
    root_chord = 0.055 # all meters
    tip_chord = 0.03
    span = 0.032
    fin_position = 0.66
    cant_angle = 0

    # Chute - NOT CONFIGURED YET
    chute_cd = 0.4
    deployment_alt = 210

    # Target and environment
    target_apogee = 228.6 # 228.6 meters = 750 feet
    env_elevation = 260 # meters from sea level
    latitude = 38
    longitude = 92
    wind_speed = 1 # m/s

    # Sim parameters
    sampling_rate: int = 20 # Control algorithm frequency
    terminate_on_apogee = True

    # Accelerometer parameters
    accel_range = 160  # m/s2
    accel_resolution = 0.01  # m/s2
    accel_noise_density = 0
    accel_noise_variance = 0.017
    accel_random_walk_density = 0.0
    accel_constant_bias = 0.0
    accel_temperature_bias = 0.0
    accel_temperature_scale_factor = 0.0
    accel_cross_axis_sensitivity = 0.0
    accel_operating_temp = 25.0  # Celsius
    accele_position = 0.3  # meters from nose

    # Barometer parameters
    barometer_range = 120000  # Pa
    barometer_resolution = 1.0  # Pa
    barometer_noise_density = 0
    barometer_noise_variance = 3 ** 2
    barometer_random_walk_density = 0.0
    barometer_constant_bias = 0.0
    barometer_operating_temperature = 25.0  # Celsius
    barometer_temperature_bias = 0.0
    barometer_temperature_scale_factor = 0.0
    barometer_position = 0.3  # meters from nose

    # Kalman filter params
    alt_std = 0.26 # Meters for bmp390
    accel_std = 0.013  # Standard deviation for accelerometer measurements (bno055 -> 0.013,
    model_y_std = 0.05
    model_v_std = 0.1
    model_a_std = 0.015

    # Control parameters
    kp = 0.012
    ki = 0.0
    kd = 0
    deadband = 0.1 # Meters

    # Controller parameters
    max_deployment_rate = 2.5   # deployment / time
    apogee_prediction_cd = 0.71   # Should match the rocket drag curve. Based on 4th flight of rocket
    airbrake_drag = 0.35    # Max Cd from airbrake. Needs to match airbrake drag curve

    # Timing parameters
    burn_time = 1.4

    # Input data
    engine_file = "input_data/AeroTech_F42T_L.eng"
    airbrake_drag_curve_file = "input_data/airbrake_drag_curve.csv"
    rocket_drag_curve_file = "input_data/rocket_drag_curve.csv"