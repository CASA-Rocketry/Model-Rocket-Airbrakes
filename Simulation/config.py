from dataclasses import dataclass
import csv
import os

def load_config_from_csv(csv_path="config.csv"):
    """Load configuration values from CSV file"""
    config_values = {}

    # Get the directory of this file
    script_dir = os.path.dirname(os.path.abspath(__file__))
    full_path = os.path.join(script_dir, csv_path)

    with open(full_path, 'r', encoding='utf-8-sig') as f:
        reader = csv.reader(f)
        for row in reader:
            if len(row) >= 2:
                key = row[0].strip()
                value = row[1].strip()
                config_values[key] = value

    return config_values

# Load values from CSV
_csv_config = load_config_from_csv()

def _get_csv_value(key, default, value_type=float):
    """Get a value from CSV config with type conversion"""
    if key in _csv_config:
        val = _csv_config[key]
        if value_type == bool:
            return val.upper() == 'T'
        return value_type(val)
    return default

@dataclass
class Config:
    # Physical parameters
    rocket_radius = 0.028
    dry_mass = 0.557 #0.573
    burnout_mass = _get_csv_value("Burnout Mass (kg)", dry_mass + 0.027)
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

    air_density = _get_csv_value("Air Density (kg/m^3)", 1.18) # kg/m3
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
    target_apogee = _get_csv_value("Target Apogee (m)", 228.6) # 228.6 meters = 750 feet
    env_elevation = 260 # meters from sea level
    latitude = 38
    longitude = 92
    wind_speed = 1.5 # m/s

    # Sim parameters
    sampling_rate: int = 20 # Control algorithm frequency
    terminate_on_apogee = True

    # Accelerometer parameters - Set up for bno055
    accel_range = 160  # m/s2
    accel_resolution = 0.01  # m/s2
    accel_noise_density = 0.0015 # m/s2/sqrthz
    accel_noise_variance = 0.0
    accel_random_walk_density = 0.0
    accel_constant_bias = 0.0
    accel_temperature_bias = 0.0003 # %/C
    accel_temperature_scale_factor = 0.0
    accel_cross_axis_sensitivity = 0.01 # %
    accel_operating_temp = 25.0  # Celsius
    accele_position = 0.3  # meters from nose
    use_orientation_correction = False  # Correct accelerometer. Uses sim orientation.

    # Barometer parameters
    barometer_range = 120000  # Pa, not based on data sheet
    barometer_resolution = 1.0  # Pa, not based on data sheet
    barometer_noise_density = 0
    barometer_noise_variance = 3 ** 2 #
    barometer_random_walk_density = 0.0
    barometer_constant_bias = 0.0
    barometer_operating_temperature = 25.0  # Celsius
    barometer_temperature_bias = 0.0
    barometer_temperature_scale_factor = 0.0
    barometer_position = 0.3  # meters from nose

    # Kalman filter params
    alt_std = _get_csv_value("Kalman Measurement Y STD", 0.26) # Meters for bmp390
    accel_std = _get_csv_value("Kalman Measurement A STD", 0.013)  # Standard deviation for accelerometer measurements (bno055 -> 0.013,
    model_y_std = _get_csv_value("Kalman Model Y STD", 0.05)
    model_v_std = _get_csv_value("Kalman Model V STD", 0.1)
    model_a_std = _get_csv_value("Kalman Model A STD", 0.015)

    use_airbrake = _get_csv_value("Airbrakes Enabled (T/F)", True, bool)

    # Control algorithm selection
    control_algorithm = "OPTIMIZERPID" # BANGBANG, PID, OPTIMIZER, OPTIMIZERPID

    # Control parameters
    kp = 4 # Set up for CD PID. For apogee PID: _get_csv_value("KP (1/s)", 0.24)
    ki = 2 #0.01 for apogee PID
    kd = 0
    deadband = 0 # Meters
    i_window = 1 # seconds
    apogee_offset = 1.87 # Meters. Accounts for difference between avionics altitude model and rocketpy one

    # Controller parameters
    max_deployment_rate = 2.5   # deployment / time
    apogee_prediction_cd = _get_csv_value("Rocket CD", 0.71)   # Should match the rocket drag curve. Based on 4th flight of rocket 0.71
    airbrake_drag = _get_csv_value("Airbrake CD", 0.8)    # Max Cd from airbrake. Needs to match airbrake drag curve 0.35

    # Timing parameters
    burn_time = _get_csv_value("Coast Lockout (s)", 1.3)

    # Input data
    engine_file = "input_data/AeroTech_F42T_L.eng"
    airbrake_drag_curve_file = "input_data/airbrake_drag_curve.csv"
    rocket_drag_curve_file = "input_data/rocket_drag_curve.csv"

    # Monte carlo
    impulse_std = 0.35 # Ns - Based on data from https://www.thrustcurve.org/motors/cert/62d80c95ac50e90004732ded/F42.pdf
    mass_std = 0.001 # kg
    com_std = 0.005 # m
    wind_std = (1, 2.5) # (nominal multiplier, std)
    rocket_cd_std = 0.05
    airbrake_cd_std = 0.1