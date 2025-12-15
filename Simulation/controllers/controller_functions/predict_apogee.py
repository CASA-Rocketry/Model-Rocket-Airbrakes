from math import pi, log

def predict_apogee(altitude_agl, velocity, current_deployment, config, combined_cd = True):
    """Apogee predictor for control algorithm with combined rocket and airbrake drag"""
    if velocity <= 0:
        return altitude_agl

    # Calculate combined drag coefficient
    rocket_cd = config.apogee_prediction_cd

    if combined_cd == True:
        # Airbrake contribution based on current deployment level
        airbrake_cd = current_deployment * config.airbrake_drag

        # Combined drag coefficient (rocket + airbrake)
        combined_cd = rocket_cd + airbrake_cd
    else:
        combined_cd = rocket_cd

    # Use rocket's reference area for drag calculation
    rocket_reference_area = pi * (config.rocket_radius ** 2)

    # Calculate drag parameter k with combined drag
    k = 0.5 * config.air_density * combined_cd * rocket_reference_area
    if k <= 0:
        return altitude_agl

    log_arg = (k * velocity ** 2) / (config.burnout_mass * 9.81) + 1
    if log_arg <= 0:
        return altitude_agl

    delta_altitude = (config.burnout_mass / (2 * k)) * log(log_arg)
    predicted_apogee_agl = altitude_agl + delta_altitude

    return predicted_apogee_agl