from rocketpy import Environment

def setup_environment(config):
    environment = Environment(
        latitude=config.latitude,
        longitude=config.longitude,
        elevation=config.env_elevation
    )

    environment.set_atmospheric_model(
        type="custom_atmosphere", wind_u=config.wind_speed, wind_v=0
    )

    return environment

