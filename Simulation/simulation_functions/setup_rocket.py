from rocketpy import Environment, Rocket, Flight, Barometer, Accelerometer
from rocketpy.motors import GenericMotor
from math import pi

def setup_rocket(config, controller):

    rocket = Rocket(
        radius=config.rocket_radius,
        mass=config.dry_mass,
        inertia=(
            config.I_xx,
            config.I_yy,
            config.I_zz
        ),
        power_off_drag=config.rocket_drag_curve_file,
        power_on_drag=config.rocket_drag_curve_file, # assume drag to be same for power on and power off
        center_of_mass_without_motor=config.com_no_motor,
        coordinate_system_orientation="nose_to_tail",
    )

    motor = GenericMotor.load_from_eng_file(
        file_name=config.engine_file,
        chamber_radius=config.chamber_radius,
        chamber_height=config.chamber_height
    )
    rocket.add_motor(motor, position=config.motor_position)

    rocket.add_nose(length=config.nosecone_length, kind=config.nosecone_type, position=0)
    rocket.add_trapezoidal_fins(
        n=config.n_fins, root_chord=config.root_chord, tip_chord=config.tip_chord,
        span=config.span,
        position=config.fin_position, cant_angle=config.cant_angle)
    rocket.add_parachute(
        name="Chute", cd_s=config.chute_cd, trigger=config.deployment_alt, sampling_rate=config.sampling_rate,
        lag=0, noise=(0, 0, 0))

    # Add barometer sensor with RocketPy's built-in pressure measurement
    barometer = Barometer(
        sampling_rate=config.sampling_rate,
        measurement_range=config.barometer_range,
        resolution=config.barometer_resolution,
        noise_density=config.barometer_noise_density,
        noise_variance=config.barometer_noise_variance,
        random_walk_density=config.barometer_random_walk_density,
        constant_bias=config.barometer_constant_bias,
        operating_temperature=config.barometer_operating_temperature,
        temperature_bias=config.barometer_temperature_bias,
        temperature_scale_factor=config.barometer_temperature_scale_factor,
        name="Barometer"
    )
    rocket.add_sensor(barometer, position=(0, 0, config.barometer_position))

    accelerometer = Accelerometer(
        sampling_rate=config.sampling_rate,
        measurement_range=config.accel_range,
        resolution=config.accel_resolution,
        noise_density=config.accel_noise_density,
        noise_variance=config.accel_noise_variance,
        random_walk_density=config.accel_random_walk_density,
        constant_bias=config.accel_constant_bias,
        operating_temperature=config.accel_operating_temp,
        temperature_bias=config.accel_temperature_bias,
        temperature_scale_factor=config.accel_temperature_scale_factor,
        cross_axis_sensitivity=config.accel_cross_axis_sensitivity,
        name="Accelerometer"
    )
    rocket.add_sensor(accelerometer, position=(0, 0, config.accele_position))

    rocket.add_air_brakes(
        drag_coefficient_curve=config.airbrake_drag_curve_file,
        controller_function=controller,
        sampling_rate=config.sampling_rate,
        reference_area=pi * config.rocket_radius ** 2,
        clamp=True,
        initial_observed_variables=[0],
        override_rocket_drag=False,
        name="Air Brakes",
    )

    return rocket, motor

