"""
Minimal Monte Carlo Script - Only outputs graph
"""

import sys
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
from rocketpy import MonteCarlo
from rocketpy.stochastic import (
    StochasticEnvironment,
    StochasticRocket,
    StochasticFlight,
    StochasticGenericMotor
)

script_dir = Path(__file__).parent
sys.path.insert(0, str(script_dir))

from config import Config
from controller import Control
from simulation_functions.setup_environment import setup_environment
from simulation_functions.setup_rocket import setup_rocket


class CustomStochasticRocket(StochasticRocket):
    def __init__(self, rocket, controller_class, config, *args, **kwargs):
        super().__init__(rocket, *args, **kwargs)
        self.controller_class = controller_class
        self.config = config
        self._airbrake_config = {
            'drag_coefficient_curve': config.airbrake_drag_curve_file,
            'sampling_rate': config.sampling_rate,
            'reference_area': 3.14159 * (config.rocket_radius ** 2),
            'clamp': True,
            'initial_observed_variables': [0],
            'override_rocket_drag': False,
            'name': "Air Brakes",
        }

    def create_object(self):
        new_rocket = super().create_object()
        from rocketpy import Barometer, Accelerometer

        barometer = Barometer(
            sampling_rate=self.config.sampling_rate,
            measurement_range=self.config.barometer_range,
            resolution=self.config.barometer_resolution,
            noise_density=self.config.barometer_noise_density,
            noise_variance=self.config.barometer_noise_variance,
            random_walk_density=self.config.barometer_random_walk_density,
            constant_bias=self.config.barometer_constant_bias,
            operating_temperature=self.config.barometer_operating_temperature,
            temperature_bias=self.config.barometer_temperature_bias,
            temperature_scale_factor=self.config.barometer_temperature_scale_factor,
            name="Barometer"
        )
        new_rocket.add_sensor(barometer, position=(0, 0, self.config.barometer_position))

        accelerometer = Accelerometer(
            sampling_rate=self.config.sampling_rate,
            measurement_range=self.config.accel_range,
            resolution=self.config.accel_resolution,
            noise_density=self.config.accel_noise_density,
            noise_variance=self.config.accel_noise_variance,
            random_walk_density=self.config.accel_random_walk_density,
            constant_bias=self.config.accel_constant_bias,
            operating_temperature=self.config.accel_operating_temp,
            temperature_bias=self.config.accel_temperature_bias,
            temperature_scale_factor=self.config.accel_temperature_scale_factor,
            cross_axis_sensitivity=self.config.accel_cross_axis_sensitivity,
            name="Accelerometer"
        )
        new_rocket.add_sensor(accelerometer, position=(0, 0, self.config.accele_position))

        controller_instance = self.controller_class(self.config)
        new_rocket.add_air_brakes(
            drag_coefficient_curve=self._airbrake_config['drag_coefficient_curve'],
            controller_function=controller_instance.controller,
            sampling_rate=self._airbrake_config['sampling_rate'],
            reference_area=self._airbrake_config['reference_area'],
            clamp=self._airbrake_config['clamp'],
            initial_observed_variables=self._airbrake_config['initial_observed_variables'],
            override_rocket_drag=self._airbrake_config['override_rocket_drag'],
            name=self._airbrake_config['name'],
        )

        return new_rocket


def run_monte_carlo(config, num_simulations=100):
    nominal_controller = Control(config)
    environment = setup_environment(config)
    rocket, motor = setup_rocket(config, nominal_controller.controller)

    from rocketpy import Flight
    nominal_flight = Flight(
        rocket=rocket,
        environment=environment,
        rail_length=config.rail_length,
        inclination=90,
        heading=0,
        terminate_on_apogee=config.terminate_on_apogee
    )

    wind_multiplier, wind_std = config.wind_std
    stochastic_env = StochasticEnvironment(
        environment=environment,
        wind_velocity_x_factor=(wind_multiplier, wind_std),
        wind_velocity_y_factor=(wind_multiplier, wind_std),
    )

    nominal_impulse = motor.total_impulse
    stochastic_motor = StochasticGenericMotor(
        generic_motor=motor,
        total_impulse=(nominal_impulse, config.impulse_std),
    )

    stochastic_rocket = CustomStochasticRocket(
        rocket=rocket,
        controller_class=Control,
        config=config,
        mass=(config.dry_mass, config.mass_std),
        center_of_mass_without_motor=(config.com_no_motor, config.com_std),
    )

    stochastic_rocket.add_motor(stochastic_motor, position=config.motor_position)

    stochastic_flight = StochasticFlight(
        flight=nominal_flight,
        inclination=(90, 0),
        heading=(0, 0),
    )

    def get_max_deployment(flight):
        """Extract max airbrake deployment from controller data"""
        try:
            if not hasattr(flight.rocket, 'air_brakes') or len(flight.rocket.air_brakes) == 0:
                return 0.0

            airbrake_item = flight.rocket.air_brakes[0]
            if isinstance(airbrake_item, tuple):
                airbrake, _ = airbrake_item
            else:
                airbrake = airbrake_item

            max_deployment = 0.0

            # Try current deployment level
            if hasattr(airbrake, 'deployment_level'):
                current = float(airbrake.deployment_level) if airbrake.deployment_level is not None else 0.0
                max_deployment = max(max_deployment, current)

            # Try to access controller data
            if hasattr(airbrake, 'controller_function') and hasattr(airbrake.controller_function, '__self__'):
                controller_instance = airbrake.controller_function.__self__
                if hasattr(controller_instance, 'data') and 'deployment' in controller_instance.data:
                    deployments = controller_instance.data['deployment']
                    if deployments:
                        valid_deployments = [d for d in deployments if isinstance(d, (int, float)) and not np.isnan(d)]
                        if valid_deployments:
                            max_deployment = max(max_deployment, max(valid_deployments))

            return float(max_deployment)

        except Exception as e:
            return 0.0

    data_collector = {
        'max_airbrake_deployment': get_max_deployment,
    }

    # Create output directory for Monte Carlo files
    mc_output_dir = Path("output/monte_carlo")
    mc_output_dir.mkdir(parents=True, exist_ok=True)

    monte_carlo = MonteCarlo(
        filename=str(mc_output_dir / "results"),
        environment=stochastic_env,
        rocket=stochastic_rocket,
        flight=stochastic_flight,
        data_collector=data_collector,
    )

    monte_carlo.simulate(number_of_simulations=num_simulations, append=False)

    return monte_carlo


def create_plot(monte_carlo, config):
    output_dir = Path("output")
    output_dir.mkdir(exist_ok=True)

    apogees_agl = np.array(monte_carlo.results['apogee']) - config.env_elevation

    # Export CSV with stochastic conditions and results
    import pandas as pd

    # Base data
    csv_data = {
        'Simulation': range(1, len(apogees_agl) + 1),
        'Apogee_AGL_m': apogees_agl,
        'Error_m': apogees_agl - config.target_apogee - 1.5,
    }

    # Extract stochastic input parameters from inputs_log
    if hasattr(monte_carlo, 'inputs_log') and monte_carlo.inputs_log:
        # inputs_log is a list of dictionaries, one per simulation
        for key in monte_carlo.inputs_log[0].keys():
            # Extract each parameter across all simulations
            csv_data[key] = [sim_inputs[key] for sim_inputs in monte_carlo.inputs_log]

    # Add deployment data
    if 'max_airbrake_deployment' in monte_carlo.results:
        csv_data['Max_Deployment'] = monte_carlo.results['max_airbrake_deployment']

    # Add other results
    if 't_final' in monte_carlo.results:
        csv_data['Flight_Time_s'] = monte_carlo.results['t_final']
    if 'max_speed' in monte_carlo.results:
        csv_data['Max_Speed_ms'] = monte_carlo.results['max_speed']
    if 'impact_velocity' in monte_carlo.results:
        csv_data['Impact_Velocity_ms'] = monte_carlo.results['impact_velocity']
    if 'frontal_surface_wind' in monte_carlo.results:
        csv_data['Frontal_Wind_ms'] = monte_carlo.results['frontal_surface_wind']
    if 'lateral_surface_wind' in monte_carlo.results:
        csv_data['Lateral_Wind_ms'] = monte_carlo.results['lateral_surface_wind']

    df = pd.DataFrame(csv_data)
    csv_path = output_dir / "monte_carlo_results.csv"
    df.to_csv(csv_path, index=False)

    # Create plots
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))

    # Apogee distribution
    ax1.hist(apogees_agl, bins=30, edgecolor='black', alpha=0.7)
    ax1.axvline(config.target_apogee - 1.5, color='r', linestyle='--', linewidth=2, label='Target')
    ax1.axvline(np.mean(apogees_agl), color='g', linestyle='--', linewidth=2, label='Mean')
    ax1.set_xlabel('Apogee (m AGL)')
    ax1.set_ylabel('Frequency')
    ax1.set_title('Apogee Distribution')
    ax1.legend()
    ax1.grid(True, alpha=0.3)

    # Max airbrake deployment distribution
    if 'max_airbrake_deployment' in monte_carlo.results:
        deployments = monte_carlo.results['max_airbrake_deployment']
        deployments = np.array([d if d is not None else 0.0 for d in deployments])
        deployments = deployments[~np.isnan(deployments)]

        ax2.hist(deployments, bins=30, edgecolor='black', alpha=0.7, color='purple')
        ax2.set_xlabel('Max Deployment (0-1)')
        ax2.set_ylabel('Frequency')
        ax2.set_title(f'Max Airbrake Deployment\n(Mean: {np.mean(deployments):.3f})')
        ax2.grid(True, alpha=0.3)
    else:
        ax2.text(0.5, 0.5, 'Airbrake Data\nNot Available', ha='center', va='center', transform=ax2.transAxes)
        ax2.set_title('Max Airbrake Deployment Distribution')

    plt.tight_layout()
    plot_path = output_dir / "monte_carlo_plots.png"
    plt.savefig(plot_path, dpi=150, bbox_inches='tight')
    plt.close('all')


if __name__ == "__main__":
    config = Config()
    monte_carlo = run_monte_carlo(config=config, num_simulations=50)
    create_plot(monte_carlo, config)