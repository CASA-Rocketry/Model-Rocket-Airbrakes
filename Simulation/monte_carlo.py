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
        import pandas as pd

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

        # Apply variance to airbrake drag coefficient
        airbrake_cd_multiplier = np.random.normal(1.0, self.config.airbrake_cd_std)

        # Load and modify airbrake drag curve
        drag_curve_file = self._airbrake_config['drag_coefficient_curve']
        df = pd.read_csv(drag_curve_file)
        # Multiply the drag coefficient values (second column) by the random multiplier
        df.iloc[:, 1] = df.iloc[:, 1] * airbrake_cd_multiplier
        # Convert to list of [deployment, drag_coefficient] pairs
        modified_drag_curve = df.values.tolist()

        controller_instance = self.controller_class(self.config)

        # Store controller instance on rocket for later access
        new_rocket._controller_instance = controller_instance

        new_rocket.add_air_brakes(
            drag_coefficient_curve=modified_drag_curve,
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
        power_off_drag_factor=(1.0, config.rocket_cd_std),
        power_on_drag_factor=(1.0, config.rocket_cd_std),
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
            # Access controller instance directly from rocket
            if hasattr(flight.rocket, '_controller_instance'):
                controller_instance = flight.rocket._controller_instance
                if hasattr(controller_instance, 'data') and 'deployment' in controller_instance.data:
                    deployments = controller_instance.data['deployment']
                    if deployments:
                        valid_deployments = [d for d in deployments if isinstance(d, (int, float)) and not np.isnan(d)]
                        if valid_deployments:
                            return float(max(valid_deployments))

            return 0.0

        except Exception as e:
            print(f"Error in get_max_deployment: {e}")
            return 0.0

    def get_deployment_timeseries(flight):
        """Extract deployment time series from controller data"""
        try:
            # Access controller instance directly from rocket
            if hasattr(flight.rocket, '_controller_instance'):
                controller_instance = flight.rocket._controller_instance
                if hasattr(controller_instance, 'data'):
                    time_data = controller_instance.data.get('time', [])
                    deployment_data = controller_instance.data.get('deployment', [])
                    if time_data and deployment_data:
                        return {'time': list(time_data), 'deployment': list(deployment_data)}

            return {'time': [], 'deployment': []}

        except Exception as e:
            print(f"Error in get_deployment_timeseries: {e}")
            return {'time': [], 'deployment': []}

    data_collector = {
        'max_airbrake_deployment': get_max_deployment,
        'deployment_timeseries': get_deployment_timeseries,
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
        'Error_m': apogees_agl - config.target_apogee + config.apogee_offset,
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
    fig, ax = plt.subplots(1, 1, figsize=(8, 6))

    # Apogee distribution with fixed bin width
    bin_width = 0.25  # meters
    bin_edges = np.arange(config.target_apogee - 5,
                          config.target_apogee + 5 + bin_width,
                          bin_width)

    ax.hist(apogees_agl, bins=bin_edges, edgecolor='black', alpha=0.7)
    ax.axvline(config.target_apogee - config.apogee_offset, color='r', linestyle='--', linewidth=2, label='Target')
    ax.axvline(np.mean(apogees_agl), color='g', linestyle='--', linewidth=2, label='Mean')
    ax.set_xlabel('Apogee (m AGL)')
    ax.set_ylabel('Frequency')
    ax.set_title('Apogee Distribution')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # Set fixed x-axis limits centered around target apogee
    ax.set_xlim(config.target_apogee - 5 - config.apogee_offset, config.target_apogee + 5 - config.apogee_offset)

    plt.tight_layout()
    plot_path = output_dir / "monte_carlo_plots.png"
    plt.savefig(plot_path, dpi=150, bbox_inches='tight')
    plt.close()

    # Create deployment overlay plot
    if 'deployment_timeseries' in monte_carlo.results:
        fig_deployment, ax_deployment = plt.subplots(1, 1, figsize=(10, 6))

        deployment_data = monte_carlo.results['deployment_timeseries']

        # Plot each simulation's deployment curve
        for i, sim_data in enumerate(deployment_data):
            if sim_data and 'time' in sim_data and 'deployment' in sim_data:
                time = sim_data['time']
                deployment = sim_data['deployment']
                if len(time) > 0 and len(deployment) > 0:
                    ax_deployment.plot(time, deployment, alpha=0.3, linewidth=0.8, color='blue')

        ax_deployment.set_xlabel('Time (s)')
        ax_deployment.set_ylabel('Deployment (0-1)')
        ax_deployment.set_title('Airbrake Deployment Over Time - All Monte Carlo Simulations')
        ax_deployment.grid(True, alpha=0.3)
        ax_deployment.set_ylim(-0.05, 1.05)

        plt.tight_layout()
        deployment_plot_path = output_dir / "monte_carlo_deployment_overlay.png"
        plt.savefig(deployment_plot_path, dpi=150, bbox_inches='tight')
        plt.close('all')


if __name__ == "__main__":
    config = Config()
    monte_carlo = run_monte_carlo(config=config, num_simulations=1000)
    create_plot(monte_carlo, config)
    std = np.std(monte_carlo.results['apogee'])
    print(f"Standard deviation: {std} m")
    average_apogee = sum(monte_carlo.results['apogee']) / len(monte_carlo.results['apogee'])
    average_error = average_apogee - config.target_apogee + config.apogee_offset - config.env_elevation
    print(f"Average error: {average_error} m")
