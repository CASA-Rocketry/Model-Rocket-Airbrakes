import sys
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
import time
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
    start_time = time.time()
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

    # Wind configuration: supports both normal and uniform distributions
    # For normal: wind_std = (multiplier, std)
    # For uniform: wind_std = (min, max, "uniform")
    if len(config.wind_std) == 3 and config.wind_std[2] == "uniform":
        # Uniform distribution: (min, max, "uniform")
        wind_min, wind_max = config.wind_std[0], config.wind_std[1]
        stochastic_env = StochasticEnvironment(
            environment=environment,
            wind_velocity_x_factor=(wind_min, wind_max, "uniform"),
            wind_velocity_y_factor=(wind_min, wind_max, "uniform"),
        )
    else:
        # Normal distribution: (multiplier, std)
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

    end_time = time.time()
    wall_time = end_time - start_time

    return monte_carlo, wall_time

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
        plt.rcParams.update({'font.size': 12})
        fig_deployment, ax_deployment = plt.subplots(1, 1, figsize=(10, 6))

        deployment_data = monte_carlo.results['deployment_timeseries']

        # Plot each simulation's deployment curve
        for i, sim_data in enumerate(deployment_data):
            if sim_data and 'time' in sim_data and 'deployment' in sim_data:
                time = sim_data['time']
                deployment = sim_data['deployment']
                if len(time) > 0 and len(deployment) > 0:
                    ax_deployment.plot(time, deployment, alpha=0.3, linewidth=2, color='blue')

        ax_deployment.set_xlabel('Time (s)')
        ax_deployment.set_ylabel('Deployment (0-1)')
        ax_deployment.set_title('Airbrake Deployments Over Time')
        ax_deployment.grid(True, alpha=0.3)
        ax_deployment.set_ylim(-0.05, 1.05)
        plt.xlim(-0.25, 6.25)
        plt.plot()
        plt.show()

        plt.tight_layout()
        deployment_plot_path = output_dir / "monte_carlo_deployment_overlay.png"
        plt.savefig(deployment_plot_path, dpi=150, bbox_inches='tight')
        plt.close('all')

def compare_monte_carlo(num):
    config = Config()

    # Run Monte Carlo simulations with different control algorithms
    print("Running Monte Carlo with BANGBANG controller...")
    config.control_algorithm = "BANGBANG"
    monte_carlo1, wall_time1 = run_monte_carlo(config=config, num_simulations=num)

    print("Running Monte Carlo with OPTIMIZERPID controller...")
    config.control_algorithm = "OPTIMIZERPID"
    monte_carlo2, wall_time2 = run_monte_carlo(config=config, num_simulations=num)

    print("Running Monte Carlo with PID controller...")
    config.control_algorithm = "PID"
    monte_carlo3, wall_time3 = run_monte_carlo(config=config, num_simulations=num)

    output_dir = Path("output")
    output_dir.mkdir(exist_ok=True)

    # Extract apogees for each algorithm
    apogees_agl_1 = np.array(monte_carlo1.results['apogee']) - config.env_elevation
    apogees_agl_2 = np.array(monte_carlo2.results['apogee']) - config.env_elevation
    apogees_agl_3 = np.array(monte_carlo3.results['apogee']) - config.env_elevation

    # Export comparison data to CSV
    import pandas as pd

    # Create dataframes for each algorithm
    df1 = pd.DataFrame({
        'Simulation': range(1, len(apogees_agl_1) + 1),
        'Algorithm': 'BANGBANG',
        'Apogee_AGL_m': apogees_agl_1,
        'Error_m': apogees_agl_1 - (config.target_apogee - config.apogee_offset),
    })

    df2 = pd.DataFrame({
        'Simulation': range(1, len(apogees_agl_2) + 1),
        'Algorithm': 'SOLVER',
        'Apogee_AGL_m': apogees_agl_2,
        'Error_m': apogees_agl_2 - (config.target_apogee - config.apogee_offset),
    })

    df3 = pd.DataFrame({
        'Simulation': range(1, len(apogees_agl_3) + 1),
        'Algorithm': 'INTEGRATOR',
        'Apogee_AGL_m': apogees_agl_3,
        'Error_m': apogees_agl_3 - (config.target_apogee - config.apogee_offset),
    })

    # Add max deployment if available
    if 'max_airbrake_deployment' in monte_carlo1.results:
        df1['Max_Deployment'] = monte_carlo1.results['max_airbrake_deployment']
    if 'max_airbrake_deployment' in monte_carlo2.results:
        df2['Max_Deployment'] = monte_carlo2.results['max_airbrake_deployment']
    if 'max_airbrake_deployment' in monte_carlo3.results:
        df3['Max_Deployment'] = monte_carlo3.results['max_airbrake_deployment']

    # Combine all dataframes
    df_combined = pd.concat([df1, df2, df3], ignore_index=True)

    # Save to CSV
    csv_path = output_dir / "monte_carlo_comparison_2.csv"
    df_combined.to_csv(csv_path, index=False)
    print(f"\nComparison data saved to: {csv_path}")

    # Create combined apogee distribution plot
    fig, ax = plt.subplots(1, 1, figsize=(12, 6))

    # Apogee distribution with fixed bin width
    bin_width = 0.05  # meters
    bin_edges = np.arange(config.target_apogee - 5,
                          config.target_apogee + 5 + bin_width,
                          bin_width)

    # Plot histograms for all three algorithms
    ax.hist(apogees_agl_1, bins=bin_edges, edgecolor='black', alpha=0.5, label='BANGBANG', color='blue')
    ax.hist(apogees_agl_2, bins=bin_edges, edgecolor='black', alpha=0.5, label='SOLVER', color='green')
    ax.hist(apogees_agl_3, bins=bin_edges, edgecolor='black', alpha=0.5, label='INTEGRATOR', color='orange')

    # Plot target and mean lines
    ax.axvline(config.target_apogee - config.apogee_offset, color='r', linestyle='--', linewidth=2, label='Target')
    ax.axvline(np.mean(apogees_agl_1), color='blue', linestyle=':', linewidth=2, alpha=0.7)
    ax.axvline(np.mean(apogees_agl_2), color='green', linestyle=':', linewidth=2, alpha=0.7)
    ax.axvline(np.mean(apogees_agl_3), color='orange', linestyle=':', linewidth=2, alpha=0.7)

    ax.set_xlabel('Apogee (m AGL)')
    ax.set_ylabel('Frequency')
    ax.set_title('Apogee Distribution Comparison - Control Algorithms')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # Set fixed x-axis limits centered around target apogee
    ax.set_xlim(config.target_apogee - 5 - config.apogee_offset, config.target_apogee + 5 - config.apogee_offset)

    plt.tight_layout()
    plot_path = output_dir / "monte_carlo_comparison_apogee.png"
    plt.savefig(plot_path, dpi=150, bbox_inches='tight')
    plt.close()

    # Create combined deployment overlay plot
    if 'deployment_timeseries' in monte_carlo1.results:
        plt.rcParams.update({'font.size': 12})
        fig_deployment, ax_deployment = plt.subplots(1, 1, figsize=(14, 6))

        # Plot BANGBANG deployments
        deployment_data_1 = monte_carlo1.results['deployment_timeseries']
        for i, sim_data in enumerate(deployment_data_1):
            if sim_data and 'time' in sim_data and 'deployment' in sim_data:
                time = sim_data['time']
                deployment = sim_data['deployment']
                if len(time) > 0 and len(deployment) > 0:
                    ax_deployment.plot(time, deployment, alpha=0.2, linewidth=1.5, color='blue',
                                      label='BANGBANG' if i == 0 else '')

        # Plot OPTIMIZERPID deployments
        deployment_data_2 = monte_carlo2.results['deployment_timeseries']
        for i, sim_data in enumerate(deployment_data_2):
            if sim_data and 'time' in sim_data and 'deployment' in sim_data:
                time = sim_data['time']
                deployment = sim_data['deployment']
                if len(time) > 0 and len(deployment) > 0:
                    ax_deployment.plot(time, deployment, alpha=0.2, linewidth=1.5, color='green',
                                      label='OPTIMIZERPID' if i == 0 else '')

        # Plot PID deployments
        deployment_data_3 = monte_carlo3.results['deployment_timeseries']
        for i, sim_data in enumerate(deployment_data_3):
            if sim_data and 'time' in sim_data and 'deployment' in sim_data:
                time = sim_data['time']
                deployment = sim_data['deployment']
                if len(time) > 0 and len(deployment) > 0:
                    ax_deployment.plot(time, deployment, alpha=0.2, linewidth=1.5, color='orange',
                                      label='PID' if i == 0 else '')

        ax_deployment.set_xlabel('Time (s)')
        ax_deployment.set_ylabel('Deployment (0-1)')
        ax_deployment.set_title('Airbrake Deployments Over Time - Control Algorithm Comparison')
        ax_deployment.legend()
        ax_deployment.grid(True, alpha=0.3)
        ax_deployment.set_ylim(-0.05, 1.05)
        ax_deployment.set_xlim(-0.25, 6.25)

        plt.tight_layout()
        deployment_plot_path = output_dir / "monte_carlo_comparison_deployment.png"
        plt.savefig(deployment_plot_path, dpi=150, bbox_inches='tight')
        plt.close('all')

    # Print statistics
    print("\n=== Monte Carlo Comparison Results ===")
    print(f"\nBANGBANG:")
    print(f"  Mean Apogee: {np.mean(apogees_agl_1):.2f} m")
    print(f"  Std Dev: {np.std(apogees_agl_1):.2f} m")
    print(f"  Mean Error: {np.mean(apogees_agl_1) - (config.target_apogee - config.apogee_offset):.2f} m")
    print(f"  Wall Time: {wall_time1:.2f} s")

    print(f"\nOPTIMIZERPID:")
    print(f"  Mean Apogee: {np.mean(apogees_agl_2):.2f} m")
    print(f"  Std Dev: {np.std(apogees_agl_2):.2f} m")
    print(f"  Mean Error: {np.mean(apogees_agl_2) - (config.target_apogee - config.apogee_offset):.2f} m")
    print(f"  Wall Time: {wall_time2:.2f} s")

    print(f"\nPID:")
    print(f"  Mean Apogee: {np.mean(apogees_agl_3):.2f} m")
    print(f"  Std Dev: {np.std(apogees_agl_3):.2f} m")
    print(f"  Mean Error: {np.mean(apogees_agl_3) - (config.target_apogee - config.apogee_offset):.2f} m")
    print(f"  Wall Time: {wall_time3:.2f} s")

    print(f"\nOutput files:")
    print(f"  - {output_dir / 'monte_carlo_comparison_2.csv'}")
    print(f"  - {output_dir / 'monte_carlo_comparison_apogee.png'}")
    print(f"  - {output_dir / 'monte_carlo_comparison_deployment.png'}")


def plot_comparison_from_csv(csv_filename="monte_carlo_comparison.csv"):
    """
    Plot comparison data from CSV file using step histograms

    Parameters:
    -----------
    csv_filename : str
        Name of the CSV file in the output directory (default: "monte_carlo_comparison.csv")
    """
    import pandas as pd

    config = Config()
    output_dir = Path("output")
    csv_path = output_dir / csv_filename

    # Read the CSV file
    if not csv_path.exists():
        print(f"Error: CSV file not found at {csv_path}")
        return

    df = pd.read_csv(csv_path)

    # Separate data by algorithm
    df_bangbang = df[df['Algorithm'] == 'BANGBANG']
    df_solver = df[df['Algorithm'] == 'SOLVER']
    df_integrator = df[df['Algorithm'] == 'INTEGRATOR']

    apogees_agl_1 = df_bangbang['Apogee_AGL_m'].values + config.apogee_offset
    apogees_agl_2 = df_solver['Apogee_AGL_m'].values + config.apogee_offset
    apogees_agl_3 = df_integrator['Apogee_AGL_m'].values + config.apogee_offset

    # Calculate statistics for each algorithm
    mean_1, std_1 = np.mean(apogees_agl_1), np.std(apogees_agl_1)
    mean_2, std_2 = np.mean(apogees_agl_2), np.std(apogees_agl_2)
    mean_3, std_3 = np.mean(apogees_agl_3), np.std(apogees_agl_3)

    # Create step histogram plot
    fig, ax = plt.subplots(1, 1, figsize=(18, 6))

    # Apogee distribution with fixed bin width
    bin_width = 0.2  # meters
    # Determine bin range based on actual data
    all_apogees = np.concatenate([apogees_agl_1, apogees_agl_2, apogees_agl_3])
    data_min = np.min(all_apogees)
    data_max = np.max(all_apogees)
    bin_edges = np.arange(np.floor(data_min) - 1,
                          np.ceil(data_max) + 1 + bin_width,
                          bin_width)

    # Plot filled transparent histograms for all three algorithms with stats in labels
    ax.hist(apogees_agl_1, bins=bin_edges, histtype='stepfilled', linewidth=2,
            alpha=0.4, label=f'BANGBANG (μ={mean_1:.2f}m, σ={std_1:.2f}m)',
            color='blue', edgecolor='blue')
    ax.hist(apogees_agl_3, bins=bin_edges, histtype='stepfilled', linewidth=2,
            alpha=0.4, label=f'INTEGRATOR (μ={mean_3:.2f}m, σ={std_3:.2f}m)',
            color='orange', edgecolor='orange')
    ax.hist(apogees_agl_2, bins=bin_edges, histtype='stepfilled', linewidth=2,
            alpha=0.4, label=f'SOLVER (μ={mean_2:.2f}m, σ={std_2:.2f}m)',
            color='green', edgecolor='green')

    # Plot target and mean lines
    target_apogee = config.target_apogee
    ax.axvline(target_apogee, color='r',
               linestyle='--', linewidth=2)
    ax.axvline(mean_1, color='blue', linestyle=':',
               linewidth=2, alpha=0.7)
    ax.axvline(mean_2, color='green', linestyle=':',
               linewidth=2, alpha=0.7)
    ax.axvline(mean_3, color='orange', linestyle=':',
               linewidth=2, alpha=0.7)

    ax.set_xlabel('Apogee (m AGL)', fontsize=16)
    ax.set_ylabel('Frequency', fontsize=16)
    ax.set_title('Comparison of Control Algorithm Apogee Distribution', fontsize=18)
    ax.legend(fontsize=14)
    ax.grid(True, alpha=0.3)
    ax.tick_params(axis='both', which='major', labelsize=14)

    # Set x-axis limits to match bin edges
    ax.set_xlim(bin_edges[0], bin_edges[-1])

    # Add text labels on the graph for mean and target lines
    y_pos = ax.get_ylim()[1] * 0.95  # Position near top of plot
    ax.text(target_apogee, y_pos, f'Target\n{target_apogee:.2f}m',
            color='r', ha='center', va='top', fontsize=14, fontweight='bold',
            bbox=dict(boxstyle='round,pad=0.3', facecolor='white', edgecolor='red', alpha=0.8),
            clip_on=False)

    # Offset mean labels vertically to prevent overlap
    ax.text(mean_1, y_pos * 0.40, f'BB μ\n{mean_1:.2f}m',
            color='blue', ha='center', va='top', fontsize=13,
            bbox=dict(boxstyle='round,pad=0.3', facecolor='white', edgecolor='blue', alpha=0.7),
            clip_on=False)
    ax.text(mean_2, y_pos * 0.70, f'Solver μ\n{mean_2:.2f}m',
            color='green', ha='center', va='top', fontsize=13,
            bbox=dict(boxstyle='round,pad=0.3', facecolor='white', edgecolor='green', alpha=0.7),
            clip_on=False)
    ax.text(mean_3, y_pos * 0.55, f'Int μ\n{mean_3:.2f}m',
            color='orange', ha='center', va='top', fontsize=13,
            bbox=dict(boxstyle='round,pad=0.3', facecolor='white', edgecolor='orange', alpha=0.7),
            clip_on=False)

    plt.tight_layout()
    plot_path = output_dir / "monte_carlo_comparison_step_histogram.png"
    plt.savefig(plot_path, dpi=150, bbox_inches='tight', pad_inches=0.2)
    plt.plot()
    plt.show()
    plt.close()

    # Print statistics
    print("\n=== Monte Carlo Comparison Results (from CSV) ===")
    print(f"\nBANGBANG:")
    print(f"  Mean Apogee: {np.mean(apogees_agl_1):.2f} m")
    print(f"  Std Dev: {np.std(apogees_agl_1):.2f} m")
    print(f"  Mean Error: {np.mean(apogees_agl_1) - config.target_apogee:.2f} m")

    print(f"\nSOLVER:")
    print(f"  Mean Apogee: {np.mean(apogees_agl_2):.2f} m")
    print(f"  Std Dev: {np.std(apogees_agl_2):.2f} m")
    print(f"  Mean Error: {np.mean(apogees_agl_2) - config.target_apogee:.2f} m")

    print(f"\nINTEGRATOR:")
    print(f"  Mean Apogee: {np.mean(apogees_agl_3):.2f} m")
    print(f"  Std Dev: {np.std(apogees_agl_3):.2f} m")
    print(f"  Mean Error: {np.mean(apogees_agl_3) - config.target_apogee:.2f} m")

    print(f"\nPlot saved to: {plot_path}")


if __name__ == "__main__":

    config = Config()
    #compare_monte_carlo(num=100)
    plot_comparison_from_csv("monte_carlo_comparison.csv")

    #monte_carlo, wall_time = run_monte_carlo(config=config, num_simulations=10)
    #create_plot(monte_carlo, config)
    #std = np.std(monte_carlo.results['apogee'])
    #print(f"Standard deviation: {std} m")
    #average_apogee = sum(monte_carlo.results['apogee']) / len(monte_carlo.results['apogee'])
    #average_error = average_apogee - config.target_apogee + config.apogee_offset - config.env_elevation
    #print(f"Average error: {average_error} m")
    #print(f"Wall time: {wall_time}")
