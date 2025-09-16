"""
Monte Carlo simulation for airbrake system performance analysis.
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from rocketpy import Environment, Rocket, Flight
from rocketpy.motors import GenericMotor
from rocketpy.sensors import Barometer, Accelerometer
from config import ControllerConfig
from controller import AirbrakeController
import time
import copy


class MonteCarloAnalysis:
    """Monte Carlo analysis with parameter variations"""

    def __init__(self, config: ControllerConfig, n_simulations: int = 100):
        self.config = config
        self.n_simulations = n_simulations
        self.results = []
        self.statistics = {}
        self.controller_data = []  # Store controller data for deployment plots

    def create_varied_config(self):
        """Create configuration with realistic variations"""
        varied_config = copy.deepcopy(self.config)

        # Center of mass variation
        com_variation = np.random.normal(0, 0.01)  # meters
        varied_config.com_no_motor += com_variation

        # Mass variation
        mass_variation = np.random.normal(0, 0.002)  # kg
        varied_config.dry_mass += mass_variation
        varied_config.burnout_mass = varied_config.dry_mass + 0.027

        # Fin cant angle variation
        cant_variation = np.random.normal(0, 4) # degrees
        varied_config.cant_angle += cant_variation

        # Moment of inertia variations
        inertia_variation = np.random.normal(0, 0.05)
        varied_config.I_xx *= (1 + inertia_variation)
        varied_config.I_yy *= (1 + inertia_variation)

        return varied_config

    def run_single_simulation(self, sim_number):
        """Run a single simulation with variations"""
        try:
            # Get varied configuration
            varied_config = self.create_varied_config()

            # Create environment with wind variations
            environment = Environment(
                latitude=varied_config.latitude,
                longitude=varied_config.longitude,
                elevation=varied_config.env_elevation,
            )

            # Add random wind
            wind_speed = np.random.uniform(0, 5)
            wind_direction = np.random.uniform(0, 360)  # degrees
            environment.set_atmospheric_model(
                type="standard_atmosphere",
                wind_u=wind_speed * np.cos(np.radians(wind_direction)),
                wind_v=wind_speed * np.sin(np.radians(wind_direction))
            )

            # Create rocket with varied parameters
            rocket = Rocket(
                radius=varied_config.rocket_radius,
                mass=varied_config.dry_mass,
                inertia=(varied_config.I_xx, varied_config.I_yy, varied_config.I_zz),
                power_off_drag="rocket_drag_curve.csv",
                power_on_drag="rocket_drag_curve.csv",
                center_of_mass_without_motor=varied_config.com_no_motor,
                coordinate_system_orientation="nose_to_tail",
            )

            # Apply rocket drag variation by scaling the drag curve
            rocket_drag_variation = np.random.normal(1.0, 0.2)
            if hasattr(rocket, 'power_off_drag_factor'):
                rocket.power_off_drag_factor *= rocket_drag_variation
                rocket.power_on_drag_factor *= rocket_drag_variation

            # Load motor with thrust variation
            motor = GenericMotor.load_from_eng_file(
                file_name="AeroTech_F42T_L.eng",
                chamber_radius=0.0145,
                chamber_height=0.083
            )

            # Apply thrust variation
            thrust_variation = np.random.normal(1.0, 0.03)
            if hasattr(motor, 'thrust_curve'):
                # Scale the thrust curve
                original_thrust = motor.thrust_curve
                if hasattr(original_thrust, 'set_data'):
                    thrust_data = original_thrust.get_data()
                    scaled_thrust = [(t, thrust * thrust_variation) for t, thrust in thrust_data]
                    motor.thrust_curve.set_data(scaled_thrust)

            rocket.add_motor(motor, position=0.77)

            # Add components
            rocket.add_nose(length=0.13, kind="von karman", position=0)
            rocket.add_trapezoidal_fins(
                n=varied_config.n_fins,
                root_chord=varied_config.root_chord,
                tip_chord=varied_config.tip_chord,
                span=varied_config.span,
                position=varied_config.fin_position,
                cant_angle=varied_config.cant_angle
            )

            rocket.add_parachute(
                name="Chute",
                cd_s=varied_config.chute_cd,
                trigger=varied_config.deployment_alt,  # Fixed
                sampling_rate=10,
                lag=0,
                noise=(0, 8.3, 0.5)
            )

            # Add sensors
            barometer = Barometer(
                sampling_rate=varied_config.sampling_rate,
                measurement_range=100000,
                resolution=0.2,
                noise_density=0.0,
                noise_variance=(0.75) ** 2,
                random_walk_density=0.0,
                constant_bias=0.0,
                operating_temperature=20,
                temperature_bias=0.0,
                temperature_scale_factor=0.0,
                name="Barometer"
            )
            rocket.add_sensor(barometer, position=(0, 0, 0.35))

            accelerometer = Accelerometer(
                sampling_rate=varied_config.sampling_rate,
                measurement_range=varied_config.accel_range,
                resolution=varied_config.accel_resolution,
                noise_density=varied_config.accel_noise_density,
                noise_variance=varied_config.accel_noise_variance,
                random_walk_density=varied_config.accel_random_walk_density,
                constant_bias=varied_config.accel_constant_bias,
                operating_temperature=20,
                temperature_bias=varied_config.accel_temperature_bias,
                temperature_scale_factor=varied_config.accel_temperature_scale_factor,
                cross_axis_sensitivity=varied_config.accel_cross_axis_sensitivity,
                name="Accelerometer"
            )
            rocket.add_sensor(accelerometer, position=(0, 0, 0.35))

            # Create controller with varied config
            controller = AirbrakeController(varied_config, self.config.burn_time)

            # Store environment reference for the controller
            env_ref = environment

            def controller_wrapper(time, sampling_rate, state, state_history,
                                   observed_variables, air_brakes, sensors):
                return controller.control(
                    time, sampling_rate, state, state_history,
                    observed_variables, air_brakes, sensors, None
                )


            # Add airbrakes with drag curve variation
            airbrake_drag_variation = np.random.normal(1.0, 0.2)

            rocket.add_air_brakes(
                drag_coefficient_curve="airbrake_drag_curve.csv",
                controller_function=controller_wrapper,
                sampling_rate=varied_config.sampling_rate,
                reference_area=varied_config.airbrake_area,
                clamp=True,
                initial_observed_variables=[0],
                override_rocket_drag=False,
                name="Air Brakes",
            )

            # Apply airbrake drag curve variation
            if hasattr(rocket, 'air_brakes') and hasattr(rocket.air_brakes[0], 'drag_coefficient_curve'):
                airbrakes = rocket.air_brakes[0]
                if hasattr(airbrakes.drag_coefficient_curve, 'set_data'):
                    # Scale the drag curve
                    original_data = airbrakes.drag_coefficient_curve.get_data()
                    scaled_data = [(x, y * airbrake_drag_variation) for x, y in original_data]
                    airbrakes.drag_coefficient_curve.set_data(scaled_data)

            # Launch angle variation
            inclination = np.random.normal(85, 0.5)

            # Run flight
            flight = Flight(
                rocket=rocket,
                environment=environment,
                rail_length=varied_config.rail_length,
                inclination=inclination,  # Small variation
                heading=0,  # Fixed
                time_overshoot=False,
                terminate_on_apogee=varied_config.terminate_on_apogee
            )

            # Extract results
            apogee_agl = flight.apogee - varied_config.env_elevation
            error = apogee_agl - varied_config.target_apogee

            # Get controller statistics
            max_deployment = max(controller.data['deployment']) if controller.data['deployment'] else 0
            mean_deployment = np.mean([d for d in controller.data['deployment'] if d > 0]) if any(
                d > 0 for d in controller.data['deployment']) else 0
            control_actions = sum(1 for d in controller.data['deployment'] if d > 0)

            # Get predicted apogees when control was active
            active_predictions = [p for p, active in zip(controller.data['predicted_apogee_agl'],
                                                         controller.data['control_active']) if active and p > 0]
            mean_predicted = np.mean(active_predictions) if active_predictions else 0

            # Store controller data for deployment plots
            self.controller_data.append({
                'sim_number': sim_number,
                'time': controller.data['time'].copy(),
                'deployment': controller.data['deployment'].copy(),
                'control_active': controller.data['control_active'].copy(),
                'apogee_agl': apogee_agl,
                'error': error
            })

            return {
                'sim_number': sim_number,
                'apogee_agl': apogee_agl,
                'error': error,
                'error_pct': 100 * error / varied_config.target_apogee,
                'flight_time': flight.t_final,
                'max_deployment': max_deployment,
                'mean_deployment': mean_deployment,
                'control_actions': control_actions,
                'mean_predicted_apogee': mean_predicted,
                'inclination': inclination,
                'mass': varied_config.dry_mass,
                'com': varied_config.com_no_motor,
                'kp': varied_config.kp_base,
                'kd': varied_config.kd_base,
                'pred_cd': varied_config.apogee_prediction_cd,
                'airbrake_cd': varied_config.airbrake_drag
            }

        except Exception as e:
            print(f"Simulation {sim_number} failed: {e}")
            import traceback
            traceback.print_exc()
            return None

    def run_monte_carlo(self):
        """Run the complete Monte Carlo analysis"""
        print(f"Starting Monte Carlo simulation with {self.n_simulations} runs...")
        print(f"Target apogee: {self.config.target_apogee:.1f}m AGL")

        start_time = time.time()

        for i in range(self.n_simulations):
            print(f"  Running simulation {i + 1}/{self.n_simulations}")

            result = self.run_single_simulation(i + 1)
            if result is not None:
                self.results.append(result)

        elapsed_time = time.time() - start_time

        print(f"\nCompleted {len(self.results)}/{self.n_simulations} successful simulations")
        print(f"Elapsed time: {elapsed_time:.1f} seconds")

        # Debug: Check deployment levels
        if self.results:
            max_deps = [r['max_deployment'] for r in self.results]

            mean_deps = [r['mean_deployment'] for r in self.results if r['mean_deployment'] > 0]
            if mean_deps:
                print(f"  Mean deployment when active: {np.mean(mean_deps):.3f}")

        if self.results:
            self.calculate_statistics()
            return True
        return False

    def calculate_statistics(self):
        """Calculate statistical analysis of results"""
        if not self.results:
            return

        errors = [r['error'] for r in self.results]
        error_pcts = [r['error_pct'] for r in self.results]
        apogees = [r['apogee_agl'] for r in self.results]

        self.statistics = {
            'n_simulations': len(self.results),
            'target_apogee': self.config.target_apogee,
            'mean_apogee': np.mean(apogees),
            'std_apogee': np.std(apogees),
            'median_apogee': np.median(apogees),
            'mean_error': np.mean(errors),
            'std_error': np.std(errors),
            'rms_error': np.sqrt(np.mean([e ** 2 for e in errors])),
            'mean_abs_error': np.mean([abs(e) for e in errors]),
            'mean_error_pct': np.mean(error_pcts),
            'std_error_pct': np.std(error_pcts),
            'min_apogee': min(apogees),
            'max_apogee': max(apogees),
            'range': max(apogees) - min(apogees),
            'within_3m': sum(1 for e in errors if abs(e) <= 3.0),
            'within_5m': sum(1 for e in errors if abs(e) <= 5.0),
            'within_10m': sum(1 for e in errors if abs(e) <= 10.0),
            'within_15m': sum(1 for e in errors if abs(e) <= 15.0),
        }

    def print_statistics(self):
        """Print statistical analysis"""
        if not self.statistics:
            print("No statistics available")
            return

        stats = self.statistics
        n = stats['n_simulations']

        print(f"\n{'=' * 60}")
        print(f"MONTE CARLO ANALYSIS RESULTS")
        print(f"{'=' * 60}")
        print(f"Simulations completed: {n}")
        print(f"Target apogee: {stats['target_apogee']:.1f}m AGL")
        print(f"")
        print(f"Mean: {stats['mean_apogee']:.2f}m")
        print(f"Standard deviation: {stats['std_apogee']:.2f}m")
        print(f"")

    def create_plots(self):
        """Create analysis plots"""
        if not self.results:
            print("No results to plot")
            return

        errors = [r['error'] for r in self.results]
        apogees = [r['apogee_agl'] for r in self.results]

        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6))

        # Apogee histogram
        ax1.hist(apogees, bins=20, alpha=0.7, color='blue', edgecolor='black')
        ax1.axvline(self.config.target_apogee, color='red', linestyle='--', linewidth=2, label='Target')
        ax1.axvline(np.mean(apogees), color='orange', linestyle='-', linewidth=2, label='Mean')
        ax1.set_xlabel('Apogee AGL (m)')
        ax1.set_ylabel('Frequency')
        ax1.set_title('Apogee Distribution')
        ax1.legend()
        ax1.grid(True, alpha=0.3)

        # Apogee vs simulation number with acceptable range
        ax2.plot(range(1, len(apogees) + 1), apogees, 'o-', alpha=0.6, markersize=3)
        ax2.axhline(self.config.target_apogee, color='red', linestyle='--', linewidth=2, label='Target')
        ax2.axhline(np.mean(apogees), color='orange', linestyle='-', linewidth=2, label='Mean')

        # Add acceptable range
        target = self.config.target_apogee
        ax2.fill_between(range(1, len(apogees) + 1), target - 0.5, target + 0.5,
                        alpha=0.2, color='green', label='±0.5m Acceptable')

        ax2.set_xlabel('Simulation Number')
        ax2.set_ylabel('Apogee AGL (m)')
        ax2.set_title('Apogee vs Simulation')
        ax2.legend()
        ax2.grid(True, alpha=0.3)

        plt.tight_layout()
        plt.show()

    def plot_deployment_curves(self):
        """Plot all deployment curves overlaid"""
        if not self.controller_data:
            print("No controller data available for deployment plots")
            return

        plt.figure(figsize=(12, 8))

        # Plot all deployment curves
        j, k, l = 0, 0, 0
        for i, data in enumerate(self.controller_data):
            times = np.array(data['time'])
            deployments = np.array(data['deployment'])

            # Color by performance (error)
            error = data['error']
            if abs(error) <= 0.5:
                color = 'green'
                alpha = 0.7
                if j == 0:
                    label = 'Within ±0.5m'
                    j = 1
                else:
                    label = None
            elif abs(error) <= 2:
                color = 'orange'
                alpha = 0.5
                if k == 0:
                    label = 'Within ±2m'
                    k = 1
                else:
                    label = None
            else:
                color = 'red'
                alpha = 0.3
                if l == 0:
                    label = 'Beyond ±2m'
                    l = 1
                else:
                    label = None

            plt.plot(times, deployments, color=color, alpha=alpha, linewidth=1, label=label)

        plt.xlabel('Time (s)')
        plt.ylabel('Deployment Level')
        plt.title('Airbrake Deployment Curves - All Monte Carlo Simulations')
        plt.ylim(0, 1.1)
        plt.grid(True, alpha=0.3)
        plt.legend()
        plt.tight_layout()
        plt.show()

    def export_results(self, filename="monte_carlo_results.csv"):
        """Export results to CSV"""
        if not self.results:
            print("No results to export")
            return

        df = pd.DataFrame(self.results)
        df.to_csv(filename, index=False)
        print(f"Results exported to {filename}")

    def run_analysis(self):
        """Run complete Monte Carlo analysis"""
        success = self.run_monte_carlo()

        if success and self.results:
            self.print_statistics()
            self.create_plots()
            self.plot_deployment_curves()
            self.export_results()
            return True
        else:
            print("Monte Carlo analysis failed")
            return False


def main():
    """Run Monte Carlo analysis"""
    from config import ControllerConfig

    config = ControllerConfig()

    # Create
    analyzer = MonteCarloAnalysis(config, n_simulations=20)

    print("Starting Monte Carlo Analysis of Airbrake System")
    print(f"Configuration: Target = {config.target_apogee}m")

    # Run analysis
    success = analyzer.run_analysis()

    if success:
        print("\nMonte Carlo analysis completed successfully!")
    else:
        print("\nMonte Carlo analysis failed!")


if __name__ == "__main__":
    main()