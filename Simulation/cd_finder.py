"""
Drag Coefficient Calculator from Real Flight Data - FIXED VERSION

Improvements:
- Proper CSV headers for RocketPy compatibility
- Robust NaN and data handling
- AGL-consistent altitude comparisons
- Normalized error metrics
- Better validation and sanity checks
"""

import pandas as pd
import numpy as np
from scipy.optimize import minimize, minimize_scalar
from scipy.interpolate import interp1d
from rocketpy import Environment, Rocket, Flight
from rocketpy.motors import GenericMotor
from controller import AirbrakeController
from config import ControllerConfig
import os
import matplotlib.pyplot as plt


class FlightConfig:
    """Configuration for a specific flight with unique mass, COM, etc."""

    def __init__(self, name, dry_mass, com_no_motor, has_airbrakes=False,
                 needs_angle_optimization=False):
        self.name = name
        self.dry_mass = dry_mass
        self.com_no_motor = com_no_motor
        self.has_airbrakes = has_airbrakes
        self.needs_angle_optimization = needs_angle_optimization


class RateLimitedController:
    """Wrapper for AirbrakeController that enforces rate limiting"""

    def __init__(self, base_controller, max_rate=1.0 / 0.3, dt=0.05):
        self.base_controller = base_controller
        self.max_rate = max_rate
        self.dt = dt
        self.last_deployment = 0.0
        self.last_time = 0.0
        self.flight_data = []
        self.data_index = 0

    def load_flight_deployments(self, times, deployments):
        """Load the airbrake deployment schedule from real flight data"""
        self.flight_data = list(zip(times, deployments))
        self.data_index = 0

    def get_rate_limited_deployment(self, time, target_deployment):
        """Apply rate limiting to deployment changes"""
        if time == 0:
            self.last_deployment = 0.0
            self.last_time = 0.0
            return 0.0

        dt = time - self.last_time
        if dt <= 0:
            return self.last_deployment

        max_change = self.max_rate * dt
        desired_change = target_deployment - self.last_deployment
        actual_change = np.clip(desired_change, -max_change, max_change)
        new_deployment = np.clip(self.last_deployment + actual_change, 0.0, 1.0)

        self.last_deployment = new_deployment
        self.last_time = time

        return new_deployment

    def control(self, time, sampling_rate, state, state_history,
                observed_variables, air_brakes, sensors, environment):
        """Control function that uses real flight data with rate limiting"""

        if self.data_index < len(self.flight_data):
            while (self.data_index < len(self.flight_data) - 1 and
                   self.flight_data[self.data_index + 1][0] <= time):
                self.data_index += 1

            _, target_deployment = self.flight_data[self.data_index]
        else:
            target_deployment = self.flight_data[-1][1] if self.flight_data else 0.0

        deployment = self.get_rate_limited_deployment(time, target_deployment)

        if self.base_controller:
            altitude_agl = state[2] - environment.elevation
            velocity = -state[5]

            self.base_controller.data['time'].append(time)
            self.base_controller.data['deployment'].append(deployment)
            self.base_controller.data['sim_altitude_agl'].append(altitude_agl)
            self.base_controller.data['sim_velocity'].append(velocity)

        return deployment


class FlightDataLoader:
    """Load and process real flight log data"""

    @staticmethod
    def detect_liftoff(df, alt_column='y estimate', alt_threshold=5.0):
        """
        Detect liftoff point in flight data using altitude threshold

        Returns index where altitude first exceeds threshold
        """
        if alt_column not in df.columns:
            print(f"Warning: Column '{alt_column}' not found")
            return 0

        altitude = df[alt_column].values

        # Find first point above threshold
        for i in range(len(altitude)):
            if not np.isnan(altitude[i]) and altitude[i] > alt_threshold:
                return i

        print("Warning: Could not detect liftoff, using index 0")
        return 0

    @staticmethod
    def load_flight_log(filepath, time_col='time stamp', alt_col='y estimate',
                        deployment_col='servo deployment'):
        """
        Load flight log and extract relevant data with robust handling

        Returns:
            dict with 'times', 'altitudes', 'deployments', 'apogee', 'ground_level'
        """
        # Read CSV file
        df = pd.read_csv(filepath, skiprows=1)

        # Clean column names (strip whitespace)
        df.columns = df.columns.str.strip()

        print(f"\nAvailable columns: {list(df.columns)}")

        # Validate required columns exist
        if time_col not in df.columns:
            raise ValueError(f"Time column '{time_col}' not found in CSV")
        if alt_col not in df.columns:
            raise ValueError(f"Altitude column '{alt_col}' not found in CSV")

        # Handle NaNs in critical columns BEFORE any processing
        df[time_col] = pd.to_numeric(df[time_col], errors='coerce')
        df[alt_col] = pd.to_numeric(df[alt_col], errors='coerce')

        # Drop rows where time or altitude is NaN
        initial_len = len(df)
        df = df.dropna(subset=[time_col, alt_col])
        if len(df) < initial_len:
            print(f"Dropped {initial_len - len(df)} rows with NaN time/altitude")

        # Calculate ground level from pre-liftoff data (median of first 50 points)
        pre_liftoff_alt = df[alt_col].iloc[:min(50, len(df))].values
        ground_level = np.median(pre_liftoff_alt[~np.isnan(pre_liftoff_alt)])
        print(f"Ground level (median): {ground_level:.2f}m")

        # Convert altitude to AGL
        df[alt_col] = df[alt_col] - ground_level

        # Detect liftoff (using 2m AGL threshold for robustness)
        liftoff_idx = FlightDataLoader.detect_liftoff(df, alt_column=alt_col, alt_threshold=2.0)

        print(f"Detected liftoff at index {liftoff_idx}")
        print(f"Liftoff time: {df[time_col].iloc[liftoff_idx]:.2f}ms")
        print(f"Liftoff altitude (AGL): {df[alt_col].iloc[liftoff_idx]:.2f}m")

        # Trim data from liftoff
        df_flight = df.iloc[liftoff_idx:].copy()

        # Convert time from milliseconds to seconds and reset to start at 0
        time_offset = df_flight[time_col].iloc[0]
        df_flight[time_col] = (df_flight[time_col] - time_offset) / 1000.0

        # Extract data
        times = df_flight[time_col].values
        altitudes = df_flight[alt_col].values

        # Handle servo deployment - may be text or numeric
        if deployment_col in df_flight.columns:
            deployments = []
            for val in df_flight[deployment_col].values:
                if pd.isna(val) or isinstance(val, str):
                    deployments.append(0.0)
                else:
                    try:
                        deployments.append(float(val))
                    except:
                        deployments.append(0.0)
            deployments = np.array(deployments)
        else:
            print(f"Warning: Deployment column '{deployment_col}' not found, assuming no airbrakes")
            deployments = np.zeros_like(times)

        # Find apogee
        apogee = np.max(altitudes)
        apogee_time = times[np.argmax(altitudes)]

        # Check if airbrakes were used
        max_deployment = np.max(deployments)
        has_airbrakes = max_deployment > 0.05

        print(f"Flight apogee (AGL): {apogee:.2f}m at t={apogee_time:.2f}s")
        print(f"Max airbrake deployment: {max_deployment:.3f}")
        print(f"Airbrakes used: {'Yes' if has_airbrakes else 'No'}")
        print(f"Data points: {len(times)}")

        return {
            'times': times,
            'altitudes': altitudes,
            'deployments': deployments,
            'apogee': apogee,
            'apogee_time': apogee_time,
            'has_airbrakes': has_airbrakes,
            'ground_level': ground_level,
            'dataframe': df_flight
        }


class DragCoefficientCalculator:
    """Calculate drag coefficients by matching simulation to real flight"""

    def __init__(self, config: ControllerConfig):
        self.config = config
        self.real_flight_data = None
        self.current_flight_config = None
        self.rocket_cd = None
        self.launch_angle = 85.0  # Default vertical
        self.airbrake_cd = None
        self.simulation_history = []

    def load_real_flight(self, filepath, **kwargs):
        """Load real flight data"""
        self.real_flight_data = FlightDataLoader.load_flight_log(filepath, **kwargs)
        print(f"\nLoaded flight data from {filepath}")
        print(f"Target apogee: {self.real_flight_data['apogee']:.2f}m AGL")

    def write_rocket_drag_curve(self, cd_value, filename='rocket_drag_curve.csv'):
        """Write rocket drag curve CSV with proper header for RocketPy"""
        with open(filename, 'w') as f:
            f.write("mach,cd\n")  # FIXED: Added proper header
            f.write(f"0.0,{cd_value:.4f}\n")
            f.write(f"0.3,{cd_value:.4f}\n")
            f.write(f"0.7,{cd_value:.4f}\n")
            f.write(f"1.0,{cd_value:.4f}\n")

    def write_airbrake_drag_curve(self, cd_max, filename='airbrake_drag_curve.csv'):
        """Write airbrake drag curve CSV with proper header and clamping"""
        abs_filename = os.path.abspath(filename)
        with open(abs_filename, 'w') as f:
            f.write("deployment_level,mach,cd\n")  # FIXED: Added proper header
            # No drag at zero deployment
            f.write(f"0.0,0.0,0.0000\n")
            f.write(f"0.0,1.0,0.0000\n")
            # Half deployment - constant across Mach
            cd_half = max(0.0, cd_max * 0.5)  # FIXED: Clamp negative values
            f.write(f"0.5,0.0,{cd_half:.4f}\n")
            f.write(f"0.5,1.0,{cd_half:.4f}\n")
            # Full deployment - constant across Mach
            cd_full = max(0.0, cd_max)  # FIXED: Clamp negative values
            f.write(f"1.0,0.0,{cd_full:.4f}\n")
            f.write(f"1.0,1.0,{cd_full:.4f}\n")
        print(f"    [DEBUG: Wrote airbrake drag curve to {abs_filename}]")
        print(f"    [DEBUG: Cd values - half: {cd_half:.4f}, full: {cd_full:.4f}]")

    def run_simulation(self, rocket_cd=None, airbrake_cd=None, launch_angle=None,
                       flight_config=None, use_airbrakes=True):
        """
        Run a single simulation with given parameters

        Returns dict with 'apogee' (AGL), 'times', 'altitudes' (AGL)
        """
        # Use stored values if not provided
        if rocket_cd is None:
            rocket_cd = self.rocket_cd if self.rocket_cd else 0.8
        if airbrake_cd is None:
            airbrake_cd = self.airbrake_cd if self.airbrake_cd else 0.5
        if launch_angle is None:
            launch_angle = self.launch_angle
        if flight_config is None:
            flight_config = self.current_flight_config

        # Write drag curves
        self.write_rocket_drag_curve(rocket_cd)
        if use_airbrakes:
            self.write_airbrake_drag_curve(airbrake_cd)

        # Setup environment
        environment = Environment(
            latitude=self.config.latitude,
            longitude=self.config.longitude,
            elevation=self.config.env_elevation
        )
        environment.set_atmospheric_model(type='standard_atmosphere')

        # Create rocket with flight-specific parameters
        rocket = Rocket(
            radius=self.config.rocket_radius,
            mass=flight_config.dry_mass,
            inertia=(self.config.I_xx, self.config.I_yy, self.config.I_zz),
            power_off_drag='rocket_drag_curve.csv',
            power_on_drag='rocket_drag_curve.csv',
            center_of_mass_without_motor=flight_config.com_no_motor,
            coordinate_system_orientation="nose_to_tail",
        )

        # Add motor
        motor = GenericMotor.load_from_eng_file(
            file_name="AeroTech_F42T_L.eng",
            chamber_radius=0.0145,
            chamber_height=0.083
        )
        rocket.add_motor(motor, position=0.77)

        # Add components
        rocket.add_nose(length=0.13, kind="von karman", position=0)
        rocket.add_trapezoidal_fins(
            n=self.config.n_fins,
            root_chord=self.config.root_chord,
            tip_chord=self.config.tip_chord,
            span=self.config.span,
            position=self.config.fin_position,
            cant_angle=self.config.cant_angle
        )

        # Add airbrakes if needed
        if use_airbrakes and flight_config.has_airbrakes:
            print(f"\n    [DEBUG: Adding airbrakes with Cd={airbrake_cd:.4f}, area={self.config.airbrake_area:.6f}m²]")
            print(f"    [DEBUG: Max deployment in flight data: {np.max(self.real_flight_data['deployments']):.3f}]")

            base_controller = AirbrakeController(self.config, self.config.burn_time)
            controller = RateLimitedController(
                base_controller,
                max_rate=1.0 / 0.3,
                dt=1.0 / self.config.sampling_rate
            )

            controller.load_flight_deployments(
                self.real_flight_data['times'],
                self.real_flight_data['deployments']
            )

            deployment_log = []

            def controller_wrapper(time, sampling_rate, state, state_history,
                                   observed_variables, air_brakes, sensors):
                deployment = controller.control(
                    time, sampling_rate, state, state_history,
                    observed_variables, air_brakes, sensors, environment
                )
                deployment_log.append((time, deployment))
                return deployment

            abs_drag_curve_path = os.path.abspath("airbrake_drag_curve.csv")
            print(f"    [DEBUG: Using drag curve file: {abs_drag_curve_path}]")
            print(f"    [DEBUG: File exists: {os.path.exists(abs_drag_curve_path)}]")

            rocket.add_air_brakes(
                drag_coefficient_curve=abs_drag_curve_path,
                controller_function=controller_wrapper,
                sampling_rate=self.config.sampling_rate,
                reference_area=self.config.airbrake_area,
                clamp=True,
                initial_observed_variables=[0],
                override_rocket_drag=False,
                name="Air Brakes",
            )

            print(f"    [DEBUG: Airbrakes added successfully to rocket]")
        else:
            print(f"\n    [DEBUG: NOT adding airbrakes - use_airbrakes={use_airbrakes}, has_airbrakes={flight_config.has_airbrakes}]")
            deployment_log = None

        # Run flight simulation
        inclination = 90 - launch_angle

        try:
            flight = Flight(
                rocket=rocket,
                environment=environment,
                rail_length=self.config.rail_length,
                inclination=inclination,
                heading=0,
                time_overshoot=False,
                terminate_on_apogee=True
            )

            # FIXED: Calculate AGL consistently (already AGL since we use env_elevation)
            apogee_agl = flight.apogee - self.config.env_elevation

            # SANITY CHECK: Print basic values
            print(f"    [Sim apogee AGL: {apogee_agl:.2f}m, t_final: {flight.t_final:.2f}s]", end='')

            # DEBUG: Check deployment log
            if deployment_log:
                deployments_only = [d for _, d in deployment_log]
                if deployments_only:
                    print(f"\n    [DEBUG: Deployment - min: {min(deployments_only):.3f}, max: {max(deployments_only):.3f}, mean: {np.mean(deployments_only):.3f}]", end='')

            # FIXED: Sample at real flight time points to avoid interpolation error
            sim_times = []
            sim_altitudes_agl = []

            real_times = self.real_flight_data['times']
            time_end = min(flight.t_final, real_times[-1])

            # Use real flight time points where possible
            for t in real_times:
                if t > time_end:
                    break
                try:
                    # Get altitude MSL, convert to AGL
                    alt_msl = flight.altitude(t)
                    alt_agl = alt_msl - self.config.env_elevation
                    sim_times.append(t)
                    sim_altitudes_agl.append(alt_msl)
                except:
                    break

            return {
                'apogee': apogee_agl,
                'times': np.array(sim_times),
                'altitudes': np.array(sim_altitudes_agl),
                'flight': flight
            }

        except Exception as e:
            print(f" [SIM FAILED: {e}]", end='')
            return None

    def calculate_trajectory_error(self, sim_result):
        """
        Calculate normalized error between simulated and real trajectories

        Returns dict with multiple error metrics
        """
        if sim_result is None:
            return {
                'combined': 1e10,
                'rms': 1e10,
                'apogee_error': 1e10,
                'normalized_rms': 1e10
            }

        real_times = self.real_flight_data['times']
        real_altitudes = self.real_flight_data['altitudes']
        sim_times = sim_result['times']
        sim_altitudes = sim_result['altitudes']

        try:
            # Use only time points where both exist
            max_time = min(real_times[-1], sim_times[-1])

            # Create interpolator for simulation
            sim_interp = interp1d(sim_times, sim_altitudes, kind='linear',
                                  bounds_error=False, fill_value='extrapolate')

            # Find common time points
            mask = real_times <= max_time
            common_times = real_times[mask]
            common_real_alt = real_altitudes[mask]
            common_sim_alt = sim_interp(common_times)

            # Calculate errors
            errors = common_sim_alt - common_real_alt
            rms_error = np.sqrt(np.mean(errors ** 2))

            # FIXED: Normalize RMS by typical altitude magnitude
            typical_altitude = np.mean(np.abs(common_real_alt))
            normalized_rms = rms_error / max(typical_altitude, 1.0)

            # Apogee error
            apogee_error = sim_result['apogee'] - self.real_flight_data['apogee']

            # FIXED: Focus on trajectory error throughout ascent, not just apogee
            combined_error = 0.5 * rms_error + 0.1 * abs(apogee_error)

            return {
                'combined': combined_error,
                'rms': rms_error,
                'apogee_error': apogee_error,
                'normalized_rms': normalized_rms,
                'max_error': np.max(np.abs(errors)),
                'mean_error': np.mean(errors)
            }

        except Exception as e:
            print(f" [Error calc failed: {e}]", end='')
            return {
                'combined': 1e10,
                'rms': 1e10,
                'apogee_error': 1e10,
                'normalized_rms': 1e10
            }

    def find_rocket_cd_and_angle(self, cd_min=0.5, cd_max=1.0,
                                 angle_min=0, angle_max=10, flight_config=None):
        """Find rocket Cd and launch angle by matching trajectory"""
        if self.real_flight_data is None:
            raise ValueError("Must load real flight data first")

        self.current_flight_config = flight_config
        optimize_angle = flight_config.needs_angle_optimization

        print(f"\n{'=' * 60}")
        print(f"STEP 1: Finding Rocket Cd for {flight_config.name}")
        if optimize_angle:
            print("(Also optimizing launch angle for weathercocking)")
        print(f"{'=' * 60}")
        print(f"Flight mass: {flight_config.dry_mass:.3f} kg")
        print(f"Flight COM: {flight_config.com_no_motor:.3f} m")
        print(f"Target apogee: {self.real_flight_data['apogee']:.2f}m AGL")
        print(f"Cd range: {cd_min:.2f} to {cd_max:.2f}")
        if optimize_angle:
            print(f"Angle range: {angle_min:.1f}° to {angle_max:.1f}° from vertical")
        print(f"{'=' * 60}\n")

        self.simulation_history = []

        def objective(params):
            """Objective function optimizing trajectory match"""
            if optimize_angle:
                rocket_cd, launch_angle = params
            else:
                rocket_cd = params[0] if hasattr(params, '__iter__') else params
                launch_angle = 0.0

            print(f"  Cd={rocket_cd:.4f}", end='')
            if optimize_angle:
                print(f", Angle={launch_angle:.2f}°", end='')
            print(" -> ", end='', flush=True)

            sim_result = self.run_simulation(
                rocket_cd=rocket_cd,
                launch_angle=launch_angle,
                flight_config=flight_config,
                use_airbrakes=False
            )

            error_dict = self.calculate_trajectory_error(sim_result)

            self.simulation_history.append({
                'type': 'rocket_cd',
                'rocket_cd': rocket_cd,
                'launch_angle': launch_angle,
                'airbrake_cd': None,
                'apogee': sim_result['apogee'] if sim_result else None,
                **error_dict
            })

            print(
                f" RMS={error_dict['rms']:.2f}m, Apogee err={error_dict['apogee_error']:+.2f}m, Combined={error_dict['combined']:.2f}")

            return error_dict['combined']

        # Run optimization
        if optimize_angle:
            bounds = [(cd_min, cd_max), (angle_min, angle_max)]
            result = minimize(
                objective,
                x0=[0.7, 2.0],
                bounds=bounds,
                method='Nelder-Mead',
                options={'maxiter': 50, 'xatol': 0.01, 'fatol': 1.0}
            )
            self.rocket_cd, self.launch_angle = result.x
        else:
            result = minimize_scalar(
                objective,
                bounds=(cd_min, cd_max),
                method='bounded',
                options={'xatol': 0.01, 'maxiter': 20}
            )
            self.rocket_cd = result.x
            self.launch_angle = 0.0

        # Get final errors
        final_sim = self.run_simulation(
            rocket_cd=self.rocket_cd,
            launch_angle=self.launch_angle,
            flight_config=flight_config,
            use_airbrakes=False
        )
        final_errors = self.calculate_trajectory_error(final_sim)

        print(f"\n{'=' * 60}")
        print("ROCKET Cd OPTIMIZATION COMPLETE")
        print(f"{'=' * 60}")
        print(f"Rocket base drag coefficient: {self.rocket_cd:.4f}")
        if optimize_angle:
            print(f"Launch angle (from vertical): {self.launch_angle:.2f}°")
        print(f"Final combined error: {final_errors['combined']:.2f}")
        print(f"Final RMS trajectory error: {final_errors['rms']:.2f}m")
        print(f"Final apogee error: {final_errors['apogee_error']:.2f}m")
        print(f"Total simulations run: {len(self.simulation_history)}")
        print(f"{'=' * 60}\n")

        return self.rocket_cd, self.launch_angle

    def find_airbrake_cd(self, cd_min=0.1, cd_max=0.6, flight_config=None):
        """Find airbrake Cd by matching trajectory"""
        if self.real_flight_data is None:
            raise ValueError("Must load real flight data first")
        if self.rocket_cd is None:
            raise ValueError("Must determine rocket Cd first")

        self.current_flight_config = flight_config

        # Get max deployment from real flight
        max_deployment = np.max(self.real_flight_data['deployments'])

        print(f"\n{'=' * 60}")
        print(f"STEP 2: Finding Airbrake Cd for {flight_config.name}")
        print(f"{'=' * 60}")
        print(f"Using rocket Cd: {self.rocket_cd:.4f}")
        print(f"Flight mass: {flight_config.dry_mass:.3f} kg")
        print(f"Flight COM: {flight_config.com_no_motor:.3f} m")
        print(f"Target apogee: {self.real_flight_data['apogee']:.2f}m AGL")
        print(f"Airbrake Cd range: {cd_min:.2f} to {cd_max:.2f}")
        print(f"Max deployment from flight: {max_deployment:.3f}")
        print(f"{'=' * 60}\n")

        def objective(airbrake_cd):
            """Objective function for airbrake Cd"""
            print(f"  Airbrake Cd={airbrake_cd:.4f} -> ", end='', flush=True)

            sim_result = self.run_simulation(
                rocket_cd=self.rocket_cd,
                airbrake_cd=airbrake_cd,
                launch_angle=0.0,  # Flight 2 should be vertical (no weathercocking)
                flight_config=flight_config,
                use_airbrakes=True
            )

            error_dict = self.calculate_trajectory_error(sim_result)

            self.simulation_history.append({
                'type': 'airbrake_cd',
                'rocket_cd': self.rocket_cd,
                'launch_angle': 0.0,  # Always vertical for airbrake flights
                'airbrake_cd': airbrake_cd,
                'apogee': sim_result['apogee'] if sim_result else None,
                **error_dict
            })

            print(
                f" RMS={error_dict['rms']:.2f}m, Apogee err={error_dict['apogee_error']:+.2f}m, Combined={error_dict['combined']:.2f}")

            return error_dict['combined']

        result = minimize_scalar(
            objective,
            bounds=(cd_min, cd_max),
            method='bounded',
            options={'xatol': 0.01, 'maxiter': 20}
        )

        self.airbrake_cd = result.x

        # Get final errors
        final_sim = self.run_simulation(
            rocket_cd=self.rocket_cd,
            airbrake_cd=self.airbrake_cd,
            launch_angle=0.0,
            flight_config=flight_config,
            use_airbrakes=True
        )
        final_errors = self.calculate_trajectory_error(final_sim)

        print(f"\n{'=' * 60}")
        print("AIRBRAKE Cd OPTIMIZATION COMPLETE")
        print(f"{'=' * 60}")
        print(f"Airbrake max drag coefficient: {self.airbrake_cd:.4f}")
        print(f"Final combined error: {final_errors['combined']:.2f}")
        print(f"Final RMS trajectory error: {final_errors['rms']:.2f}m")
        print(f"Final apogee error: {final_errors['apogee_error']:.2f}m")
        print(f"{'=' * 60}\n")

        return self.airbrake_cd

    def plot_trajectory_comparison(self, filename='trajectory_comparison.png'):
        """Plot simulated vs real trajectory"""
        if self.real_flight_data is None:
            print("No flight data to plot")
            return

        sim_result = self.run_simulation(
            rocket_cd=self.rocket_cd,
            airbrake_cd=self.airbrake_cd,
            launch_angle=self.launch_angle,
            flight_config=self.current_flight_config,
            use_airbrakes=self.current_flight_config.has_airbrakes
        )

        if sim_result is None:
            print("Failed to generate final simulation")
            return

        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10))

        # Plot altitude vs time
        ax1.plot(self.real_flight_data['times'], self.real_flight_data['altitudes'],
                 'b-', linewidth=2, label='Real Flight', alpha=0.7)
        ax1.plot(sim_result['times'], sim_result['altitudes'],
                 'r--', linewidth=2, label='Simulation', alpha=0.7)
        ax1.set_xlabel('Time (s)')
        ax1.set_ylabel('Altitude AGL (m)')
        ax1.set_title(f'Trajectory Comparison - {self.current_flight_config.name}')
        ax1.legend()
        ax1.grid(True, alpha=0.3)

        # Plot error
        sim_interp = interp1d(sim_result['times'], sim_result['altitudes'],
                              kind='linear', bounds_error=False, fill_value='extrapolate')

        errors = sim_interp(self.real_flight_data['times']) - self.real_flight_data['altitudes']

        ax2.plot(self.real_flight_data['times'], errors, 'g-', linewidth=2)
        ax2.axhline(y=0, color='k', linestyle='--', alpha=0.3)
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Error (Sim - Real) (m)')
        ax2.set_title('Altitude Error Throughout Flight')
        ax2.grid(True, alpha=0.3)

        plt.tight_layout()
        plt.savefig(filename, dpi=150)
        print(f"Trajectory comparison saved to {filename}")
        plt.close()

    def export_results(self, output_file='drag_coefficient_results.csv'):
        """Export optimization history to CSV"""
        if not self.simulation_history:
            print("No results to export")
            return

        df = pd.DataFrame(self.simulation_history)
        df.to_csv(output_file, index=False)
        print(f"Results exported to {output_file}")


def main():
    """Main function to run drag coefficient calculation"""

    # Initialize base configuration
    config = ControllerConfig()

    # Define flight-specific configurations
    flight1_config = FlightConfig(
        name="Flight 1",
        dry_mass=0.618,  # kg
        com_no_motor=0.300,  # m
        has_airbrakes=False,
        needs_angle_optimization=False
    )

    flight2_config = FlightConfig(
        name="Flight 2",
        dry_mass=0.618,  # kg
        com_no_motor=0.300,  # m
        has_airbrakes=True,
        needs_angle_optimization=False
    )

    calculator = DragCoefficientCalculator(config)

    # ========================================================================
    # STEP 1: Find rocket base Cd from Flight 1 (no airbrakes)
    # ========================================================================

    flight1_path = "Flight2.CSV"

    if not os.path.exists(flight1_path):
        print(f"Error: Flight log file not found: {flight1_path}")
        return

    print("\n" + "=" * 60)
    print("LOADING FLIGHT 1 (No Airbrakes)")
    print("=" * 60)

    calculator.load_real_flight(
        flight1_path,
        time_col='time stamp',
        alt_col='y estimate',
        deployment_col='servo deployment'
    )

    # Find rocket Cd (and launch angle for weathercocking)
    rocket_cd, launch_angle = calculator.find_rocket_cd_and_angle(
        cd_min=0.5,
        cd_max=2,
        angle_min=0,
        angle_max=30,
        flight_config=flight1_config
    )

    # Plot trajectory comparison
    calculator.plot_trajectory_comparison('flight1_trajectory.png')

    # ========================================================================
    # STEP 2: Find airbrake Cd from Flight 2 (with airbrakes)
    # ========================================================================

    flight2_path = "Flight1.CSV"

    if os.path.exists(flight2_path):
        print("\n" + "=" * 60)
        print("LOADING FLIGHT 2 (With Airbrakes)")
        print("=" * 60)

        calculator.load_real_flight(
            flight2_path,
            time_col='time stamp',
            alt_col='y estimate',
            deployment_col='servo deployment'
        )

        # Find airbrake Cd
        airbrake_cd = calculator.find_airbrake_cd(
            cd_min=0.1,
            cd_max=2,
            flight_config=flight2_config
        )

        # Plot trajectory comparison
        calculator.plot_trajectory_comparison('flight2_trajectory.png')

        # Export all results
        calculator.export_results('drag_coefficient_optimization.csv')

        # Final summary
        print("\n" + "=" * 60)
        print("FINAL OPTIMIZED RESULTS")
        print("=" * 60)
        print(f"Rocket base drag coefficient:    {rocket_cd:.4f}")
        print(f"Launch angle (weathercocking):    {launch_angle:.2f}° from vertical")
        print(f"Airbrake max drag coefficient:   {airbrake_cd:.4f}")
        print("=" * 60)
        print("\n📝 UPDATE YOUR CONFIGURATION FILES:")
        print("-" * 40)
        print("config.py:")
        print(f"  apogee_prediction_cd = {rocket_cd:.4f}")
        print(f"  airbrake_drag = {airbrake_cd:.4f}")
        print("\nrocket_drag_curve.csv:")
        print(f"  mach,cd")
        print(f"  0.0,{rocket_cd:.4f}")
        print(f"  0.3,{rocket_cd:.4f}")
        print(f"  0.7,{rocket_cd:.4f}")
        print(f"  1.0,{rocket_cd:.4f}")
        print("\nairbrake_drag_curve.csv:")
        print(f"  deployment_level,mach,cd")
        print(f"  0.0,0.0,0.0000")
        print(f"  0.0,1.0,0.0000")
        print(f"  0.5,0.0,{airbrake_cd * 0.5:.4f}")
        print(f"  0.5,0.5,{airbrake_cd * 0.4:.4f}")
        print(f"  1.0,0.0,{airbrake_cd:.4f}")
        print(f"  1.0,0.5,{airbrake_cd * 0.8:.4f}")
        print("=" * 60)

    else:
        print(f"\nFlight 2 not found: {flight2_path}")
        print("Only rocket Cd was calculated.")

        # Export results
        calculator.export_results('drag_coefficient_optimization.csv')

        print("\n" + "=" * 60)
        print("PARTIAL RESULTS")
        print("=" * 60)
        print(f"Rocket base drag coefficient: {rocket_cd:.4f}")
        print(f"Launch angle (Flight 1):      {launch_angle:.2f}°")
        print("\nRun again with Flight 2 to find airbrake Cd")
        print("=" * 60)


if __name__ == "__main__":
    main()