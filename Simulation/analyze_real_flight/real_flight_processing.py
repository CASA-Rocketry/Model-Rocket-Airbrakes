import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import sys
import os
from scipy.interpolate import interp1d

# Add parent directory to path to import config and kalman filter
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from config import Config
from controllers.controller_functions.kalman_filter import KalmanAltitudeFilter


class ThrustCurveInterpolator:
    """
    Loads and interpolates thrust curve from .eng motor file

    The thrust curve provides expected thrust force over time, which is used
    to calculate acceleration during motor burn: a(t) = (thrust(t) - drag(t)) / mass(t) - g

    IMPORTANT: Accounts for changing rocket mass as propellant burns.
    - Propellant mass is extracted from .eng file header
    - Mass flow rate proportional to thrust: dm/dt ∝ thrust(t)
    - Propellant consumed = propellant_mass × (impulse_consumed / total_impulse)
    - More accurate than linear time model: accounts for varying burn rate
    - This is critical because as mass decreases, acceleration increases (F=ma)
    - Maximum acceleration occurs at motor burnout when mass is minimum

    ALSO accounts for aerodynamic drag:
    - Drag = 0.5 * ρ * v² * Cd * A
    - Air density varies with altitude (standard atmosphere model)
    """

    def __init__(self, eng_file_path: str, rocket_mass: float, rocket_diameter: float = 0.066, drag_coefficient: float = 0.5):
        """
        Initialize thrust curve interpolator

        Args:
            eng_file_path: Path to .eng motor file (RASP format)
            rocket_mass: INITIAL rocket mass in kg at ignition (includes propellant)
                        Example: 0.670 kg for rocket + loaded motor
                        NOT the burnout mass! This should be the measured launch mass.
            rocket_diameter: Rocket diameter in meters (default: 0.066m = 66mm)
            drag_coefficient: Drag coefficient Cd (default: 0.5)
        """
        self.eng_file_path = eng_file_path
        self.initial_rocket_mass = rocket_mass  # Mass at ignition
        self.rocket_diameter = rocket_diameter
        self.drag_coefficient = drag_coefficient
        self.reference_area = np.pi * (rocket_diameter / 2) ** 2  # Cross-sectional area
        self.time_data = []
        self.thrust_data = []
        self.interpolator = None
        self.burn_time = 0.0
        self.propellant_mass = 0.0  # Will be loaded from file
        self.total_impulse = 0.0
        self.cumulative_impulse = None  # Cumulative impulse over time
        self.impulse_interpolator = None  # Interpolator for impulse(t)

        self._load_thrust_curve()

    def _load_thrust_curve(self):
        """Load thrust curve from .eng file and extract motor parameters"""
        with open(self.eng_file_path, 'r') as f:
            lines = f.readlines()

        # Skip comment lines (start with ;)
        data_lines = [line.strip() for line in lines if line.strip() and not line.startswith(';')]

        # First data line is the motor header
        # Format: Name Diameter(mm) Length(mm) Delays PropMass(kg) TotalMass(kg) Manufacturer
        header = data_lines[0].split()
        if len(header) >= 6:
            self.propellant_mass = float(header[4])  # Propellant mass in kg
            print(f"    Loaded motor: {header[0]}, propellant mass: {self.propellant_mass:.4f} kg")
        else:
            print(f"    Warning: Could not parse propellant mass from motor file, assuming 0.027 kg")
            self.propellant_mass = 0.027  # Default fallback

        # Load thrust curve data points
        for line in data_lines[1:]:
            parts = line.split()
            if len(parts) >= 2:
                time = float(parts[0])
                thrust = float(parts[1])
                self.time_data.append(time)
                self.thrust_data.append(thrust)

        self.time_data = np.array(self.time_data)
        self.thrust_data = np.array(self.thrust_data)
        self.burn_time = self.time_data[-1]

        # Calculate total impulse using trapezoidal integration
        self.total_impulse = np.trapz(self.thrust_data, self.time_data)
        print(f"    Total impulse: {self.total_impulse:.1f} N·s")

        # Calculate cumulative impulse at each time point
        # This tells us how much propellant has been consumed up to time t
        # Assumption: mass flow rate ∝ thrust, so propellant consumed ∝ impulse
        self.cumulative_impulse = np.zeros_like(self.time_data)
        for i in range(1, len(self.time_data)):
            # Integrate thrust from 0 to current time
            self.cumulative_impulse[i] = np.trapz(
                self.thrust_data[:i+1],
                self.time_data[:i+1]
            )

        # Create interpolators
        self.interpolator = interp1d(
            self.time_data,
            self.thrust_data,
            kind='linear',
            bounds_error=False,
            fill_value=0.0  # Return 0 thrust outside burn time
        )

        self.impulse_interpolator = interp1d(
            self.time_data,
            self.cumulative_impulse,
            kind='linear',
            bounds_error=False,
            fill_value=(0.0, self.total_impulse)  # 0 before burn, total after
        )

    def get_thrust(self, time: float) -> float:
        """
        Get interpolated thrust at given time

        Args:
            time: Time in seconds

        Returns:
            Thrust force in Newtons
        """
        return float(self.interpolator(time))

    def get_current_mass(self, time: float) -> float:
        """
        Get rocket mass at given time, accounting for propellant consumption

        Uses thrust-based mass consumption model:
        - Assumes mass flow rate is proportional to thrust: dm/dt ∝ thrust(t)
        - Propellant consumed = propellant_mass × (impulse_consumed / total_impulse)
        - More physically accurate than linear time-based model

        This accounts for the fact that when thrust is high, more propellant
        is being burned per second, and vice versa.

        Args:
            time: Time in seconds

        Returns:
            Current rocket mass in kg
        """
        if time <= 0:
            return self.initial_rocket_mass
        elif time >= self.burn_time:
            # All propellant consumed
            return self.initial_rocket_mass - self.propellant_mass
        else:
            # Thrust-based propellant consumption model
            # Get cumulative impulse up to this time
            impulse_consumed = float(self.impulse_interpolator(time))

            # Fraction of propellant burned = fraction of total impulse delivered
            fraction_burned = impulse_consumed / self.total_impulse if self.total_impulse > 0 else 0

            # Calculate mass consumed
            propellant_consumed = self.propellant_mass * fraction_burned

            return self.initial_rocket_mass - propellant_consumed

    def get_air_density(self, altitude: float) -> float:
        """
        Get air density at given altitude using standard atmosphere model

        Args:
            altitude: Altitude in meters AGL

        Returns:
            Air density in kg/m³
        """
        # Standard atmosphere approximation
        # ρ = ρ₀ * exp(-altitude / H) where H ≈ 8500m (scale height)
        rho_0 = 1.225  # Sea level density kg/m³
        scale_height = 8500.0  # meters
        return rho_0 * np.exp(-altitude / scale_height)

    def get_drag_force(self, velocity: float, altitude: float) -> float:
        """
        Calculate drag force at given velocity and altitude

        Args:
            velocity: Velocity in m/s (positive = upward)
            altitude: Altitude in meters AGL

        Returns:
            Drag force in Newtons (always opposes velocity)
        """
        rho = self.get_air_density(altitude)
        # Drag = 0.5 * ρ * v² * Cd * A
        # Sign: drag always opposes motion
        drag_magnitude = 0.5 * rho * velocity**2 * self.drag_coefficient * self.reference_area
        # Return negative if going up (drag opposes upward motion)
        return -drag_magnitude if velocity > 0 else drag_magnitude

    def get_acceleration(self, time: float, velocity: float = 0.0, altitude: float = 0.0) -> float:
        """
        Get net acceleration at given time, accounting for thrust, drag, mass, and gravity

        Returns NET acceleration to match what accelerometer measures.
        a_net = (F_thrust - F_drag) / m - g

        Args:
            time: Time in seconds since motor ignition
            velocity: Current velocity in m/s (for drag calculation, default 0)
            altitude: Current altitude in m AGL (for air density, default 0)

        Returns:
            Net acceleration in m/s² (thrust/mass - drag/mass - gravity)
        """
        thrust = self.get_thrust(time)
        current_mass = self.get_current_mass(time)

        if current_mass > 0:
            # Calculate drag force (opposes velocity)
            drag = self.get_drag_force(velocity, altitude)

            # Net acceleration = (thrust + drag) / mass - gravity
            # Note: drag is negative when going up, so it reduces acceleration
            return (thrust + drag) / current_mass - 9.81
        else:
            return -9.81  # Just gravity if no mass

    def is_burning(self, time: float) -> bool:
        """Check if motor is still burning at given time"""
        return 0.0 <= time <= self.burn_time


class RealFlightProcessor:
    """Tool for processing real flight data with Kalman filtering and acceleration integration"""

    def __init__(self, config: Config, accel_saturation_threshold=4.0):
        """
        Initialize the processor

        Args:
            config: Configuration object with Kalman filter parameters
            accel_saturation_threshold: Acceleration threshold (in g) for saturation detection
        """
        self.config = config
        self.accel_saturation_threshold = accel_saturation_threshold
        self.kalman_filter = None

    def load_flight_data(self, csv_file_path: str,
                        time_col='timestamp',
                        altitude_col='altitude',
                        accel_col='accelerometer',
                        accel_x_col=None,
                        accel_y_col=None,
                        state_col=None,
                        time_unit='us'):
        """
        Load flight data from CSV file

        Args:
            csv_file_path: Path to CSV file
            time_col: Name of timestamp column
            altitude_col: Name of altitude column
            accel_col: Name of accelerometer column (vertical acceleration)
            accel_x_col: Name of X acceleration column (optional)
            accel_y_col: Name of Y acceleration column (optional)
            state_col: Name of flight state column (optional, for state machine tracking)
            time_unit: Unit of time column ('us' for microseconds, 'ms' for milliseconds, 's' for seconds)

        Returns:
            DataFrame with flight data
        """
        df = pd.read_csv(csv_file_path)

        # Validate required columns exist
        required_cols = [time_col, altitude_col, accel_col]
        missing_cols = [col for col in required_cols if col not in df.columns]
        if missing_cols:
            raise ValueError(f"Missing required columns: {missing_cols}")

        # Rename columns to standard names
        rename_dict = {
            time_col: 'time',
            altitude_col: 'altitude',
            accel_col: 'accel'
        }

        # Add optional acceleration columns if provided
        if accel_x_col and accel_x_col in df.columns:
            rename_dict[accel_x_col] = 'accel_x'
        if accel_y_col and accel_y_col in df.columns:
            rename_dict[accel_y_col] = 'accel_y'
        if state_col and state_col in df.columns:
            rename_dict[state_col] = 'state'

        df = df.rename(columns=rename_dict)

        # Sort by time
        df = df.sort_values('time')

        # Reset index
        df = df.reset_index(drop=True)

        # Convert time to seconds and normalize to start from 0
        if time_unit == 'us':
            # Convert microseconds to seconds
            df['time'] = (df['time'] - df['time'].iloc[0]) / 1e6
        elif time_unit == 'ms':
            # Convert milliseconds to seconds
            df['time'] = (df['time'] - df['time'].iloc[0]) / 1e3
        elif time_unit == 's':
            # Already in seconds, just normalize to start from 0
            df['time'] = df['time'] - df['time'].iloc[0]
        else:
            raise ValueError(f"Invalid time_unit '{time_unit}'. Must be 'us', 'ms', or 's'")

        return df

    def run_filter(self, flight_data: pd.DataFrame,
                   initial_altitude=None,
                   handle_saturation=True,
                   override_deployment=None,
                   no_airbrake_decel=-12.0,
                   rate_limit_deployment=None,
                   thrust_curve_interpolator=None,
                   deployment_model_y_std=None,
                   deployment_model_v_std=None,
                   deployment_model_a_std=None,
                   deployment_alt_std=None,
                   deployment_accel_std=None,
                   deployment_filter_duration=1.0,
                   deployment_filter_ranges=None,
                   deployment_alt_std_velocity_scale=None):
        """
        Run Kalman filter on flight data

        Args:
            flight_data: DataFrame with 'time', 'altitude', 'accel' columns
            initial_altitude: Initial altitude for filter (if None, uses first measurement)
            handle_saturation: If True, set accel std to inf when accel >= threshold
            override_deployment: If not None, override all deployment values with this value (e.g., 0.0)
            no_airbrake_decel: Deceleration (m/s²) to use for apogee prediction without airbrakes (default: -12.0)
            rate_limit_deployment: If not None, apply rate limiting to deployment (units/second, e.g., 1.0)
            thrust_curve_interpolator: Optional ThrustCurveInterpolator object. If provided, the Kalman filter
                                      will use thrust-based acceleration MEASUREMENT during motor burn.
                                      HOW IT WORKS:
                                      - Only active when flight state == 2 (powered flight)
                                      - Thrust curve time t=0 starts when state first becomes 2
                                      - During state 2 AND motor burn, thrust curve acceleration REPLACES
                                        the measured acceleration (which is typically saturated)
                                      - This provides accurate acceleration data when the accelerometer saturates
                                      - After burnout OR state change, reverts to actual accelerometer measurements
                                      - Much more accurate than using saturated accelerometer readings
            deployment_filter_ranges: Optional list of (start_time, end_time) tuples specifying when to use
                                     deployment filter parameters. If None, will auto-detect based on state 3.
                                     Example: [(1.5, 2.5), (3.0, 4.0)] uses deployment filter from 1.5-2.5s and 3.0-4.0s

        Returns:
            DataFrame with original data plus filter estimates
        """
        # Initialize filter
        self.kalman_filter = KalmanAltitudeFilter(self.config)

        if initial_altitude is None:
            initial_altitude = flight_data['altitude'].iloc[0]

        self.kalman_filter.initialize(initial_altitude, self.config.sampling_rate)

        # Storage for results
        filtered_altitude = []
        filtered_velocity = []
        filtered_accel = []
        saturated_flags = []
        thrust_accel_values = []  # Store thrust accelerations for analysis
        thrust_used_count = 0  # Counter for how many times thrust curve was used

        # Check if state column exists
        has_state = 'state' in flight_data.columns

        # Find ignition time (when state first becomes 2)
        ignition_time = None
        state_3_time = None
        if has_state and thrust_curve_interpolator:
            # Debug: show unique state values
            unique_states = flight_data['state'].unique()
            print(f"\n  Debug: Unique state values in data: {sorted(unique_states)}")

            state_2_mask = flight_data['state'] == 2
            num_state_2 = state_2_mask.sum()
            print(f"  Debug: Found {num_state_2} data points with state==2")

            if state_2_mask.any():
                ignition_time = flight_data.loc[state_2_mask, 'time'].iloc[0]
                print(f"  Detected ignition at t={ignition_time:.3f}s (first state==2)")
            else:
                print(f"  Warning: No state==2 found in data, thrust curve will not be used!")

        # Determine deployment filter time ranges
        if deployment_filter_ranges is not None:
            # Use user-specified ranges
            print(f"\n  Using custom deployment filter ranges:")
            for i, (start, end) in enumerate(deployment_filter_ranges):
                print(f"    Range {i+1}: t={start:.3f}s to t={end:.3f}s ({end-start:.1f}s duration)")
        else:
            # Auto-detect based on state 3 if available
            if has_state:
                state_3_mask = flight_data['state'] == 3
                if state_3_mask.any():
                    state_3_time = flight_data.loc[state_3_mask, 'time'].iloc[0]
                    print(f"  Detected state 3 at t={state_3_time:.3f}s")
                    if deployment_model_y_std or deployment_model_v_std or deployment_model_a_std or deployment_alt_std or deployment_accel_std:
                        print(f"  Deployment filter will be active from t={state_3_time:.3f}s to t={state_3_time + deployment_filter_duration:.3f}s ({deployment_filter_duration:.1f}s duration)")
                        # Create single range based on state 3
                        deployment_filter_ranges = [(state_3_time, state_3_time + deployment_filter_duration)]

        # Print thrust curve info if provided
        if thrust_curve_interpolator:
            print(f"\n  Using thrust curve interpolation:")
            print(f"    Motor burn time: {thrust_curve_interpolator.burn_time:.2f} s")
            print(f"    Initial rocket mass: {thrust_curve_interpolator.initial_rocket_mass:.3f} kg")
            print(f"    Final rocket mass: {thrust_curve_interpolator.get_current_mass(999):.3f} kg")
            print(f"    Propellant mass: {thrust_curve_interpolator.propellant_mass:.4f} kg")
            print(f"    Max thrust: {np.max(thrust_curve_interpolator.thrust_data):.1f} N")
            print(f"    Max acceleration: {np.max(thrust_curve_interpolator.thrust_data) / thrust_curve_interpolator.get_current_mass(thrust_curve_interpolator.burn_time):.1f} m/s²")
            if has_state:
                print(f"    Thrust curve ONLY used when state == 2 (powered flight)")
                if ignition_time is not None:
                    print(f"    Thrust curve t=0 starts at flight time t={ignition_time:.3f}s")
            print(f"    During burn: thrust accel REPLACES saturated accelerometer measurement")
            print(f"    After burn: switches back to accelerometer measurement")

        # Process each data point
        for idx, row in flight_data.iterrows():
            time = row['time']
            altitude = row['altitude']
            accel_measured = row['accel']

            # Get flight state if available (check for NaN)
            flight_state = int(row['state']) if has_state and 'state' in row and pd.notna(row['state']) else None

            # Get thrust acceleration if interpolator provided
            thrust_accel = 0.0
            is_burning = False
            if thrust_curve_interpolator:
                # Calculate time relative to ignition (when state first became 2)
                if ignition_time is not None:
                    thrust_time = time - ignition_time
                else:
                    # No state column, use absolute time
                    thrust_time = time

                # Get current velocity and altitude estimates from Kalman filter (if available)
                # Use previous estimates for drag calculation
                if len(filtered_velocity) > 0:
                    current_velocity = filtered_velocity[-1]
                    current_altitude = filtered_altitude[-1]
                else:
                    current_velocity = 0.0
                    current_altitude = altitude

                thrust_accel = thrust_curve_interpolator.get_acceleration(thrust_time, current_velocity, current_altitude)
                is_burning = thrust_curve_interpolator.is_burning(thrust_time)
                thrust_accel_values.append(thrust_accel)

            # Determine which acceleration measurement to use
            # ONLY use thrust curve when:
            # 1. Thrust interpolator is provided
            # 2. Motor is burning (based on time since state==2)
            # 3. If state column exists: state must == 2 (powered flight)
            #    If no state column: always use thrust curve during burn
            if has_state:
                use_thrust_curve = (
                    thrust_curve_interpolator and
                    is_burning and
                    flight_state == 2
                )
            else:
                # No state column - use thrust curve whenever burning
                use_thrust_curve = (
                    thrust_curve_interpolator and
                    is_burning
                )

            # Debug first few uses of thrust curve
            if use_thrust_curve and thrust_used_count < 5:
                print(f"  DEBUG: Using thrust curve at t={time:.3f}s, state={flight_state}, thrust_time={thrust_time:.3f}s, thrust_accel={thrust_accel:.1f} m/s²")

            if use_thrust_curve:
                # During motor burn in state 2: USE THRUST CURVE instead of saturated accelerometer
                accel_to_use = thrust_accel
                # Don't flag as saturated since we're using thrust model
                is_saturated = False
                thrust_used_count += 1
            else:
                # Not in powered flight state or after burnout: use actual accelerometer
                accel_to_use = accel_measured
                # Check for accelerometer saturation (convert accel to g's for comparison)
                accel_in_g = accel_measured / 9.81
                is_saturated = handle_saturation and abs(accel_in_g) >= self.accel_saturation_threshold

                # Debug why thrust curve NOT used (only first few times during potential burn period)
                if thrust_curve_interpolator and thrust_used_count < 5 and idx < 1000:
                    if has_state and flight_state != 2:
                        if idx % 100 == 0:  # Only print every 100th to avoid spam
                            print(f"  DEBUG: NOT using thrust at t={time:.3f}s - state={flight_state} (not 2)")
                    elif not is_burning:
                        if thrust_used_count == 0 and idx % 100 == 0:
                            print(f"  DEBUG: NOT using thrust at t={time:.3f}s - not burning (thrust_time={thrust_time:.3f}s, burn_time={thrust_curve_interpolator.burn_time:.3f}s)")

            saturated_flags.append(is_saturated)

            # Check if current time falls within any deployment filter range
            deployment_filter_active = False
            if deployment_filter_ranges is not None:
                for start_time, end_time in deployment_filter_ranges:
                    if start_time <= time <= end_time:
                        deployment_filter_active = True
                        break

            # Store original Q and R matrices if we need to modify them
            original_Q = None
            original_R = None

            # Switch to deployment-specific filter variances if within active window
            if deployment_filter_active and (deployment_model_y_std or deployment_model_v_std or
                                            deployment_model_a_std or deployment_alt_std or deployment_accel_std):
                original_Q = self.kalman_filter.Q.copy()
                original_R = self.kalman_filter.R.copy()

                # Update process noise covariance (Q matrix) if provided
                if deployment_model_y_std is not None:
                    self.kalman_filter.Q[0, 0] = deployment_model_y_std ** 2
                if deployment_model_v_std is not None:
                    self.kalman_filter.Q[1, 1] = deployment_model_v_std ** 2
                if deployment_model_a_std is not None:
                    self.kalman_filter.Q[2, 2] = deployment_model_a_std ** 2

                # Update measurement noise covariance (R matrix) if provided
                if deployment_alt_std is not None:
                    # Get current velocity estimate for velocity scaling
                    if deployment_alt_std_velocity_scale is not None and len(filtered_velocity) > 0:
                        current_vel = filtered_velocity[-1]
                        # σ_total = σ_base * v²
                        alt_std_scaled = deployment_alt_std * (current_vel ** 2 + 1)
                        self.kalman_filter.R[0, 0] = deployment_alt_std ** 2 * (current_vel ** 2 + 1)
                    else:
                        # No velocity scaling, use base value
                        self.kalman_filter.R[0, 0] = deployment_alt_std ** 2

                if deployment_accel_std is not None:
                    self.kalman_filter.R[1, 1] = deployment_accel_std ** 2

            # Override R matrix if saturated (takes precedence over deployment)
            if is_saturated:
                if original_R is None:
                    original_R = self.kalman_filter.R.copy()
                # Set accel measurement variance to very large value (effectively infinite)
                self.kalman_filter.R[1, 1] = 1e10

            # Update filter with chosen acceleration measurement
            alt_est, vel_est = self.kalman_filter.update(
                altitude,
                accel_to_use,  # <-- Use thrust accel during burn, measured accel after
                time,
                motor_burn_time=0
            )

            # Restore Q and R matrices if they were modified
            if original_Q is not None:
                self.kalman_filter.Q = original_Q
            if original_R is not None:
                self.kalman_filter.R = original_R

            # Store estimates
            filtered_altitude.append(alt_est)
            filtered_velocity.append(vel_est)
            filtered_accel.append(self.kalman_filter.getAEstimate())

        # Add results to dataframe
        results = flight_data.copy()
        results['filtered_altitude'] = filtered_altitude
        results['filtered_velocity'] = filtered_velocity
        results['filtered_accel'] = filtered_accel
        results['saturated'] = saturated_flags

        # Add thrust acceleration if it was used
        if thrust_curve_interpolator:
            results['thrust_accel'] = thrust_accel_values
            print(f"\n  Thrust curve was used for {thrust_used_count}/{len(results)} data points")

        # Calculate predicted apogee with no airbrakes for each timestep
        # Using kinematic equation: apogee = current_altitude + v²/(2*|a|)
        # Only calculate when velocity > 0 (ascending)
        predicted_apogee = []
        for alt, vel in zip(filtered_altitude, filtered_velocity):
            if vel > 0:
                # Ballistic trajectory: how much higher will it go with constant deceleration?
                delta_h = -(vel ** 2) / (2 * no_airbrake_decel)
                apogee = alt + delta_h
            else:
                # Descending or at apogee, predicted apogee is current max altitude
                apogee = alt
            predicted_apogee.append(apogee)

        results['predicted_apogee_no_airbrake'] = predicted_apogee

        # Integrate horizontal accelerations if available
        if 'accel_x' in flight_data.columns and 'accel_y' in flight_data.columns:
            time_array = flight_data['time'].values
            ax = flight_data['accel_x'].values
            ay = flight_data['accel_y'].values

            # Integrate acceleration to get velocity
            # Using cumulative trapezoidal integration
            vx = np.zeros_like(ax)
            vy = np.zeros_like(ay)
            x = np.zeros_like(ax)
            y = np.zeros_like(ay)

            for i in range(1, len(time_array)):
                dt = time_array[i] - time_array[i-1]
                # Trapezoidal integration for velocity
                vx[i] = vx[i-1] + 0.5 * (ax[i-1] + ax[i]) * dt
                vy[i] = vy[i-1] + 0.5 * (ay[i-1] + ay[i]) * dt
                # Trapezoidal integration for position
                x[i] = x[i-1] + 0.5 * (vx[i-1] + vx[i]) * dt
                y[i] = y[i-1] + 0.5 * (vy[i-1] + vy[i]) * dt

            results['integrated_vx'] = vx
            results['integrated_vy'] = vy
            results['integrated_x'] = x
            results['integrated_y'] = y

        # Override deployment values if requested
        if override_deployment is not None:
            # Find deployment columns (case-insensitive search)
            deployment_cols = [col for col in results.columns if 'deployment' in col.lower()]
            for col in deployment_cols:
                results[col] = override_deployment
                print(f"  Overriding '{col}' column with value: {override_deployment}")

        # Apply rate limiting to deployment if requested
        if rate_limit_deployment is not None:
            # Find deployment columns (case-insensitive search)
            deployment_cols = [col for col in results.columns if 'deployment' in col.lower()]
            time_array = results['time'].values

            for col in deployment_cols:
                original_deployment = results[col].values
                rate_limited = np.zeros_like(original_deployment)
                rate_limited[0] = original_deployment[0]

                # Apply rate limiting
                for i in range(1, len(original_deployment)):
                    dt = time_array[i] - time_array[i-1]
                    max_change = rate_limit_deployment * dt
                    desired_change = original_deployment[i] - rate_limited[i-1]

                    # Clamp the change to the rate limit
                    actual_change = np.clip(desired_change, -max_change, max_change)
                    rate_limited[i] = rate_limited[i-1] + actual_change

                # Add as new column
                new_col_name = f"{col}_rate_limited"
                results[new_col_name] = rate_limited
                print(f"  Added rate-limited deployment column: '{new_col_name}' (rate: {rate_limit_deployment}/s)")

        return results

    def plot_results(self, results: pd.DataFrame, save_path=None):
        """
        Plot filter results

        Args:
            results: DataFrame from run_filter()
            save_path: If provided, save plot to this path
        """
        fig, axes = plt.subplots(3, 1, figsize=(12, 10))

        # Altitude plot
        ax = axes[0]
        ax.plot(results['time'], results['altitude'], 'b.', label='Measured', alpha=0.5)
        ax.plot(results['time'], results['filtered_altitude'], 'r-', label='Filtered', linewidth=2)
        ax.set_ylabel('Altitude (m)')
        ax.set_title('Kalman Filter Performance')
        ax.legend()
        ax.grid(True)

        # Velocity plot
        ax = axes[1]
        ax.plot(results['time'], results['filtered_velocity'], 'g-', label='Filtered Velocity', linewidth=2)
        ax.set_ylabel('Velocity (m/s)')
        ax.legend()
        ax.grid(True)

        # Acceleration plot
        ax = axes[2]
        ax.plot(results['time'], results['accel'], 'b.', label='Measured', alpha=0.5)
        ax.plot(results['time'], results['filtered_accel'], 'r-', label='Filtered', linewidth=2)

        # Plot thrust acceleration if available
        if 'thrust_accel' in results.columns:
            ax.plot(results['time'], results['thrust_accel'], 'm--', label='Thrust Accel (Model)', linewidth=1.5)

        # Highlight saturated regions (convert threshold from g to m/s²)
        if results['saturated'].any():
            saturation_threshold_ms2 = self.accel_saturation_threshold * 9.81
            saturated_mask = results['saturated']
            ax.axhspan(saturation_threshold_ms2, results['accel'].max() * 1.1,
                      alpha=0.2, color='red', label=f'Saturation Region (±{self.accel_saturation_threshold}g)')
            ax.axhspan(-saturation_threshold_ms2, results['accel'].min() * 1.1,
                      alpha=0.2, color='red')

        ax.set_ylabel('Acceleration (m/s²)')
        ax.set_xlabel('Time (s)')
        ax.legend()
        ax.grid(True)

        plt.tight_layout()

        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            print(f"Plot saved to {save_path}")

        plt.show()

    def calculate_metrics(self, results: pd.DataFrame):
        """
        Calculate performance metrics

        Args:
            results: DataFrame from run_filter()

        Returns:
            Dictionary of metrics
        """
        # Altitude error statistics
        altitude_error = results['altitude'] - results['filtered_altitude']

        metrics = {
            'altitude_rmse': np.sqrt(np.mean(altitude_error**2)),
            'altitude_mae': np.mean(np.abs(altitude_error)),
            'altitude_max_error': np.max(np.abs(altitude_error)),
            'num_saturated_points': results['saturated'].sum(),
            'saturation_percentage': 100 * results['saturated'].sum() / len(results)
        }

        return metrics


def main():
    """Example usage"""
    # Load configuration
    config = Config()

    # Example: modify filter parameters for tuning
    config.alt_std = 2.0
    config.accel_std = 0.5
    config.model_y_std = 0.1
    config.model_v_std = 0.5
    config.model_a_std = 1.0

    # Initialize processor
    processor = RealFlightProcessor(config, accel_saturation_threshold=2.95)

    # Load flight data
    # REPLACE THIS PATH with your actual log file
    csv_path = "12.23.25 - Flight 2.CSV"

    print(f"Loading flight data from {csv_path}...")
    flight_data = processor.load_flight_data(
        csv_path,
        time_col='Time (us)',      # Adjust column names as needed
        altitude_col='Altitude AGL (m)',
        accel_col='IMU Global Acceleration z',
        accel_x_col='IMU Global Acceleration x',     # Optional: X acceleration column
        accel_y_col='IMU Global Acceleration y',     # Optional: Y acceleration column
        time_unit='us'             # 'us' for microseconds, 'ms' for milliseconds, 's' for seconds
    )

    print(f"Loaded {len(flight_data)} data points")
    print(f"Time range: {flight_data['time'].min():.2f}s to {flight_data['time'].max():.2f}s")

    # Run filter and integration
    print("\nProcessing flight data...")
    results = processor.run_filter(flight_data, handle_saturation=True, override_deployment=0)

    # Calculate metrics
    metrics = processor.calculate_metrics(results)
    print("\nFilter Performance Metrics:")
    print(f"  Altitude RMSE: {metrics['altitude_rmse']:.3f} m")
    print(f"  Altitude MAE: {metrics['altitude_mae']:.3f} m")
    print(f"  Max Altitude Error: {metrics['altitude_max_error']:.3f} m")
    print(f"  Saturated Points: {metrics['num_saturated_points']} ({metrics['saturation_percentage']:.1f}%)")

    # Save results
    output_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)), "output")
    output_path = os.path.join(output_dir, "real_flight_results.csv")
    results.to_csv(output_path, index=False)
    print(f"\nResults saved to {output_path}")

    # Plot results
    print("\nGenerating plots...")
    processor.plot_results(results, save_path=os.path.join(output_dir, "real_flight_analysis.png"))


if __name__ == "__main__":
    main()