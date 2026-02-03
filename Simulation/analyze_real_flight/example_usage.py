"""
Process real flight data with Kalman filtering and thrust curve interpolation
"""

from real_flight_processing import RealFlightProcessor, ThrustCurveInterpolator
import sys
import os

# Add parent directory to path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from config import Config


def process_flight():
    """Process real flight data with thrust curve and Kalman filtering"""

    # Create config
    config = Config()

    # Filter parameters - no deployment
    config.alt_std = 0.26
    config.accel_std = 0.05
    config.model_y_std = 0.00025
    config.model_v_std = 0.00125
    config.model_a_std = 0.0065

    # Filter parameters - with deployment (when airbrakes deployed)
    deployment_alt_std = 0.26
    deployment_accel_std = 0.05
    deployment_model_y_std = 0.25
    deployment_model_v_std = 0.0125
    deployment_model_a_std = 0.00065

    # Initialize processor
    processor = RealFlightProcessor(config, accel_saturation_threshold=2.95)

    # Load flight data
    csv_path = "flight4mod.csv"
    print(f"Loading flight data from {csv_path}...")
    flight_data = processor.load_flight_data(
        csv_path,
        time_col='Time (us)',
        altitude_col='Altitude AGL (m)',
        accel_col='IMU Global Acceleration z',
        accel_x_col='IMU Global Acceleration x',
        accel_y_col='IMU Global Acceleration y',
        state_col='Mode',
        time_unit='s'
    )

    print(f"Loaded {len(flight_data)} data points")

    # Create thrust curve interpolator
    script_dir = os.path.dirname(os.path.abspath(__file__))
    eng_file_path = os.path.join(script_dir, "..", config.engine_file)
    initial_rocket_mass = config.burnout_mass + 0.027  # kg

    print(f"\nInitializing thrust curve interpolator...")
    print(f"  Engine file: {eng_file_path}")
    print(f"  Initial rocket mass (with propellant): {initial_rocket_mass:.3f} kg")

    thrust_interpolator = ThrustCurveInterpolator(
        eng_file_path,
        initial_rocket_mass,
        rocket_diameter=config.rocket_radius * 2,
        drag_coefficient=config.apogee_prediction_cd
    )

    # Define custom time ranges for deployment filter (optional)
    # If None, will auto-detect based on state 3
    # Example: [(1.5, 2.5), (3.0, 4.0)] uses deployment filter from 1.5-2.5s and 3.0-4.0s
    deployment_filter_ranges = [(1.5, 8)] #[(2, 4), (4.1, 6.1)]#[(1.5, 3.1), (3.15, 7)]  # Set to None to auto-detect, or specify custom ranges

    # Run filter
    print("\nProcessing flight data...")
    results = processor.run_filter(
        flight_data,
        handle_saturation=True,
        rate_limit_deployment=2.5,
        thrust_curve_interpolator=thrust_interpolator,
        deployment_model_y_std=deployment_model_y_std,
        deployment_model_v_std=deployment_model_v_std,
        deployment_model_a_std=deployment_model_a_std,
        deployment_alt_std=deployment_alt_std,
        deployment_accel_std=deployment_accel_std,
        deployment_filter_duration=3.0,  # Used if deployment_filter_ranges is None
        deployment_filter_ranges=deployment_filter_ranges,  # Custom time ranges or None for auto-detect
        deployment_alt_std_velocity_scale=1  # Scale altitude std by v² (σ_total = σ_base * v²)
    )

    # Calculate metrics
    metrics = processor.calculate_metrics(results)
    print("\nPerformance Metrics:")
    print(f"  Altitude RMSE: {metrics['altitude_rmse']:.3f} m")
    print(f"  Altitude MAE: {metrics['altitude_mae']:.3f} m")
    print(f"  Max Error: {metrics['altitude_max_error']:.3f} m")
    print(f"  Saturated Points: {metrics['num_saturated_points']} ({metrics['saturation_percentage']:.1f}%)")

    # Save results
    output_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)), "output")
    results.to_csv(os.path.join(output_dir, "real_flight_results.csv"), index=False)
    print(f"\nResults saved to real_flight_results.csv")

    # Plot results
    processor.plot_results(results, save_path=os.path.join(output_dir, "real_flight_plots.png"))


if __name__ == "__main__":
    process_flight()