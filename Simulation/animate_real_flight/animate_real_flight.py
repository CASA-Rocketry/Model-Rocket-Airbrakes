import sys
from pathlib import Path

# Get the directory paths
script_dir = Path(__file__).parent
project_root = script_dir.parent
assets_dir = project_root / 'animation_assets'

sys.path.insert(0, str(project_root))
from Simulation.analysis.animation import animate_rocket_flight

animate_rocket_flight(
    rocket_body_path=str(assets_dir / 'red_parts.stl'),
    rocket_black_path=str(assets_dir / 'black_parts.stl'),
    airbrake_leaf_paths=[
        str(assets_dir / 'brake_4.stl'),
        str(assets_dir / 'brake_1.stl'),
        str(assets_dir / 'brake_2.stl'),
        str(assets_dir / 'brake_3.stl')
    ],
    airbrake_max_extension=0.0165,
    csv_path=str(project_root / 'output' / 'real_flight_results.csv'),
    output_path=str(project_root / 'output' / 'real_flight_animation.mp4'),
    csv_time_col='time',
    csv_altitude_col='filtered_altitude',
    csv_deployment_col='Servo deployment_rate_limited',
    csv_velocity_col='filtered_velocity',
    csv_e0_col='IMU Quat W',
    csv_e1_col='IMU Quat X',
    csv_e2_col='IMU Quat Y',
    csv_e3_col='IMU Quat Z',
    ground_size=500,
    camera_option=1,
    fps=30
)