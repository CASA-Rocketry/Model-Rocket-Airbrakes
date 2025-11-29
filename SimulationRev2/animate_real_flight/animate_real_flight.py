import sys
from pathlib import Path

# Get the directory paths
script_dir = Path(__file__).parent
project_root = script_dir.parent
assets_dir = project_root / 'animation_assets'

sys.path.insert(0, str(project_root))
from SimulationRev2.analysis.animation import animate_rocket_flight

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
    output_path='flight.mp4',
    csv_time_col='Time',
    csv_altitude_col='Altitude',
    csv_deployment_col='Deployment',
    csv_velocity_col='Velocity',
    csv_path='flight3.CSV',
    ground_size=500,
    camera_option=1
)