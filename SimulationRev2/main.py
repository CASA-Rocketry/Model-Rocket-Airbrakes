from SimulationRev2.simulation_functions.simulation import run_simulation
from config import Config
from controller import Control
from SimulationRev2.analysis.export_flight import export_to_csv
from SimulationRev2.analysis.animation import animate_rocket_flight
from SimulationRev2.analysis.plot_deployment import plot_deployment

# Create instances
config = Config()
controller = Control(config)

# Run simulation
flight = run_simulation(config, controller)
export_to_csv(flight, controller, config)

if flight is not None:
    print(f"\n{'='*60}")
    print("Simulation Results")
    print(f"{'='*60}")
    print(f"Apogee: {flight.apogee - config.env_elevation:.2f} m AGL")
    print(f"Max velocity: {flight.max_speed:.2f} m/s")
    print(f"Flight time: {flight.t_final:.2f} s")
    print(f"Target apogee: {config.target_apogee:.2f} m AGL")
    print(f"Real error: {(flight.apogee - config.env_elevation) - config.target_apogee:.2f} m")
    print(f"Error relative to avionics: {max(controller.data['filtered_altitude_agl']) - config.target_apogee:.2f} m")
    print(f"{'='*60}\n")
else:
    print("Simulation failed!")

plot_deployment("output/sim_flight_data.csv", "output/deployment.png")

animate_rocket_flight(
    rocket_body_path='animation_assets/red_parts.stl',
    rocket_black_path='animation_assets/black_parts.stl',
    airbrake_leaf_paths=[
        'animation_assets/brake_4.stl',
        'animation_assets/brake_1.stl',
        'animation_assets/brake_2.stl',
        'animation_assets/brake_3.stl'
    ],
    flight=flight,
    controller=controller,
    airbrake_max_extension=0.0165,
    output_path='output/flight.mp4',
    ground_size = 500,
    camera_option=1,
)

