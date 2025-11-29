from rocketpy import Flight
from Simulation.simulation_functions.setup_rocket import setup_rocket
from Simulation.simulation_functions.setup_environment import setup_environment

def run_simulation(config, controller):
    try:
        environment = setup_environment(config)
        rocket, motor = setup_rocket(config, controller.controller)

        print("Running simulation...")

        flight = Flight(
            rocket=rocket,
            environment=environment,
            rail_length=config.rail_length,
            inclination=90,
            heading=0,
            time_overshoot=False,
            terminate_on_apogee=config.terminate_on_apogee
        )

        return flight

    except Exception as e:
        print(f"Simulation error: {e}")
        import traceback
        traceback.print_exc()
        return None
