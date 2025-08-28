import logging
from config import ControllerConfig
from simulation import SimulationRunner
from plotting import FlightPlotter

logging.basicConfig(level=logging.WARNING)


def main():
    config = ControllerConfig()
    runner = SimulationRunner(config)

    # Run simulation
    flight = runner.run_full_simulation()

    # Create plots using FlightPlotter
    print("Creating plots...")
    plotter = FlightPlotter(runner.controller, config, flight)
    plotter.create_all_plots()

    # Print summary
    runner.print_summary(flight)


if __name__ == "__main__":
    main()