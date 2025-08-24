import logging
from config import ControllerConfig
from simulation import SimulationRunner

logging.basicConfig(level=logging.WARNING)

def main():
    config = ControllerConfig()
    runner = SimulationRunner(config)
    runner.run_full_simulation()

if __name__ == "__main__":
    main()