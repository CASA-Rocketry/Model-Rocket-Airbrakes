"""
Example: Running simulation with deployment data from file

This script shows how to replay deployment data from a real flight
(or any CSV file) in the simulation instead of using active control.

Useful for:
- Comparing simulated vs real flight with same deployment profile
- Testing "what if" scenarios with different deployment profiles
- Analyzing the effect of specific deployment sequences
"""

from Simulation.simulation_functions.simulation import run_simulation
from config import Config
from controller import Control
from Simulation.analysis.export_flight import export_to_csv
from Simulation.analysis.plot_deployment import plot_deployment

# Create config
config = Config()

deployment_file_path = "output/real_flight_results.csv"

# Create file-based controller
controller = Control(config, deployment_file=deployment_file_path)

# ============================================================================
# Option 2: Or use a custom deployment profile CSV
# ============================================================================
# Create a CSV with your own deployment profile:
# time,deployment
# 0.0,0.0
# 0.5,0.0
# 1.0,0.3
# 1.5,0.7
# 2.0,1.0
# 3.0,1.0
#
# custom_profile = "my_deployment_profile.csv"
# controller = Control(config, deployment_file=custom_profile)

# ============================================================================
# Run simulation
# ============================================================================
print("\nRunning simulation with file-based deployment...")
print(f"Deployment file: {deployment_file_path}")
print("="*60)

flight = run_simulation(config, controller)
export_to_csv(flight, controller, config)

if flight is not None:
    print(f"\n{'='*60}")
    print("Simulation Results (with file-based deployment)")
    print(f"{'='*60}")
    print(f"Apogee: {flight.apogee - config.env_elevation:.2f} m AGL")
    print(f"Max velocity: {flight.max_speed:.2f} m/s")
    print(f"Flight time: {flight.t_final:.2f} s")
    print(f"Target apogee: {config.target_apogee:.2f} m AGL")
    print(f"Error: {(flight.apogee - config.env_elevation) - config.target_apogee:.2f} m")
    print(f"{'='*60}\n")
else:
    print("Simulation failed!")

# Plot deployment
plot_deployment("output/sim_flight_data.csv", "output/deployment_from_file.png")

print("\nNote: This simulation used the deployment profile from the file,")
print("not active control. Compare this to a simulation with active control")
print("to see the difference!")
