from monte_carlo import run_monte_carlo
from config import Config
import csv
import numpy as np
import pandas as pd
import os

burnout_mass_low = 0.525
burnout_mass_high = 0.625
n_points = 10
n_simulations = 1000
config = Config()

delta_mass = (burnout_mass_high - burnout_mass_low) / (n_points - 1)

summary_data = []

# clear data point folder
for filename in os.listdir("output/mass finder/data points"):
    file_path = os.path.join("output/mass finder/data points", filename)
    os.unlink(file_path)

for n in range(n_points):

    print(f"Running data point {n+1} of {n_points}")

    burnout_mass = burnout_mass_low + n * delta_mass
    config = Config(burnout_mass=burnout_mass)
    monte_carlo, wall_time = run_monte_carlo(config=config, num_simulations=n_simulations)

    apogees_agl = np.array(monte_carlo.results['apogee']) - config.env_elevation
    errors = apogees_agl - config.target_apogee + config.apogee_offset
    deployments = np.array(monte_carlo.results['max_airbrake_deployment'])

    # Create detailed CSV for this mass configuration
    detail_filename = f'data_point_{burnout_mass:.4f}_kg.csv'
    detail_filepath = os.path.join('output', 'mass finder', 'data points', detail_filename)

    detail_data = {
        'Simulation': range(1, len(apogees_agl) + 1),
        'Apogee_AGL_m': apogees_agl + config.apogee_offset,
        'Error_m': errors,
        'Deployment_pct': deployments
    }

    detail_df = pd.DataFrame(detail_data)
    detail_df.to_csv(detail_filepath, index=False)

    # Add summary statistics to summary data
    summary_data.append({
        'Burnout_Mass_kg': burnout_mass,
        'Mean_Apogee_m': np.mean(apogees_agl) + config.apogee_offset,
        'Std_Apogee_m': np.std(apogees_agl),
        'Mean_Error_m': np.mean(errors),
        'Mean_Deployment_pct': np.mean(deployments),
        'Detail_File': detail_filename,
        'Wall_Time': wall_time
    })



# Write summary CSV
summary_df = pd.DataFrame(summary_data)
summary_filepath = os.path.join('output', 'mass finder', 'mass_finder_summary.csv')
summary_df.to_csv(summary_filepath, index=False)

print(f"Mass finder complete. Summary saved to {summary_filepath}")