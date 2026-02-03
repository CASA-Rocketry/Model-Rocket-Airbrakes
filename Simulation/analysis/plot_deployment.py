import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path
from config import Config


def plot_deployment(csv_path='sim_flight_data.csv', save_path='deployment_plot.png', config=Config):
    try:
        # Read the CSV file
        df = pd.read_csv(csv_path)

        # Filter to only show data where control is active AND after coast lockout
        config_instance = config() if callable(config) else config
        coast_lockout = config_instance.burn_time
        df_filtered = df[(df['Control_Active'] == 1) & (df['Time_s'] >= coast_lockout)].copy()

        if len(df_filtered) == 0:
            print(f"Warning: No data points with Control_Active == 1 after coast lockout ({coast_lockout}s) found.")
            return None

        # Extract relevant columns
        time = df_filtered['Time_s']
        deployment = df_filtered['Deployment']
        desired_deployment = df_filtered['Desired_Deployment']
        airbrake_force = df_filtered['V_Sim'] ** 2 * 0.5 * config_instance.air_density * config_instance.airbrake_drag * (config_instance.rocket_radius ** 2 * 3.1415) * deployment
        error = df_filtered['Error']  # Predicted apogee error with current deployment held constant

        # Create figure with subplots
        fig, ax1 = plt.subplots(1, 1, figsize=(12, 8), sharex=True)

        # Deployment comparison
        ax1.plot(time, desired_deployment, 'c--', label='Desired Deployment', linewidth=2, alpha=0.8)
        ax1.plot(time, deployment, 'b-', label='Actual Deployment', linewidth=1.5)
        ax1.set_ylabel('Deployment (0-1)', color='blue')
        ax1.set_title('Airbrake Deployment Over Time', fontsize=14, fontweight='bold')
        ax1.legend(loc='best', fontsize=10)
        ax1.grid(True, alpha=0.3)
        ax1.set_ylim(-0.05, 1.05)
        ax1.tick_params(axis='y', labelcolor='blue')

        ax2 = ax1.twinx()

        # Plot the second dataset on the second axes
        ax2.plot(time, airbrake_force, color='red', label='Series 2')
        ax2.set_ylabel('Drag Force (N)', color='red')
        ax2.tick_params(axis='y', labelcolor='red')

        ax3 = ax1.twinx()

        # Offset ax3 to prevent label overlap with ax2
        ax3.spines['right'].set_position(('outward', 60))

        ax3.plot(time, error, color='orange', label='Series 3')
        ax3.axhline(y=0, color='orange', linestyle='--', linewidth=1, alpha=0.5)
        ax3.set_ylabel('Error (m)', color='orange')
        ax3.tick_params(axis='y', labelcolor='orange')

        plt.tight_layout()
        # Create output directory if it doesn't exist
        Path(save_path).parent.mkdir(parents=True, exist_ok=True)
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"\nDeployment plot saved to '{save_path}'")

        plt.show()

        return fig, ax1

    except FileNotFoundError:
        print(f"Error: CSV file '{csv_path}' not found.")
        print("Please run the simulation first to generate the CSV file.")
        return None
    except KeyError as e:
        print(f"Error: Required column {e} not found in CSV file.")
        print("Make sure the CSV was generated with the latest controller code.")
        return None
    except Exception as e:
        print(f"Error plotting deployment: {e}")
        import traceback
        traceback.print_exc()
        return None


def plot_deployment_with_altitude(csv_path='sim_flight_data.csv', save_path='deployment_altitude_plot.png', config=Config):
    try:
        # Read the CSV file
        df = pd.read_csv(csv_path)

        # Get coast lockout time
        config_instance = config() if callable(config) else config
        coast_lockout = config_instance.burn_time

        # Extract all altitude data (for full flight profile)
        time_full = df['Time_s']
        altitude_full = df['Alt_Sim']

        # Filter deployment data to only after coast lockout
        df_filtered = df[df['Time_s'] >= coast_lockout].copy()
        time = df_filtered['Time_s']
        deployment = df_filtered['Deployment']
        desired_deployment = df_filtered['Desired_Deployment']
        altitude = df_filtered['Alt_Sim']

        # Create figure with subplots
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8), sharex=True)

        # Plot 1: Deployment
        ax1.plot(time, desired_deployment, 'b-', label='Desired Deployment', linewidth=2, alpha=0.8)
        ax1.plot(time, deployment, 'r--', label='Actual Deployment', linewidth=1.5)
        ax1.set_ylabel('Deployment (0-1)', fontsize=12, color='blue')
        ax1.tick_params(axis='y', labelcolor='blue')
        ax1.set_title('Airbrake Deployment and Altitude Over Time', fontsize=14, fontweight='bold')
        ax1.legend(loc='upper left', fontsize=10)
        ax1.grid(True, alpha=0.3)
        ax1.set_ylim(-0.05, 1.05)

        # Plot 2: Altitude (show full flight profile)
        ax2.plot(time_full, altitude_full, 'g-', linewidth=2)
        ax2.set_xlabel('Time (s)', fontsize=12)
        ax2.set_ylabel('Altitude AGL (m)', fontsize=12, color='green')
        ax2.tick_params(axis='y', labelcolor='green')
        ax2.grid(True, alpha=0.3)

        # Mark coast lockout time
        ax2.axvline(x=coast_lockout, color='orange', linestyle='--', alpha=0.5, linewidth=1.5, label=f'Coast Lockout: {coast_lockout:.1f}s')

        # Mark apogee
        apogee_idx = np.argmax(altitude_full)
        apogee_time = time_full.iloc[apogee_idx]
        apogee_alt = altitude_full.iloc[apogee_idx]
        ax2.axvline(x=apogee_time, color='red', linestyle='--', alpha=0.5, label=f'Apogee: {apogee_alt:.1f}m')
        ax2.legend(loc='upper right', fontsize=10)

        plt.tight_layout()
        # Create output directory if it doesn't exist
        Path(save_path).parent.mkdir(parents=True, exist_ok=True)
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"\nDeployment-Altitude plot saved to '{save_path}'")

        plt.show()

        return fig, (ax1, ax2)

    except Exception as e:
        print(f"Error plotting deployment with altitude: {e}")
        import traceback
        traceback.print_exc()
        return None


if __name__ == "__main__":
    print("="*60)
    print("Airbrake Deployment Plotting Tool")
    print("="*60)
    print("\n1. Plot Deployment Only (Desired vs Actual)")
    print("2. Plot Deployment with Altitude")
    print("3. Plot Both")

    choice = input("\nSelect option (1/2/3): ").strip()

    csv_file = input("CSV file path (press Enter for 'sim_flight_data.csv'): ").strip()
    if not csv_file:
        csv_file = '../sim_flight_data.csv'

    if choice == "1":
        plot_deployment(csv_file)
    elif choice == "2":
        plot_deployment_with_altitude(csv_file)
    elif choice == "3":
        plot_deployment(csv_file)
        plot_deployment_with_altitude(csv_file)
    else:
        print("Invalid choice. Exiting.")