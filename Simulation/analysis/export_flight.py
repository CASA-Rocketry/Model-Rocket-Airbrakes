import pandas as pd
import numpy as np
from pathlib import Path

def export_to_csv(flight, controller, config):
    """Export flight data to CSV with temperature data"""

    try:
        # Create DataFrame from controller data
        df = pd.DataFrame({
            'Time_s': controller.data['time'],
            'Alt_Sim': controller.data['sim_altitude_agl'],
            'Alt_Raw': controller.data['raw_altitude_agl'],
            'Alt_Filtered': controller.data['filtered_altitude_agl'],
            'V_Sim': controller.data['sim_velocity'],
            'V_Filtered': controller.data['filtered_velocity'],
            'A_Sim': controller.data['sim_acceleration'],
            'A_Raw': controller.data['raw_acceleration'],
            'A_Filtered': controller.data['filtered_acceleration'],
            'Deployment': controller.data['deployment'],
            'Desired_Deployment': controller.data['desired_deployment'],
            'Predicted_Apogee': controller.data['predicted_apogee'],
            'Predicted_Apogee_No_Brake': controller.data['predicted_apogee_no_brake'],
            'Control_Active': controller.data['control_active']
        })

        # Export to CSV
        script_dir = Path(__file__).parent
        output_dir = script_dir.parent / 'output'
        output_dir.mkdir(parents=True, exist_ok=True)
        filename = output_dir / 'sim_flight_data.csv'
        df.to_csv(filename, index=False)
        print(f"\nData exported to {filename}")
        print(f"Rows: {len(df)}, Columns: {len(df.columns)}")

    except Exception as e:
        print(f"Error exporting CSV: {e}")
        import traceback
        traceback.print_exc()
