import matplotlib.pyplot as plt
import numpy as np
from rocketpy import Flight
from config import ControllerConfig


class FlightPlotter:
    """Plotting for flight simulation data"""

    def __init__(self, controller, config: ControllerConfig, flight: Flight = None):
        self.controller = controller
        self.config = config
        self.flight = flight

    def plot_tracking_and_error_analysis(self):
        """Window 1: Combined tracking and error analysis in 3x2 layout"""
        if not self.controller or not self.controller.data['time']:
            print("No controller data available for plotting")
            return

        times = np.array(self.controller.data['time'])

        fig, ((ax1, ax2, ax3), (ax4, ax5, ax6)) = plt.subplots(2, 3, figsize=(18, 10))

        # Altitude tracking
        ax1.plot(times, self.controller.data['sim_altitude_agl'], 'blue', label='True AGL', linewidth=2)
        ax1.plot(times, self.controller.data['filtered_altitude_agl'], 'red', label='Filtered AGL', linewidth=2)

        # Add raw barometer data
        raw_times = []
        raw_altitudes = []
        for i, raw_alt in enumerate(self.controller.data['raw_altitude_agl']):
            if raw_alt is not None:
                raw_times.append(times[i])
                raw_altitudes.append(raw_alt)

        if raw_times:
            ax1.scatter(raw_times, raw_altitudes, c='orange', s=10, alpha=0.6, label='Raw Barometer', zorder=3)

        ax1.set_ylabel("Altitude AGL (m)")
        ax1.set_xlabel("Time (s)")
        ax1.set_title("Altitude Tracking")
        ax1.legend()
        ax1.grid(True, alpha=0.3)

        # Velocity tracking
        ax2.plot(times, self.controller.data['sim_velocity'], 'darkgreen', label='True', linewidth=2)
        ax2.plot(times, self.controller.data['filtered_velocity'], 'orange', label='Filtered', linewidth=2)
        ax2.set_ylabel("Velocity (m/s)")
        ax2.set_xlabel("Time (s)")
        ax2.set_title("Velocity Tracking")
        ax2.legend()
        ax2.grid(True, alpha=0.3)

        # Acceleration tracking
        if self.flight:
            try:
                flight_acceleration = self.flight.acceleration(times)
                ax3.plot(times, flight_acceleration, 'blue', label='True Acceleration', linewidth=2)
            except Exception as e:
                print(f"Could not get flight acceleration: {e}")

        # Add filtered acceleration
        filtered_accel = np.array(self.controller.data['filtered_acceleration'])
        filtered_accel_abs = np.abs(filtered_accel)
        ax3.plot(times, filtered_accel_abs, 'red', label='Filtered Acceleration', linewidth=2)

        ax3.set_ylabel("Acceleration (m/s²)")
        ax3.set_xlabel("Time (s)")
        ax3.set_title("Acceleration Tracking")
        ax3.legend()
        ax3.grid(True, alpha=0.3)

        # Altitude errors
        sim_alt = np.array(self.controller.data['sim_altitude_agl'])
        filtered_alt = np.array(self.controller.data['filtered_altitude_agl'])
        alt_errors = np.abs(sim_alt - filtered_alt)

        ax4.plot(times, alt_errors, 'red', linewidth=2)
        ax4.set_ylabel("Altitude Error (m)")
        ax4.set_xlabel("Time (s)")
        ax4.set_title("Altitude Error")
        ax4.grid(True, alpha=0.3)

        # Velocity errors
        sim_vel = np.array(self.controller.data['sim_velocity'])
        filtered_vel = np.array(self.controller.data['filtered_velocity'])
        vel_errors = np.abs(sim_vel - filtered_vel)

        ax5.plot(times, vel_errors, 'blue', linewidth=2)
        ax5.set_ylabel("Velocity Error (m/s)")
        ax5.set_xlabel("Time (s)")
        ax5.set_title("Velocity Error")
        ax5.grid(True, alpha=0.3)

        # Acceleration errors
        if self.flight and 'filtered_acceleration' in self.controller.data:
            try:
                true_accel = self.flight.acceleration(times)
                filtered_accel = np.array(self.controller.data['filtered_acceleration'])
                accel_errors = np.abs(true_accel - np.abs(filtered_accel))

                ax6.plot(times, accel_errors, 'purple', linewidth=2)
                ax6.set_ylabel("Acceleration Error (m/s²)")
                ax6.set_xlabel("Time (s)")
                ax6.set_title("Acceleration Error")
                ax6.grid(True, alpha=0.3)
            except Exception as e:
                print(f"Could not plot acceleration error: {e}")
                ax6.text(0.5, 0.5, 'Acceleration Error\nData Not Available',
                         ha='center', va='center', transform=ax6.transAxes, fontsize=12)
                ax6.set_title("Acceleration Error")
        else:
            ax6.text(0.5, 0.5, 'Acceleration Error\nData Not Available',
                     ha='center', va='center', transform=ax6.transAxes, fontsize=12)
            ax6.set_title("Acceleration Error")

        plt.tight_layout()
        plt.show()

    def plot_control_analysis(self):
        """Window 2: Deployment level, apogee prediction (from control active), and airbrake drag force"""
        if not self.controller or not self.controller.data['time']:
            print("No controller data available for control analysis")
            return

        times = np.array(self.controller.data['time'])
        control_active = np.array(self.controller.data['control_active'])

        fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(18, 6))

        # Deployment level
        deployment = np.array(self.controller.data['deployment'])
        ax1.plot(times, deployment, 'purple', linewidth=2)
        ax1.set_ylabel("Deployment Level")
        ax1.set_xlabel("Time (s)")
        ax1.set_title("Airbrake Deployment Level")
        ax1.set_ylim(0, 1.1)
        ax1.grid(True, alpha=0.3)

        # Apogee prediction (only from when control is active)
        control_times = times[control_active]
        control_apogee = np.array(self.controller.data['predicted_apogee_agl'])[control_active]

        if len(control_times) > 0:
            ax2.plot(control_times, control_apogee, 'darkred', linewidth=2, label='Predicted AGL')
            ax2.axhline(y=self.config.target_apogee, color='forestgreen', linestyle='--', linewidth=2,
                        label='Target AGL')
        else:
            ax2.text(0.5, 0.5, 'No Control Active\nPeriod Found',
                     ha='center', va='center', transform=ax2.transAxes, fontsize=12)

        ax2.set_ylabel("Apogee AGL (m)")
        ax2.set_xlabel("Time (s)")
        ax2.set_title("Apogee Prediction (Control Active Only)")
        ax2.legend()
        ax2.grid(True, alpha=0.3)

        # Airbrake drag force
        if self.flight:
            try:
                # Calculate airbrake drag force
                air_density = self.config.air_density
                airbrake_area = self.config.airbrake_area

                # Get velocity from flight data
                velocities = self.flight.speed(times)

                # Estimate Cd from deployment level
                cd_values = deployment * self.config.airbrake_drag

                drag_forces = 0.5 * air_density * velocities ** 2 * cd_values * airbrake_area

                ax3.plot(times, drag_forces, 'brown', linewidth=2)
                ax3.set_ylabel("Airbrake Drag Force (N)")
                ax3.set_xlabel("Time (s)")
                ax3.set_title("Airbrake Drag Force")
                ax3.grid(True, alpha=0.3)
            except Exception as e:
                print(f"Could not calculate airbrake drag force: {e}")
                ax3.text(0.5, 0.5, 'Drag Force\nCalculation Failed',
                         ha='center', va='center', transform=ax3.transAxes, fontsize=12)
                ax3.set_title("Airbrake Drag Force")
        else:
            ax3.text(0.5, 0.5, 'Flight Data\nNot Available',
                     ha='center', va='center', transform=ax3.transAxes, fontsize=12)
            ax3.set_title("Airbrake Drag Force")

        plt.tight_layout()
        plt.show()

    def plot_flight_trajectory(self):
        """Window 3: 3D flight trajectory"""
        if not self.flight:
            print("No flight object available for trajectory plotting")
            return

        try:
            # 3D trajectory plot
            self.flight.plots.trajectory_3d()
        except Exception as e:
            print(f"Could not create trajectory plot: {e}")
            print("Available plot methods:",
                  [method for method in dir(self.flight.plots) if not method.startswith('_')])

    def create_all_plots(self):
        """Create all plots in the specified order"""
        try:
            print("Creating tracking and error analysis plots (Window 1)...")
            self.plot_tracking_and_error_analysis()
        except Exception as e:
            print(f"Error creating tracking and error analysis plots: {e}")

        try:
            print("Creating control analysis plots (Window 2)...")
            self.plot_control_analysis()
        except Exception as e:
            print(f"Error creating control analysis plots: {e}")

        if self.flight:
            try:
                print("Creating 3D flight trajectory (Window 3)...")
                self.plot_flight_trajectory()
            except Exception as e:
                print(f"Error creating flight trajectory plot: {e}")
        else:
            print("No flight object available - skipping 3D trajectory plot")


def create_flight_plots(controller, config: ControllerConfig, flight: Flight = None):
    """Convenience function to create flight plots"""
    plotter = FlightPlotter(controller, config, flight)
    plotter.create_all_plots()
    return plotter