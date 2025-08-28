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

    def plot_controller_analysis(self):
        """Create the 2x2 controller analysis plots"""
        if not self.controller.data['time']:
            print("No controller data available for plotting")
            return

        times = np.array(self.controller.data['time'])

        fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(15, 10))

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
        ax1.set_title("Altitude Tracking (Above Ground Level)")
        ax1.legend()
        ax1.grid(True, alpha=0.3)

        # Velocity tracking
        ax2.plot(times, self.controller.data['sim_velocity'], 'darkgreen', label='True', linewidth=2)
        ax2.plot(times, self.controller.data['filtered_velocity'], 'orange', label='Filtered', linewidth=2)
        ax2.set_ylabel("Velocity (m/s)")
        ax2.set_title("Velocity Tracking")
        ax2.legend()
        ax2.grid(True, alpha=0.3)

        # Deployment
        ax3.plot(times, self.controller.data['deployment'], 'purple', linewidth=2)
        ax3.set_ylabel("Deployment Level")
        ax3.set_xlabel("Time (s)")
        ax3.set_title("Air Brake Deployment")
        ax3.grid(True, alpha=0.3)

        # Apogee prediction
        ax4.plot(times, self.controller.data['predicted_apogee_agl'], 'darkred', linewidth=2, label='Predicted AGL')
        ax4.axhline(y=self.config.target_apogee, color='forestgreen', linestyle='--', linewidth=2, label='Target AGL')
        ax4.set_ylabel("Apogee AGL (m)")
        ax4.set_xlabel("Time (s)")
        ax4.set_title("Apogee Prediction (Above Ground Level)")
        ax4.legend()
        ax4.grid(True, alpha=0.3)

        plt.tight_layout()
        plt.show()

    def plot_flight_trajectory(self):
        """Plot 3D flight trajectory"""
        if not self.flight:
            print("No flight object available for trajectory plotting")
            return

        try:
            # Basic trajectory plot (use trajectory_3d instead of trajectory)
            self.flight.plots.trajectory_3d()
        except Exception as e:
            print(f"Could not create trajectory plot: {e}")
            print("Available plot methods:",
                  [method for method in dir(self.flight.plots) if not method.startswith('_')])

    def plot_comprehensive_flight_analysis(self):
        """Create comprehensive flight analysis plots using RocketPy"""
        if not self.flight:
            print("No flight object available for comprehensive analysis")
            return

        # Uncomment desired plots below:

        # Basic flight info and summary
        # self.flight.info()
        # self.flight.prints.initial_conditions()
        # self.flight.prints.numerical_integration_settings()
        # self.flight.prints.surface_wind_conditions()
        # self.flight.prints.launch_rail_conditions()
        # self.flight.prints.out_of_rail_conditions()
        # self.flight.prints.burn_out_conditions()
        # self.flight.prints.apogee_conditions()
        # self.flight.prints.maximum_values()
        # self.flight.prints.impact_conditions()

        # Trajectory plots
        # self.flight.plots.trajectory_3d()  # 3D trajectory
        # self.flight.plots.linear_kinematics()  # Position, velocity, acceleration vs time

        # Additional plots (uncomment as needed)
        # self.flight.plots.angular_kinematics()  # Angular position, velocity, acceleration
        # self.flight.plots.attitude_angle()  # Attitude angles vs time
        # self.flight.plots.aerodynamic_forces()  # Drag, normal forces
        # self.flight.plots.energy()  # Kinetic, potential, total energy
        # self.flight.plots.atmospheric_model()  # Temperature, pressure, density vs altitude
        # self.flight.plots.flight_path_angle()  # Flight path angle vs time
        # self.flight.plots.velocity_components()  # Velocity components
        # self.flight.plots.static_margin()  # Static margin vs time
        # self.flight.plots.stability_and_control()  # Stability derivatives

        # Ground track and landing analysis
        # self.flight.plots.xy()  # Ground track (x vs y)

        print(
            "To enable comprehensive flight analysis, uncomment desired plots in plot_comprehensive_flight_analysis() method")

    def plot_sensor_comparison(self):
        """Plot comparison between true flight data and sensor readings"""
        if not self.flight or not self.controller.data['time']:
            print("Need both flight object and controller data for sensor comparison")
            return

        times = np.array(self.controller.data['time'])

        fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(15, 10))

        # Altitude comparison
        try:
            flight_alt_asl = self.flight.z(times)
            flight_alt_agl = flight_alt_asl - self.config.env_elevation

            ax1.plot(times, flight_alt_agl, 'blue', label='True Flight AGL', linewidth=2)
            ax1.plot(times, self.controller.data['sim_altitude_agl'], 'green', label='Sim Truth AGL', linewidth=2)
            ax1.plot(times, self.controller.data['filtered_altitude_agl'], 'red', label='Filtered AGL', linewidth=2)

            # Raw barometer data
            raw_times = [times[i] for i, raw_alt in enumerate(self.controller.data['raw_altitude_agl']) if
                         raw_alt is not None]
            raw_altitudes = [raw_alt for raw_alt in self.controller.data['raw_altitude_agl'] if raw_alt is not None]
            if raw_times:
                ax1.scatter(raw_times, raw_altitudes, c='orange', s=10, alpha=0.6, label='Raw Barometer')

            ax1.set_ylabel("Altitude AGL (m)")
            ax1.set_title("Altitude: Flight vs Simulation vs Sensors")
            ax1.legend()
            ax1.grid(True, alpha=0.3)
        except Exception as e:
            print(f"Could not plot altitude comparison: {e}")

        # Velocity comparison
        try:
            flight_velocity = self.flight.speed(times)

            ax2.plot(times, flight_velocity, 'blue', label='True Flight Speed', linewidth=2)
            ax2.plot(times, self.controller.data['sim_velocity'], 'green', label='Sim Truth Velocity', linewidth=2)
            ax2.plot(times, self.controller.data['filtered_velocity'], 'red', label='Filtered Velocity', linewidth=2)
            ax2.set_ylabel("Velocity (m/s)")
            ax2.set_title("Velocity: Flight vs Simulation vs Filter")
            ax2.legend()
            ax2.grid(True, alpha=0.3)
        except Exception as e:
            print(f"Could not plot velocity comparison: {e}")

        # Mach number and dynamic pressure
        try:
            flight_mach = self.flight.mach_number(times)
            flight_dyn_pressure = self.flight.dynamic_pressure(times)

            ax3.plot(times, flight_mach, 'purple', linewidth=2)
            ax3.set_ylabel("Mach Number")
            ax3.set_title("Flight Mach Number")
            ax3.grid(True, alpha=0.3)

            ax4.plot(times, flight_dyn_pressure, 'brown', linewidth=2)
            ax4.set_ylabel("Dynamic Pressure (Pa)")
            ax4.set_xlabel("Time (s)")
            ax4.set_title("Dynamic Pressure")
            ax4.grid(True, alpha=0.3)
        except Exception as e:
            print(f"Could not plot Mach/pressure: {e}")

        plt.tight_layout()
        plt.show()

    def plot_control_analysis(self):
        """Plot simplified control system analysis"""
        if not self.controller.data['time']:
            print("No controller data available")
            return

        times = np.array(self.controller.data['time'])

        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 5))

        # Motor burning and control active phases
        motor_burning = np.array(self.controller.data['motor_burning'])
        control_active = np.array(self.controller.data['control_active'])

        ax1.fill_between(times, 0, 1, where=motor_burning, alpha=0.3, color='red', label='Motor Burning')
        ax1.fill_between(times, 0, 1, where=control_active, alpha=0.3, color='green', label='Control Active')
        ax1.set_ylabel("Phase")
        ax1.set_xlabel("Time (s)")
        ax1.set_title("Flight Phases")
        ax1.set_ylim(0, 1.1)
        ax1.legend()
        ax1.grid(True, alpha=0.3)

        # Simple deployment over time
        ax2.plot(times, self.controller.data['deployment'], 'purple', linewidth=2)
        ax2.set_ylabel("Deployment Level")
        ax2.set_xlabel("Time (s)")
        ax2.set_title("Air Brake Deployment Over Time")
        ax2.grid(True, alpha=0.3)

        plt.tight_layout()
        plt.show()

    def plot_filter_performance(self):
        """Plot simplified Kalman filter performance analysis"""
        if not self.controller.data['time']:
            print("No controller data available")
            return

        times = np.array(self.controller.data['time'])

        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 5))

        # Altitude errors
        sim_alt = np.array(self.controller.data['sim_altitude_agl'])
        filtered_alt = np.array(self.controller.data['filtered_altitude_agl'])
        alt_errors = np.abs(sim_alt - filtered_alt)

        ax1.plot(times, alt_errors, 'red', linewidth=2)
        ax1.set_ylabel("Altitude Error (m)")
        ax1.set_xlabel("Time (s)")
        ax1.set_title("Kalman Filter Altitude Error")
        ax1.grid(True, alpha=0.3)

        # Velocity errors
        sim_vel = np.array(self.controller.data['sim_velocity'])
        filtered_vel = np.array(self.controller.data['filtered_velocity'])
        vel_errors = np.abs(sim_vel - filtered_vel)

        ax2.plot(times, vel_errors, 'blue', linewidth=2)
        ax2.set_ylabel("Velocity Error (m/s)")
        ax2.set_xlabel("Time (s)")
        ax2.set_title("Kalman Filter Velocity Error")
        ax2.grid(True, alpha=0.3)

        plt.tight_layout()
        plt.show()

    def create_all_plots(self):
        """Create all available plots with error handling"""
        try:
            print("Creating controller analysis plots...")
            self.plot_controller_analysis()
        except Exception as e:
            print(f"Error creating controller analysis plots: {e}")

        try:
            print("Creating sensor comparison plots...")
            self.plot_sensor_comparison()
        except Exception as e:
            print(f"Error creating sensor comparison plots: {e}")

        try:
            print("Creating control analysis plots...")
            self.plot_control_analysis()
        except Exception as e:
            print(f"Error creating control analysis plots: {e}")

        try:
            print("Creating filter performance plots...")
            self.plot_filter_performance()
        except Exception as e:
            print(f"Error creating filter performance plots: {e}")

        if self.flight:
            try:
                print("Creating flight trajectory plot...")
                self.plot_flight_trajectory()
            except Exception as e:
                print(f"Error creating flight trajectory plot: {e}")

            # Uncomment to enable comprehensive flight analysis
            # try:
            #     print("Creating comprehensive flight analysis...")
            #     self.plot_comprehensive_flight_analysis()
            # except Exception as e:
            #     print(f"Error creating comprehensive flight analysis: {e}")
        else:
            print("No flight object available - skipping flight-specific plots")


def create_flight_plots(controller, config: ControllerConfig, flight: Flight = None):
    """Convenience function to create flight plots"""
    plotter = FlightPlotter(controller, config, flight)
    plotter.create_all_plots()
    return plotter