#!/usr/bin/env python3
"""
Simplified Barometric Calibrator using ISA atmospheric model

Dependencies: ambiance, numpy, sklearn
Install with: pip install ambiance numpy scikit-learn
"""

import numpy as np
from sklearn.preprocessing import PolynomialFeatures
from sklearn.linear_model import LinearRegression
from sklearn.pipeline import Pipeline
from ambiance import Atmosphere


class BarometricCalibrator:
    def __init__(self, ground_altitude_m=0, max_altitude_agl=5000, num_points=200):
        """
        Initialize and generate calibration functions

        Args:
            ground_altitude_m: Launch site altitude above sea level (meters)
            max_altitude_agl: Maximum altitude above ground level for calibration (meters)
            num_points: Number of calibration points to generate
        """
        self.ground_altitude_m = ground_altitude_m

        # Get ground conditions
        ground_atm = Atmosphere(ground_altitude_m)
        self.ground_pressure = ground_atm.pressure[0]  # Pa
        self.ground_temperature = ground_atm.temperature[0]  # K

        # Generate calibration data
        self._generate_calibration_data(max_altitude_agl, num_points)

        # Fit models
        self.pressure_model = self._fit_pressure_model()
        self.pressure_temp_model = self._fit_pressure_temp_model()

    def _generate_calibration_data(self, max_altitude_agl, num_points):
        """Generate ISA atmospheric data for calibration"""
        altitudes_agl = np.linspace(0, max_altitude_agl, num_points)
        altitudes_msl = altitudes_agl + self.ground_altitude_m

        atmosphere = Atmosphere(altitudes_msl)

        self.cal_altitudes = altitudes_agl
        self.cal_pressures = atmosphere.pressure
        self.cal_temperatures = atmosphere.temperature

    def _fit_pressure_model(self, degree=4):
        """Fit polynomial model: pressure → altitude"""
        pressure_ratios = self.cal_pressures / self.ground_pressure

        model = Pipeline([
            ('poly', PolynomialFeatures(degree=degree)),
            ('linear', LinearRegression())
        ])

        model.fit(pressure_ratios.reshape(-1, 1), self.cal_altitudes)
        return model

    def _fit_pressure_temp_model(self, degree=3):
        """Fit polynomial model: pressure + temperature → altitude"""
        pressure_ratios = self.cal_pressures / self.ground_pressure
        temp_ratios = self.cal_temperatures / self.ground_temperature

        features = np.column_stack([pressure_ratios, temp_ratios])

        model = Pipeline([
            ('poly', PolynomialFeatures(degree=degree)),
            ('linear', LinearRegression())
        ])

        model.fit(features, self.cal_altitudes)
        return model

    def print_functions(self):
        """Print the altitude functions with their mathematical expressions"""
        # Pressure-only function
        coeffs = self.pressure_model.named_steps['linear'].coef_
        intercept = self.pressure_model.named_steps['linear'].intercept_

        print("PRESSURE-ONLY ALTITUDE FUNCTION:")
        print("-" * 40)
        print(f"Ground pressure: {self.ground_pressure:.1f} Pa")
        print(f"Ground temperature: {self.ground_temperature:.1f} K")
        print()
        print("def get_altitude_pressure_only(pressure_pa):")
        print(f"    p_ratio = pressure_pa / {self.ground_pressure:.1f}")
        print(f"    altitude = {intercept:.6f}", end="")

        for i, coeff in enumerate(coeffs[1:], 1):
            if coeff >= 0:
                print(f" + {coeff:.6f} * (p_ratio^{i})", end="")
            else:
                print(f" - {abs(coeff):.6f} * (p_ratio^{i})", end="")
        print()
        print("    return altitude")
        print()

        # Pressure + temperature function
        coeffs = self.pressure_temp_model.named_steps['linear'].coef_
        intercept = self.pressure_temp_model.named_steps['linear'].intercept_
        feature_names = self.pressure_temp_model.named_steps['poly'].get_feature_names_out(['p_ratio', 't_ratio'])

        print("PRESSURE + TEMPERATURE ALTITUDE FUNCTION:")
        print("-" * 40)
        print("def get_altitude_pressure_temp(pressure_pa, temperature_k):")
        print(f"    p_ratio = pressure_pa / {self.ground_pressure:.1f}")
        print(f"    t_ratio = temperature_k / {self.ground_temperature:.1f}")
        print(f"    altitude = {intercept:.6f}", end="")

        for coeff, feature in zip(coeffs, feature_names):
            if feature == '1':  # Skip intercept
                continue

            sign = " + " if coeff >= 0 else " - "
            coeff_abs = abs(coeff)

            if feature == 'p_ratio':
                print(f"{sign}{coeff_abs:.6f} * p_ratio", end="")
            elif feature == 't_ratio':
                print(f"{sign}{coeff_abs:.6f} * t_ratio", end="")
            elif '^' in feature and ' ' not in feature:  # Pure powers like p_ratio^2
                if feature.startswith('p_ratio^'):
                    power = int(feature.split('^')[1])
                    print(f"{sign}{coeff_abs:.6f} * (p_ratio^{power})", end="")
                elif feature.startswith('t_ratio^'):
                    power = int(feature.split('^')[1])
                    print(f"{sign}{coeff_abs:.6f} * (t_ratio^{power})", end="")
            elif ' ' in feature:  # Interaction terms like 'p_ratio t_ratio' or 'p_ratio^2 t_ratio'
                terms = feature.replace(' ', ' * ')
                print(f"{sign}{coeff_abs:.6f} * {terms}", end="")
            else:  # Fallback for any other terms
                print(f"{sign}{coeff_abs:.6f} * {feature}", end="")

        print()
        print("    return altitude")
        print()

    def plot_accuracy(self):
        """Plot model accuracy vs ISA truth"""
        import matplotlib.pyplot as plt

        # Create test data (different from training data)
        test_altitudes = np.linspace(0, self.cal_altitudes[-1], 50)
        test_altitudes_msl = test_altitudes + self.ground_altitude_m
        test_atm = Atmosphere(test_altitudes_msl)

        test_pressures = test_atm.pressure
        test_temperatures = test_atm.temperature

        # Get model predictions
        pressure_ratios = test_pressures / self.ground_pressure
        pred_pressure_only = self.pressure_model.predict(pressure_ratios.reshape(-1, 1))

        temp_ratios = test_temperatures / self.ground_temperature
        features = np.column_stack([pressure_ratios, temp_ratios])
        pred_pressure_temp = self.pressure_temp_model.predict(features)

        # Calculate errors
        error_pressure_only = pred_pressure_only - test_altitudes
        error_pressure_temp = pred_pressure_temp - test_altitudes

        # Create plots
        fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(15, 10))

        # Plot 1: Pressure vs Altitude with model fit
        ax1.plot(test_pressures, test_altitudes, 'b-', linewidth=2, label='ISA Truth')
        ax1.plot(test_pressures, pred_pressure_only, 'r--', linewidth=2, label='Pressure Model')
        ax1.set_xlabel('Pressure (Pa)')
        ax1.set_ylabel('Altitude (m AGL)')
        ax1.set_title('Pressure Model vs ISA Truth')
        ax1.legend()
        ax1.grid(True, alpha=0.3)

        # Plot 2: Temperature effect
        ax2.plot(test_pressures, test_altitudes, 'b-', linewidth=2, label='ISA Truth')
        ax2.plot(test_pressures, pred_pressure_only, 'r--', linewidth=2, label='Pressure Only')
        ax2.plot(test_pressures, pred_pressure_temp, 'g:', linewidth=2, label='Pressure + Temp')
        ax2.set_xlabel('Pressure (Pa)')
        ax2.set_ylabel('Altitude (m AGL)')
        ax2.set_title('Model Comparison')
        ax2.legend()
        ax2.grid(True, alpha=0.3)

        # Plot 3: Pressure-only model errors
        ax3.plot(test_altitudes, error_pressure_only, 'ro-', markersize=4, label='Pressure Model')
        ax3.axhline(y=0, color='black', linestyle='-', alpha=0.5)
        ax3.set_xlabel('True Altitude (m AGL)')
        ax3.set_ylabel('Error (m)')
        ax3.set_title(f'Pressure Model Error (RMS: {np.sqrt(np.mean(error_pressure_only ** 2)):.2f}m)')
        ax3.grid(True, alpha=0.3)

        # Plot 4: Pressure+temp model errors
        ax4.plot(test_altitudes, error_pressure_temp, 'go-', markersize=4, label='Pressure + Temp Model')
        ax4.axhline(y=0, color='black', linestyle='-', alpha=0.5)
        ax4.set_xlabel('True Altitude (m AGL)')
        ax4.set_ylabel('Error (m)')
        ax4.set_title(f'Pressure+Temp Model Error (RMS: {np.sqrt(np.mean(error_pressure_temp ** 2)):.2f}m)')
        ax4.grid(True, alpha=0.3)

        plt.tight_layout()
        plt.show()

        # Print accuracy summary
        print("\nMODEL ACCURACY SUMMARY:")
        print("-" * 40)
        print(f"Pressure-only model:")
        print(f"  RMS Error: {np.sqrt(np.mean(error_pressure_only ** 2)):.2f} m")
        print(f"  Max Error: {np.max(np.abs(error_pressure_only)):.2f} m")
        print(f"  Mean Error: {np.mean(error_pressure_only):.2f} m")

        print(f"\nPressure+Temperature model:")
        print(f"  RMS Error: {np.sqrt(np.mean(error_pressure_temp ** 2)):.2f} m")
        print(f"  Max Error: {np.max(np.abs(error_pressure_temp)):.2f} m")
        print(f"  Mean Error: {np.mean(error_pressure_temp):.2f} m")

    def get_altitude(self, pressure_pa, temperature_k=None):
        """
        Calculate altitude from pressure and optionally temperature

        Args:
            pressure_pa: Measured pressure in Pascals
            temperature_k: Measured temperature in Kelvin (optional)

        Returns:
            float: Altitude above ground level in meters
        """
        if temperature_k is not None:
            # Use pressure + temperature model
            pressure_ratio = pressure_pa / self.ground_pressure
            temp_ratio = temperature_k / self.ground_temperature
            features = np.array([[pressure_ratio, temp_ratio]])
            return self.pressure_temp_model.predict(features)[0]
        else:
            # Use pressure-only model
            pressure_ratio = pressure_pa / self.ground_pressure
            return self.pressure_model.predict([[pressure_ratio]])[0]

    def get_altitude_with_static_temp(self, pressure_pa, static_temperature_k):
        """
        Calculate altitude using a fixed temperature value

        Args:
            pressure_pa: Measured pressure in Pascals
            static_temperature_k: Fixed temperature value in Kelvin

        Returns:
            float: Altitude above ground level in meters
        """
        return self.get_altitude(pressure_pa, static_temperature_k)


# Example usage
if __name__ == "__main__":
    # Initialize calibrator for your launch site
    calibrator = BarometricCalibrator(
        ground_altitude_m=200,  # Your launch site altitude
        max_altitude_agl=5000,  # Max altitude you expect
        num_points=200  # Calibration resolution
    )

    # Example measurements
    test_pressure = 95000  # Pa
    test_temperature = 285  # K (about 12°C)

    # Test the functions
    altitude_with_temp = calibrator.get_altitude(test_pressure, test_temperature)
    altitude_pressure_only = calibrator.get_altitude(test_pressure)

    print(f"Test: Altitude with temp = {altitude_with_temp:.1f} m")
    print(f"Test: Altitude pressure-only = {altitude_pressure_only:.1f} m")
    print("\n" + "=" * 50)

    # Print the functions
    calibrator.print_functions()

    # Show accuracy plots
    calibrator.plot_accuracy()