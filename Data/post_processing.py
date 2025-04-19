
"""
Post processing script for analyzing and visualizing data from a CSV file.
"""



import numpy as np
import pandas as pd
from matplotlib import pyplot as plt



# CSV Data Loading and Labeling ===================================================================
# Read the CSV file and Assign Column Names -------------------------------------------------------
filename = 'log.csv'
df = pd.read_csv(filename)
df.columns = [
    "Time [s]", "Altitude [m]", "Pressure [Pa]", "Temperature [K]",
    "Accel X [m/s^2]", "Accel Y [m/s^2]", "Accel Z [m/s^2]",
    "Jerk X [m/s^3]", "Jerk Y [m/s^3]", "Jerk Z [m/s^3]"
]


# Compute and Print Statistics --------------------------------------------------------------------
stats = df.describe()
stats.loc['range'] = stats.loc['max'] - stats.loc['min']
stats.loc['var'] = df.var()
pd.set_option("display.float_format", "{:.3f}".format)



# Standard Atmosphere Calculation =================================================================
g = 9.80665  # Gravity [m/s^2]
R = 287.05   # Specific Gas Constant [J/kg·K]
T0 = 288.16  # Sea Level Temperature [K]
P0 = 101325  # Sea Level Pressure [Pa]

heights = [0, 11000, 20000, 32000]   # Section Heights [m]
lapse_rates = [-0.0065, 0.0, 0.001]  # Standard Lapse Rates [K/m]

altitudes = df["Altitude [m]"].values
ideal_T = []
ideal_P = []

T_base = T0
P_base = P0
layer = 0
for h in altitudes:
    if ((layer < len(heights) - 2) and (h > heights[layer + 1])):
        layer += 1
        T_base = T0
        P_base = P0

    h0 = heights[layer]
    a = lapse_rates[layer]

    if (a != 0):
        T = T_base + a * (h - h0)
        P = P_base * (T / T_base) ** (-g / (a * R))
    else:
        T = T_base
        P = P_base * np.exp(-g * (h - h0) / (R * T))

    ideal_T.append(T)
    ideal_P.append(P)



# Data Plotting ===================================================================================
# Time vs Acceleration ----------------------------------------------------------------------------
plt.figure()
plt.plot(df["Time [s]"], df["Accel X [m/s^2]"], label = "Accel X")
plt.plot(df["Time [s]"], df["Accel Y [m/s^2]"], label = "Accel Y")
plt.plot(df["Time [s]"], df["Accel Z [m/s^2]"], label = "Accel Z")
plt.xlabel("Time [s]")
plt.ylabel("Acceleration [m/s^2]")
plt.title("Time vs Acceleration")
plt.legend()
plt.grid()
plt.tight_layout()


# Time vs Jerk ----------------------------------------------------------------------------
plt.figure()
plt.plot(df["Time [s]"], df["Jerk X [m/s^3]"], label = "Jerk X")
plt.plot(df["Time [s]"], df["Jerk Y [m/s^3]"], label = "Jerk Y")
plt.plot(df["Time [s]"], df["Jerk Z [m/s^3]"], label = "Jerk Z")
plt.xlabel("Time [s]")
plt.ylabel("Jerk [m/s^3]")
plt.title("Time vs Jerk")
plt.legend()
plt.grid()
plt.tight_layout()


# Time vs Altitude ----------------------------------------------------------------------------
plt.figure()
plt.plot(df["Time [s]"], df["Altitude [m]"])
plt.xlabel("Time [s]")
plt.ylabel("Altitude [m]")
plt.title("Time vs Altitude")
plt.grid()
plt.tight_layout()


# Altitude vs Measured & Ideal Temperature ----------------------------------------------------------------------------
plt.figure()
plt.plot(df["Temperature [K]"], df["Altitude [m]"], label = "Measured")
plt.plot(ideal_T, df["Altitude [m]"], label = "Ideal", linestyle = '--')
plt.xlabel("Temperature [K]")
plt.ylabel("Altitude [m]")
plt.title("Altitude vs Temperature (Measured vs Ideal)")
plt.legend()
plt.grid()
plt.tight_layout()


# Altitude vs Measured & Ideal Pressure ----------------------------------------------------------------------------
plt.figure()
plt.plot(df["Pressure [Pa]"], df["Altitude [m]"], label = "Measured")
plt.plot(ideal_P, df["Altitude [m]"], label = "Ideal", linestyle = '--')
plt.xlabel("Pressure [Pa]")
plt.ylabel("Altitude [m]")
plt.title("Altitude vs Pressure (Measured vs Ideal)")
plt.legend()
plt.grid()
plt.tight_layout()


# Altitude vs Acceleration X, Y, and Z ----------------------------------------------------------------------------
plt.figure()
plt.plot(df["Accel X [m/s^2]"], df["Altitude [m]"], label = "Accel X")
plt.plot(df["Accel Y [m/s^2]"], df["Altitude [m]"], label = "Accel Y")
plt.plot(df["Accel Z [m/s^2]"], df["Altitude [m]"], label = "Accel Z")
plt.xlabel("Acceleration [m/s^2]")
plt.ylabel("Altitude [m]")
plt.title("Altitude vs Accel X, Y, Z")
plt.legend()
plt.grid()
plt.tight_layout()


# Altitude vs Jerk X, Y, and Z ----------------------------------------------------------------------------
plt.figure()
plt.plot(df["Jerk X [m/s^2]"], df["Altitude [m]"], label = "Jerk X")
plt.plot(df["Jerk Y [m/s^2]"], df["Altitude [m]"], label = "Jerk Y")
plt.plot(df["Jerk Z [m/s^2]"], df["Altitude [m]"], label = "Jerk Z")
plt.xlabel("Jerk [m/s^3]")
plt.ylabel("Altitude [m]")
plt.title("Altitude vs Jerk X, Y, Z")
plt.legend()
plt.grid()
plt.tight_layout()



# Final Rendering and Stats =======================================================================
plt.show()
print(stats)
