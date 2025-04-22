"""
Post processing script for analyzing and visualizing data from a CSV file.
"""



import numpy as np
import pandas as pd
from matplotlib import pyplot as plt
from scipy.ndimage import gaussian_filter1d



# CSV Data Loading and Labeling ===================================================================
# Read the CSV file and Assign Column Names -------------------------------------------------------
filename = r"C:\Users\maxwe\OneDrive\Documents\PlatformIO\Projects\Balloon\Data\dynamic_data_test.csv"
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
fig, axs = plt.subplots(3, 1, figsize = (10, 8), sharex = True)
for i, (axis, color) in enumerate(zip(['X', 'Y', 'Z'], ['blue', 'green', 'orange'])):
    raw = df[f"Accel {axis} [m/s^2]"]
    smooth = gaussian_filter1d(raw, sigma = 2)
    axs[i].scatter(df["Time [s]"], raw, alpha = 0.3, s = 10, color = "gray", label = f"Accel {axis} (raw)")
    axs[i].plot(df["Time [s]"], smooth, color = color, label = f"Accel {axis} (smooth)")
    axs[i].set_ylabel(f"A{axis} [m/s^2]")
    axs[i].legend()
    axs[i].grid()
axs[-1].set_xlabel("Time [s]")
fig.suptitle("Time vs Acceleration")
plt.savefig(r"C:\Users\maxwe\OneDrive\Documents\PlatformIO\Projects\Balloon\Images\Time_vs_Acceleration.png")


# Time vs Jerk ----------------------------------------------------------------------------
fig, axs = plt.subplots(3, 1, figsize = (10, 8), sharex = True)
for i, (axis, color) in enumerate(zip(['X', 'Y', 'Z'], ['blue', 'green', 'orange'])):
    raw = df[f"Jerk {axis} [m/s^3]"]
    smooth = gaussian_filter1d(raw, sigma = 2)
    axs[i].scatter(df["Time [s]"], raw, alpha = 0.3, s = 10, color = "gray", label = f"Jerk {axis} (raw)")
    axs[i].plot(df["Time [s]"], smooth, color = color, label = f"Jerk {axis} (smooth)")
    axs[i].set_ylabel(f"J{axis} [m/s^3]")
    axs[i].legend()
    axs[i].grid()
axs[-1].set_xlabel("Time [s]")
fig.suptitle("Time vs Jerk")
plt.savefig(r"C:\Users\maxwe\OneDrive\Documents\PlatformIO\Projects\Balloon\Images\Time_vs_Jerk.png")


# Time vs Altitude ----------------------------------------------------------------------------
plt.figure()
raw = df["Altitude [m]"]
smooth = gaussian_filter1d(raw, sigma = 2)
plt.scatter(df["Time [s]"], raw, alpha = 0.3, s = 10, color = "gray", label = "Raw")
plt.plot(df["Time [s]"], smooth, label = "Smoothed")
plt.xlabel("Time [s]")
plt.ylabel("Altitude [m]")
plt.title("Time vs Altitude")
plt.legend()
plt.grid()
plt.savefig(r"C:\Users\maxwe\OneDrive\Documents\PlatformIO\Projects\Balloon\Images\Time_vs_Altitude.png")


# Altitude vs Measured & Ideal Temperature ----------------------------------------------------------------------------
plt.figure()
raw = df["Temperature [K]"]
smooth = gaussian_filter1d(raw, sigma = 2)
plt.scatter(raw, df["Altitude [m]"], alpha = 0.3, s = 10, color = "gray", label = "Measured (raw)")
plt.plot(smooth, df["Altitude [m]"], label = "Measured (smoothed)")
plt.plot(ideal_T, df["Altitude [m]"], label = "Ideal", linestyle = "--")
plt.xlabel("Temperature [K]")
plt.ylabel("Altitude [m]")
plt.title("Altitude vs Temperature (Measured vs Ideal)")
plt.legend()
plt.grid()
plt.savefig(r"C:\Users\maxwe\OneDrive\Documents\PlatformIO\Projects\Balloon\Images\Altitude_vs_Temperature.png")


# Altitude vs Measured & Ideal Pressure ----------------------------------------------------------------------------
plt.figure()
raw = df["Pressure [Pa]"]
smooth = gaussian_filter1d(raw, sigma = 2)
plt.scatter(raw, df["Altitude [m]"], alpha = 0.3, s = 10, color = "gray", label = "Measured (raw)")
plt.plot(smooth, df["Altitude [m]"], label = "Measured (smoothed)")
plt.plot(ideal_P, df["Altitude [m]"], label = "Ideal", linestyle = "--")
plt.xlabel("Pressure [Pa]")
plt.ylabel("Altitude [m]")
plt.title("Altitude vs Pressure (Measured vs Ideal)")
plt.legend()
plt.grid()
plt.savefig(r"C:\Users\maxwe\OneDrive\Documents\PlatformIO\Projects\Balloon\Images\Altitude_vs_Pressure.png")


# Altitude vs Acceleration X, Y, and Z ----------------------------------------------------------------------------
fig, axs = plt.subplots(3, 1, figsize = (10, 8), sharex = True)
for i, (axis, color) in enumerate(zip(['X', 'Y', 'Z'], ['blue', 'green', 'orange'])):
    raw = df[f"Accel {axis} [m/s^2]"]
    smooth = gaussian_filter1d(raw, sigma = 2)
    axs[i].scatter(raw, df["Altitude [m]"], alpha = 0.3, s = 10, color = color, label = f"Accel {axis} (raw)")
    axs[i].set_ylabel("Altitude [m]")
    axs[i].legend()
    axs[i].grid()
axs[-1].set_xlabel("Acceleration [m/s^2]")
fig.suptitle("Altitude vs Acceleration")
plt.savefig(r"C:\Users\maxwe\OneDrive\Documents\PlatformIO\Projects\Balloon\Images\Altitude_vs_Acceleration.png")


# Altitude vs Jerk X, Y, and Z ----------------------------------------------------------------------------
fig, axs = plt.subplots(3, 1, figsize = (10, 8), sharex = True)
for i, (axis, color) in enumerate(zip(['X', 'Y', 'Z'], ['blue', 'green', 'orange'])):
    raw = df[f"Jerk {axis} [m/s^3]"]
    smooth = gaussian_filter1d(raw, sigma = 2)
    axs[i].scatter(raw, df["Altitude [m]"], alpha = 0.3, s = 10, color = color, label = f"Jerk {axis} (raw)")
    axs[i].set_ylabel("Altitude [m]")
    axs[i].legend()
    axs[i].grid()
axs[-1].set_xlabel("Jerk [m/s^3]")
fig.suptitle("Altitude vs Jerk")
plt.savefig(r"C:\Users\maxwe\OneDrive\Documents\PlatformIO\Projects\Balloon\Images\Altitude_vs_Jerk.png")



# Final Rendering and Stats =======================================================================
plt.show()
print(stats)
