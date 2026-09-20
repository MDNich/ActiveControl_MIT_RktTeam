import pandas as pd
import numpy as np
import os
import matplotlib.pyplot as plt 

def generate_PID_gains(KP, KI, KD, Cn_alphas, densities, velocities, S, y, n_control_surfaces):
    '''
    Inputs: KP, KI, KD (PID gains) from Simplified_plant_dynamics.py
            min_V, max_V (velocity range from predicted flight profile)
            min_alt, max_alt (altitude range for predicted flight profile)
            n_steps (number of steps in the gain schedule)
    Outputs: gain_schedule.csv - PID gains lookup table for a given plant and expected flight profile.
    '''
    gain_schedule = []
    for i in range(len(velocities)):
        V = velocities[i]
        rho = densities[i]
        # Calculate aerodynamic gain for current conditions
        Ka = get_aerodynamic_gain(Cn_alphas[i], rho, V, S, y, n_control_surfaces)
        # Avoid division by zero for Ka
        if Ka == 0:
            kp_out, ki_out, kd_out = 0, 0, 0
        else:
            kp_out = KP / Ka
            ki_out = KI / Ka
            kd_out = KD / Ka
        gain_schedule.append([
            V, 
            rho, 
            round(kp_out, 5), 
            round(ki_out, 5), 
            round(kd_out, 5)
        ])
    gain_schedule = np.array(gain_schedule)

    # Save to CSV
    df = pd.DataFrame(gain_schedule, columns=['velocity', 'density', 'KP', 'KI', 'KD'])
    current_dir = os.path.dirname(os.path.abspath(__file__))
    csv_path = os.path.join(current_dir, 'gain_schedule.csv')
    df.to_csv(csv_path, index=False)
    print("Gain schedule saved to gain_schedule.csv")

    return None

def get_aerodynamic_gain(Cn_alpha, rho, V, S, y, n_control_surfaces):
    '''
    Helper function to calculate aerodynamic gain based on air density, velocity, and control surface area.
    Inputs: Cn_alpha (normal force to AoA slope), rho (air density), V (velocity), S (control surface area), y (distance from center of mass to control surface), n_control_surfaces (number of control surfaces)
    Outputs: Ka (aerodynamic gain)
    '''
    q = 0.5 * rho * V**2  # Dynamic pressure [N/m^2]
    Ka = Cn_alpha * q * S * y * n_control_surfaces # Aerodynamic gain [N*m]
    return Ka

# USED ONLY FOR DIRECTLY CALCULATED AERODYNAMICS

def get_density(altitude):
    '''
    Helper function to get air density based on altitude.
    Inputs: altitude (in meters)
    Outputs: rho (air density in kg/m^3)
    '''
    # International Standard Atmosphere (ISA) model up to 40,000 m
    # Valid for altitudes up to 47,000 m (stratosphere)
    # Source: https://www.grc.nasa.gov/www/k-12/airplane/atmosmet.html

    # Define layer base values
    layers = [
        (0,      288.15, 101325,   -0.0065),   # Troposphere (0-11km)
        (11000,  216.65, 22632.1,   0.0),      # Lower Stratosphere (11-20km)
        (20000,  216.65, 5474.89,   0.001),    # Mid Stratosphere (20-32km)
        (32000,  228.65, 868.019,   0.0028),   # Upper Stratosphere (32-47km)
        (47000,  270.65, 110.906,   0.0),      # Above 47km (not needed here)
    ]

    R = 287.05  # Specific gas constant for dry air [J/(kg·K)]
    g0 = 9.80665  # Standard gravity [m/s^2]

    h = float(altitude)
    if h < 0:
        h = 0
    if h > 40000:
        h = 40000

    # Find the correct layer
    for i in range(len(layers)-1):
        h_base, T_base, P_base, lapse = layers[i]
        h_next = layers[i+1][0]
        if h >= h_base and h < h_next:
            if lapse == 0.0:
                # Isothermal layer
                T = T_base
                P = P_base * np.exp(-g0 * (h - h_base) / (R * T))
            else:
                # Gradient layer
                T = T_base + lapse * (h - h_base)
                P = P_base * (T / T_base) ** (-g0 / (R * lapse))
            rho = P / (R * T)
            return rho

    # If above last layer, use last layer's values
    h_base, T_base, P_base, lapse = layers[-2]
    T = T_base + lapse * (h - h_base)
    P = P_base * (T / T_base) ** (-g0 / (R * lapse))
    rho = P / (R * T)
    return rho

# PLOTTER FUNCTION FOR VISUALIZATION
def plot_gain_schedule():
    '''
    Function to visualize the gain schedule.
    '''


# INPUTS:

KP, KI, KD = 10, 1, 5 # GAINS FETCHED FROM Simplified_plant_dynamics.py

Cn_alphas = [5.0, 4.5, 4.0, 3.5, 3.0] # Normal force to AoA slope [N*m/rad] FROM CFD, THEORY, OR TESTING
densities = [1.225, 0.9093, 0.7364, 0.5887, 0.4664]  # Air densities at different altitudes [kg/m^3]
velocities = [0, 20, 40, 60, 80]  # Gain schedule velocities [m/s] should correspond to the expected flight profile and CFD data
S = 0.001  # Control surface area [m^2]
y = 0.2  # Distance from the center of mass to the control surface [m]
n_control_surfaces = 2  # Number of control surfaces

# Generate the gain schedule
generate_PID_gains(KP, KI, KD, Cn_alphas, densities, velocities, S, y, n_control_surfaces)