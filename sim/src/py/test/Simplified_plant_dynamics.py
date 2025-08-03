from control import tf, feedback, step_response, bode_plot
from control.matlab import rlocus
import numpy as np
import matplotlib.pyplot as plt
from PID_coefficients import find_PID

# TODO: Rewrite this as a CLASS called Rocket_Controller

# Define the rocket's parameters
Jxx = 0.0006  # Moment of inertia about the x-axis (get value from OpenRocket) [kg*m^2]
C = 2.0 # Damping coefficient for subsonic case  [N*m*s/rad] (middle of the range value :P )

# Define the "Plant" (rocket system) transfer function
G_plant = tf([1], [Jxx, C, 0])

def get_plant_info(G_plant, k=1):
    '''
    
    '''
    # TODO: Print some other useful characteristics like w_n

    # Test dynamics of pure gain closed-loop system
    k = 1
    G_plant_cl = k * G_plant / (1 + k * G_plant)  # Closed-loop transfer function

    t_out_G_plant, phi_out_G_plant = step_response(G_plant, T=np.linspace(0, 25, 1000))
    t_out_G_cl, phi_out_G_cl = step_response(G_plant_cl, T=np.linspace(0, 25, 1000))

    # Plot the step response of the plant
    plt.figure(figsize=(10, 5))
    plt.title('Step Response of the Plant G_plant')
    plt.xlabel('Time (s)')
    plt.ylabel('Angle (rad)')
    plt.grid()
    plt.plot(t_out_G_plant, phi_out_G_plant, label='Step Response of Plant G_plant')
    plt.plot(t_out_G_cl, phi_out_G_cl, label='Step Response of Closed Loop G_cl', linestyle='--')

    # Plot the Root Locus of the plant
    plt.figure(figsize=(10, 5))
    rlocus(G_plant, plot=True)

    # Plot the Bode for the plant
    bode_plot(G_plant, dB = False)
    plt.show()

def set_dominant_poles(t_s, Mp):
    '''
    Sets dominant pole locations using settling time (t_s) and max peak (Mp) requirements
    Assumes second order system behavior, generally a good approximation
    Returns: s0 [Complex]
    Coordinates of positive desired pole in the pole pair (they're complex conjugates)
    
    Areas to improve: Look into setting the damping ratio directly to 0.707 (aerospace damping number) to avoid overshoot althogether
    '''
    s_real = -4.0/t_s # Real part of pole pair
    zeta = np.sqrt((np.log(Mp)**2) / (np.pi**2 + np.log(Mp)**2))
    # zeta = 0.707 # Aerospace damping number boiiii
    w_n = -1*s_real/zeta
    s_imag = w_n*np.sqrt(1 + zeta**2)
    s0 = s_real + s_imag*1j
    return s0

def set_PID_coeffs(G_plant, gamma, s0, show):
    '''
    Computes P,I, and D gains to get the root locus to pass through desired CL poles

    Areas to improve: Right now we are setting the ratio of the two zeros as a design choice.
    Could have more freedom/better performance by not doing that
    '''
    Kd, Kp, Ki, z1_opt, K, Gc_PID = find_PID(G_plant, gamma, s0)

    if show:
        print("Results for G1:")
        print(f"Kd: {Kd}, Kp: {Kp}, Ki: {Ki}, z1_opt: {z1_opt}, K: {K}")
        print(f"Gc_PID: {Gc_PID}")

    return Gc_PID


def return_PID_coeffs(G_plant, gamma, s0, show):
    '''
    Computes P,I, and D gains to get the root locus to pass through desired CL poles

    Areas to improve: Right now we are setting the ratio of the two zeros as a design choice.
    Could have more freedom/better performance by not doing that
    '''
    Kd, Kp, Ki, z1_opt, K, Gc_PID = find_PID(G_plant, gamma, s0)

    if show:
        print("Results for G1:")
        print(f"Kd: {Kd}, Kp: {Kp}, Ki: {Ki}, z1_opt: {z1_opt}, K: {K}")
        print(f"Gc_PID: {Gc_PID}")

    return Kp, Ki, Kd

def get_CL_control_info(G, Gc, s0):
    '''
    Visualize characteristics of controlled system and controller
    '''
    # Plot root locus and desired poles for loop function
    L = G * Gc
    plt.figure()
    rlocus(G * Gc, gains=np.linspace(0, 5, 500))
    plt.plot(np.real(s0), np.imag(s0), 'ro', label='s0')
    plt.plot(np.real(s0), -1*np.imag(s0), 'ro', label='Conjugate of s0')
    plt.title("Root Locus for L")
    plt.xlabel("Real Axis")
    plt.ylabel("Imaginary Axis")
    plt.xlim(-20, 20)
    plt.ylim(-20, 20)
    plt.grid()
    plt.legend()

    # Plot step response for both systems
    T = np.linspace(0, 5, 500)  # Time vector for step response

    # Step response for G1
    T1, y1 = step_response(feedback(L), T)
    plt.figure()
    plt.plot(T1, y1, label="L")
    plt.show()

if __name__ == "__main__":
    ts_des = 0.5 # Desired settling time of 0.5 seconds
    Mp_des = 0.2 # Desired max peak of less than 20%
    s0 = set_dominant_poles(ts_des, Mp_des)
    print(f'Desired Poles: {s0} and its conjugate')

    gamma = 2 # Assume ratio between two compensator zeros.  Between 1-3 said to be a heuristic range
    Gc_PID = set_PID_coeffs(G_plant, gamma, s0, show=True)

    get_CL_control_info(G_plant, Gc_PID, s0)

# TODO: Figure out how the gain schedule is implemented
# Calculate some base parameters that are scaled by Ka as a function of speed and altitude
Ka = 1.0  # Gain scaling factor (can be adjusted based on speed and altitude)

# Make gain lookup table that can return the gain for a given speed and altitude
# Avoid using interpolation for simplicity