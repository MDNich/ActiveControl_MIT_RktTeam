import control as ctl
import numpy as np
from scipy.optimize import minimize

def find_PID(G, gamma, s0):
    
    def phase_cost(x):
        z1 = x[0]  
        num = [1, (1 + gamma) * z1, gamma * z1**2]
        den = [1, 0]
        Gc_unscaled = ctl.tf(num, den)
        L = G * Gc_unscaled
        L_val = ctl.evalfr(L, s0)
        phase_err = np.angle(L_val) - np.pi
        # phase_err %= 2 * np.pi  # Ensure phase error is in the range [0, 2*pi]
        return phase_err


    z0 = np.array([s0.real / 2])  ### Initial Guess
    
    res = minimize(phase_cost, x0=z0, method='Nelder-Mead', tol=1e-3,
                   options={'maxiter': 1000}) #### Strategy from controls.py in 06 github
    z1_opt = res.x[0]


    num = [1, (1 + gamma) * z1_opt, gamma * z1_opt**2]
    den = [1, 0]
    Gc_unscaled = ctl.tf(num, den)
    L_unscaled = G * Gc_unscaled
    L_val = ctl.evalfr(L_unscaled, s0)
    K = 1.0 / np.abs(L_val)
    

    Gc_PID = K * Gc_unscaled

    Kd = K
    Kp = K * (1 + gamma) * z1_opt
    Ki = K * gamma * z1_opt**2


    return Kd, Kp, Ki, z1_opt, K, Gc_PID

if __name__ == "__main__":
    # Define two example transfer functions G1 and G2

    ki = 0.999
    wn = 4.873
    zeta = 0.099
    G = ctl.tf([ki * wn ** 2], [1, 2 * zeta * wn, wn ** 2])

    # Define gamma and s0 for both cases
    gamma_1 = 2
    s0_1 = -6 + 8j

    gamma_2 = 2
    s0_2 = -8 + 9j

    # Call find_PID for G1
    Kd1, Kp1, Ki1, z1_opt1, K1, Gc_PID1 = find_PID(G, gamma_1, s0_1)
    print("Results for G1:")
    print(f"Kd: {Kd1}, Kp: {Kp1}, Ki: {Ki1}, z1_opt: {z1_opt1}, K: {K1}")
    print(f"Gc_PID: {Gc_PID1}")

    # Call find_PID for G2
    Kd2, Kp2, Ki2, z1_opt2, K2, Gc_PID2 = find_PID(G, gamma_2, s0_2)
    print("\nResults for G2:")
    print(f"Kd: {Kd2}, Kp: {Kp2}, Ki: {Ki2}, z1_opt: {z1_opt2}, K: {K2}")
    print(f"Gc_PID: {Gc_PID2}")

    import matplotlib.pyplot as plt

    # Plot root locus for G1
    plt.figure()
    ctl.rlocus(G * Gc_PID1, gains=np.linspace(0, 5, 500))
    plt.plot(s0_1.real, s0_1.imag, 'ro', label='s0')
    plt.plot(s0_1.real, -s0_1.imag, 'bo', label='Conjugate of s0')
    plt.title("Root Locus for G1")
    plt.xlabel("Real Axis")
    plt.ylabel("Imaginary Axis")
    plt.xlim(-20, 20)
    plt.ylim(-20, 20)
    plt.grid()
    plt.legend()

    # Plot root locus for G2
    plt.figure()
    ctl.rlocus(G * Gc_PID2, gains=np.linspace(0, 5, 500))
    plt.plot(s0_2.real, s0_2.imag, 'ro', label='s0')
    plt.plot(s0_2.real, -s0_2.imag, 'bo', label='Conjugate of s0')
    plt.title("Root Locus for G2")
    plt.xlabel("Real Axis")
    plt.ylabel("Imaginary Axis")
    plt.xlim(-20, 20)
    plt.ylim(-20, 20)
    plt.grid()
    plt.legend()

    # Plot step response for both systems
    T = np.linspace(0, 5, 500)  # Time vector for step response

    # Step response for G1
    T1, y1 = ctl.step_response(ctl.feedback(G * Gc_PID1), T)
    plt.figure()
    plt.plot(T1, y1, label="G1 (Less Aggressive)")

    # Step response for G2
    T2, y2 = ctl.step_response(ctl.feedback(G * Gc_PID2), T)
    plt.plot(T2, y2, label="G2 (More Aggressive)")

    plt.title("Step Response Comparison")
    plt.xlabel("Time (s)")
    plt.ylabel("Response")
    plt.grid()
    plt.legend()

    plt.show()