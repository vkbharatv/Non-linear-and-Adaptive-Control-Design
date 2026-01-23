# %%
"""
Delay Control System Simulation
This module defines a DelayControlSystem class that simulates a control system with input delay. It includes methods for stepping through the simulation, resetting the system, checking marginal stability, and calculating gain and phase margins.
Developed for educational and research purposes.
Author: Dr. Bharat Verma
Date: 2026-10-01
Institution: The LNMIIT, Jaipur, India
ORCID: https://orcid.org/0000-0001-7600-7872
"""

# %%
import control as ctrl
import numpy as np
import matplotlib.pyplot as plt
import scipy.signal as sp


# %%
class DelayControlSystem:

    def __init__(self, tf, delay_time, dt=0.01, Total_time=10):
        self.plant = tf
        self.delay_time = delay_time
        self.A, self.B, self.C, self.D = ctrl.ssdata(ctrl.tf2ss(self.plant))
        self.n = self.A.shape[0]
        self.p = self.C.shape[0]
        self.sim_time = Total_time
        self.dt = dt
        # Use dynamic storage so arrays grow as needed.
        self.t = [0]
        self.u = [0]  # Time series of actual inputs (with delay applied)
        self.y = [np.zeros(self.p)]
        self.x = [np.zeros(self.n)]
        self.i = 0
        # Delay buffer for storing past inputs
        self.delay_steps = (
            int(np.ceil(self.delay_time / self.dt)) if delay_time > 0 else 0
        )
        self.u_delay_buffer = (
            np.zeros(self.delay_steps) if self.delay_steps > 0 else np.array([])
        )
        self.buffer_index = 0

    def reset(self):
        self.delay_time = 0
        self.t = [0]
        self.u = [0]
        self.y = [np.zeros(self.p)]
        self.x = [np.zeros(self.n)]
        self.i = 0
        self.buffer_index = 0
        if self.delay_steps > 0:
            self.u_delay_buffer = np.zeros(self.delay_steps)

    def step(self, u=1.0):
        # Get current input from input signal
        u_current_input = (
            np.asarray(u).item()
            if np.asarray(u).size == 1
            else np.asarray(u).flatten()[0]
        )

        # Apply delay using buffer
        if self.delay_steps > 0:
            self.u_delay_buffer = np.roll(self.u_delay_buffer, 1)
            self.u_delay_buffer[0] = u_current_input
            u_current = self.u_delay_buffer[-1]
        else:
            u_current = u_current_input

        x_prev = self.x[-1]
        x_next = x_prev + self.dt * (
            self.A @ x_prev + np.asarray(self.B).flatten() * u_current
        )

        # Output: y = C*x + D*u
        y_next = self.C @ x_next + np.asarray(self.D).flatten()[0] * u_current

        next_t = (self.t[-1] + self.dt) if self.t else self.dt
        self.i = self.i + 1

        self.t.append(next_t)
        self.x.append(x_next)
        self.y.append(y_next)
        self.u.append(u_current)
        return x_next, y_next, next_t

    def marginal_stability(self):
        eigvals = np.linalg.eigvals(self.A)
        for val in eigvals:
            if np.real(val) > 0:
                return False
        return True

    def gain_phase_margin(self, omega=np.logspace(-3, 3, 10000)):
        num = self.plant.num[0][0]
        den = self.plant.den[0][0]
        tau = self.delay_time

        # Define the frequency range for analysis
        s = 1j * omega

        # Calculate the transfer function L(s) = N(s) / D(s) * e^(-tau * s)
        L = (np.polyval(num, s) / np.polyval(den, s)) * np.exp(-tau * s)

        # Calculate magnitude and phase
        magnitude = ctrl.mag2db(np.abs(L))
        phase = np.angle(L, deg=True)
        phase = np.unwrap(np.deg2rad(phase)) * (180 / np.pi)

        # Find gain crossover frequency (where magnitude is 0 dB)
        gain_crossover_idx = np.where(np.isclose(magnitude, 0, atol=0.01))[0]
        if len(gain_crossover_idx) == 0:
            phase_margin = np.inf
            gain_crossover_freq = None
        else:
            gain_crossover_freq = omega[gain_crossover_idx[0]]
            phase_margin = 180 + phase[gain_crossover_idx[0]]

        # Find phase crossover frequency (where phase is -180 degrees)
        phase_crossover_idx = np.where(np.isclose(phase, -180, atol=1))[0]
        if len(phase_crossover_idx) == 0:
            gain_margin = np.inf
            phase_crossover_freq = None
        else:
            phase_crossover_freq = omega[phase_crossover_idx[0]]
            gain_margin = 0 - magnitude[phase_crossover_idx[0]]

        return gain_margin, phase_margin, gain_crossover_freq, phase_crossover_freq


# %% The PID Controller Class
class PIDController:
    def __init__(self, Kp=1.0, Ki=0.0, Kd=0.0, dt=0.01, N=100):
        self.Kp = np.array(Kp)
        self.Ki = np.array(Ki)
        self.Kd = np.array(Kd)
        self.dt = dt
        self.integral = 0.0
        self.prev_error = 0.0
        self.N = N  # Derivative filter coefficient
        self.derivative_filtered = 0.0

    def __call__(self):
        print("The PID controller Gains are")
        print(f"Kp: {self.Kp}, Ki: {self.Ki}, Kd: {self.Kd}")

    def compute(self, setpoint, measurement):
        error = setpoint - measurement
        self.integral += error * self.dt
        derivative = (error - self.prev_error) / self.dt if self.dt > 0 else 0.0
        self.derivative_filtered = (
            (self.N * self.dt * derivative + self.derivative_filtered)
            / (self.N * self.dt + 1)
            if hasattr(self, "derivative_filtered")
            else derivative
        )
        output = (
            (self.Kp * error)
            + (self.Ki * self.integral)
            + (self.Kd * self.derivative_filtered)
        )
        self.prev_error = error
        return output

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.derivative_filtered = 0.0

    def update_gains(self, Kp, Ki, Kd):
        self.Kp = np.array(Kp)
        self.Ki = np.array(Ki)
        self.Kd = np.array(Kd)

    # %% IMC-PID Tuning Method for FOPDT Systems with Talyor Series Approximation
    def IMC_tune(self, lambda_c, K, tau, theta):
        K_c = tau / (K * (lambda_c + theta))
        tau_i = tau
        Kp = K_c
        Ki = K_c / tau_i
        Kd = 0.0
        self.update_gains(Kp, Ki, Kd)


# %%
if __name__ == "__main__":
    s = ctrl.TransferFunction.s
    G = 1 / (s + 1) ** 2
    C = 1 + 0.5 / s

    # Define delay time
    delay_time = 1  # seconds
    dt = 0.01  # time step
    f = 1 / 200
    sim_time = 100
    r = sp.square(2 * np.pi * f * np.arange(0, sim_time, dt))
    # Create DelayControlSystem instance
    delay_system = DelayControlSystem(G, delay_time, dt=dt)
    controller = DelayControlSystem(C, 0, dt=dt)
    delay_system.reset()
    controller.reset()
    u_c = 0
    e = np.array([0])
    Ref = np.array([0])
    for i in range(r.size):
        Ref = np.append(Ref, r[i])
        e = np.append(e, Ref[-1] - delay_system.y[-1])
        x, y, t = delay_system.step(u_c)
        _, u_c, _ = controller.step(e[i])
    ise = np.sum(e**2) * dt
    iae = np.sum(np.abs(e)) * dt

    plt.plot(delay_system.t, delay_system.y, "b-", label="Output y(t)")
    plt.plot(delay_system.t, e, "r--", label="Error e(t)")
    plt.plot(delay_system.t, Ref, "g--", label="Reference r(t)")
    plt.grid(True, which="both")
    plt.minorticks_on()
    plt.grid(True, which="minor", alpha=0.3)
    plt.legend(loc="best")
    plt.show()
    print("Performance Metrics:")
    print(f"ISE: {ise:.4f} and IAE: {iae:.4f}")
    loop_tf = DelayControlSystem(ctrl.series(C, G), delay_time, dt=dt)
    gm, pm, w_gc, w_pc = loop_tf.gain_phase_margin()
    print(f"GM = {gm:.2f}, PM = {pm:.2f}, \nw_gc = {w_gc:.2f}, w_pc = {w_pc:.2f}")
