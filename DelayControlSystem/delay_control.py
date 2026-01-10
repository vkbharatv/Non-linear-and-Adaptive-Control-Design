# %%
import control as ctrl
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import lti, step, square
import scipy.signal as sp

# %%
class DelayControlSystem:

    def __init__(self, tf, delay_time, dt=0.01, Total_time=10):
        self.tf = tf
        self.delay_time = delay_time
        self.A,self.B,self.C,self.D = ctrl.ssdata(ctrl.tf2ss(self.tf))
        self.n = self.A.shape[0]
        self.p = self.C.shape[0]
        self.sim_time = Total_time
        self.dt = dt
        # Use dynamic storage so arrays grow as needed.
        self.t = [0.0]
        self.u = [0.0]  # Time series of actual inputs (with delay applied)
        self.y = [np.zeros(self.p)]
        self.x = [np.zeros(self.n)]
        self.i = 0
        # Delay buffer for storing past inputs
        self.delay_steps = int(np.ceil(self.delay_time / self.dt)) if delay_time > 0 else 0
        self.u_delay_buffer = np.zeros(self.delay_steps) if self.delay_steps > 0 else np.array([])
        self.buffer_index = 0

    def reset(self):
        self.tf = None
        self.delay_time = 0
        self.t = [0.0]
        self.u = [0.0]
        self.y = [np.zeros(self.p)]
        self.x = [np.zeros(self.n)]
        self.i = 0
        self.buffer_index = 0
        if self.delay_steps > 0:
            self.u_delay_buffer = np.zeros(self.delay_steps)

    def step(self,u=1.0):
        self.i = self.i + 1
        dt = self.dt 
        next_t = self.t[-1] + dt
        
        # Get current input from input signal
        u_current_input = np.asarray(u).item() if np.asarray(u).size == 1 else np.asarray(u).flatten()[0]
        
        # Apply delay using buffer
        if self.delay_steps > 0:
            self.u_delay_buffer[self.buffer_index] = u_current_input
            u_current = self.u_delay_buffer[(self.buffer_index + 1) % self.delay_steps]
            self.buffer_index = (self.buffer_index + 1) % self.delay_steps
        else:
            u_current = u_current_input

        x_prev = self.x[-1]
        x_next = x_prev + dt * (self.A @ x_prev + np.asarray(self.B).flatten() * u_current)

        # Output: y = C*x + D*u
        y_next = self.C @ x_next + np.asarray(self.D).flatten()[0] * u_current

        self.t.append(next_t)
        self.x.append(x_next)
        self.y.append(y_next)
        self.u.append(u_current)
        return x_next, y_next, next_t


# %%
if __name__ == "__main__":
    s = ctrl.TransferFunction.s
    G = 1/(s+1)**2
    C = 1+0.1/s

    # Define delay time
    delay_time = 1 # seconds
    dt = 0.001  # time step
    f = 1/200
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
        _,u_c,_ = controller.step(e[i])
    ise = np.sum(e**2)*dt
    print(f"ISE: {ise}")
    plt.plot(delay_system.t, delay_system.y)
    plt.plot(delay_system.t, e,'r--')
    plt.plot(delay_system.t, Ref,'g--')
    plt.grid(True, which='both')
    plt.minorticks_on()
    plt.grid(True, which='minor', alpha=0.3)
    plt.show()