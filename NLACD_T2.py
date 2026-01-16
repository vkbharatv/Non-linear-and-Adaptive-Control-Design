# %%

import control as ctrl
import numpy as np
import matplotlib.pyplot as plt
from DelayControlSystem.delay_control import *
plt.rcParams["mathtext.fontset"] = "cm"
plt.rcParams["font.family"] = "serif"
plt.rcParams["font.family"] = "Times New Roman"
plt.style.use("seaborn-v0_8")
# %%
"""
1. Delay simulation and effect of approximation
2. Simulate with the Step-by-step method
3. PID control Implementation with delay
"""
s=ctrl.TransferFunction.s
G = 1/(s+1)**2
theta = 2
D1 = 1-theta*s  #Taylor approx
D2 = (1-0.5*theta*s)/(1+0.5*theta*s) # Pade approx

G1 =G*D1 
G2 =G*D2
T = 10
dt = 0.01
t = np.linspace(0,T,int(T/dt)) 
t1, y1 = ctrl.step_response(G1, t)
t2, y2 = ctrl.step_response(G2, t)

# %%
G3 = DelayControlSystem(G, delay_time=theta, dt=dt)
G3.reset()
y_t = []
t_n = []
for t_i in t:
    x, y, _ = G3.step(u=1.0)
    y_t.append(y[0])
    t_n.append(t_i)

plt.plot(t1, y1, label="Taylor Approximation")
plt.plot(t2, y2, label="Pade Approximation")
plt.plot(t_n, y_t, label='Actual')

plt.legend()
plt.show()
"""
For smaller delay: Taylor Approximation is simpler
For larger delay: Pade Approximation is always better and also reccommended
"""

"""
Analysis: Effect of delay on system performance
1. plot the error between the actual delayed system and the approximated system
e1 = |y_actual - y_taylor|
e2 = |y_actual - y_pade|
"""
# %%
print(len(t_n), len(y_t))
print(len(t1), len(y1))
print(len(t2), len(y2))
e1 = y_t - y1
e2 = y_t - y2
plt.plot(t_n, e1, label="Error $\\theta$: Taylor Approximation") 
plt.plot(t_n, e2, label="Error $\\theta$: Pade Approximation")
plt.legend()
plt.show()
# %%

G_delay = DelayControlSystem(G, delay_time=theta, dt=dt)
s = ctrl.TransferFunction.s  # Ensure s is defined as a transfer function variable
C = 0.2 + 0.2 / s
C = DelayControlSystem(C, delay_time=0, dt=dt)
G_delay.reset()
C.reset()
y_T = [0]
T = 100
t_sim = np.linspace(0, T, int(T / dt))
r = np.ones_like(t_sim)

for i, t_i in enumerate(t_sim):
    e = r[i] - y_T[-1]
    _, u_c, _ = C.step(e)
    _, y, _ = G_delay.step(u_c)
    y_T.append(y[0])

plt.plot(t_sim, y_T[1:], label="Output y(t) with Delay")
plt.plot(t_sim, r, "r--", label="Reference r(t)")
plt.grid(True, which="minor", alpha=0.3)
plt.grid(True, which="major", alpha=0.7)
plt.xlabel("Time (s)")
plt.minorticks_on()
plt.ylabel("Output")
plt.title("System Response with Delay and PID Control")
plt.show()

# %%
