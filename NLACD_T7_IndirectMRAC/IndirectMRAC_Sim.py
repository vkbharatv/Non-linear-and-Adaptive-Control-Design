# %%
import controlsim as csm
import control as ctrl
import numpy as np
import matplotlib.pyplot as plt

plt.rcParams["mathtext.fontset"] = "cm"
plt.rcParams["font.family"] = "serif"
plt.rcParams["font.size"] = 8
# %%
A = -2
B = 1
C = 1
D = 0
sys = ctrl.ss(A, B, C, D)
G = ctrl.ss2tf(sys)
dt = 0.01
plant = csm.ProcessDefinition(G, 0.0, dt=dt)
# %%
Am = -2
Bm = 2
Cm = 1
Dm = 0
sysm = ctrl.ss(Am, Bm, Cm, Dm)
plant_ref = csm.ProcessDefinition(sysm, 0.0, dt=dt)

print(f"True Plant parmeters - A: {A}, B: {B}")

# %% Ideal Parameteres
K_x = (Am - A) / B
K_r = (Bm) / B
print(f"Ideal K_x: {K_x:.2f}\nIdeal K_r: {K_r:.2f}")
A_cl = A + B * K_x
B_cl = B * K_r
sys_cl = ctrl.ss(A_cl, B_cl, C, D)
G_cl = ctrl.ss2tf(sys_cl)

# %% Indirect MRAC Simulation
sim_time = 100
t = np.arange(0, sim_time, dt)
R = np.sign(np.sin(2 * np.pi * 0.05 * t))
G_f = ctrl.tf([1], [0.1, 1])

x = []
y = []
Ahat_h = []
Bhat_h = []
e_h = []
u_h = []
Ahat = 0.0
Bhat = 1  # Initialize to non-zero value to prevent division by zero
b_min = 0.5
K_x = 0.0
K_r = 0.0
x = 0
Gamma = 0.05  # Adaptation gain
plant.reset()
plant_ref.reset()
spf = csm.ProcessDefinition(G_f, 0.0, dt=dt)
spf.reset()
A_n = 0.1 # Noice Magnitude

for r in R:
    _, ref, t = spf.step(r)
    u = K_x * x + K_r * ref +np.random.random(1)*A_n
    x, y, t = plant.step(u)
    x_m, y_m, t_m = plant_ref.step(ref)
    e = y - y_m

    Ahat += Gamma * e * y  # Update Ahat
    Bhat_dot = Gamma * e * u  # Update Bhat with projection operator

    if abs(Bhat) > b_min:
        Bhat += Bhat_dot
    elif abs(Bhat) == b_min and Bhat_dot * np.sign(Bhat) >= 0:
        Bhat += Bhat_dot
    elif abs(Bhat) <= b_min and Bhat_dot * np.sign(Bhat) <= 0:
        pass  # Stop update of Bhat
    else:
        Bhat += Bhat_dot

    K_x = (Am - Ahat) / Bhat
    K_r = Bm / Bhat
    # save the results
    e_h.append(e)
    Ahat_h = np.append(Ahat_h, Ahat)
    Bhat_h = np.append(Bhat_h, Bhat)
    u_h = np.append(u_h, u)

# %%
t = plant.t[0 : len(R)]
plt.figure(figsize=(10, 10))
plt.subplot(4, 1, 1)
plt.plot(spf.t, spf.y, label="Reference")
plt.plot(plant.t, plant.y, label="Plant Output")
plt.plot(plant_ref.t, plant_ref.y, label="Reference Model Output", linestyle="--")
plt.title("Indirect MRAC Simulation")
plt.ylabel("Output")
plt.legend()
plt.subplot(4, 1, 2)

plt.axhline(A, color="r", linestyle="--", label=r"True $A$")
plt.axhline(B, color="g", linestyle="--", label=r"True $B$")
plt.plot(t, Ahat_h, label=r"$\hat{A}$")
plt.plot(t, Bhat_h, label=r"$\hat{B}$")
plt.plot(t,(A-Ahat_h), label=r"$\tilde{A}$", linestyle=":")
plt.plot(t,(B-Bhat_h), label=r"$\tilde{B}$",linestyle=":")
plt.ylabel("Values")
plt.legend()
plt.subplot(4, 1, 3)
plt.plot(t, e_h, label=r"Tracking Error")
plt.ylabel("Error")
plt.legend()
plt.subplot(4, 1, 4)
plt.plot(t, u_h, label="Control Input")
plt.xlabel("Time (s)")
plt.ylabel("Control Input")
plt.legend()
plt.savefig("Figure_1.png", dpi=300)
plt.show()

print(f"Final Estimated Ahat: {Ahat_h[-1]:.2f} and True A: {A:.2f}")
print(f"Final Estimated Bhat: {Bhat_h[-1]:.2f} and True B: {B:.2f}")

