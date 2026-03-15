# %%
import controlsim as csm
import control as ctrl
import numpy as np
import matplotlib.pyplot as plt

plt.rcParams["mathtext.fontset"] = "cm"
plt.rcParams["font.family"] = "serif"
plt.rcParams["font.size"] = 10
dt = 0.01
# %% Plant Definition
A = np.matrix([[0, 1], [-1, -2]], dtype=float)
B = np.matrix([[0], [0.5]], dtype=float)
C = np.matrix([[1, 0]], dtype=float)
D = np.matrix([[0]], dtype=float)
sys = ctrl.ss(A, B, C, D)
plant = csm.ProcessDefinition(sys, 0.0, dt=dt)
# %% Reference Model Definition
Am = np.matrix([[0, 1], [-1, -3]], dtype=float)
Bm = np.matrix([[0], [1]], dtype=float)
Cm = np.matrix([[1, 1]], dtype=float)
Dm = np.matrix([[0]], dtype=float)

K_x = -ctrl.place(A, B, np.linalg.eigvals(Am))
K_r = -np.linalg.inv(Cm @ np.linalg.inv(Am) @ B)
K_x = np.matrix(K_x).reshape(1, 2)
K_r = np.matrix(K_r).reshape(1, 1)
sysm = ctrl.ss(Am, Bm, Cm, Dm)
plant_ref = csm.ProcessDefinition(sysm, 0.0, dt=dt)
# %% Simulation of nominal system
sim_time = 100
t = np.arange(0, sim_time, dt)
plant.reset()
plant_ref.reset()
for i in range(len(t) - 1):
    u = np.sign(np.sin(2 * np.pi * 0.05 * t[i]))
    plant.step(u)
    plant_ref.step(u)


plt.figure()
plt.plot(plant_ref.t, plant_ref.x, label="Reference State 1")
plt.xlabel("Time")
plt.ylabel("State 1")
plt.legend()
plt.show()

P = ctrl.lyap(A.transpose(), np.eye(2))
print("Lyapunov Matrix P:\n", P)


# %% Uncertainty Introduction
def get_alpha(x, t):
    alpha = np.zeros(3)
    alpha[0] = np.sin(2 * 10 * x[0, 0] * t)
    alpha[1] = np.cos(2 * 15 * x[1, 0] * t)
    alpha[2] = 1.0
    return np.matrix(alpha, dtype=float).reshape(3, 1)


def run_sumulation(gamma, sim_time):
    What = np.matrix([0.0, 0.0, 0.0], dtype=float).reshape(3, 1)
    x = np.matrix([0.0, 0.0], dtype=float).reshape(2, 1)
    xm = np.matrix([0.0, 0.0], dtype=float).reshape(2, 1)
    u_t = []
    T_t = []
    X = np.empty((2, 0), dtype=float)
    Xm = np.empty((2, 0), dtype=float)
    t = np.arange(0, sim_time, dt)
    W_hat = np.empty((3, 0), dtype=float)
    plant.reset()
    plant_ref.reset()
    # gamma = 50.0  # Adaptation gain

    for i in range(len(t) - 1):
        Ref = np.sign(np.sin(2 * np.pi * 0.01 * t[i]))
        u = K_r * Ref + K_x @ x - What.T @ get_alpha(x, t[i])
        x, _, ts = plant.step(u + W @ get_alpha(x, t[i]))
        xm, _, _ = plant_ref.step(Ref)
        x = np.matrix(x).reshape(2, 1)
        xm = np.matrix(xm).reshape(2, 1)
        e = x - xm
        What_dot = (
            gamma * get_alpha(x, t[i]) * e.T * P * B
        )  # Adding a small decay term to prevent unbounded growth
        What += What_dot * dt
        u_t.append(u.item())
        T_t.append(t[i])
        W_hat = np.append(W_hat, What, axis=1)
        X = np.append(np.asarray(X), x, axis=1)
        Xm = np.append(np.asarray(Xm), xm, axis=1)
    return X, Xm, u_t, T_t, W_hat


# %% Response

sim_time = 300
W = np.matrix([3, 0.3, -0.5], dtype=float).reshape(1, 3)
x, xm, u_t, T_t, W_hat = run_sumulation(gamma=0.0, sim_time=sim_time)  # Adaptive OFF
x_1, xm_1, u_t_1, T_t_1, W_hat_1 = run_sumulation(
    gamma=50.0, sim_time=sim_time
)  # Adaptive ON

# %% plot
fig, axs = plt.subplots(3, 1, figsize=(10, 10))

n = xm.shape[0]

# Generate dynamic label lists
labels_ref = [rf"$x_m{i+1}$" for i in range(n)]
labels_off = [rf"$x_{i+1}$ (Adaptive: OFF)" for i in range(n)]
labels_on = [rf"$x_{i+1}$ (Adaptive: ON)" for i in range(n)]

# Plotting
axs[0].plot(T_t, xm.T, label=labels_ref, linestyle="--")
axs[0].plot(T_t, x.T, label=labels_off, alpha=0.5)
axs[0].plot(T_t_1, x_1.T, label=labels_on)
axs[0].set_ylabel("States")
axs[0].legend(ncol=3)

axs[1].plot(T_t, u_t, label=r"$u(t)$ (Adaptive: OFF)")
axs[1].plot(T_t_1, u_t_1, label=r"$u(t)$ (Adaptive: ON)", alpha=0.5)
axs[1].set_ylabel("Control Input")
axs[1].legend()

axs[2].plot(T_t, np.asarray(W_hat_1[0]).flatten(), label=r"$\hat{W}_1$")
axs[2].plot(T_t, np.asarray(W_hat_1[1]).flatten(), label=r"$\hat{W}_2$")
axs[2].plot(T_t, np.asarray(W_hat_1[2]).flatten(), label=r"$\hat{W}_3$")
axs[2].hlines(W[0, 0], T_t[0], T_t[-1], colors="r", linestyles="dashed", label=r"$W_1$")
axs[2].hlines(W[0, 1], T_t[0], T_t[-1], colors="b", linestyles="dashed", label=r"$W_2$")
axs[2].hlines(W[0, 2], T_t[0], T_t[-1], colors="k", linestyles="dashed", label=r"$W_3$")
axs[2].set_xlabel("Time")
axs[2].set_ylabel("Estimated Parameters")
axs[2].legend()
plt.tight_layout()
plt.savefig("robust_mrac_response.png", dpi=300)
plt.show()
