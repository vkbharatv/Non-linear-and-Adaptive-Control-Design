%% System Definition
clc,clear

A = -0.5;
B = 0.5;
C = 1;
D=0;

sys =  ss(A,B,C,D);

%% Reference Plant
A_m = -1;
B_m = 1;
C_m = 1;
D_m=0;
sys_ref = ss(A_m, B_m, C_m, D_m);

%% Closed Loop system
Kx = (A_m - A) / B;
Kr = B_m / B;
A_cl = A+B*Kx;

%% Simulation
model_path = "simulation_MRAC.slx";
Gamma = 20;
load_system(model_path);
sim_out = sim(model_path, "StopTime", "100");
%%
figure, hold on
fprintf("Ideal Kx: %f\n", Kx)
fprintf("Ideal Kr: %f\n", Kr)
fprintf("Final Kx_hat: %f\n", sim_out.Kx_hat.Data(end))
fprintf("Final Kr_hat: %f\n", sim_out.Kr_hat.Data(end))
plot(sim_out.Kx_hat)
yline(Kx,'LineStyle',':')
yline(Kr,'LineStyle',':')
plot(sim_out.Kr_hat)
% ylim([Kx-0.1,Kr+0.1])
