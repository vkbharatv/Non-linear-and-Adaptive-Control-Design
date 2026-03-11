%% System Definition
clc,clear

A = -1
B = 0.5
C = 1
D=0

sys =  ss(A,B,C,D);

%% Reference Plant
A_m = -1
B_m = 1
C_m = 1
D_m=0
sys_ref = ss(A_m, B_m, C_m, D_m)

