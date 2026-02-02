clear; close all; clc;

num_pitch = [0.0347 -2.379 78.92];
den_pitch = [1 6.256 683.5];

[A_p, B_p, C_p, D_p] = tf2ss(num_pitch, den_pitch)

Ts = 0.0317;
sys_p = ss(A_p, B_p, C_p, D_p);
sys_pd = c2d(sys_p, Ts, 'zoh')
[Ad_p, Bd_p, Cd_p, ~] = ssdata(sys_pd);

Q_p = diag([1, 2]);
R_p = 1;
[K_p, ~, ~] = lqr(A_p, B_p, Q_p, R_p)

Qn_p = diag([0.1, 0.1]);
Rn_p = 0.5;
[L_p, ~, ~] = lqe(A_p, eye(2), C_p, Qn_p, Rn_p)

Ld_p = L_p * Ts
