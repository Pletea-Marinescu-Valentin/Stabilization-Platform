clear; close all; clc;

num_roll = [0.02478 -1.665 86.37];
den_roll = [1 7.67 2023];

[A_r, B_r, C_r, D_r] = tf2ss(num_roll, den_roll)

Ts = 0.0317;
sys_r = ss(A_r, B_r, C_r, D_r);
sys_rd = c2d(sys_r, Ts, 'zoh')
[Ad_r, Bd_r, Cd_r, ~] = ssdata(sys_rd);

Q_r = diag([1, 2]);
R_r = 1;
[K_r, ~, ~] = lqr(A_r, B_r, Q_r, R_r)

Qn_r = diag([0.1, 0.1]);
Rn_r = 2.0;
[L_r, ~, ~] = lqe(A_r, eye(2), C_r, Qn_r, Rn_r)
Ld_r = L_r * Ts
