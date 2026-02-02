clear; clc;

num = [-0.02478 1.665 -86.37];
den = [1 7.67 2023];
num = -num;
G = tf(num, den);

Ts = 0.0317;
Gz = c2d(G, Ts, 'tustin');

[num_d, den_d] = tfdata(Gz, 'v');
A = den_d;
B = num_d;

RS0 = [0 0.05 0.01 0.05];

stepCost = @(RS) rstCost(A, B, RS(1), RS(2), RS(3), RS(4), Ts);
RS_opt = fminsearch(stepCost, RS0);

R = [1 RS_opt(1)];
S = [RS_opt(2) RS_opt(3)];
T = RS_opt(4);

desired_T = 0.1;
k = desired_T / T;

R_impl = R * k
S_impl = S * k
T_impl = T * k

function J = rstCost(A, B, r1, s0, s1, T, Ts)
    R = [1 r1];
    S = [s0 s1];
    
    num_cl = T * B;
    den_cl = conv(A, R) + conv(B, S);
    
    sys_cl = tf(num_cl, den_cl, Ts);
    
    [y, ~] = step(sys_cl, 2);
    ref = ones(size(y));
    
    J = sum((y - ref).^2);
end
