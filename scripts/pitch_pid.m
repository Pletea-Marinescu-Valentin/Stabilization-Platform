clear; close all; clc;

num_sys2 = [-0.0347   2.379    -78.92];
den_sys2 = [1         6.256    683.5];

num = -num_sys2;
den =  den_sys2;

G = tf(num, den)
G = minreal(G);

a1 = den(2);
a0 = den(3);

w0 = sqrt(a0)
xi = a1 / (2*w0)
K  = dcgain(G)

lambda = 2.0;
Ti = 10*lambda;
Td = 1/w0;

Kp = (1/K) * (2*xi) / (w0*lambda)
Ki = Kp / Ti
Kd = Kp * Td

s = tf('s');
PID = Kp * (1 + 1/(Ti*s) + Td*s)

CL = feedback(PID*G, 1);

t = 0:0.01:8;
step(CL, t);
grid on;
title('Closed-loop step response');
xlabel('Time [s]');
ylabel('y(t)');
