clear; clc;

num = [-0.02478 1.665 -86.37];
den = [1 7.67 2023];
num = -num;
[Ac,Bc,Cc,Dc] = tf2ss(num,den);

b_est = Cc * Bc;

wn = 2;
zeta = 0.95;

Aref = [0 1; -wn^2 -2*zeta*wn];
Bref = [0; wn^2];
Cref = [1 0];

tau = 1 / wn;
gamma_practical = 0.001;
K_init_practical = 0.02;

dt = 0.001;
Tsim = 10;
N = round(Tsim/dt);

x  = [0;0];
xr = [0;0];
K = K_init_practical;

Klog = zeros(N,1);
y  = zeros(1,N);
yr = zeros(1,N);
r = zeros(1,N);
r(0.5/dt:end) = -3;

for k = 1:N
    yr(k) = Cref * xr;
    y(k)  = Cc * x;
    
    e = r(k) - y(k);
    e_track = yr(k) - y(k);
    
    u = K * e;
    u = max(min(u, 10), -10);
    
    xr = xr + dt*(Aref*xr + Bref*r(k));
    x  = x  + dt*(Ac*x  + Bc*u);
    
    K = K + gamma_practical * e_track * e * dt;
    K = max(min(K, 0.15), 0.01);
    
    Klog(k) = K;
end

K_final = mean(Klog(end-500:end));
t = (0:N-1)*dt;

figure('Position', [100, 100, 1200, 800]);

subplot(3,1,1);
plot(t, y, 'b-', 'LineWidth', 1.5); hold on;
plot(t, yr, 'r--', 'LineWidth', 1.5);
plot(t, r, 'k:', 'LineWidth', 1);
xlabel('Time (s)'); ylabel('Output (deg)');
title('MRAC System Response');
legend('Plant Output', 'Reference Model', 'Command');
grid on;

subplot(3,1,2);
plot(t, Klog, 'g-', 'LineWidth', 1.4);
xlabel('Time (s)'); ylabel('K(t)');
title('Adaptive Gain Evolution');
grid on;

subplot(3,1,3);
plot(t, yr-y, 'm-', 'LineWidth', 1.4);
xlabel('Time (s)'); ylabel('Tracking Error (deg)');
title('e_{track} = y_m - y');
grid on;
