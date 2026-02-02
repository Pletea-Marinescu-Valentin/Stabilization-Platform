clear; clc; close all;

T = readtable("pitch_id.csv");

t = T.time_ms / 1000;
u = T.u_deg;
y_deg = T.pitch_deg;

idx = ~isnan(t) & ~isnan(u) & ~isnan(y_deg);
t = t(idx);  u = u(idx);  y_deg = y_deg(idx);

y_rad = deg2rad(y_deg);
y = unwrap(y_rad);

idx = t > (t(1)+1.0);
t = t(idx);  u = u(idx);  y = y(idx);

y = detrend(y, 1);
u = detrend(u);

Ts = mean(diff(t));

data_id = iddata(y, u, Ts);

bestFit = -Inf;
bestModel = [];

for na = 1:5
    for nb = 1:5
        for nk = 0:3
            try
                m = oe(data_id, [na nb nk]);
                fit = m.Report.Fit.FitPercent;
                if fit > bestFit
                    bestFit = fit;
                    bestModel = m;
                end
            end
        end
    end
end

for nx = 1:10
    try
        m = n4sid(data_id, nx, 'DisturbanceModel','estimate');
        fit = m.Report.Fit.FitPercent;
        if fit > bestFit
            bestFit = fit;
            bestModel = m;
        end
    end
end

figure;
[y_pred, ~] = predict(bestModel, data_id, 1);
plot(t, y, 'b', 'LineWidth', 1.3); hold on;
plot(t, y_pred.y, 'r', 'LineWidth', 1.3);
legend("Real system", "Model prediction", "Location", "Best");
title(sprintf("Model Prediction Fit = %.2f%%", bestFit));
xlabel("Time [s]");
ylabel("Pitch [rad]");
grid on;

sysd = ss(bestModel);
sysc = d2c(sysd);
sysc_min = minreal(sysc);

sysc_tf = tf(sysc_min);
sysc_tf = minreal(sysc_tf);

sys2 = balred(sysc_tf, 2);
