clear; clc; close all;

filename = "log_adaptive.csv";

TARGET_PITCH = -0.2;
TARGET_ROLL  = -175.44;

wrapTo180 = @(a) mod(a+180,360)-180;

data = readtable(filename);
t = data.time_ms / 1000;

pitch = data.pitch_deg;
roll  = data.roll_deg;
u_pitch = data.control_pitch;
u_roll  = data.control_roll;

e_pitch = pitch - TARGET_PITCH;
e_roll  = wrapTo180(roll - TARGET_ROLL);

steady_range     = t <= 20;
disturb_range    = t > 20;

metrics_steady_pitch  = compute_metrics_steady(t(steady_range), e_pitch(steady_range), u_pitch(steady_range));
metrics_steady_roll   = compute_metrics_steady(t(steady_range), e_roll(steady_range),  u_roll(steady_range));

metrics_dist_pitch    = compute_metrics_disturbance(t(disturb_range), e_pitch(disturb_range), u_pitch(disturb_range));
metrics_dist_roll     = compute_metrics_disturbance(t(disturb_range), e_roll(disturb_range),  u_roll(disturb_range));

[metrics_steady_pitch.cpa,  metrics_steady_pitch.breakdown]  = compute_cpa_score(metrics_steady_pitch);
[metrics_dist_pitch.cpa,    metrics_dist_pitch.breakdown]    = compute_cpa_score(metrics_dist_pitch);

[metrics_steady_roll.cpa,   metrics_steady_roll.breakdown]   = compute_cpa_score(metrics_steady_roll);
[metrics_dist_roll.cpa,     metrics_dist_roll.breakdown]     = compute_cpa_score(metrics_dist_roll);

results.filename = filename;
results.t = t;
results.e_pitch = e_pitch;
results.e_roll  = e_roll;
results.u_pitch = u_pitch;
results.u_roll  = u_roll;

results.steady.pitch = metrics_steady_pitch;
results.steady.roll  = metrics_steady_roll;
results.dist.pitch   = metrics_dist_pitch;
results.dist.roll    = metrics_dist_roll;

matname = erase(filename,".csv") + ".mat";
save(matname, "results");
