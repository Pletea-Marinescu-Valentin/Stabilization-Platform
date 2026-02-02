clear; clc; close all;

files = {"log_pid.mat"; "log_rst.mat"; "log_lqr.mat"; "log_adaptive.mat"};
names  = {"PID","RST","LQG","MRAC"}; 
colors = lines(4);

N = numel(files);

CPA_pitch_steady  = zeros(N,1);
CPA_roll_steady   = zeros(N,1);
CPA_pitch_dist    = zeros(N,1);
CPA_roll_dist     = zeros(N,1);

all_results = cell(N,1);
all_metrics = struct;

for i = 1:N
    S = load(files{i});
    R = S.results;
    all_results{i} = R;

    CPA_pitch_steady(i) = R.steady.pitch.cpa;
    CPA_roll_steady(i)  = R.steady.roll.cpa;
    CPA_pitch_dist(i)   = R.dist.pitch.cpa;
    CPA_roll_dist(i)    = R.dist.roll.cpa;

    all_metrics(i).name          = names{i};
    all_metrics(i).steady_pitch  = R.steady.pitch;
    all_metrics(i).steady_roll   = R.steady.roll;
    all_metrics(i).dist_pitch    = R.dist.pitch;
    all_metrics(i).dist_roll     = R.dist.roll;
end

T = table(names', CPA_pitch_steady, CPA_roll_steady, CPA_pitch_dist, CPA_roll_dist, ...
    'VariableNames', {'Controller','CPA_Pitch_Steady','CPA_Roll_Steady','CPA_Pitch_Dist','CPA_Roll_Dist'});
disp(T);

for i = 1:N
    fprintf("\n%s:\n", names{i});
    disp_metrics_steady(all_metrics(i).steady_pitch, "Pitch (Steady)");
    disp_metrics_steady(all_metrics(i).steady_roll, "Roll (Steady)");
    disp_metrics_disturbance(all_metrics(i).dist_pitch, "Pitch (Disturbance)");
    disp_metrics_disturbance(all_metrics(i).dist_roll, "Roll (Disturbance)");
end

t_steady_start = 0;
t_steady_end   = 20;
t_dist_start   = 20;
t_dist_end     = 60;

t_ref = all_results{1}.t;
t_max = max(t_ref);
t_steady_end = min(t_steady_end, t_max);
t_dist_start = min(t_dist_start, t_max);
t_dist_end   = min(t_dist_end, t_max);

figure('Color','w','Position',[100 100 1200 400]); hold on;
h_pitch = gobjects(N,1);
for i = 1:N
    R = all_results{i};
    h_pitch(i) = plot(R.t, R.e_pitch, 'LineWidth', 1.4, 'Color', colors(i,:));
end
yline(2,'k--','LineWidth',1);
yline(-2,'k--','LineWidth',1);
yl = ylim;
if t_steady_end > t_steady_start
    patch([t_steady_start t_steady_end t_steady_end t_steady_start], [yl(1) yl(1) yl(2) yl(2)], ...
        [0.8 0.8 0.9], 'FaceAlpha', 0.15, 'EdgeColor', 'none', 'HandleVisibility','off');
end
if t_dist_end > t_dist_start
    patch([t_dist_start t_dist_end t_dist_end t_dist_start], [yl(1) yl(1) yl(2) yl(2)], ...
        [0.9 0.8 0.8], 'FaceAlpha', 0.15, 'EdgeColor', 'none', 'HandleVisibility','off');
end
uistack(h_pitch,'top');
title('Pitch Error - All Controllers');
xlabel('Time [s]'); ylabel('Error [deg]');
grid on; legend(h_pitch, names, 'Location','best');

figure('Color','w','Position',[100 100 1200 400]); hold on;
h_roll = gobjects(N,1);
for i = 1:N
    R = all_results{i};
    h_roll(i) = plot(R.t, R.e_roll, 'LineWidth', 1.4, 'Color', colors(i,:));
end
yline(2,'k--','LineWidth',1);
yline(-2,'k--','LineWidth',1);
yl = ylim;
if t_steady_end > t_steady_start
    patch([t_steady_start t_steady_end t_steady_end t_steady_start], [yl(1) yl(1) yl(2) yl(2)], ...
        [0.8 0.8 0.9], 'FaceAlpha', 0.15, 'EdgeColor', 'none', 'HandleVisibility','off');
end
if t_dist_end > t_dist_start
    patch([t_dist_start t_dist_end t_dist_end t_dist_start], [yl(1) yl(1) yl(2) yl(2)], ...
        [0.9 0.8 0.8], 'FaceAlpha', 0.15, 'EdgeColor', 'none', 'HandleVisibility','off');
end
uistack(h_roll,'top');
title('Roll Error - All Controllers');
xlabel('Time [s]'); ylabel('Error [deg]');
grid on; legend(h_roll, names, 'Location','best');

figure('Color','w','Position',[100 100 1200 400]); hold on;
for i = 1:N
    R = all_results{i};
    plot(R.t, R.u_pitch, 'LineWidth', 1.2, 'Color', colors(i,:));
end
title('Control Signal - Pitch');
xlabel('Time [s]'); ylabel('u_{pitch}');
grid on; legend(names, 'Location','best');

figure('Color','w','Position',[100 100 1200 400]); hold on;
for i = 1:N
    R = all_results{i};
    plot(R.t, R.u_roll, 'LineWidth', 1.2, 'Color', colors(i,:));
end
title('Control Signal - Roll');
xlabel('Time [s]'); ylabel('u_{roll}');
grid on; legend(names, 'Location','best');

figure('Color','w');
subplot(2,2,1); bar(CPA_pitch_steady);
title('CPA Pitch Steady'); ylabel('Score'); set(gca,'XTickLabel',names);
subplot(2,2,2); bar(CPA_roll_steady);
title('CPA Roll Steady'); ylabel('Score'); set(gca,'XTickLabel',names);
subplot(2,2,3); bar(CPA_pitch_dist);
title('CPA Pitch Disturbance'); ylabel('Score'); set(gca,'XTickLabel',names);
subplot(2,2,4); bar(CPA_roll_dist);
title('CPA Roll Disturbance'); ylabel('Score'); set(gca,'XTickLabel',names);
