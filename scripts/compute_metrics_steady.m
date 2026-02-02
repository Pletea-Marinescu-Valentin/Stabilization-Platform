function M = compute_metrics_steady(t, e, u)

M.RMSE = sqrt(mean(e.^2));
M.MAE  = mean(abs(e));
M.STD  = std(e);
M.mean_abs_u = mean(abs(u));

if numel(u) >= 2
    M.control_effort = sum(abs(diff(u)));
else
    M.control_effort = 0;
end

[max_val, idx] = max(abs(e));
M.peak_deviation = max_val;
M.overshoot_abs = max_val;
M.peak_time = t(idx);

M.IAE = trapz(t, abs(e));
M.ISE = trapz(t, e.^2);

M.percent_in_1deg = 100 * sum(abs(e) <= 1) / numel(e);
M.percent_in_2deg = 100 * sum(abs(e) <= 2) / numel(e);

s = sign(e);
s(s==0) = 1;
M.num_oscillations = sum(abs(diff(s)) > 0);

nTail = min(50, length(e));
M.steady_state_error = mean(e(end - nTail + 1 : end));

end
