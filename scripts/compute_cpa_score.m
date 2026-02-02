function [score, breakdown] = compute_cpa_score(M)

IAE_max = 1000;
overshoot_max = 30;
RMSE_max = 10;
control_effort_max = 50;

w.IAE       = 0.45;
w.Overshoot = 0.25;
w.RMSE      = 0.20;
w.Effort    = 0.10;

fields = {'IAE','overshoot_abs','RMSE','control_effort'};
for f = fields
    if ~isfield(M, f{1}) || isnan(M.(f{1}))
        M.(f{1}) = 0;
    end
end

n.IAE       = min(M.IAE              / IAE_max,           1);
n.Overshoot = min(M.overshoot_abs    / overshoot_max,     1);
n.RMSE      = min(M.RMSE             / RMSE_max,          1);
n.Effort    = min(M.control_effort   / control_effort_max,1);

num = w.IAE*n.IAE + w.Overshoot*n.Overshoot + w.RMSE*n.RMSE + w.Effort*n.Effort;
den = w.IAE + w.Overshoot + w.RMSE + w.Effort;

score = num / den;

breakdown.normalized = n;
breakdown.weights = w;
breakdown.raw = M;
end
