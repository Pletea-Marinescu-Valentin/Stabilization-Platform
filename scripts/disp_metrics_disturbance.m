function disp_metrics_disturbance(M, label)

fprintf("\n--- %s ---\n", label);
fprintf("Max deviation: %.3f\n", M.max_deviation);
fprintf("RMSE: %.4f\n", M.RMSE);
fprintf("MAE: %.4f\n", M.MAE);
fprintf("STD: %.4f\n", M.std);
fprintf("IAE: %.3f\n", M.IAE);
fprintf("Mean |u|: %.4f\n", M.mean_abs_u);
fprintf("Control effort: %.3f\n", M.control_effort);

end
