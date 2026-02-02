function disp_metrics_steady(M, label)

fprintf("\n--- %s ---\n", label);
fprintf("Peak deviation: %.3f at t = %.3f\n", M.peak_deviation, M.peak_time);
fprintf("RMSE: %.4f\n", M.RMSE);
fprintf("MAE: %.4f\n", M.MAE);
fprintf("STD: %.4f\n", M.STD);
fprintf("IAE: %.3f\n", M.IAE);
fprintf("Mean |u|: %.4f\n", M.mean_abs_u);
fprintf("Percent in ±1 deg: %.2f%%\n", M.percent_in_1deg);
fprintf("Percent in ±2 deg: %.2f%%\n", M.percent_in_2deg);
fprintf("Oscillations: %d\n", M.num_oscillations);
fprintf("Steady-state error: %.4f\n", M.steady_state_error);

end
