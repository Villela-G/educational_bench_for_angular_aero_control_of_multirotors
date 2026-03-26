function fig = plot_bench_response(sim, metrics, gains, cost)
%PLOT_BENCH_RESPONSE Plot the final bench step response after optimization.

fig = figure('Name', 'Bench Step Response', 'Color', 'w');
plot(sim.t, rad2deg(sim.theta), 'LineWidth', 1.8, 'Color', [0.00 0.35 0.70]);
hold on;
stairs(sim.t, rad2deg(sim.theta_sp), '--', 'LineWidth', 1.5, 'Color', [0.80 0.20 0.10]);
grid on;
box on;
xlabel('Time [s]');
ylabel('Angle [deg]');
legend('Response', 'Step', 'Location', 'best');
title(sprintf(['Final Response | KP=%.4f  KI=%.4f  KD=%.4f' ...
    ' | Cost=%.4f | Rise=%.3fs | Overshoot=%.2f%%'], ...
    gains(1), gains(2), gains(3), cost, metrics.rise_time, 100 * metrics.overshoot));
end
