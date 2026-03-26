function result = optimizer(varargin)
%OPTIMIZER Optimize STM-style PID gains and plot the final step response.

[params, cfg] = parse_optimizer_inputs(varargin{:});

options = optimset( ...
    'Display', cfg.display, ...
    'TolX', 1e-7, ...
    'TolFun', 1e-7, ...
    'MaxIter', cfg.max_iter);

[gainsOpt, costOpt] = optimize_gains(params, cfg, options);

simulation = simulate_bench_step(gainsOpt, params, ...
    'step_amplitude', cfg.step_amplitude, ...
    'step_time', cfg.step_time, ...
    'stop_time', cfg.stop_time, ...
    'sample_time', cfg.sample_time);

metrics = evaluate_bench_response(simulation);

if cfg.show_plot
    plot_bench_response(simulation, metrics, gainsOpt, costOpt);
end

result = struct();
result.controller_type = 'stm_discrete_pid';
result.gains = struct('KP', gainsOpt(1), 'KI', gainsOpt(2), 'KD', gainsOpt(3));
result.cost = costOpt;
result.params = params;
result.config = cfg;
result.simulation = simulation;
result.metrics = metrics;

fprintf('=== Bench Optimization Complete ===\n');
fprintf('KP = %.6f\n', gainsOpt(1));
fprintf('KI = %.6f\n', gainsOpt(2));
fprintf('KD = %.6f\n', gainsOpt(3));
fprintf('Cost = %.6g\n', costOpt);
end

function [bestGains, bestCost] = optimize_gains(params, cfg, options)
starts = build_start_points(cfg.initial_gains);
bestGains = starts(1, :);
bestCost = inf;

for i = 1:size(starts, 1)
    [candidateGains, candidateCost] = fminsearch( ...
        @(g) objective(g, params, cfg), starts(i, :), options);

    if candidateCost < bestCost
        bestGains = candidateGains;
        bestCost = candidateCost;
    end
end

for i = 1:cfg.num_restarts
    [candidateGains, candidateCost] = fminsearch( ...
        @(g) objective(g, params, cfg), bestGains, options);

    if candidateCost < bestCost - 1e-9
        bestGains = candidateGains;
        bestCost = candidateCost;
    else
        break
    end
end
end

function J = objective(gains, params, cfg)
gains = double(gains(:).');

if any(~isfinite(gains)) || any(gains < 0)
    J = 1e12;
    return
end

sim = simulate_bench_step(gains, params, ...
    'step_amplitude', cfg.step_amplitude, ...
    'step_time', cfg.step_time, ...
    'stop_time', cfg.stop_time, ...
    'sample_time', cfg.sample_time);

regSim = simulate_bench_step(gains, params, ...
    'step_amplitude', 0, ...
    'step_time', cfg.stop_time + cfg.sample_time, ...
    'stop_time', cfg.stop_time, ...
    'sample_time', cfg.sample_time, ...
    'initial_theta', -cfg.step_amplitude);

if any(~isfinite(sim.theta)) || any(abs(sim.theta) > cfg.theta_guard) || ...
        any(~isfinite(regSim.theta)) || any(abs(regSim.theta) > cfg.theta_guard)
    J = 1e11;
    return
end

metrics = evaluate_bench_response(sim);
if ~isfinite(metrics.ISE) || ~isfinite(metrics.RMS)
    J = 1e10;
    return
end

dt = max(sim.dt, eps);
controlEffort = sum(sim.control_u_action.^2, 'omitnan') * dt;
saturationPenalty = mean(double(sim.saturated), 'omitnan');
terminalError = abs(sim.theta(end) - sim.theta_sp(end));
overshootPenalty = metrics.overshoot.^2;
settlingPenalty = metrics.settling_time / max(cfg.stop_time, eps);
gainPenalty = sum(gains.^2, 'omitnan');
oscillationPenalty = count_oscillations(sim, cfg);
regulationRms = sqrt(mean(regSim.theta.^2, 'omitnan'));
regulationTerminal = abs(regSim.theta(end));

J = cfg.w_ise * metrics.ISE + ...
    cfg.w_rms * (metrics.RMS.^2) + ...
    cfg.w_overshoot * overshootPenalty + ...
    cfg.w_settling * settlingPenalty + ...
    cfg.w_terminal * (terminalError.^2) + ...
    cfg.w_regulation * (regulationRms.^2) + ...
    cfg.w_reg_terminal * (regulationTerminal.^2) + ...
    cfg.w_oscillation * oscillationPenalty + ...
    cfg.w_control * controlEffort + ...
    cfg.w_saturation * saturationPenalty + ...
    cfg.w_gain * gainPenalty;

if ~isfinite(J)
    J = 1e10;
end

function penalty = count_oscillations(sim, cfg)
mask = sim.t >= cfg.step_time;
if ~any(mask)
    penalty = 0;
    return
end

error = sim.theta_sp(mask) - sim.theta(mask);
tolerance = 0.02 * max(abs(cfg.step_amplitude), eps);
active = abs(error) > tolerance;
error = error(active);

if numel(error) < 3
    penalty = 0;
    return
end

signs = sign(error);
signs = signs(signs ~= 0);

if numel(signs) < 3
    penalty = 0;
    return
end

signChanges = sum(signs(2:end) ~= signs(1:end-1));
penalty = max(signChanges - 1, 0);
end
end

function [params, cfg] = parse_optimizer_inputs(varargin)
cfg = default_optimizer_config();
paramOverrides = struct();

i = 1;
while i <= numel(varargin)
    arg = varargin{i};

    if isstruct(arg)
        paramOverrides = merge_structs(paramOverrides, arg);
        i = i + 1;
        continue
    end

    if ~(ischar(arg) || isstring(arg))
        error('optimizer:InvalidInput', ...
            'Expected structs or name/value pairs.');
    end

    if i == numel(varargin)
        error('optimizer:InvalidInput', ...
            'Missing value for parameter "%s".', char(string(arg)));
    end

    name = char(string(arg));
    value = varargin{i+1};
    key = lower(name);

    if isfield(cfg, key)
        cfg.(key) = value;
    else
        paramOverrides.(name) = value;
    end

    i = i + 2;
end

params = load_hardwarex_bench_params(paramOverrides);
cfg = finalize_config(cfg, params);
end

function cfg = default_optimizer_config()
cfg = struct();
cfg.initial_gains = [1.5 1e-4 0.03];
cfg.step_amplitude = deg2rad(10);
cfg.step_time = 1.5;
cfg.stop_time = 150.0;
cfg.sample_time = NaN;
cfg.max_iter = 600;
cfg.num_restarts = 2;
cfg.display = 'off';
cfg.show_plot = true;
cfg.theta_guard = deg2rad(80);

cfg.w_ise = 1.0;
cfg.w_rms = 0.5;
cfg.w_overshoot = 5.0;
cfg.w_settling = 1.0;
cfg.w_terminal = 6.0;
cfg.w_regulation = 4.0;
cfg.w_reg_terminal = 8.0;
cfg.w_oscillation = 2.0;
cfg.w_control = 5e-5;
cfg.w_saturation = 1.0;
cfg.w_gain = 1e-7;
end

function cfg = finalize_config(cfg, params)
cfg.initial_gains = double(cfg.initial_gains(:).');
cfg.step_amplitude = double(cfg.step_amplitude);
cfg.step_time = double(cfg.step_time);
cfg.stop_time = double(cfg.stop_time);
cfg.max_iter = max(1, round(double(cfg.max_iter)));
cfg.num_restarts = max(0, round(double(cfg.num_restarts)));
cfg.display = char(string(cfg.display));
cfg.show_plot = logical(cfg.show_plot);
cfg.theta_guard = double(cfg.theta_guard);

if ~isfinite(cfg.sample_time)
    cfg.sample_time = 1 / max(params.control_rate_hz, eps);
else
    cfg.sample_time = double(cfg.sample_time);
end

weightFields = {'w_ise', 'w_rms', 'w_overshoot', 'w_settling', ...
    'w_terminal', 'w_regulation', 'w_reg_terminal', ...
    'w_oscillation', 'w_control', 'w_saturation', 'w_gain'};
for i = 1:numel(weightFields)
    key = weightFields{i};
    cfg.(key) = double(cfg.(key));
end
end

function starts = build_start_points(initialGains)
starts = [ ...
    initialGains(:).'; ...
    [0.5 0 0.02]; ...
    [1.5 1e-4 0.03]; ...
    [3.0 0 0.05]; ...
    [5.0 0 0.10]; ...
    [10.0 0 0.30]; ...
    [20.0 0.01 1.0]];
starts = unique(starts, 'rows', 'stable');
end

function base = merge_structs(base, extra)
names = fieldnames(extra);
for i = 1:numel(names)
    base.(names{i}) = extra.(names{i});
end
end
