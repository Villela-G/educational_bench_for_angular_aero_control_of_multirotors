function sim = simulate_bench_step(gains, params, varargin)
%SIMULATE_BENCH_STEP Simulate the 1 DoF bench with the STM PID structure.

params = load_hardwarex_bench_params(params);
cfg = parse_config(params, varargin{:});

gains = double(gains(:).');
if numel(gains) ~= 3
    error('simulate_bench_step:InvalidGains', ...
        'Expected gains = [KP KI KD].');
end

KP = gains(1);
KI = gains(2);
KD = gains(3);

t = (0:cfg.sample_time:cfg.stop_time).';
N = numel(t);

theta = zeros(N, 1);
thetaDot = zeros(N, 1);
thetaSp = zeros(N, 1);
thetaMeasured = zeros(N, 1);
thetaMeasuredUnits = zeros(N, 1);
controlSignal = zeros(N, 1);
uAction = zeros(N, 1);
motorRight = zeros(N, 1);
motorLeft = zeros(N, 1);
forceRight = zeros(N, 1);
forceLeft = zeros(N, 1);
tauApplied = zeros(N, 1);
saturated = false(N, 1);
measurementNoise = build_measurement_noise(N, params);

thetaNow = cfg.initial_theta;
thetaDotNow = cfg.initial_theta_dot;
integralErr = 0;
filteredDErr = 0;
thetaRef0 = cfg.step_amplitude * double(0 >= cfg.step_time);
prevErr = angle_to_control_units(thetaRef0, params) - ...
    (angle_to_control_units(thetaNow, params) + measurementNoise(1));

for k = 1:N
    tNow = t(k);
    thetaRefNow = cfg.step_amplitude * double(tNow >= cfg.step_time);
    thetaRefMeasuredUnits = angle_to_control_units(thetaRefNow, params);
    thetaMeasuredUnits(k) = angle_to_control_units(thetaNow, params) + measurementNoise(k);
    thetaMeasured(k) = control_units_to_angle(thetaMeasuredUnits(k), params);

    theta(k) = thetaNow;
    thetaDot(k) = thetaDotNow;
    thetaSp(k) = thetaRefNow;

    err = thetaRefMeasuredUnits - thetaMeasuredUnits(k);
    integralErr = min(max(integralErr + err, params.ierr_min), params.ierr_max);
    rawDErr = err - prevErr;
    filteredDErr = params.derivative_filter_alpha * filteredDErr + ...
        (1 - params.derivative_filter_alpha) * rawDErr;

    controlSignal(k) = KP * err + KI * integralErr + KD * filteredDErr;
    uAction(k) = controlSignal(k) / params.control_divisor;

    motorRight(k) = clip_command(params.motor_hover_command + uAction(k), params);
    motorLeft(k) = clip_command(params.motor_hover_command - uAction(k), params);
    saturated(k) = motorRight(k) ~= params.motor_hover_command + uAction(k) || ...
        motorLeft(k) ~= params.motor_hover_command - uAction(k);

    forceRight(k) = command_to_force(motorRight(k), params);
    forceLeft(k) = command_to_force(motorLeft(k), params);
    tauApplied(k) = params.L_d * forceRight(k) - params.L_e * forceLeft(k);

    prevErr = err;

    if k < N
        xNext = rk4_step([thetaNow; thetaDotNow], tauApplied(k), cfg.sample_time, params);
        thetaNow = xNext(1);
        thetaDotNow = xNext(2);
    end
end

sim = struct();
sim.t = t;
sim.theta = theta;
sim.theta_dot = thetaDot;
sim.theta_sp = thetaSp;
sim.theta_measured = thetaMeasured;
sim.theta_measured_units = thetaMeasuredUnits;
sim.measurement_noise = measurementNoise;
sim.control_signal = controlSignal;
sim.control_u_action = uAction;
sim.motor_right = motorRight;
sim.motor_left = motorLeft;
sim.force_right = forceRight;
sim.force_left = forceLeft;
sim.control_tau_cmd = tauApplied;
sim.control_tau = tauApplied;
sim.saturated = saturated;
sim.source_type = 'bench_simulation_stm_pid';
sim.controller_type = 'stm_discrete_pid';
sim.params = params;
sim.gains = gains;
sim.N = N;
sim.dt = cfg.sample_time;
end

function cfg = parse_config(params, varargin)
p = inputParser;
p.FunctionName = mfilename;

addParameter(p, 'step_amplitude', deg2rad(10), @(x) isnumeric(x) && isscalar(x));
addParameter(p, 'step_time', 0.25, @(x) isnumeric(x) && isscalar(x) && x >= 0);
addParameter(p, 'stop_time', 4.0, @(x) isnumeric(x) && isscalar(x) && x > 0);
addParameter(p, 'sample_time', 1 / params.control_rate_hz, @(x) isnumeric(x) && isscalar(x) && x > 0);
addParameter(p, 'initial_theta', 0, @(x) isnumeric(x) && isscalar(x));
addParameter(p, 'initial_theta_dot', 0, @(x) isnumeric(x) && isscalar(x));

parse(p, varargin{:});
cfg = p.Results;

cfg.step_amplitude = double(cfg.step_amplitude);
cfg.step_time = double(cfg.step_time);
cfg.stop_time = double(cfg.stop_time);
cfg.sample_time = double(cfg.sample_time);
cfg.initial_theta = double(cfg.initial_theta);
cfg.initial_theta_dot = double(cfg.initial_theta_dot);
end

function value = angle_to_control_units(thetaRad, params)
switch lower(strtrim(params.angle_units))
    case 'deg'
        value = rad2deg(thetaRad);
    case 'rad'
        value = thetaRad;
    otherwise
        error('simulate_bench_step:InvalidAngleUnits', ...
            'Unsupported angle_units "%s".', params.angle_units);
end
end

function thetaRad = control_units_to_angle(value, params)
switch lower(strtrim(params.angle_units))
    case 'deg'
        thetaRad = deg2rad(value);
    case 'rad'
        thetaRad = value;
    otherwise
        error('simulate_bench_step:InvalidAngleUnits', ...
            'Unsupported angle_units "%s".', params.angle_units);
end
end

function noise = build_measurement_noise(N, params)
stream = RandStream('mt19937ar', 'Seed', params.noise_seed);
noise = params.measurement_noise_std * randn(stream, N, 1);
end

function command = clip_command(command, params)
command = min(max(command, params.motor_idle_command), params.motor_max_command);
end

function force = command_to_force(command, params)
ratio = (command - params.motor_idle_command) / ...
    max(params.motor_max_command - params.motor_idle_command, eps);
ratio = min(max(ratio, 0), 1);
force = params.force_min_n + ratio * (params.force_max_n - params.force_min_n);
end

function xNext = rk4_step(xNow, tauNow, dt, params)
k1 = dynamics(xNow, tauNow, params);
k2 = dynamics(xNow + 0.5 * dt * k1, tauNow, params);
k3 = dynamics(xNow + 0.5 * dt * k2, tauNow, params);
k4 = dynamics(xNow + dt * k3, tauNow, params);
xNext = xNow + (dt / 6) * (k1 + 2 * k2 + 2 * k3 + k4);
end

function dx = dynamics(xNow, tauNow, params)
thetaDotNow = xNow(2);
thetaDDotNow = (tauNow - params.B_b * thetaDotNow) / params.Itot;
dx = [thetaDotNow; thetaDDotNow];
end
