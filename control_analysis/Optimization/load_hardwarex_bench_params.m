function params = load_hardwarex_bench_params(varargin)
%LOAD_HARDWAREX_BENCH_PARAMS Load the hard-coded 1 DoF bench parameters.

overrides = parse_inputs(varargin{:});
params = default_params();
params = apply_overrides(params, overrides);
params = apply_aliases(params);
params = finalize_params(params);
end

function params = default_params()
params = struct();
params.source = 'hard_coded_symmetric_bench';

% Symmetric bench identified from the current HardwareX setup.
params.Ixx = 3.2e-3;
params.m_d = 0.03;
params.m_e = 0.03;
params.L_d = 0.4;
params.L_e = 0.4;
params.B_b = 2.0e-3;

% STM-side controller.
params.control_rate_hz = 100;
params.angle_units = 'deg';
params.measurement_noise_std = 10.0;
params.noise_seed = 23;
params.motor_idle_command = 200;
params.motor_hover_command = 1000;
params.motor_max_command = 1800;
params.force_min_n = 2.0;
params.force_max_n = 3.0;
params.control_divisor = 2.0;
params.derivative_filter_alpha = 0.7;
params.ierr_min = -1000;
params.ierr_max = 1000;
end

function overrides = parse_inputs(varargin)
overrides = struct();

i = 1;
while i <= numel(varargin)
    arg = varargin{i};

    if isstruct(arg)
        overrides = merge_structs(overrides, arg);
        i = i + 1;
        continue
    end

    if ~(ischar(arg) || isstring(arg))
        error('load_hardwarex_bench_params:InvalidInput', ...
            'Expected structs or name/value pairs.');
    end

    if i == numel(varargin)
        error('load_hardwarex_bench_params:InvalidInput', ...
            'Missing value for parameter "%s".', char(string(arg)));
    end

    name = char(string(arg));
    value = varargin{i+1};

    if strcmpi(name, 'AllowMissing')
        i = i + 2;
        continue
    end

    overrides.(name) = value;
    i = i + 2;
end
end

function base = merge_structs(base, extra)
names = fieldnames(extra);
for i = 1:numel(names)
    base.(names{i}) = extra.(names{i});
end
end

function params = apply_overrides(params, overrides)
names = fieldnames(overrides);
for i = 1:numel(names)
    params.(names{i}) = overrides.(names{i});
end
end

function params = apply_aliases(params)
if isfield(params, 'm') && isfinite_scalar(params.m)
    params.m_d = double(params.m);
    params.m_e = double(params.m);
end

if isfield(params, 'L') && isfinite_scalar(params.L)
    params.L_d = double(params.L);
    params.L_e = double(params.L);
end

if isfield(params, 'B') && isfinite_scalar(params.B)
    params.B_b = double(params.B);
end

if isfield(params, 'idle_command') && isfinite_scalar(params.idle_command)
    params.motor_idle_command = double(params.idle_command);
end

if isfield(params, 'measurement_noise_std') && isfinite_scalar(params.measurement_noise_std)
    params.measurement_noise_std = double(params.measurement_noise_std);
end

if isfield(params, 'noise_seed') && isfinite_scalar(params.noise_seed)
    params.noise_seed = double(params.noise_seed);
end

if isfield(params, 'hover_command') && isfinite_scalar(params.hover_command)
    params.motor_hover_command = double(params.hover_command);
end

if isfield(params, 'max_command') && isfinite_scalar(params.max_command)
    params.motor_max_command = double(params.max_command);
end

if isfield(params, 'force_min') && isfinite_scalar(params.force_min)
    params.force_min_n = double(params.force_min);
end

if isfield(params, 'force_max') && isfinite_scalar(params.force_max)
    params.force_max_n = double(params.force_max);
end

if isfield(params, 'd_filter_alpha') && isfinite_scalar(params.d_filter_alpha)
    params.derivative_filter_alpha = double(params.d_filter_alpha);
end
end

function params = finalize_params(params)
numericFields = { ...
    'Ixx', 'm_d', 'm_e', 'L_d', 'L_e', 'B_b', ...
    'control_rate_hz', 'measurement_noise_std', 'noise_seed', ...
    'motor_idle_command', 'motor_hover_command', ...
    'motor_max_command', 'force_min_n', 'force_max_n', ...
    'control_divisor', 'derivative_filter_alpha', 'ierr_min', 'ierr_max'};

for i = 1:numel(numericFields)
    key = numericFields{i};
    if isfield(params, key)
        params.(key) = double(params.(key));
    end
end

params.angle_units = char(string(params.angle_units));
params.Itot = params.Ixx + params.m_e * params.L_e^2 + params.m_d * params.L_d^2;

commandSpan = params.motor_max_command - params.motor_idle_command;
if commandSpan <= 0
    error('load_hardwarex_bench_params:InvalidCommands', ...
        'motor_max_command must be greater than motor_idle_command.');
end

params.force_per_command_n = (params.force_max_n - params.force_min_n) / commandSpan;
params.hover_force_n = params.force_min_n + ...
    (params.motor_hover_command - params.motor_idle_command) * params.force_per_command_n;
params.torque_per_command = (params.L_d + params.L_e) * params.force_per_command_n;
end

function tf = isfinite_scalar(value)
tf = isnumeric(value) && isscalar(value) && isfinite(value);
end
