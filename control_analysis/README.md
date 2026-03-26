# Control_Repo

This repo is now a minimal 1 DoF bench tuner.

The optimizer is matched to the STM controller structure:

- angle error in controller units
- integral as a running sum
- derivative from error difference with first-order filtering
- motor mixing with `u_action = control / 2`
- motor clipping before the plant update

## Simplest Use

From MATLAB:

```matlab
cd('.../Control_Repo/Optimization')
result = optimizer;
```

After optimization, a figure opens automatically with the final step response.

The optimizer returns STM-style gains:

```matlab
result.gains.KP
result.gains.KI
result.gains.KD
```

## Hard-Coded Bench Parameters

The default bench configuration is hard-coded in:

[`Optimization/load_hardwarex_bench_params.m`](Optimization/load_hardwarex_bench_params.m)

Current values:

- `Ixx = 3.2e-3`
- `m_d = m_e = 0.03 kg`
- `L_d = L_e = 0.4 m`
- `B_b = 2e-3`
- `control_rate_hz = 100`
- `angle_units = 'deg'`
- `measurement_noise_std = 1.0 deg`
- `motor_idle_command = 200`
- `motor_hover_command = 1000`
- `motor_max_command = 1800`
- `force_min_n = 2.0`
- `force_max_n = 3.0`
- `control_divisor = 2.0`
- `derivative_filter_alpha = 0.7`
- `ierr_min = -1000`
- `ierr_max = 1000`

## How To Change Parameters

### 1. Physical bench parameters

Edit `default_params()` in:

[`Optimization/load_hardwarex_bench_params.m`](Optimization/load_hardwarex_bench_params.m)

Main physical parameters:

- `Ixx`: total rigid-body inertia around the bench axis
- `m_d`, `m_e`: effective moving mass on each side
- `L_d`, `L_e`: distance from pivot to each motor
- `B_b`: viscous damping term

### 2. STM/controller parameters

Also in `default_params()`:

- `control_rate_hz`: controller sample rate
- `angle_units`: `deg` or `rad`
- `measurement_noise_std`: white sensor noise standard deviation in controller units
- `motor_idle_command`: minimum usable motor command
- `motor_hover_command`: center command used for symmetric mixing
- `motor_max_command`: maximum allowed motor command
- `force_min_n`, `force_max_n`: linear thrust model limits used by the simulator
- `control_divisor`: the divisor used before motor mixing
- `derivative_filter_alpha`: derivative smoothing factor
- `ierr_min`, `ierr_max`: integral clamp

### 3. Optimization parameters

Edit `default_optimizer_config()` in:

[`Optimization/optimizer.m`](Optimization/optimizer.m)

Main tuning settings:

- `initial_gains`
- `step_amplitude`
- `step_time`
- `stop_time`
- `sample_time`
- `max_iter`
- `num_restarts`
- `show_plot`
- `theta_guard`

Cost weights:

- `w_ise`
- `w_rms`
- `w_overshoot`
- `w_settling`
- `w_terminal`
- `w_regulation`
- `w_reg_terminal`
- `w_oscillation`
- `w_control`
- `w_saturation`
- `w_gain`

You can also override any of these when calling `optimizer`.

Example:

```matlab
result = optimizer( ...
    'initial_gains', [2 0 0.1], ...
    'step_amplitude', deg2rad(10), ...
    'max_iter', 600);
```

## Files

- `Optimization/optimizer.m`: gain optimization
- `Optimization/simulate_bench_step.m`: STM-matched simulator
- `Optimization/evaluate_bench_response.m`: step metrics
- `Optimization/load_hardwarex_bench_params.m`: hard-coded bench parameters
- `Optimization/plot_bench_response.m`: final response figure
