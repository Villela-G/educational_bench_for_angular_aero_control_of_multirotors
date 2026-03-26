function metrics = evaluate_bench_response(sim)
%EVALUATE_BENCH_RESPONSE Compute step-response metrics for the 1 DoF bench.

t = sim.t(:);
theta = sim.theta(:);
thetaSp = sim.theta_sp(:);

if numel(t) < 20
    error('evaluate_bench_response:SignalTooShort', ...
        'Simulation is too short for evaluation.');
end

[stepIdx, tStep, stepAmplitude, stepSign, u0, uf] = detect_step(t, thetaSp);
tRel = t - tStep;

baselineMask = tRel < 0 & tRel >= -0.2;
if ~any(baselineMask)
    baselineMask = tRel < 0;
end

if any(baselineMask)
    baseline = mean(theta(baselineMask), 'omitnan');
else
    baseline = theta(1);
end

if ~isfinite(baseline)
    baseline = 0;
end

thetaAligned = theta - baseline;
reference = stepAmplitude * double(tRel >= 0);

tStepRel = t(stepIdx:end) - t(stepIdx);
thetaStep = thetaAligned(stepIdx:end);
[tStepRel, uniqueIdx] = unique(tStepRel);
thetaStep = thetaStep(uniqueIdx);

if numel(thetaStep) < 10
    error('evaluate_bench_response:SegmentTooShort', ...
        'Step segment is too short for evaluation.');
end

epsValue = 1e-9;
yFinal = mean(thetaStep(round(0.8 * numel(thetaStep)):end), 'omitnan');
if abs(stepAmplitude) < epsValue
    stepAmplitude = uf - u0;
end
if abs(stepAmplitude) < epsValue
    stepAmplitude = 1;
end

ySigned = stepSign * thetaStep;
finalSigned = stepSign * yFinal;
yNorm = ySigned / max(abs(stepAmplitude), epsValue);

idx10 = find(yNorm >= 0.1, 1, 'first');
idx90 = find(yNorm >= 0.9, 1, 'first');
if isempty(idx10) || isempty(idx90)
    riseTime = NaN;
else
    riseTime = tStepRel(idx90) - tStepRel(idx10);
end

tolerance = 0.02 * max(abs(stepAmplitude), epsValue);
lastOutside = find(abs(thetaStep - yFinal) > tolerance, 1, 'last');
if isempty(lastOutside)
    settlingTime = 0;
else
    settlingTime = tStepRel(lastOutside);
end

[peakSigned, peakIdx] = max(ySigned);
peakValue = thetaStep(peakIdx);
peakTime = tStepRel(peakIdx);
minSigned = min(ySigned);
referenceMagnitude = max(abs(finalSigned), epsValue);
overshoot = max((peakSigned - finalSigned) / referenceMagnitude, 0);
undershoot = max(-minSigned / referenceMagnitude, 0);

mask = tRel >= 0;
error = thetaAligned(mask) - reference(mask);
dt = mean(diff(t(mask)), 'omitnan');
if ~isfinite(dt) || dt <= 0
    ISE = NaN;
    RMS = NaN;
else
    ISE = sum(error.^2, 'omitnan') * dt;
    RMS = sqrt(mean(error.^2, 'omitnan'));
end

metrics = struct();
metrics.step_index = stepIdx;
metrics.t_step = tStep;
metrics.step_amplitude = uf - u0;
metrics.step_sign = stepSign;
metrics.baseline = baseline;
metrics.rise_time = riseTime;
metrics.settling_time = settlingTime;
metrics.overshoot = overshoot;
metrics.undershoot = undershoot;
metrics.peak = peakValue;
metrics.peak_time = peakTime;
metrics.final_value = yFinal;
metrics.ISE = ISE;
metrics.RMS = RMS;
metrics.t_rel = tRel;
metrics.theta_aligned = thetaAligned;
metrics.reference = reference;
metrics.error = error;
end

function [stepIdx, tStep, stepAmplitude, stepSign, u0, uf] = detect_step(t, u)
u = fillmissing(u(:), 'nearest');

if numel(u) < 2
    error('evaluate_bench_response:MissingStep', ...
        'Setpoint is too short.');
end

du = diff(u);
[stepMagnitude, stepIdxRaw] = max(abs(du));
if isempty(stepIdxRaw) || ~isfinite(stepMagnitude) || stepMagnitude <= 0
    error('evaluate_bench_response:MissingStep', ...
        'Could not detect a meaningful step in theta_sp.');
end

stepIdx = min(stepIdxRaw + 1, numel(t));
tStep = t(stepIdx);

preIdx = max(1, stepIdx - min(25, stepIdx - 1)):max(stepIdx - 1, 1);
postIdx = max(stepIdx, numel(u) - min(25, numel(u) - stepIdx + 1) + 1):numel(u);

u0 = mean(u(preIdx), 'omitnan');
uf = mean(u(postIdx), 'omitnan');
stepAmplitude = uf - u0;

if abs(stepAmplitude) < 1e-9
    stepAmplitude = du(stepIdxRaw);
end

stepSign = sign(stepAmplitude);
if stepSign == 0
    stepSign = 1;
end
end
