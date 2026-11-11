function metrics = evaluate_current_capture(csvPath, axisName, outdir)
% Single abrupt step, average_count=1. Input columns match MCU sample struct.
% Supply only samples[0:sample_count-1], never the whole unfilled array.
arguments
    csvPath (1,:) char
    axisName (1,1) string {mustBeMember(axisName,["d","q"])}
    outdir (1,:) char
end
T = readtable(csvPath);
t = T.time_s; seq = T.source_sequence;
r = T.(axisName+"_reference_a"); y = T.(axisName+"_current_a");
u = T.(axisName+"_voltage_pu");
assert(all(isfinite([t;seq;r;y;u])) && all(diff(t)>0));
assert(all(diff(seq)==1),'Missing samples or averaging: use full-rate capture.');
changes = find(abs(diff(r))>1e-6);
assert(numel(changes)==1,'Expected one step; sine/ramp requires a different analysis.');
k = changes(1)+1;
assert(k>=6 && height(T)-k>=10,'Insufficient baseline or post-step samples.');
t = t-t(k); delta = r(end)-r(1);
assert(abs(delta)>1e-6);
baseline = mean(y(1:k-1));
info = stepinfo((y(k:end)-baseline)/delta,t(k:end),1, ...
    'SettlingTimeThreshold',0.02);
metrics = info;
metrics.error_tail_a = mean(r(end-9:end)-y(end-9:end));
metrics.IAE_as = trapz(t(k:end),abs(r(k:end)-y(k:end)));
metrics.voltage_peak_pu = max(abs(u));
metrics.duration_after_step_s = t(end);
metrics.warning = 'Finite record only; verify metadata saturation_count=0. No gain/phase margin inferred from step.';
if ~isfolder(outdir), mkdir(outdir); end
save(fullfile(outdir,'capture_metrics.mat'),'metrics');
fig = figure('Visible','off');
tiledlayout(2,1);
nexttile; plot(t,r,t,y); grid on; ylabel('Current (A)'); legend('Reference','Feedback');
nexttile; plot(t,u); grid on; xlabel('Time relative to step (s)'); ylabel('Voltage (pu)');
exportgraphics(fig,fullfile(outdir,'capture_step.png')); close(fig);
disp(metrics)
end
