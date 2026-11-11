function result = run_current_pi_demo()
% Synthetic motor only. NEVER copy these gains to unmeasured hardware.
cfg = current_pi_config();
cfg.R = 0.5;
cfg.L = 1e-3;
cfg.Vdc = 24;
cfg.measured = false;
root = fileparts(fileparts(mfilename('fullpath')));
result = tune_current_pi(cfg, fullfile(root, 'results', 'synthetic_demo'));
assert(result.closed_loop_stable && result.worst_pm_deg >= cfg.min_pm_deg);
assert(result.worst_overshoot_pct <= cfg.max_overshoot_pct);
assert(~result.measured);
end
