function cfg = current_pi_config()
% Unknown motor parameters deliberately remain NaN. SI units throughout.
cfg.R = NaN;                         % Ohm, equivalent phase resistance
cfg.L = NaN;                         % H, fixed-frame effective inductance
cfg.Vdc = NaN;                       % V, measured bus voltage
cfg.measured = false;                % Set true only with measured R/L/Vdc
cfg.Ts = 50e-6;
cfg.extra_delay_s = 50e-6;           % Additional command delay, EXCLUDES ZOH
cfg.voltage_limit_pu = 0.02;         % Matches commissioning mode
cfg.integral_limit_pu = 0.01;
cfg.step_a = 0.05;                  % Single-axis test; other axis reference = 0
cfg.duration_s = 0.05;
cfg.min_pm_deg = 55;
cfg.min_gm_db = 6;
cfg.max_overshoot_pct = 10;
cfg.max_settling_s = 0.02;
cfg.R_factors = [0.8 1.2];           % Provisional uncertainty, replace after measurement
cfg.L_factors = [0.8 1.2];
cfg.V_factors = [0.9 1.1];
cfg.fc_hz = logspace(log10(30), log10(800), 24);
cfg.ki_factors = [0.5 0.75 1 1.25 1.5];
end
