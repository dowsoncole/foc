function selftest_current_pi()
% Reproducible synthetic-only tests; requires Control + Identification toolboxes.
root = fileparts(fileparts(mfilename('fullpath')));
out = fullfile(root,'results','synthetic_identification');
if ~isfolder(out), mkdir(out); end
Ts = 50e-6; time_s = (0:Ts:0.05)';
G = c2d(tf(24,[1e-3 0.5]),Ts,'zoh');
u_pu = zeros(size(time_s));
u_pu(time_s>=0.005 & time_s<0.025) = 0.002;
i_a = lsim(G,u_pu,time_s);
writetable(table(time_s,u_pu,i_a),fullfile(out,'train.csv'));
u_pu = zeros(size(time_s));
u_pu(time_s>=0.008 & time_s<0.03) = -0.0015;
i_a = lsim(G,u_pu,time_s);
writetable(table(time_s,u_pu,i_a),fullfile(out,'validation.csv'));
r = identify_current_rl(fullfile(out,'train.csv'),fullfile(out,'validation.csv'),24,out);
assert(abs(r.R_ohm/0.5-1)<0.05 && abs(r.L_h/0.001-1)<0.05);
assert(r.validation_fit_pct>95);
% Test the documented MCU CSV layout and reject a dropped sample.
source_sequence = (0:numel(time_s)-1)';
d_reference_a = 0.05*double(time_s>=0.005);
d_current_a = lsim(c2d(tf(1000,[1 1000]),Ts,'zoh'),d_reference_a,time_s);
d_voltage_pu = 0.01*d_reference_a;
capture = table(source_sequence,time_s,d_reference_a,d_current_a,d_voltage_pu);
writetable(capture,fullfile(out,'capture.csv'));
m = evaluate_current_capture(fullfile(out,'capture.csv'),"d",out);
assert(m.SettlingTime>0 && m.SettlingTime<0.01);
capture(20,:) = [];
writetable(capture,fullfile(out,'capture_dropped.csv'));
rejected = false;
try
    evaluate_current_capture(fullfile(out,'capture_dropped.csv'),"d",out);
catch
    rejected = true;
end
assert(rejected,'Dropped capture sample must be rejected.');
run_current_pi_demo();
cfg = current_pi_config();
failed = false;
try
    tune_current_pi(cfg,fullfile(root,'results','invalid_input'));
catch
    failed = true;
end
assert(failed,'Unknown motor must be rejected.');
fprintf('PASS: synthetic identification, robust PI search, discrete equivalence, saturation recovery, unknown-motor rejection.\n');
end
