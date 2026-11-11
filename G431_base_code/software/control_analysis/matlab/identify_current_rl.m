function result = identify_current_rl(trainingCsv, validationCsv, Vdc, outdir)
% OPEN-LOOP fixed-rotor small-signal voltage -> current records only.
% Columns: time_s,u_pu,i_a. Commands are held starting at the row timestamp.
% Feedback-loop data require a suitable closed-loop identification method.
validateattributes(Vdc, {'double'}, {'positive','finite','scalar'});
assert(~strcmp(trainingCsv,validationCsv),'Use an independent validation experiment.');
[ze,Ts] = readExperiment(trainingCsv);
[zv,Tv] = readExperiment(validationCsv);
assert(abs(Ts-Tv)/Ts < 0.01,'Sampling rates differ.');
model = procest(ze,'P1D');            % Kp*exp(-Td*s)/(1+Tp1*s)
assert(model.Kp > 0 && model.Tp1 > 0,'Nonphysical R/L; inspect polarity and data.');
[~,fit] = compare(zv,model);
result.R_ohm = Vdc/model.Kp;
result.L_h = result.R_ohm*model.Tp1;
result.fitted_delay_s = model.Td;    % Includes timing conventions; do not add ZOH twice
result.validation_fit_pct = fit;
result.Ts = Ts;
result.training_csv = trainingCsv;
result.validation_csv = validationCsv;
result.note = 'Effective fixed-frame R/L. Verify rotor, units, deadtime and residuals before tuning.';
if ~isfolder(outdir), mkdir(outdir); end
save(fullfile(outdir,'identified_rl.mat'),'model','result');
f = figure('Visible','off'); compare(zv,model);
exportgraphics(f,fullfile(outdir,'validation.png')); close(f);
f = figure('Visible','off'); resid(zv,model);
exportgraphics(f,fullfile(outdir,'residuals.png')); close(f);
disp(result)
end

function [z,Ts] = readExperiment(path)
tbl = readtable(path);
assert(all(ismember({'time_s','u_pu','i_a'},tbl.Properties.VariableNames)));
assert(height(tbl) >= 100,'At least 100 samples required.');
t = tbl.time_s; u = tbl.u_pu; y = tbl.i_a;
assert(all(isfinite([t;u;y])) && all(diff(t)>0));
Ts = median(diff(t));
assert(max(abs(diff(t)-Ts))/Ts < 0.01, ...
    'Nonuniform data: examine timestamp jitter and resample explicitly first.');
assert(std(u)>1e-6,'Insufficient voltage excitation.');
% The initial section must be a settled, constant operating point.
n = max(5,floor(height(tbl)*0.05));
assert(max(u(1:n))-min(u(1:n)) < 1e-6,'Start with a settled baseline.');
u = u-mean(u(1:n)); y = y-mean(y(1:n));
z = iddata(y,u,Ts);
z.InterSample = 'zoh';
end
