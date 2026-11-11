function matlab_tune_speed_response(rootdir,tau_s,stem,pm_target,prefix)
% Measured closed-loop FRF and externally excited plant FRF. Not a claim that
% fitted open-loop margins have been independently measured on the motor.
set(groot,'defaultFigureVisible','off');
if nargin<2,tau_s=.01;end
if nargin<3,stem='matlab_design';end
if nargin<4,pm_target=65;end
if nargin<5,prefix='baseline';end
files=dir(fullfile(rootdir,[prefix '_f*'],'summary.json'));
assert(numel(files)>=5,'Need at least five measured frequency points');
a=struct([]);
for k=1:numel(files)
    v=jsondecode(fileread(fullfile(files(k).folder,files(k).name)));
    assert(v.saturated_rows==0 && v.measured_cycles>=4,'Unqualified experiment');
    a=[a;v]; %#ok<AGROW>
end
[f,order]=sort([a.frequency_hz]); a=a(order);
plant=[a.plant_real]+1i*[a.plant_imag];
% Explicit design-only replacement of the KNOWN speed-estimator pole.
% This is a prediction until fresh firmware measurements validate it.
source_tau_s=a(1).estimator_tau_ms/1000;
assert(all([a.estimator_tau_ms]==a(1).estimator_tau_ms));
plant=plant.*(1+1i*2*pi*f*source_tau_s)./(1+1i*2*pi*f*tau_s);
closed=10.^([a.gain_db]/20).*exp(1i*deg2rad([a.phase_deg]));
data=idfrd(reshape(plant,1,1,[]),2*pi*f,0,'FrequencyUnit','rad/s');
data.InputName={'Iq reference A'}; data.OutputName={'Filtered speed rad/s'};
% A current command -> filtered speed fit includes current-loop dynamics and
% speed-estimator delay. Closed-loop external excitation supplies an
% instrumental FRF ratio; no independent white-noise Iq assumption is made.
opt=tfestOptions('EnforceStability',true,'Display','off');
relative=Inf;
for orders=[2 0;3 1;4 2].'
    trial=tfest(data,orders(1),orders(2),opt);
    pred=reshape(freqresp(trial,2*pi*f),1,[]);
    err=norm((pred-plant)./max(abs(plant),1))/sqrt(numel(f));
    if err<relative, model=trial; relative=err; end
end
measuredPlant=frd(reshape(plant,1,1,[]),2*pi*f);
opts=pidtuneOptions('PhaseMargin',pm_target,'DesignFocus','reference-tracking','NumUnstablePoles',0);
designs=struct([]);
if tau_s<.01,targets=[10 15 20 25 30 40 50 60];else,targets=[5 8 10 12 15 20];end
for hz=targets
    [controller,info]=pidtune(measuredPlant,'PI',2*pi*hz,opts);
    loop=controller*model; cl=feedback(loop,1);
    [gm,pm,wcg,wcp]=margin(loop);
    si=stepinfo(cl,'SettlingTimeThreshold',.02);
    d=struct('requested_crossover_hz',hz,'kp',controller.Kp,'ki',controller.Ki,...
        'stable',isstable(cl),'phase_margin_deg',pm,'gain_margin_db',20*log10(gm),...
        'gain_crossover_hz',wcp/(2*pi),'phase_crossover_hz',wcg/(2*pi),...
        'predicted_rise_s',si.RiseTime,'predicted_settling_s',si.SettlingTime,...
        'predicted_overshoot_pct',si.Overshoot,'pidtune_achieved_pm',info.PhaseMargin,...
        'candidate_inside_initial_test_envelope',controller.Kp>=.001 && controller.Kp<=.2 &&...
        controller.Ki>=0 && controller.Ki<=2);
    designs=[designs;d]; %#ok<AGROW>
end
out=struct('source','Measured external sine excitation; fit-based predictions only',...
    'frequency_hz',f,'measured_closed_gain_db',[a.gain_db],...
    'measured_closed_phase_deg',[a.phase_deg],...
    'plant_real',real(plant),'plant_imag',imag(plant),...
    'fit_relative_complex_error',relative,'estimator_tau_s',tau_s,...
    'source_prefix',prefix,'source_estimator_tau_s',source_tau_s,'candidates',designs);
fid=fopen(fullfile(rootdir,[stem '.json']),'w');
cleanup=onCleanup(@() fclose(fid));
fwrite(fid,jsonencode(out,PrettyPrint=true),'char'); clear cleanup
save(fullfile(rootdir,[stem '.mat']),'model','data','designs','closed','f');
fig=figure; tiledlayout(2,1);
nexttile; semilogx(f,[a.gain_db],'-o'); ylabel('Closed-loop dB'); grid on;
nexttile; semilogx(f,rad2deg(unwrap(angle(closed))),'-o');
ylabel('Closed-loop phase deg'); xlabel('Hz'); grid on;
exportgraphics(fig,fullfile(rootdir,[stem '_closed.png'])); close(fig);
fig=figure; bode(data,model); grid on;
exportgraphics(fig,fullfile(rootdir,[stem '_plant_fit.png'])); close(fig);
disp(out); disp(struct2table(designs));
end
