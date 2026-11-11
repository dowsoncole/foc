function matlab_identify_speed(dataset,tuningOnly)
warning('Historical pipeline: its selected PI failed hardware validation. For current re-identification use matlab_retry_speed; do not deploy these legacy gains.');
% Real PRBS -> System Identification Toolbox -> Control System Toolbox.
% No manual or Python model fitting. Measurements and predictions stay separate.
if nargin<1, dataset='speed_prbs'; end
if nargin<2, tuningOnly=false; end
close all; set(groot,'defaultFigureVisible','off');
r=fullfile(fileparts(fileparts(mfilename('fullpath'))),'results');
[ze,rawE]=loadprbs(r,[dataset '_est']);
[zv,rawV]=loadprbs(r,[dataset '_val']);
r=fullfile(r,[dataset '_matlab_aligned']); if ~isfolder(r), mkdir(r); end
fprintf('Estimation samples %d, validation samples %d, Ts %.6f s\n',size(ze,1),size(zv,1),ze.Ts);
if ~tuningOnly
fig=figure; tiledlayout(2,1); nexttile; plot(rawE(:,1),rawE(:,2)); ylabel('Iq command (A)'); grid on;
nexttile; plot(rawE(:,1),rawE(:,3)); ylabel('Encoder speed (rad/s)'); xlabel('DWT time (s)'); grid on;
exportgraphics(fig,fullfile(r,'matlab_prbs_measured.png'));
opt=tfestOptions('Display','off','EnforceStability',true);
models={}; fits=[]; orders=[]; labels={};
for np=1:4
 for nz=0:min(np-1,1)
  try
   m=tfest(ze,np,nz,opt);
   [~,fit]=compare(zv,m);
   fprintf('np=%d nz=%d validation NRMSE fit=%.3f%%, dcgain=%.4g\n',np,nz,fit,dcgain(m));
   if isfinite(fit) && isstable(m) && dcgain(m)>0
    models{end+1}=m; fits(end+1)=fit; orders(end+1,:)=[np nz]; %#ok<SAGROW>
    labels{end+1}=sprintf('continuous TF %d/%d',np,nz);
   end
  catch err
   fprintf('Model np=%d nz=%d rejected: %s\n',np,nz,err.message);
  end
 end
end
% User-validated demo_coder workflow: discrete TF, ARX, OE and spectral FRF.
for kind=1:3
 for order=2:4
  try
   if kind==1
    m=tfest(ze,order,order-1,'Ts',ze.Ts,opt); label=sprintf('discrete TF %d',order);
   elseif kind==2
    m=arx(ze,[order order 1]); label=sprintf('ARX %d',order);
   else
    m=oe(ze,[order order 1]); label=sprintf('OE %d',order);
   end
   [~,fit]=compare(zv,m);
   fprintf('%s validation NRMSE fit=%.3f%%, dcgain=%.4g\n',label,fit,dcgain(m));
   if isfinite(fit) && isstable(m) && dcgain(m)>0
    models{end+1}=m; fits(end+1)=fit; orders(end+1,:)=[order -kind]; labels{end+1}=label; %#ok<SAGROW>
   end
  catch err
   fprintf('Discrete model rejected: %s\n',err.message);
  end
 end
end
spectral=spafdr(ze); spectralValidation=spafdr(zv);
fig=figure; bopt=bodeoptions; bopt.FreqUnits='Hz'; bode(spectral,spectralValidation,bopt); grid on;
legend('Estimation run spafdr','Validation run spafdr'); exportgraphics(fig,fullfile(r,'matlab_spafdr.png'));
[coh,freq]=mscohere(ze.InputData,ze.OutputData,hann(64),32,256,1/ze.Ts);
fig=figure; semilogx(freq,coh); ylim([0 1]); xlim([1 60]); grid on;
xlabel('Frequency (Hz)'); ylabel('Magnitude-squared coherence'); exportgraphics(fig,fullfile(r,'matlab_coherence.png'));
save(fullfile(r,'matlab_spectral_data.mat'),'spectral','spectralValidation','coh','freq');
assert(~isempty(models),'No valid stable positive-gain model. Do not deploy gains.');
[bestFit,idx]=max(fits); model=models{idx}; G=ctplant(model);
disp(G);
fig=figure; compare(zv,model); exportgraphics(fig,fullfile(r,'matlab_model_validation.png'));
fig=figure; resid(zv,model); exportgraphics(fig,fullfile(r,'matlab_model_residuals.png'));
fig=figure; bode(model,{2*pi*.5,2*pi*40}); grid on; exportgraphics(fig,fullfile(r,'matlab_plant_bode.png'));
save(fullfile(r,'matlab_identified_plant.mat'),'ze','zv','rawE','rawV','models','fits','orders','labels','model','G');
summary=struct('validationFitPct',bestFit,'orders',orders,'fits',fits,'selectedOrder',orders(idx,:),...
 'input','Iq command A, speed PI bypassed, current loop retained',...
 'output','Filtered encoder mechanical speed rad/s',...
 'samplePeriod_s',ze.Ts,'selectedModel',labels{idx},'modelNumerator',G.Numerator{1},'modelDenominator',G.Denominator{1},...
 'validationNote','Separate physical run with shifted PRBS; compare estimates initial conditions; same low-speed operating envelope');
writejson(fullfile(r,'matlab_identification_summary.json'),summary);
else
 load(fullfile(r,'matlab_identified_plant.mat'),'models','fits','orders','labels','model','G');
 bestFit=max(fits);
end
assert(bestFit>=50,'Validation fit below provisional 50%% gate. Do not tune/deploy an unreliable model.');
% Search model-based target crossovers, NOT physical gain sweeps.
options=pidtuneOptions('PhaseMargin',65,'DesignFocus','reference-tracking');
candidates=struct([]); bestScore=inf; chosen=[];
for hz=[.5 .8 1 1.5 2 3 4 6 8]
 try
  [C,info]=pidtune(G,'PI',2*pi*hz,options);
  if C.Kp<0 || C.Kp>.2 || C.Ki<0 || C.Ki>2, continue; end
  T=feedback(C*G,1); if ~isstable(T), continue; end
  [gm,pm,~,wcp]=margin(C*G); si=stepinfo(T);
  y=step(T,(0:.002:4)'); score=trapz((0:.002:4)',abs(1-y));
  % Reject unstable alternatives among models with comparable validation fit.
  robust=true;
  for j=find(fits>=bestFit-10)
   if ~isstable(feedback(C*ctplant(models{j}),1)), robust=false; end
  end
  item=struct('targetCrossoverHz',hz,'kp',C.Kp,'ki',C.Ki,'predictedIAE',score,...
   'predictedOvershootPct',si.Overshoot,'predictedSettling_s',si.SettlingTime,...
   'predictedBandwidthHz',bandwidth(T)/(2*pi),'phaseMarginDeg',pm,...
   'gainMarginFactor',gm,'actualCrossoverHz',wcp/(2*pi),'stableOnAlternativeModels',robust);
  if isempty(candidates), candidates=item; else, candidates(end+1)=item; end %#ok<SAGROW>
  if robust && pm>=60 && si.Overshoot<=20 && score<bestScore
   bestScore=score; chosen=item; controller=C; closedLoop=T; tuningInfo=info;
  end
 catch err
  fprintf('Target %.2f Hz rejected: %s\n',hz,getReport(err,'extended'));
 end
end
writejson(fullfile(r,'matlab_pi_candidates.json'),candidates);
assert(~isempty(chosen),'No PI satisfies provisional robustness and firmware limits. Do not deploy.');
writejson(fullfile(r,'matlab_pi_selected.json'),chosen);
save(fullfile(r,'matlab_speed_design.mat'),'model','G','controller','closedLoop','tuningInfo','chosen','candidates');
fig=figure; step(closedLoop,4); grid on; title('MODEL PREDICTION: PI closed-loop unit step');
exportgraphics(fig,fullfile(r,'matlab_predicted_step.png'));
fig=figure; margin(controller*G); grid on; title('MODEL PREDICTION: open-loop margins');
exportgraphics(fig,fullfile(r,'matlab_predicted_margins.png'));
fig=figure; bode(closedLoop,{2*pi*.2,2*pi*40}); grid on; title('MODEL PREDICTION: closed-loop response');
exportgraphics(fig,fullfile(r,'matlab_predicted_closed_bode.png'));
disp(chosen);
end

function [z,a]=loadprbs(r,tag)
 log=fileread(fullfile(r,[tag '.read.log']));
 assert(~isempty(regexp(log,'\$1 = 3\s','once')),'Capture did not complete');
 tok=regexp(log,'\$3 = (\d+)','tokens','once'); n=str2double(tok{1});
 f=fopen(fullfile(r,[tag '.bin']),'rb'); assert(f>=0); a=fread(f,[3 Inf],'single=>double')'; fclose(f); a=a(1:n,:);
 assert(all(isfinite(a),'all') && all(diff(a(:,1))>0));
 assert(numel(unique(a(a(:,1)>.21,2)))==2,'Excitation is not bipolar PRBS');
 % Samples have <0.1 ms positive timestamp jitter around the 8 ms grid.
 % 'previous' at nominal grid times would shift the command by an entire sample.
 % Snap BOTH measured channels to their nearest common nominal sample instead.
 t=(.216:.008:a(end,1))'; idx=interp1(a(:,1),(1:n)',t,'nearest');
 assert(max(abs(a(idx,1)-t))<.001,'Sample gap too large to snap safely');
 u=a(idx,2); y=a(idx,3);
 z=iddata(y,u,.008,'TimeUnit','seconds','InputName','Iq command','OutputName','encoder speed');
 z=detrend(z,0); % remove each run's measured DC offsets; no invented torque constant
 fprintf('%s: peak speed %.3f rad/s, sample interval %.6f..%.6f s\n',tag,max(abs(a(:,3))),min(diff(a(:,1))),max(diff(a(:,1))));
end
function writejson(path,value)
 f=fopen(path,'w','n','UTF-8'); assert(f>=0); fprintf(f,'%s\n',jsonencode(value,'PrettyPrint',true)); fclose(f);
end
function g=ctplant(m)
 g=tf(m);
 if g.Ts>0, g=d2c(g,'tustin'); end
end
