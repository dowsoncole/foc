function matlab_retry_speed(dataset)
% Fresh PRBS identification, validation first. Never deploy from fit alone.
if nargin<1,dataset='speed_prbs_retry';end
close all;set(groot,'defaultFigureVisible','off');
r=fullfile(fileparts(fileparts(mfilename('fullpath'))),'results');
out=fullfile(r,[dataset '_analysis']);if ~isfolder(out),mkdir(out);end
[ze,ae,repeatE,inputRepeatE]=capture(r,[dataset '_est']);[zv,av,repeatV,inputRepeatV]=capture(r,[dataset '_val']);
fprintf('Period repeatability, est %.2f%% val %.2f%%\n',repeatE,repeatV);
f=(1:0.25:30)';w=2*pi*f;
ge=spafdr(ze,2*pi*2,w);gv=spafdr(zv,2*pi*2,w);
[ce,cf]=mscohere(ze.InputData,ze.OutputData,hann(254),127,1024,125);
[cv,~]=mscohere(zv.InputData,zv.OutputData,hann(254),127,1024,125);
opt=tfestOptions('Display','off','EnforceStability',true,'InitialCondition','estimate');
models={};labels={}; rows=struct([]);
for np=1:4
 for nz=0:min(np-1,1)
  try
   m=tfest(ze,np,nz,opt); addmodel(m,sprintf('CT TF %d/%d',np,nz));
  catch err
   fprintf('TF rejected: %s\n',err.message);
  end
 end
end
for np=[2 3 4 5 6 8]
 try
  op=ssestOptions('Display','off','EnforceStability',true);
  m=ssest(ze,np,'Ts',ze.Ts,'Feedthrough',false,op);addmodel(m,sprintf('DT SS %d',np));
 catch err
  fprintf('SS rejected: %s\n',err.message);
 end
 if np<=4
  try
   m=oe(ze,[np np 1]);addmodel(m,sprintf('DT OE %d',np));
  catch err
   disp(err.message);
  end
 end
end
assert(~isempty(rows));disp(struct2table(rows));
[~,best]=max([rows.validationFit]);model=models{best};
fig=figure;compare(zv,model);exportgraphics(fig,fullfile(out,'validation.png'));
fig=figure;resid(zv,model);exportgraphics(fig,fullfile(out,'residual.png'));
fig=figure;tiledlayout(2,1);nexttile;plot(ae(:,1),ae(:,2),av(:,1),av(:,2));grid on;ylabel('A');
nexttile;plot(ae(:,1),ae(:,3),av(:,1),av(:,3));grid on;ylabel('rad/s');xlabel('s');exportgraphics(fig,fullfile(out,'raw_prbs.png'));
he=squeeze(freqresp(ge,w));hv=squeeze(freqresp(gv,w));hm=squeeze(freqresp(model,w));
fig=figure;tiledlayout(3,1);nexttile;semilogx(f,20*log10(abs([he hv hm])));grid on;ylabel('dB');legend('est spectral','validation spectral','model');
nexttile;semilogx(f,unwrap(angle([he hv hm]))*180/pi);grid on;ylabel('deg');
nexttile;semilogx(cf,ce,cf,cv);grid on;xlim([1 30]);ylim([0 1]);ylabel('coherence');xlabel('Hz');exportgraphics(fig,fullfile(out,'frequency_validation.png'));
% Engineering gate, deliberately not claimed to be a MathWorks default.
% Both run coherence and repeatability are checked before any tuning.
band=f>=2 & f<=10; coherent=min(interp1(cf,ce,f),interp1(cf,cv,f));
gainError=20*log10(abs(hm./hv));phaseError=angle(hm./hv)*180/pi;
gate=inputRepeatE==1 && inputRepeatV==1 && rows(best).validationFit>=70 && repeatE>=70 && repeatV>=70 && ...
 mean(coherent(band)>=.7)>=.8 && median(abs(gainError(band)))<=3 && median(abs(phaseError(band)))<=20;
summary=struct('selected',labels{best},'models',rows,'repeatabilityEstPct',repeatE,...
 'repeatabilityValPct',repeatV,'inputRepeatEst',inputRepeatE,'inputRepeatVal',inputRepeatV,...
 'gatePassed',gate,'medianGainErrorDb',median(abs(gainError(band))),...
 'medianPhaseErrorDeg',median(abs(phaseError(band))),'coherentFraction2to10Hz',mean(coherent(band)>=.7),...
 'estPeakSpeed',max(abs(ae(:,3))),'valPeakSpeed',max(abs(av(:,3))),...
 'note','No tuning/deployment unless repeatability, independent simulation fit and frequency checks all pass.');
save(fullfile(out,'identification.mat'),'ze','zv','model','models','rows','labels','summary','ge','gv','ce','cv','cf');
writejson(fullfile(out,'summary.json'),summary);disp(summary);
if ~gate,fprintf('MODEL VALIDATION FAILED: no PI exported.\n');return;end
% ZOH input model -> 1 ms digital PI matching firmware update-before-output.
g=tf(model);if g.Ts>0,g=d2c(g,'zoh');end
gd=c2d(g,.001,'zoh');c0=pid(1,1,'Ts',.001,'IFormula','BackwardEuler');
[controller,info]=pidtune(gd,c0,2*pi*2,pidtuneOptions('PhaseMargin',75));
assert(controller.Kp>=0&&controller.Kp<=.2&&controller.Ki>=0&&controller.Ki<=2);
T=feedback(controller*gd,1);[gm,pm]=margin(controller*gd);si=stepinfo(T);
assert(isstable(T)&&pm>=60&&gm>=2&&si.Overshoot<=20,'Candidate fails model performance gate');
candidate=struct('kp',controller.Kp,'ki',controller.Ki,'targetCrossoverHz',2,'info',info,...
 'predictedPhaseMargin',pm,'predictedGainMargin',gm,'predictedStepinfo',si);
writejson(fullfile(out,'candidate.json'),candidate);save(fullfile(out,'design.mat'),'controller','gd','info','candidate');
 function addmodel(m,label)
  if ~isstable(m)||dcgain(m)<=0
   fprintf('%s rejected: stable=%d, DC gain %.6g\n',label,isstable(m),dcgain(m));return;
  end
  [~,fit]=compare(zv,m,Inf,compareOptions('InitialCondition','estimate'));
  [~,fit0]=compare(zv,m,Inf,compareOptions('InitialCondition','zero'));
  models{end+1}=m;labels{end+1}=label;
  rows=[rows;struct('label',label,'validationFit',fit,'zeroICFit',fit0,'dcGain',dcgain(m))];
  fprintf('%s validation %.2f%%, zero IC %.2f%%\n',label,fit,fit0);
 end
end
function [z,a,repeat,inputRepeat]=capture(r,tag)
log=fileread(fullfile(r,[tag '.read.log']));assert(~isempty(regexp(log,'\$1 = 3\s','once')));
tok=regexp(log,'\$3 = (\d+)','tokens','once');n=str2double(tok{1});
file=fopen(fullfile(r,[tag '.bin']),'rb');a=fread(file,[3 Inf],'single=>double')';fclose(file);a=a(1:n,:);
% Discard complete first PRBS period; retain exactly four full repeated periods.
t=(.208+2.032+(0:1015)*.008)';idx=interp1(a(:,1),(1:n)',t,'nearest');
assert(all(isfinite(idx))&&max(abs(a(idx,1)-t))<.001);
u=a(idx,2);y=a(idx,3);up=reshape(u,254,4);yp=reshape(y,254,4);
inputRepeat=mean(abs(up(:,2:end)-up(:,1))<1e-6,'all');
fprintf('%s input repeat match %.4f%%\n',tag,100*inputRepeat);
repeat=100*(1-norm(yp-mean(yp,2),'fro')/norm(yp-mean(yp,'all'),'fro'));
z=detrend(iddata(y,u,.008,'InterSample','zoh','InputName','Iq command','OutputName','encoder speed'),0);
end
function writejson(path,x)
file=fopen(path,'w');assert(file>=0);fprintf(file,'%s',jsonencode(x,'PrettyPrint',true));fclose(file);
end
