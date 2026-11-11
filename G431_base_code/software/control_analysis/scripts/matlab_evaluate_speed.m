function matlab_evaluate_speed
% Final measured evaluation with MATLAB tools, not model predictions disguised as tests.
close all; set(groot,'defaultFigureVisible','off');
r=fullfile(fileparts(fileparts(mfilename('fullpath'))),'results');
design=load(fullfile(r,'speed_prbs_matlab_aligned','matlab_speed_design.mat'));
outdir=fullfile(r,'matlab_measured_evaluation'); if ~isfolder(outdir),mkdir(outdir);end
names={'speed_matlab_pi_step_pilot','speed_matlab_pi_step_positive','speed_matlab_pi_step_negative'};
metrics=struct([]); fig=figure; tiledlayout(3,1);
for k=1:numel(names)
 a=readcapture(r,names{k},8); t=a(:,1); ref=a(:,2); y=a(:,3); target=ref(end);
 keep=t>=.2; tail=(length(t)-floor(length(t)/4)+1):length(t);
 si=stepinfo(y(keep),t(keep)-.2,target,'SettlingTimeThreshold',.025);
 band=max(.02*abs(target),.05); bad=find(keep & abs(y-target)>band); settle=NaN;
 if isempty(bad),settle=0;
 elseif bad(end)<length(t) && t(end)-t(bad(end)+1)>=.5,settle=t(bad(end)+1)-.2;end
 m=struct('tag',names{k},'target_rad_s',target,'mean_rad_s',mean(y(tail)),...
 'std_rad_s',std(y(tail),1),'meanError_rad_s',target-mean(y(tail)),...
 'IAE_rad',trapz(t,abs(ref-y)),'matlabStepinfo',si,'qualifiedSettling_s',settle,...
 'settlingBand_rad_s',band,'minimumHold_s',.5,'samplePeriod_s',median(diff(t)));
 if isempty(metrics),metrics=m;else,metrics(end+1)=m;end %#ok<AGROW>
 nexttile; plot(t,ref,'--',t,y); grid on; ylabel('rad/s'); title(names{k},'Interpreter','none');
end
xlabel('Measured DWT time (s)'); exportgraphics(fig,fullfile(outdir,'measured_steps.png'));
writejson(fullfile(outdir,'measured_step_metrics.json'),metrics);
a=readcapture(r,'speed_matlab_pi_closed_prbs',3);
% Drop first second to exclude startup from the closed-loop spectral estimate.
t=(1:.008:a(end,1))'; idx=interp1(a(:,1),(1:size(a,1))',t,'nearest');
assert(max(abs(a(idx,1)-t))<.001);
z=iddata(a(idx,3),a(idx,2),.008,'TimeUnit','seconds','InputName','speed reference','OutputName','speed feedback');
z=detrend(z,0); g=spafdr(z);
[coh,cf]=mscohere(z.InputData,z.OutputData,hann(256),128,1024,125);
f=logspace(log10(.5),log10(40),160); w=2*pi*f;
[mag,phase]=bode(g,w); mag=squeeze(mag); phase=squeeze(phase);
[predMag,predPhase]=bode(design.closedLoop,w); predMag=squeeze(predMag); predPhase=squeeze(predPhase);
phase=rad2deg(unwrap(deg2rad(phase))); phase=phase-360*round(phase(1)/360);
predPhase=rad2deg(unwrap(deg2rad(predPhase))); predPhase=predPhase-360*round(predPhase(1)/360);
cohGrid=interp1(cf,coh,f,'linear'); db=20*log10(mag);
% A local empirical -3 dB bracket, qualified by coherence at both neighboring points.
cross=find(db(1:end-1)>-3 & db(2:end)<=-3 & cohGrid(1:end-1)'>.6 & cohGrid(2:end)'>.6,1);
bracket=[NaN NaN]; if ~isempty(cross),bracket=f([cross cross+1]);end
toolBandwidth=NaN; bwMessage='';
try,toolBandwidth=bandwidth(g,-3)/(2*pi);catch err,bwMessage=err.message;end
[peakDb,peakIndex]=max(db);
freqSummary=struct('method','MATLAB spafdr on actual closed-loop PRBS; includes encoder filter',...
 'qualifiedMinus3dBBracketHz',bracket,'matlabBandwidthHz',toolBandwidth,'bandwidthToolMessage',bwMessage,...
 'measuredPeakGainDb',peakDb,'measuredPeakFrequencyHz',f(peakIndex),...
 'lowestSpectralFrequencyHz',g.Frequency(1)/(2*pi),'lowestSpectralGainDb',20*log10(abs(g.ResponseData(1))),...
 'duration_s',t(end)-t(1),'samplePeriod_s',.008,'bias_rad_s',2,'amplitude_rad_s',.5,...
 'note','Coherence threshold .6 is an engineering screening rule. Finite-data spectral estimate, not exact continuous sweep.');
writejson(fullfile(outdir,'measured_frequency_metrics.json'),freqSummary);
fig=figure; tiledlayout(3,1);
nexttile; semilogx(f,db,f,20*log10(predMag),'--'); xlim([.5 40]); yline(-3,':'); ylabel('Gain (dB)');grid on;legend('Measured spafdr','Model prediction');
nexttile; semilogx(f,phase,f,predPhase,'--'); xlim([.5 40]); ylabel('Phase (deg)');grid on;
nexttile; semilogx(cf,coh);yline(.6,':');xlim([.5 40]);ylim([0 1]);ylabel('Coherence');xlabel('Hz');grid on;
exportgraphics(fig,fullfile(outdir,'measured_vs_predicted_frequency.png'));
fig=figure; plot(a(:,1),a(:,2),'--',a(:,1),a(:,3)); grid on; xlabel('DWT time (s)');ylabel('rad/s');legend('PRBS reference','Measured speed');
exportgraphics(fig,fullfile(outdir,'measured_closed_prbs.png'));
save(fullfile(outdir,'matlab_measured_evaluation.mat'),'metrics','z','g','cf','coh','freqSummary','a','design');
disp(metrics);disp(freqSummary);
end
function a=readcapture(r,tag,width)
 log=fileread(fullfile(r,[tag '.read.log'])); assert(~isempty(regexp(log,'\$1 = 3\s','once')));
 tok=regexp(log,'\$3 = (\d+)','tokens','once');n=str2double(tok{1});
 f=fopen(fullfile(r,[tag '.bin']),'rb');assert(f>=0);a=fread(f,[width Inf],'single=>double')';fclose(f);a=a(1:n,:);
 assert(all(isfinite(a),'all')&&all(diff(a(:,1))>0));
end
function writejson(path,value)
 f=fopen(path,'w','n','UTF-8');assert(f>=0);fprintf(f,'%s\n',jsonencode(value,'PrettyPrint',true));fclose(f);
end
