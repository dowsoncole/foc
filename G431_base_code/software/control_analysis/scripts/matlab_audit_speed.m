function matlab_audit_speed
% Audit saved measurements before any new physical tuning.
close all; set(groot,'defaultFigureVisible','off');
r=fullfile(fileparts(fileparts(mfilename('fullpath'))),'results');
out=fullfile(r,'matlab_method_audit'); if ~isfolder(out),mkdir(out);end
s=load(fullfile(r,'speed_prbs_matlab_aligned','matlab_identified_plant.mat'));
d=load(fullfile(r,'speed_prbs_matlab_aligned','matlab_speed_design.mat'));
fprintf('MATLAB %s\n',version); disp(ver('ident')); disp(ver('control'));
fprintf('Original selected discrete plant:\n');disp(tf(s.model));
methods={'tustin','zoh'}; rows=struct([]); f=[1 2 4 8 12 20];
for k=1:2
 g=d2c(tf(s.model),methods{k}); gd=c2d(g,.001,'zoh');
 c=pid(d.controller.Kp,d.controller.Ki,'Ts',.001,'IFormula','BackwardEuler');
 T=feedback(c*gd,1); [gm,pm]=margin(c*gd); si=stepinfo(T);
 [~,~,~,D]=ssdata(g);
 row=struct('method',methods{k},'feedthrough',D,'polesReal',real(pole(g)),'polesImag',imag(pole(g)),...
  'zerosReal',real(zero(g)),'zerosImag',imag(zero(g)),...
  'phaseMarginDeg',pm,'gainMargin',gm,'overshootPct',si.Overshoot,...
  'magnitude',abs(squeeze(freqresp(g,2*pi*f)))','phaseDeg',angle(squeeze(freqresp(g,2*pi*f)))'*180/pi);
 rows=[rows;row]; %#ok<AGROW>
end
disp(rows); fprintf('Tustin vs ZOH at 8 Hz: gain %.3f dB, phase %.3f deg\n',...
20*log10(rows(1).magnitude(4)/rows(2).magnitude(4)),rows(1).phaseDeg(4)-rows(2).phaseDeg(4));
validation=struct([]);
for j=1:numel(s.models)
 [~,fitDefault]=compare(s.zv,s.models{j});
 [~,fitZero]=compare(s.zv,s.models{j},compareOptions('InitialCondition','zero'));
 validation=[validation;struct('label',s.labels{j},'fitEstimatedIC',fitDefault,'fitZeroIC',fitZero)]; %#ok<AGROW>
end
disp(struct2table(validation));
advice(s.ze); advice(s.model,s.zv);
names={'speed_matlab_pi_step_pilot','speed_matlab_pi_step_positive','speed_matlab_pi_step_negative'};
inner=struct([]);
for j=1:numel(names)
 a=readcapture(r,names{j},8); q=a(:,5); qr=a(:,6); ii=a(:,1)>.3;
 item=struct('tag',names{j},'iqRMSE_A',rms(q(ii)-qr(ii)),...
 'iqCommandPeak_A',max(abs(qr)),'iqMeasuredPeak_A',max(abs(q)),...
 'commandSaturatedFraction',mean(abs(qr)>=.299),'uqPeak_pu',max(abs(a(:,8))),...
 'speedPeak_rad_s',max(abs(a(:,3)))); inner=[inner;item]; %#ok<AGROW>
end
disp(struct2table(inner));
ge=spafdr(s.ze); gv=spafdr(s.zv);
[ce,ff]=mscohere(s.ze.InputData,s.ze.OutputData,hann(64),32,256,125);
[cv,~]=mscohere(s.zv.InputData,s.zv.OutputData,hann(64),32,256,125);
fig=figure; tiledlayout(3,1);
fg=logspace(0,log10(40),150); w=2*pi*fg;
he=squeeze(freqresp(ge,w)); hv=squeeze(freqresp(gv,w)); hm=squeeze(freqresp(s.model,w));
nexttile; semilogx(fg,20*log10(abs(he)),fg,20*log10(abs(hv)),fg,20*log10(abs(hm)));grid on;ylabel('dB');legend('PRBS est','PRBS val','selected model');
nexttile; semilogx(fg,unwrap(angle(squeeze(freqresp(ge,w))))*180/pi,fg,unwrap(angle(squeeze(freqresp(gv,w))))*180/pi,fg,unwrap(angle(squeeze(freqresp(s.model,w))))*180/pi);grid on;ylabel('deg');
nexttile;semilogx(ff,ce,ff,cv);xlim([1 40]);ylim([0 1]);grid on;ylabel('Coherence');xlabel('Hz');
exportgraphics(fig,fullfile(out,'old_open_prbs_audit.png'));
result=struct('conversion',rows,'validation',validation,'innerLoop',inner,...
 'frequenciesHz',f,'coherenceEstAtHz',interp1(ff,ce,f),'coherenceValAtHz',interp1(ff,cv,f));
file=fopen(fullfile(out,'audit.json'),'w');fprintf(file,'%s',jsonencode(result,'PrettyPrint',true));fclose(file);
save(fullfile(out,'audit.mat'),'result','ge','gv','s','d');
end
function a=readcapture(r,tag,width)
log=fileread(fullfile(r,[tag '.read.log']));tok=regexp(log,'\$3 = (\d+)','tokens','once');n=str2double(tok{1});
file=fopen(fullfile(r,[tag '.bin']),'rb');a=fread(file,[width Inf],'single=>double')';fclose(file);a=a(1:n,:);
end
