function matlab_prbs_amplitude_check
close all;set(groot,'defaultFigureVisible','off');
r=fullfile(fileparts(fileparts(mfilename('fullpath'))),'results');out=fullfile(r,'prbs_amplitude_check');if ~isfolder(out),mkdir(out);end
tags={'speed_prbs_amp_low','speed_prbs_sync_est','speed_prbs_amp_high_pilot'};
rows=struct([]);fig=figure;tiledlayout(2,1);
noise=readbin(fullfile(r,'speed_prbs_zero_noise.bin'),8);noise=noise(noise(:,1)>.5 & noise(:,1)<2,:);
fprintf('Zero-command noise: speed std %.6g rad/s, Iq std %.6g A\n',std(noise(:,3)),std(noise(:,5)));
for j=1:numel(tags)
 tag=tags{j};log=fileread(fullfile(r,[tag '.read.log']));tok=regexp(log,'\$3 = (\d+)','tokens','once');n=str2double(tok{1});
 a=readbin(fullfile(r,[tag '.bin']),3);a=a(1:n,:);b=readbin(fullfile(r,[tag '.inner.bin']),8);
 amp=max(abs(a(:,2))); keep=a(:,1)>.208;
 % Exclude edge samples. Remaining samples are NOT necessarily settled.
 phase=mod(round(b(:,1)*1000)-200,16);bk=b(:,1)>.21 & phase>=4 & phase<=12;
 q=b(bk,5);qr=b(bk,6);iqrmse=rms(q-qr);
 z=detrend(iddata(a(keep,3),a(keep,2),.008),0);g=spafdr(z,2*pi*2,2*pi*(1:.25:30));
 [coh,cf]=mscohere(z.InputData,z.OutputData,hann(128),64,512,125);
 row=struct('tag',tag,'amplitudeA',amp,'peakSpeed',max(abs(a(:,3))),...
 'speedRms',rms(z.OutputData),'fractionBelowPoint2Rad_s',mean(abs(a(keep,3))<.2),...
 'iqNonEdgeRMSE_A',iqrmse,'iqNonEdgeRelativeRMSE',iqrmse/amp,'iqNonEdgePeak_A',max(abs(q)),...
 'iqNonEdgeRegressionRatio',sum(q.*qr)/sum(qr.^2),'iqZeroCommandStd_A',std(noise(:,5)),...
 'speedNoiseStd_rad_s',std(noise(:,3)),'speedVsStandstillDb',20*log10(rms(z.OutputData)/std(noise(:,3))),...
 'recordedVoltagePeak_pu',max(abs(b(:,8))),'coherenceAtHz1_2_4_8_12_20',interp1(cf,coh,[1 2 4 8 12 20]));
 rows=[rows;row]; %#ok<AGROW>
 fprintf('%.2f A non-edge ref / measured (first 8; not guaranteed settled):\n',amp);disp([b(find(bk,8),1) qr(1:min(8,end)) q(1:min(8,end))]);
 nexttile(1);plot(a(:,1),a(:,3)/amp,'DisplayName',sprintf('%.2f A',amp));hold on;grid on;ylabel('speed / amplitude');
 nexttile(2);semilogx(g.Frequency/(2*pi),20*log10(abs(squeeze(g.ResponseData))),'DisplayName',sprintf('%.2f A',amp));hold on;grid on;ylabel('Plant dB');xlabel('Hz');
end
nexttile(1);legend;nexttile(2);legend;
exportgraphics(fig,fullfile(out,'amplitude_comparison.png'));
disp(struct2table(rows));f=fopen(fullfile(out,'summary.json'),'w');fprintf(f,'%s',jsonencode(rows,'PrettyPrint',true));fclose(f);
save(fullfile(out,'amplitude.mat'),'rows');
end
function a=readbin(path,width)
f=fopen(path,'rb');assert(f>=0);a=fread(f,[width Inf],'single=>double')';fclose(f);
end
