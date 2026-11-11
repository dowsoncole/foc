function matlab_evaluate_speed_response(rootdir,selected,prefix,stem)
% MATLAB time/frequency evaluation of measured records, not a simulated motor.
set(groot,'defaultFigureVisible','off');
if nargin<3,prefix='final';end
if nargin<4,stem='';end
if iscell(selected),tags=[{'baseline_step_fenced'} selected];
else,tags={'baseline_step_fenced',selected};end
metrics=struct([]); fig=figure('Position',[100 100 1000 270*numel(tags)]); tiledlayout(numel(tags),1);
for k=1:numel(tags)
    folder=fullfile(rootdir,tags{k});
    summary=jsondecode(fileread(fullfile(folder,'summary.json')));
    fid=fopen(fullfile(folder,'samples.bin'),'rb'); assert(fid>=0);
    rows=fread(fid,[8 Inf],'single=>double').'; fclose(fid);
    t=rows(:,2); applied=rows(:,4); y=rows(:,5);
    base=mean(y(t>=summary.delay_s-.08 & t<summary.delay_s));
    keep=t>=summary.delay_s;
    measured=stepinfo(y(keep)-base,t(keep)-summary.delay_s,...
        applied(end)-base,'SettlingTimeThreshold',.02,'RiseTimeLimits',[.1 .9]);
    m=struct('tag',tags{k},'matlab_stepinfo',measured,...
        'qualified_metrics',summary,'speed_is_encoder_filtered',true);
    metrics=[metrics;m]; %#ok<AGROW>
    nexttile; plot(t,applied,'--',t,y); grid on; ylabel('Motor rpm');
    title(tags{k},'Interpreter','none');
end
xlabel('MCU elapsed seconds');
exportgraphics(fig,fullfile(rootdir,[stem 'measured_steps_matlab.png'])); close(fig);
freqs={dir(fullfile(rootdir,'baseline_f*','summary.json')),...
       dir(fullfile(rootdir,[prefix '_f*'],'summary.json'))};
labels={'baseline PI 0.032/1, tau 10 ms',prefix};
spectra=struct([]); fig=figure; tiledlayout(2,1);
for k=1:2
    files=freqs{k}; if isempty(files),continue;end
    s=struct([]);
    for n=1:numel(files),s=[s;jsondecode(fileread(fullfile(files(n).folder,files(n).name)))];end %#ok<AGROW>
    [f,index]=sort([s.frequency_hz]);s=s(index);
    h=10.^([s.gain_db]/20).*exp(1i*deg2rad([s.phase_deg]));
    measured=frd(reshape(h,1,1,[]),2*pi*f);
    [mag,phase]=bode(measured,2*pi*f);
    db=20*log10(reshape(mag,1,[]));
    unwrapped=rad2deg(unwrap(deg2rad(reshape(phase,1,[]))));
    index=find(db(1:end-1)>-3 & db(2:end)<=-3,1);
    bracket=[];crossing=[];
    if ~isempty(index)
        bracket=f(index:index+1);
        crossing=exp(interp1(db(index:index+1),log(bracket),-3));
    end
    crossings=struct([]);
    for n=1:numel(f)-1
        if (db(n)+3)*(db(n+1)+3)<=0 && db(n)~=db(n+1)
            crossings=[crossings;struct('bracket_hz',f(n:n+1),...
                'interpolated_hz',exp(interp1(db(n:n+1),log(f(n:n+1)),-3)),...
                'downward',db(n+1)<db(n))]; %#ok<AGROW>
        end
    end
    spectra=[spectra;struct('dataset',k,'label',labels{k},'frequency_hz',f,'gain_db',db,...
        'unwrapped_phase_deg',unwrapped,'bandwidth_bracket_hz',bracket,...
        'interpolated_bandwidth_hz',crossing,'all_minus3_crossings',crossings,...
        'peak_gain_db',max(db))]; %#ok<AGROW>
    if k==2
        gp=[s.plant_real]+1i*[s.plant_imag];
        controller=pid(s(1).kp,s(1).ki);
        estimatedLoop=controller*frd(reshape(gp,1,1,[]),2*pi*f);
        [gm,pm,wcg,wcp]=margin(estimatedLoop);
        loop=gp.*(s(1).kp+s(1).ki./(1i*2*pi*f));
        ss=1./(1+loop);
        open=struct('source','Derived from measured Iq-reference FRF; not an independently opened-loop experiment',...
            'phase_margin_deg',pm,'gain_margin_db',20*log10(gm),...
            'gain_crossover_hz',wcp/(2*pi),'phase_crossover_hz',wcg/(2*pi),...
            'sampled_sensitivity_peak',max(abs(ss)),...
            'sampled_complementary_peak',max(abs(1-ss)),...
            'measured_frequency_range_hz',[min(f) max(f)]);
        figOpen=figure; margin(estimatedLoop); grid on;
        exportgraphics(figOpen,fullfile(rootdir,[stem 'derived_open_loop_matlab.png']));close(figOpen);
        figNy=figure;nyquist(estimatedLoop);grid on;
        exportgraphics(figNy,fullfile(rootdir,[stem 'derived_nyquist_matlab.png']));close(figNy);
    end
    nexttile(1); semilogx(f,db,'-o','DisplayName',labels{k}); hold on;
    ylabel('Closed-loop dB'); grid on; legend('Location','best');
    nexttile(2); semilogx(f,unwrapped,'-o','DisplayName',labels{k}); hold on;
    ylabel('Closed-loop phase deg'); xlabel('Hz'); grid on;
end
exportgraphics(fig,fullfile(rootdir,[stem 'measured_frequency_matlab.png'])); close(fig);
result=struct('time_domain',metrics,'frequency_domain',spectra,...
    'note','Measured closed-loop phases are NOT open-loop phase margins. Sparse -3dB interpolation is not a precision bandwidth measurement.');
if exist('open','var'),result.derived_open_loop=open;end
fid=fopen(fullfile(rootdir,[stem 'matlab_measured_evaluation.json']),'w');
fwrite(fid,jsonencode(result,PrettyPrint=true),'char');fclose(fid);
disp(result);
end
