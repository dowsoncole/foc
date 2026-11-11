function result = tune_current_pi(cfg, outdir)
% Finite-grid robust PI search; not a proof of global optimum.
% C(z)=Kp+Ki*Ts/(1-z^-1): matches firmware's update-integral-before-output.
% Plant = ZOH{Vdc/(L*s+R)} plus ceil(extra_delay_s/Ts) command delays.
% Assumes fixed rotor, linear modulation, one excited axis, no cross coupling.
arguments
    cfg struct
    outdir (1,:) char
end
validateattributes([cfg.R cfg.L cfg.Vdc cfg.Ts cfg.step_a cfg.duration_s], ...
    {'double'}, {'positive','finite'});
validateattributes(cfg.extra_delay_s, {'double'}, {'nonnegative','finite'});
validateattributes(cfg.integral_limit_pu, {'double'}, {'positive','finite'});
assert(cfg.integral_limit_pu <= cfg.voltage_limit_pu);
assert(cfg.duration_s > cfg.max_settling_s);
assert(cfg.step_a <= 0.5, 'Commissioning current limit is 0.5 A.');
assert(all([cfg.R_factors cfg.L_factors cfg.V_factors] > 0));
if ~cfg.measured
    warning('SYNTHETIC/UNVERIFIED MODEL: results are not hardware-approved gains.');
end
if ~isfolder(outdir), mkdir(outdir); end
t = (0:cfg.Ts:cfg.duration_s)';
z = tf('z', cfg.Ts);
delay = ceil(cfg.extra_delay_s / cfg.Ts);
plants = {plant(cfg.R, cfg.L, cfg.Vdc, cfg.Ts, delay)};
for r = cfg.R_factors
    for l = cfg.L_factors
        for v = cfg.V_factors
            for n = [delay delay+1]
                plants{end+1} = plant(cfg.R*r, cfg.L*l, cfg.Vdc*v, cfg.Ts, n); %#ok<AGROW>
            end
        end
    end
end
rows = [];
best = Inf;
for fc = cfg.fc_hz
    if fc >= 0.1/cfg.Ts, continue; end
    for kscale = cfg.ki_factors
        kp = cfg.L*2*pi*fc/cfg.Vdc;
        ki = cfg.R*2*pi*fc/cfg.Vdc*kscale;
        C = ss(kp + ki*cfg.Ts*z/(z-1));
        worstPM = Inf; worstGM = Inf; worstOS = 0; worstST = 0; score = 0;
        valid = true;
        for n = 1:numel(plants)
            G = plants{n}; loop = C*G;
            closed = feedback(loop, 1);
            if ~isstable(closed), valid = false; break; end
            [gm, pm] = margin(loop);
            if isnan(gm) || isnan(pm), valid = false; break; end
            gmDB = 20*log10(gm);
            worstPM = min(worstPM, pm); worstGM = min(worstGM, gmDB);
            if pm < cfg.min_pm_deg || gmDB < cfg.min_gm_db
                valid = false; break;
            end
            y = step(closed*cfg.step_a, t);
            u = step(feedback(C,G)*cfg.step_a, t);
            im = u - kp*(cfg.step_a-y);
            si = stepinfo(y,t,cfg.step_a,'SettlingTimeThreshold',0.02);
            if ~isfinite(si.SettlingTime) || si.SettlingTime > cfg.max_settling_s || ...
                    si.Overshoot > cfg.max_overshoot_pct || ...
                    max(abs(u)) > cfg.voltage_limit_pu || ...
                    max(abs(im)) > cfg.integral_limit_pu
                valid = false; break;
            end
            worstOS = max(worstOS, si.Overshoot);
            worstST = max(worstST, si.SettlingTime);
            % Worst-case normalized ITAE plus a small normalized effort cost.
            cost = trapz(t,t.*abs(cfg.step_a-y))/(cfg.step_a*cfg.duration_s^2) + ...
                0.001*trapz(t,u.^2)/(cfg.voltage_limit_pu^2*cfg.duration_s);
            score = max(score,cost);
        end
        if valid
            rows(end+1,:) = [kp ki fc kscale score worstPM worstGM worstOS worstST]; %#ok<AGROW>
            if score < best, best = score; winner = rows(end,:); end
        end
    end
end
assert(~isempty(rows), 'No feasible PI. Review model, step, limits and requirements.');
tab = array2table(rows,'VariableNames', ...
    {'Kp','Ki','seed_fc_hz','ki_factor','cost','worst_pm_deg', ...
     'worst_gm_db','worst_overshoot_pct','worst_settling_s'});
tab = sortrows(tab,'cost');
writetable(tab,fullfile(outdir,'feasible_candidates.csv'));
kp = winner(1); ki = winner(2);
C = ss(kp + ki*cfg.Ts*z/(z-1));
G = plants{1}; closed = feedback(C*G,1);
yLinear = step(closed*cfg.step_a,t);
[y,u] = saturatedSimulation(cfg,kp,ki,delay,numel(t));
% Positive/negative step and overload recovery exercise the integrator limits.
[yn,un] = saturatedSimulation(cfg,kp,ki,delay,numel(t),-cfg.step_a*ones(size(t)));
assert(max(abs(y+yn)) < 1e-8 && max(abs(u+un)) < 1e-8);
ref = cfg.step_a*ones(size(t));
ref(1:floor(numel(t)/3)) = 0.5;
[yr,ur] = saturatedSimulation(cfg,kp,ki,delay,numel(t),ref);
assert(all(isfinite(yr)) && max(abs(ur)) <= cfg.voltage_limit_pu+1e-12);
assert(abs(yr(end)-cfg.step_a) < 0.02*cfg.step_a, 'Overload recovery failed.');
assert(max(abs(y-yLinear)) < 1e-7, 'Discrete simulation differs from LTI model.');
result = table2struct(tab(1,:));
result.measured = cfg.measured;
result.closed_loop_stable = isstable(closed);
result.scope = 'Best of finite grid under specified model, corners and constraints; not global optimum';
result.controller = 'parallel PI, backward Euler, pu/A and pu/(A*s)';
result.nominal_stepinfo = stepinfo(y,t,cfg.step_a);
result.cfg = cfg;
save(fullfile(outdir,'design.mat'),'cfg','result','C','G','closed','plants');
fid = fopen(fullfile(outdir,'design.json'),'w');
assert(fid >= 0);
cleanup = onCleanup(@() fclose(fid));
fprintf(fid,'%s',jsonencode(result,PrettyPrint=true));
clear cleanup
writetable(table(t,y,u,yLinear,ref,yr,ur),fullfile(outdir,'simulation.csv'));
fig = figure('Visible','off');
tiledlayout(2,1);
nexttile; plot(t,y,t,cfg.step_a*ones(size(t)),'--'); grid on
xlabel('Time (s)'); ylabel('Current (A)'); title('Synthetic/model prediction: single-axis step');
nexttile; plot(t,u); grid on; xlabel('Time (s)'); ylabel('Voltage (pu)');
exportgraphics(fig,fullfile(outdir,'step.png')); close(fig);
fig = figure('Visible','off'); margin(C*G); grid on;
exportgraphics(fig,fullfile(outdir,'loop_margin.png')); close(fig);
disp(result);
end

function G = plant(R,L,V,Ts,delay)
G = c2d(tf(V,[L R]),Ts,'zoh');
G.InputDelay = delay;
G = absorbDelay(ss(G)); % Expand discrete delays into states before feedback/isstable.
end

function [y,u] = saturatedSimulation(cfg,kp,ki,delay,N,ref)
if nargin < 6, ref = cfg.step_a*ones(N,1); end
y = zeros(N,1); u = zeros(N,1); integral = 0;
a = exp(-cfg.R*cfg.Ts/cfg.L);
b = cfg.Vdc/cfg.R*(1-a);
for k = 1:N
    e = ref(k)-y(k);
    old = integral;
    integral = min(max(old+ki*cfg.Ts*e,-cfg.integral_limit_pu),cfg.integral_limit_pu);
    cmd = kp*e+integral;
    applied = min(max(cmd,-cfg.voltage_limit_pu),cfg.voltage_limit_pu);
    if (integral-old)*(cmd-applied) > 0
        integral = old;
        applied = min(max(kp*e+integral,-cfg.voltage_limit_pu),cfg.voltage_limit_pu);
    end
    u(k) = applied;
    j = k-delay;
    if k < N
        y(k+1) = a*y(k);
        if j > 0, y(k+1) = y(k+1)+b*u(j); end
    end
end
end
