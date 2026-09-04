function main_platooning_FULL_ONE_FILE_COMPARISON_V2
% MAIN_PLATOONING_FULL_ONE_FILE_COMPARISON
% One-file journal comparison study for the 6-vehicle 16-DOF platoon.
%
% Cases:
%   1) Conventional CACC
%   2) CACC + FTDO
%   3) Proposed robust controller without variable headway / preview
%   4) Proposed full controller
%
% All cases use the SAME plant, DLC, 0.20-s V2V delay, 10% packet loss,
% initial conditions, disturbance realization, and random seed. This keeps
% the comparison reproducible and focuses the comparison on the controller.

clc; close all;
rng(11);

caseCfg = struct( ...
    'id',{1,2,3,4}, ...
    'tag',{'01_CACC','02_CACC_FTDO','03_Proposed_NoVH_NoPreview','04_Proposed_Full'}, ...
    'label',{'Conventional CACC','CACC + FTDO', ...
             'Proposed robust (no VH/preview)','Proposed full'});

nCase = numel(caseCfg);
caseResults = cell(1,nCase);

fprintf('\n===============================================================\n');
fprintf(' JOURNAL CONTROLLER COMPARISON STUDY\n');
fprintf(' 6 vehicles | 16-DOF | DLC | 0.20 s delay | 10%% packet loss\n');
fprintf('===============================================================\n');

for c = 1:nCase

    fprintf('\n===============================================================\n');
    fprintf('Running case %d/%d\n', c, nCase);
    fprintf('===============================================================\n');

    tmpResult = run_platoon_case(caseCfg(c));

    % Cell storage avoids MATLAB's dissimilar-structure assignment issue.
    caseResults{c} = tmpResult;

end

%% Aggregate comparison metrics
metricNames = { ...
    'RMS gap error (m)', ...
    'Max gap error (m)', ...
    'RMS velocity error (m/s)', ...
    'Max velocity error (m/s)', ...
    'RMS lateral error (m)', ...
    'Max lateral error (m)', ...
    'Max roll (deg)', ...
    'Max articulation (deg)', ...
    'Max axle LTR', ...
    'Max string RMS ratio', ...
    'Mean packet loss (%)', ...
    'Mean FTDO d_v RMSE', ...
    'Mean FTDO d_a RMSE', ...
    'Peak drive torque (Nm)', ...
    'Peak total brake torque (Nm)', ...
    'Peak anti-roll moment (Nm)'};

C = zeros(nCase,numel(metricNames));
for c = 1:nCase
    R = caseResults{c};
    C(c,1)  = mean(R.metrics.rms_gap_error,'omitnan');
    C(c,2)  = max(R.metrics.max_gap_error);
    C(c,3)  = mean(R.metrics.rms_velocity_error,'omitnan');
    C(c,4)  = max(R.metrics.max_velocity_error);
    C(c,5)  = mean(R.metrics.rms_lat_error,'omitnan');
    C(c,6)  = max(R.metrics.max_lat_error);
    C(c,7)  = max(R.metrics.max_roll_deg);
    C(c,8)  = max(R.metrics.max_gamma_deg);
    C(c,9)  = max([R.metrics.max_LTR_front(:); R.metrics.max_LTR_rear(:)]);
    C(c,10) = max(R.metrics.string_rms_ratio,[],'omitnan');
    C(c,11) = mean(R.metrics.packet_loss_rate,'omitnan');
    C(c,12) = mean(R.metrics.rms_obs_v,'omitnan');
    C(c,13) = mean(R.metrics.rms_obs_a,'omitnan');
    C(c,14) = max(R.metrics.peak_drive_torque);
    C(c,15) = max(R.metrics.peak_brake_torque);
    C(c,16) = max(R.metrics.peak_Mact);
end

Tcomparison = array2table(C,'VariableNames',{ ...
    'RMS_gap_m','Max_gap_m','RMS_vel_mps','Max_vel_mps', ...
    'RMS_lat_m','Max_lat_m','Max_roll_deg','Max_articulation_deg', ...
    'Max_LTR','Max_string_RMS_ratio','Mean_packet_loss_pct', ...
    'Mean_FTDO_dv_RMSE','Mean_FTDO_da_RMSE','Peak_drive_Nm', ...
    'Peak_brake_Nm','Peak_Mact_Nm'});
Tcomparison = addvars(Tcomparison, ...
    {caseCfg.label}.','Before',1,'NewVariableNames','Controller');

fprintf('\n================ COMPARISON TABLE ================\n');
disp(Tcomparison);

%% Export comparison table
outDir = fullfile(pwd,'export_journal','comparison_study');
if ~exist(outDir,'dir'), mkdir(outDir); end
writetable(Tcomparison,fullfile(outDir,'controller_comparison_summary.csv'));
save(fullfile(outDir,'controller_comparison_results.mat'), ...
    'caseResults','Tcomparison','caseCfg');

%% Comparison plots
T = caseResults{1}.time.T;
labels = {caseCfg.label};

% 101: V1 gap error
figure(101); clf; hold on;
for c = 1:nCase
    plot(T,caseResults{c}.errors.e_gap(1,:), ...
        'LineWidth',1.5,'DisplayName',labels{c});
end
yline(0,'k--','HandleVisibility','off');
grid on; box on;
xlabel('Time (s)'); ylabel('e_d (m)');
title('Controller Comparison - V1 CTH Gap Error');
legend('Location','best');

% 102: V6 gap error (propagation through the platoon)
figure(102); clf; hold on;
for c = 1:nCase
    plot(T,caseResults{c}.errors.e_gap(6,:), ...
        'LineWidth',1.5,'DisplayName',labels{c});
end
yline(0,'k--','HandleVisibility','off');
grid on; box on;
xlabel('Time (s)'); ylabel('e_d (m)');
title('Controller Comparison - V6 CTH Gap Error');
legend('Location','best');

% 103: velocity error of V1 and V6
figure(103); clf;
tiledlayout(2,1);
nexttile; hold on;
for c = 1:nCase
    plot(T,caseResults{c}.errors.vel_err_to_leader(1,:), ...
        'LineWidth',1.4,'DisplayName',labels{c});
end
yline(0,'k--','HandleVisibility','off'); grid on; box on;
ylabel('V1 error (m/s)'); title('Velocity Error to Leader');
legend('Location','best');
nexttile; hold on;
for c = 1:nCase
    plot(T,caseResults{c}.errors.vel_err_to_leader(6,:), ...
        'LineWidth',1.4,'DisplayName',labels{c});
end
yline(0,'k--','HandleVisibility','off'); grid on; box on;
xlabel('Time (s)'); ylabel('V6 error (m/s)');

% 104: string RMS ratio by vehicle
figure(104); clf; hold on;
for c = 1:nCase
    plot(1:caseResults{c}.meta.Nveh,caseResults{c}.metrics.string_rms_ratio, ...
        '-o','LineWidth',1.4,'DisplayName',labels{c});
end
yline(1,'k--','DisplayName','String-stability boundary');
grid on; box on;
xlabel('Vehicle index'); ylabel('RMS ratio');
title('String Stability Comparison');
legend('Location','best');

% 105: maximum LTR by controller
figure(105); clf;
bar(C(:,9));
yline(caseResults{1}.meta.LTR_max,'k--','DisplayName','LTR limit');
grid on; box on;
set(gca,'XTick',1:nCase,'XTickLabel',labels);
xtickangle(20);
ylabel('max |LTR|'); title('Maximum Axle LTR Comparison');
legend('Location','best');

% 106: maximum lateral error
figure(106); clf;
bar(C(:,6));
grid on; box on;
set(gca,'XTick',1:nCase,'XTickLabel',labels);
xtickangle(20);
ylabel('max |e_y| (m)'); title('Maximum Lateral Tracking Error');

% 107: peak actuation
figure(107); clf;
tiledlayout(3,1);
nexttile; bar(C(:,14)); grid on; box on;
ylabel('Nm'); title('Peak Drive Torque');
set(gca,'XTick',1:nCase,'XTickLabel',labels); xtickangle(20);
nexttile; bar(C(:,15)); grid on; box on;
ylabel('Nm'); title('Peak Total Brake Torque');
set(gca,'XTick',1:nCase,'XTickLabel',labels); xtickangle(20);
nexttile; bar(C(:,16)); grid on; box on;
ylabel('Nm'); title('Peak Active Anti-Roll Moment');
set(gca,'XTick',1:nCase,'XTickLabel',labels); xtickangle(20);

% 108: FTDO estimation RMSE (case 1 should be zero because FTDO is off)
figure(108); clf;
tiledlayout(2,1);
nexttile; bar(C(:,12)); grid on; box on;
ylabel('RMSE'); title('Disturbance Observer Velocity RMSE');
set(gca,'XTick',1:nCase,'XTickLabel',labels); xtickangle(20);
nexttile; bar(C(:,13)); grid on; box on;
ylabel('RMSE'); title('Disturbance Observer Acceleration RMSE');
set(gca,'XTick',1:nCase,'XTickLabel',labels); xtickangle(20);

fprintf('\nComparison study completed. Results exported to:\n  %s\n',outDir);
end

function results = run_platoon_case(caseCfg)
rng(11);

%% =========================================================================
% MAIN_PLATOONING_FULL_ONE_FILE
%
% Single-file version of the 6-vehicle articulated-platoon simulation.
%
% Included in THIS file:
%   1) Vehicle parameters
%   2) 16-DOF articulated vehicle plant
%   3) Full control allocation
%   4) PF/CACC-like longitudinal controller
%   5) Fixed-time disturbance observer (FTDO)
%   6) V2V transport delay + packet loss + ZOH
%   7) Delayed-state prediction
%   8) Variable time headway
%   9) Leader acceleration preview
%  10) Double-lane-change lateral controller
%  11) Full 9-channel actuator logging
%  12) Physical axle LTR from the 16-DOF plant output
%  13) Journal-oriented plots
%  14) MAT + CSV export
%
% IMPORTANT:
%   - This file preserves the supplied plant_16dof.m equations.
%   - The supplied allocator uses a roll-angle-based LTR proxy to trigger
%     M_act. The physical LTR plotted below comes from the wheel vertical
%     loads calculated inside plant_16dof().
%
% 16-DOF state:
%   1 vx, 2 vy, 3 r_t, 4 r_s, 5 gamma, 6 psi, 7 Y, 8 X,
%   9 omega_fL, 10 omega_fR, 11 omega_rL, 12 omega_rR,
%   13 omega_sL, 14 omega_sR, 15 phi_t, 16 dphi_t
%
% 9 inputs:
%   1 delta_f
%   2 T_drive
%   3 M_act
%   4 Tb_fL
%   5 Tb_fR
%   6 Tb_rL
%   7 Tb_rR
%   8 Tb_sL
%   9 Tb_sR
%% =========================================================================

%% 1. SIMULATION / FEATURES
doPic = false;

Nveh    = 6;
Tstep  = 0.01;
SimTime = 30;
Nstep   = round(SimTime/Tstep) + 1;

useFTDO            = true;
useCommDelay       = true;
usePacketLoss      = true;
useVariableHeadway = true;
useLeaderAccelPreview = true;

tau_comm         = 0.20;
packet_loss_prob = 0.10;
comm_delay_steps = round(tau_comm/Tstep);

printControlSignals = true;
print_interval_s = 1.0;
print_interval_steps = max(round(print_interval_s/Tstep),1);

exportResults = true;
export_folder = fullfile(pwd,'export_journal',caseCfg.tag);

ctrl_channel_names = { ...
    'delta_f_rad', ...
    'T_drive_Nm', ...
    'M_act_Nm', ...
    'Tb_fL_Nm', ...
    'Tb_fR_Nm', ...
    'Tb_rL_Nm', ...
    'Tb_rR_Nm', ...
    'Tb_sL_Nm', ...
    'Tb_sR_Nm'};

%% 2. VEHICLE PARAMETERS
p = vehicle_params_onefile();

%% 3. PLATOON PARAMETERS
d0 = 20.0;

h0    = 0.70;
h_min = 0.60;
h_max = 0.95;

k_v_headway   = 0.03;
Delta_h_kappa = 0.04;
Delta_h_gamma = 0.02;

a_y_scale   = 2.5;
gamma_scale = deg2rad(5);

v0  = 16.0;
v00 = 16.0;

d_initial = d0 + h0*v0;

%% 4. INITIAL ROAD-LOAD COMPENSATION
F_a0 = 0.5*p.C_D*p.A_a*p.rho_a*v0^2;
F_roll0 = p.F_zf_static*p.a_f + ...
          p.F_zr_static*p.a_r + ...
          p.F_zs_static*p.a_s;
a_eq0 = (F_a0 + F_roll0)/p.m_tot;

%% 5. LONGITUDINAL CONTROLLER
k1      = 0.18;
epsilon = 0.20;
q       = 0.80;
ppp     = 0.70;
delta   = 0.10;

k_gap   = 0.12;
k_slide = 0.015;

k_af        = 0.65;
k_af_leader = 0.85;

am = -3.0;
aM =  3.0;
max_jerk = 3.0;

max_velocity_error_for_control = 1.0;

% Optional old velocity-tracking patch; disabled in the current tuned case.
useVxTrackPatch = false;
k_vx_1015 = 0.75;
vx_err_max = 0.80;

%% CASE-SPECIFIC CONTROLLER CONFIGURATION
% All cases use the same 16-DOF plant, DLC controller, V2V delay, and
% packet-loss realization. Only the longitudinal control architecture is
% changed so the comparison remains controlled and reproducible.
switch caseCfg.id
    case 1  % Conventional CACC
        controller_name = 'Conventional CACC';
        useFTDO = false;
        useVariableHeadway = false;
        useLeaderAccelPreview = false;
        use_robust_sliding = false;
    case 2  % Conventional CACC + FTDO
        controller_name = 'CACC + FTDO';
        useFTDO = true;
        useVariableHeadway = false;
        useLeaderAccelPreview = false;
        use_robust_sliding = false;
    case 3  % Proposed robust longitudinal controller without VH/preview
        controller_name = 'Proposed robust (no VH/preview)';
        useFTDO = true;
        useVariableHeadway = false;
        useLeaderAccelPreview = false;
        use_robust_sliding = true;
    case 4  % Full proposed controller
        controller_name = 'Proposed full';
        useFTDO = true;
        useVariableHeadway = true;
        useLeaderAccelPreview = true;
        use_robust_sliding = true;
    otherwise
        error('Unknown comparison case id.');
end

% Gains of the validated proposed longitudinal controller.  They are kept
% identical in Cases 3 and 4 so the ablation isolates VH/preview effects.
k_rel_v = 0.85;
k_gap_tuned = 0.32;
k_d_gap = 0.35;
k_slide_tuned = 0.005;
k_af_tuned = 0.45;
k_af_leader_tuned = 0.10;
max_jerk_tuned = 5.0;
max_velocity_error_for_control_tuned = 2.0;

if use_robust_sliding
    k_gap = k_gap_tuned;
    k_slide = k_slide_tuned;
    k_af = k_af_tuned;
    k_af_leader = k_af_leader_tuned;
    max_jerk = max_jerk_tuned;
    max_velocity_error_for_control = max_velocity_error_for_control_tuned;
end

%% 6. FIXED-TIME DISTURBANCE OBSERVER
alpha_ft = 0.75;
beta_ft  = 1.50;

FTDO_X.a1 = 3.8;
FTDO_X.a2 = 0.08;
FTDO_X.b1 = 6.0;
FTDO_X.b2 = 0.10;

FTDO_V.a1 = 3.8;
FTDO_V.a2 = 0.08;
FTDO_V.b1 = 6.0;
FTDO_V.b2 = 0.10;

observer_comp_gain = 0.01;
d_v_hat_limit = 0.50;
d_a_hat_limit = 0.50;
FTDO_warmup = 0.50;

%% 7. DOUBLE-LANE-CHANGE
lane_width = 3.5;
A_lane = lane_width/2;
k_lane = 0.055;
x_lane_start = 50;
x_lane_end   = 120;

K_stanley = 0.85;
Kpsi   = 0.95;
Kr     = 0.20;
Kvy    = 0.010;
Kgamma = 0.08;
Krs    = 0.02;

lookahead = 8.0;
delta_limit = deg2rad(5.5);

%% 8. MEMORY / LOGGING
T = (0:Nstep-1)*Tstep;

x16 = zeros(Nveh,Nstep,16);
x   = zeros(Nveh,Nstep,3);

X0 = zeros(1,Nstep);
v0_hist = zeros(1,Nstep);
a0 = zeros(1,Nstep);
Y0 = zeros(1,Nstep);
psi0 = zeros(1,Nstep);

u_cmd = zeros(Nveh,Nstep);
u_actual = zeros(Nveh,Nstep);
u16_prev = zeros(Nveh,9);
u16_hist = zeros(Nveh,Nstep,9);

s = zeros(Nveh,Nstep);
S = zeros(Nveh,Nstep);
intOfE = zeros(Nveh,1);

e_gap = zeros(Nveh,Nstep);
e_v   = zeros(Nveh,Nstep);

h_hist = zeros(Nveh,Nstep);
dstar_hist = zeros(Nveh,Nstep);

X_hist = zeros(Nveh,Nstep);
Y_hist = zeros(Nveh,Nstep);

dparallel_hist = zeros(Nveh,Nstep);
dperp_hist = zeros(Nveh,Nstep);
deuclid_hist = zeros(Nveh,Nstep);

y_ref_hist = zeros(Nveh,Nstep);
psi_ref_hist = zeros(Nveh,Nstep);
kappa_ref_hist = zeros(Nveh,Nstep);
ey_hist = zeros(Nveh,Nstep);
epsi_hist = zeros(Nveh,Nstep);
delta_hist = zeros(Nveh,Nstep);

D_v_true = zeros(Nveh,Nstep);
D_a_true = zeros(Nveh,Nstep);
D_v_hat  = zeros(Nveh,Nstep);
D_a_hat  = zeros(Nveh,Nstep);

ax_actual_hist = zeros(Nveh,Nstep);
Ui_af_hist = zeros(Nveh,Nstep);
Ui_af_leader_hist = zeros(Nveh,Nstep);
Ui_kv_hist = zeros(Nveh,Nstep);
Ui_gap_hist = zeros(Nveh,Nstep);
Ui_dist_hist = zeros(Nveh,Nstep);
Ui_vx_track_hist = zeros(Nveh,Nstep);

% Physical LTR returned by plant:
LTR_f_hist = zeros(Nveh,Nstep);
LTR_r_hist = zeros(Nveh,Nstep);

% Vertical loads for journal diagnostics:
Fzf_L_hist = zeros(Nveh,Nstep);
Fzf_R_hist = zeros(Nveh,Nstep);
Fzr_L_hist = zeros(Nveh,Nstep);
Fzr_R_hist = zeros(Nveh,Nstep);
Fzs_L_hist = zeros(Nveh,Nstep);
Fzs_R_hist = zeros(Nveh,Nstep);

% Control breakdown
Tdrive_hist = zeros(Nveh,Nstep);
Mact_hist = zeros(Nveh,Nstep);
Tb_fL_hist = zeros(Nveh,Nstep);
Tb_fR_hist = zeros(Nveh,Nstep);
Tb_rL_hist = zeros(Nveh,Nstep);
Tb_rR_hist = zeros(Nveh,Nstep);
Tb_sL_hist = zeros(Nveh,Nstep);
Tb_sR_hist = zeros(Nveh,Nstep);
Tbrake_total_hist = zeros(Nveh,Nstep);
brake_asym_front = zeros(Nveh,Nstep);
brake_asym_rear = zeros(Nveh,Nstep);

% Communication
rx_X = zeros(Nveh,1);
rx_Y = zeros(Nveh,1);
rx_psi = zeros(Nveh,1);
rx_vx = zeros(Nveh,1);
rx_vy = zeros(Nveh,1);
rx_gamma = zeros(Nveh,1);
rx_r = zeros(Nveh,1);
rx_acc = zeros(Nveh,1);

rx_time_idx = ones(Nveh,1);
packet_received = zeros(Nveh,Nstep);
packet_lost = zeros(Nveh,Nstep);
comm_delay_hist = zeros(Nveh,Nstep);

% FTDO internal states
zX0 = zeros(Nveh,1);
zX1 = zeros(Nveh,1);
zV0 = zeros(Nveh,1);
zV1 = zeros(Nveh,1);

%% 9. LEADER INITIAL CONDITION
X_leader_initial = Nveh*d_initial;

X0(1) = X_leader_initial;
v0_hist(1) = v00;
a0(1) = 0;

[Y0(1),~,~,psi0(1),~] = dlc_reference( ...
    X0(1), ...
    X_leader_initial+x_lane_start, ...
    X_leader_initial+x_lane_end, ...
    A_lane,k_lane);

%% 10. FOLLOWER INITIALIZATION
for i = 1:Nveh

    init_X = X_leader_initial - i*d_initial;
    init_vx = v0;

    x16(i,1,1) = init_vx;
    x16(i,1,2) = 0;
    x16(i,1,3) = 0;
    x16(i,1,4) = 0;
    x16(i,1,5) = 0;
    x16(i,1,6) = 0;
    x16(i,1,7) = 0;
    x16(i,1,8) = init_X;

    x16(i,1,9)  = init_vx/p.R_f;
    x16(i,1,10) = init_vx/p.R_f;
    x16(i,1,11) = init_vx/p.R_r;
    x16(i,1,12) = init_vx/p.R_r;
    x16(i,1,13) = init_vx/p.R_s;
    x16(i,1,14) = init_vx/p.R_s;
    x16(i,1,15) = 0;
    x16(i,1,16) = 0;

    x(i,1,:) = [init_X,init_vx,0];

    X_hist(i,1) = init_X;
    Y_hist(i,1) = 0;

    zX0(i) = init_X;
    zX1(i) = 0;
    zV0(i) = init_vx;
    zV1(i) = 0;

    if i == 1
        rx_X(i) = X0(1);
        rx_Y(i) = Y0(1);
        rx_psi(i) = psi0(1);
        rx_vx(i) = v0_hist(1);
        rx_vy(i) = 0;
        rx_gamma(i) = 0;
        rx_r(i) = 0;
        rx_acc(i) = a_eq0;
    else
        j = i-1;
        rx_X(i) = x16(j,1,8);
        rx_Y(i) = 0;
        rx_psi(i) = 0;
        rx_vx(i) = v0;
        rx_vy(i) = 0;
        rx_gamma(i) = 0;
        rx_r(i) = 0;
        rx_acc(i) = a_eq0;
    end

    rx_time_idx(i) = 1;
    h_hist(i,1) = h0;

    u0_16 = allocate_control_onefile( ...
        a_eq0,0,init_vx,0,0,p);

    u16_prev(i,:) = u0_16(:).';
    u16_hist(i,1,:) = u0_16(:).';

    % Initial plant output gives physical LTR and wheel loads.
    [~,y0plant,Fz0] = plant_16dof_onefile( ...
        reshape(x16(i,1,:),16,1),u0_16,p);

    LTR_f_hist(i,1) = y0plant(6);
    LTR_r_hist(i,1) = y0plant(7);

    Fzf_L_hist(i,1) = Fz0(1);
    Fzf_R_hist(i,1) = Fz0(2);
    Fzr_L_hist(i,1) = Fz0(3);
    Fzr_R_hist(i,1) = Fz0(4);
    Fzs_L_hist(i,1) = Fz0(5);
    Fzs_R_hist(i,1) = Fz0(6);

end

%% 11. MAIN CLOSED LOOP
for n = 2:Nstep

    t = T(n);

    % Smooth leader acceleration profile.
    a0(n) = leader_accel_profile(t,1.5);

    v0_hist(n) = v0_hist(n-1) + a0(n)*Tstep;

    X0(n) = X0(n-1) + ...
            v0_hist(n-1)*Tstep + ...
            0.5*a0(n)*Tstep^2;

    [Y0(n),~,~,psi0(n),~] = dlc_reference( ...
        X0(n), ...
        X_leader_initial+x_lane_start, ...
        X_leader_initial+x_lane_end, ...
        A_lane,k_lane);

    for i = 1:Nveh

        %% Current state
        x_prev = reshape(x16(i,n-1,:),16,1);

        vx = x_prev(1);
        vy = x_prev(2);
        r_t = x_prev(3);
        r_s = x_prev(4);
        gamma = x_prev(5);
        psi = x_prev(6);
        Ypos = x_prev(7);
        Xpos = x_prev(8);
        phi = x_prev(15);

        %% External disturbances
        Dv = 0.30*sin(0.65*t + 0.17*i)*exp(-((t-12)/4.5)^2);
        Da = 0.35*sin(1.15*t + 0.13*i)*exp(-((t-17)/3.8)^2);

        D_v_true(i,n) = Dv;
        D_a_true(i,n) = Da;

        %% Fixed-time disturbance observer
        if useFTDO

            [dx_nom,~,~] = plant_16dof_onefile( ...
                x_prev,u16_prev(i,:),p);

            fX_nom = dx_nom(8);
            fV_nom = dx_nom(1);

            eOX = zX0(i)-Xpos;

            zX0_dot = fX_nom + zX1(i) ...
                - FTDO_X.a1*sigpow(eOX,alpha_ft) ...
                - FTDO_X.a2*sigpow(eOX,beta_ft);

            zX1_dot = ...
                - FTDO_X.b1*sigpow(eOX,2*alpha_ft-1) ...
                - FTDO_X.b2*sigpow(eOX,2*beta_ft-1);

            zX0(i) = zX0(i) + zX0_dot*Tstep;
            zX1(i) = zX1(i) + zX1_dot*Tstep;

            d_v_hat = max(min(zX1(i),d_v_hat_limit),-d_v_hat_limit);

            eOV = zV0(i)-vx;

            zV0_dot = fV_nom + zV1(i) ...
                - FTDO_V.a1*sigpow(eOV,alpha_ft) ...
                - FTDO_V.a2*sigpow(eOV,beta_ft);

            zV1_dot = ...
                - FTDO_V.b1*sigpow(eOV,2*alpha_ft-1) ...
                - FTDO_V.b2*sigpow(eOV,2*beta_ft-1);

            zV0(i) = zV0(i) + zV0_dot*Tstep;
            zV1(i) = zV1(i) + zV1_dot*Tstep;

            d_a_hat = max(min(zV1(i),d_a_hat_limit),-d_a_hat_limit);

        else
            d_v_hat = 0;
            d_a_hat = 0;
        end

        D_v_hat(i,n) = d_v_hat;
        D_a_hat(i,n) = d_a_hat;

        %% V2V packet with fixed delay + packet loss + ZOH
        if useCommDelay
            tx_idx = max(n-comm_delay_steps,1);
        else
            tx_idx = n-1;
        end

        packet_ok = (~usePacketLoss) || ...
                    (rand >= packet_loss_prob);

        if packet_ok

            if i == 1

                rx_X(i) = X0(tx_idx);
                rx_Y(i) = Y0(tx_idx);
                rx_psi(i) = psi0(tx_idx);
                rx_vx(i) = v0_hist(tx_idx);
                rx_vy(i) = 0;
                rx_gamma(i) = 0;
                rx_r(i) = 0;
                rx_acc(i) = a0(tx_idx);

            else

                j = i-1;

                rx_X(i) = x16(j,tx_idx,8);
                rx_Y(i) = x16(j,tx_idx,7);
                rx_psi(i) = x16(j,tx_idx,6);
                rx_vx(i) = x16(j,tx_idx,1);
                rx_vy(i) = x16(j,tx_idx,2);
                rx_gamma(i) = x16(j,tx_idx,5);
                rx_r(i) = x16(j,tx_idx,3);

                % Desired acceleration command is transmitted.
                if tx_idx >= 2
                    rx_acc(i) = u_cmd(j,tx_idx);
                else
                    rx_acc(i) = a_eq0;
                end
            end

            rx_time_idx(i) = tx_idx;
            packet_received(i,n) = 1;
            packet_lost(i,n) = 0;

        else

            % Zero-order hold: receiver memory remains unchanged.
            packet_received(i,n) = 0;
            packet_lost(i,n) = 1;

        end

        age_comm = (n-rx_time_idx(i))*Tstep;
        comm_delay_hist(i,n) = age_comm;

        Xp = rx_X(i);
        Yp = rx_Y(i);
        vx_p = rx_vx(i);
        acc_p = rx_acc(i);

        %% Delayed predecessor prediction
        prediction_horizon = min(age_comm,0.40);

        Xp_pred = Xp + ...
                  prediction_horizon*vx_p + ...
                  0.5*prediction_horizon^2*acc_p;

        Yp_pred = Yp;

        dX = Xp_pred-Xpos;
        dY = Yp_pred-Ypos;

        t_hat = [cos(psi);sin(psi)];
        n_hat = [-sin(psi);cos(psi)];

        d_parallel = dX*t_hat(1) + dY*t_hat(2);
        d_perp = dX*n_hat(1) + dY*n_hat(2);
        d_euclid = hypot(dX,dY);

        dparallel_hist(i,n) = d_parallel;
        dperp_hist(i,n) = d_perp;
        deuclid_hist(i,n) = d_euclid;

        %% Variable time headway
        if useVariableHeadway

            % Keep longitudinal CTH reference independent of velocity error.
            % The variable-headway mechanism is driven only by lateral/risk
            % terms in the validated proposed controller.
            h_v = 0;

            X0v_tmp = X0_vehicle_start( ...
                i,X_leader_initial,d_initial);

            [~,~,~,~,kappa_tmp] = dlc_reference( ...
                Xpos+lookahead, ...
                X0v_tmp+x_lane_start, ...
                X0v_tmp+x_lane_end, ...
                A_lane,k_lane);

            risk_kappa = tanh( ...
                vx^2*abs(kappa_tmp)/max(a_y_scale,1e-6));

            risk_gamma = tanh( ...
                abs(gamma)/max(gamma_scale,1e-6));

            h_raw = h0 + h_v ...
                + Delta_h_kappa*risk_kappa ...
                + Delta_h_gamma*risk_gamma;

        else
            h_raw = h0;
        end

        h_prev = h_hist(i,n-1);
        h_i = h_prev + 0.05*(h_raw-h_prev);
        h_i = min(max(h_i,h_min),h_max);

        h_hist(i,n) = h_i;

        %% CTH spacing error
        d_star = d0 + h_i*max(vx,0);

        e_i = d_star-d_parallel;

        dstar_hist(i,n) = d_star;
        e_gap(i,n) = e_i;

        %% Predictive relative velocity
        vx_p_pred = vx_p + prediction_horizon*acc_p;

        e_v_pred = vx_p_pred-vx;

        e_v_pred = max(min( ...
            e_v_pred,max_velocity_error_for_control), ...
            -max_velocity_error_for_control);

        e_v(i,n) = e_v_pred;

        %% Coupled sliding surface
        ur = k1*e_i;

        intOfE(i) = intOfE(i) + ur*Tstep;
        intOfE(i) = max(min(intOfE(i),20),-20);

        s(i,n) = e_i + epsilon*intOfE(i);

        if i == 1
            s_pred = 0;
        else
            s_pred = s(i-1,n);
        end

        S(i,n) = q*s_pred-s(i,n);

        %% Gap-error derivative for damping
        e_gap_dot_raw = (e_i-e_gap(i,n-1))/Tstep;

        if n == 2
            e_gap_dot = 0;
        else
            e_gap_dot = 0.90*e_gap_dot_hist(i,n-1) ...
                       + 0.10*e_gap_dot_raw;
        end

        e_gap_dot = max(min(e_gap_dot,3.0),-3.0);
        e_gap_dot_hist(i,n) = e_gap_dot;

        %% Leader acceleration preview
        if useLeaderAccelPreview && i == 1
            leader_tx_idx = max(n-comm_delay_steps,1);
            aL_pred = a0(leader_tx_idx);
        else
            aL_pred = 0;
        end

        %% Longitudinal controller
        Ui_slide = 0;
        % Baseline cases use a conventional CACC law. Cases 3-4 use the
        % tuned robust sliding formulation already validated in the
        % single-case simulation.
        if use_robust_sliding

            % Tuned robust CTH feedback used in the validated
            % single-case simulation.  The signs are physically consistent:
            %   e_v_pred > 0  -> predecessor faster -> accelerate
            %   e_i > 0       -> follower too close -> decelerate
            Ui_fb = ...
                  k_rel_v*e_v_pred ...
                - k_gap*e_i ...
                - k_d_gap*e_gap_dot;

            % Sliding term is deliberately kept small so it acts as a
            % robustness correction rather than dominating the CTH loop.
            Ui_slide = -k_slide*sigpow(S(i,n),ppp);

            Ui_af = k_af*acc_p;

        else

            % Conventional CACC: spacing + relative-speed feedback and
            % predecessor acceleration feedforward. The same CTH policy
            % and the same communication degradation are used for fairness.
            % Stable conventional CACC baseline. For the CTH model used here,
            % k_v must be high enough relative to h to avoid an unstable
            % gap/relative-speed mode under the 0.20-s communication delay.
            k_cacc_gap = 0.32;
            k_cacc_v   = 0.95;
            k_cacc_af  = 0.45;

            Ui_fb = -k_cacc_gap*e_i + k_cacc_v*e_v_pred;
            Ui_af = k_cacc_af*acc_p;

        end

        Ui_afL = k_af_leader*aL_pred;

        if t < FTDO_warmup || ~useFTDO
            Ui_dist = 0;
        else
            Ui_dist = ...
                -observer_comp_gain*d_a_hat ...
                -observer_comp_gain*d_v_hat/max(h_i,0.45);
        end

        Ui_vx_track = 0;

        if useVxTrackPatch && t >= 9 && t <= 16
            e_vx_leader = v0_hist(n)-vx;
            e_vx_leader = max(min(e_vx_leader,vx_err_max),-vx_err_max);

            w_vx = sin(pi*(t-9)/7)^2;

            Ui_vx_track = ...
                k_vx_1015*w_vx*e_vx_leader;
        end

        % Leader preview is only active in the full proposed case.
        if ~useLeaderAccelPreview
            Ui_afL = 0;
        end

        if use_robust_sliding
            % Validated proposed controller: no extra road-load bias.
            Ui_des = Ui_fb + Ui_slide + Ui_af + Ui_afL ...
                   + Ui_dist + Ui_vx_track;
        else
            % The allocator already converts desired acceleration to the
            % required longitudinal force including aerodynamic and rolling
            % resistance. Therefore zero feedback command is the correct
            % constant-speed equilibrium; do NOT add a_eq0 here.
            Ui_des = Ui_fb + Ui_af + Ui_afL ...
                   + Ui_dist + Ui_vx_track;
        end

        Ui_af_hist(i,n) = Ui_af;
        Ui_af_leader_hist(i,n) = Ui_afL;
        Ui_kv_hist(i,n) = Ui_fb;
        if use_robust_sliding
            Ui_kv_hist(i,n) = k_rel_v*e_v_pred;
            Ui_gap_hist(i,n) = -k_gap*e_i - k_d_gap*e_gap_dot;
        else
            Ui_gap_hist(i,n) = -k_cacc_gap*e_i;
        end
        Ui_dist_hist(i,n) = Ui_dist;
        Ui_vx_track_hist(i,n) = Ui_vx_track;

        Ui_des = max(min(Ui_des,aM),am);

        if n == 2
            Ui_prev = 0;
        else
            Ui_prev = u_cmd(i,n-1);
        end

        max_step = max_jerk*Tstep;

        Ui = max(min(Ui_des,Ui_prev+max_step), ...
                 Ui_prev-max_step);

        Ui = max(min(Ui,aM),am);

        u_cmd(i,n) = Ui;

        %% DLC lateral controller
        X0v = X0_vehicle_start( ...
            i,X_leader_initial,d_initial);

        X_start_i = X0v+x_lane_start;
        X_end_i   = X0v+x_lane_end;

        [y_ref_actual,~,~,~,~] = dlc_reference( ...
            Xpos,X_start_i,X_end_i,A_lane,k_lane);

        X_lookahead = Xpos+lookahead;

        [y_ref,~,~,psi_ref,kappa_ref] = dlc_reference( ...
            X_lookahead,X_start_i,X_end_i,A_lane,k_lane);

        e_y = y_ref-Ypos;

        e_psi = atan2( ...
            sin(psi_ref-psi), ...
            cos(psi_ref-psi));

        delta_ff = atan(p.L_t*kappa_ref);

        vx_safe = max(abs(vx),2.0);

        delta_cte = atan2(K_stanley*e_y,vx_safe);

        delta_f = ...
              delta_ff ...
            + delta_cte ...
            + Kpsi*e_psi ...
            - Kr*r_t ...
            - Kvy*vy ...
            - Kgamma*gamma ...
            - Krs*r_s;

        delta_f = max(min(delta_f,delta_limit),-delta_limit);

        y_ref_hist(i,n) = y_ref_actual;
        psi_ref_hist(i,n) = psi_ref;
        kappa_ref_hist(i,n) = kappa_ref;

        ey_hist(i,n) = e_y;
        epsi_hist(i,n) = e_psi;
        delta_hist(i,n) = delta_f;

        %% Full control allocation
        u16 = allocate_control_onefile( ...
            Ui,delta_f,max(vx,1.0),r_t,phi,p);

        u16(1) = max(min(u16(1),delta_limit),-delta_limit);

        u16(2) = max(min( ...
            u16(2),p.Fx_cmd_max*p.R_f),0);

        u16_hist(i,n,:) = u16(:).';

        %% Control logs
        Tdrive_hist(i,n) = u16(2);
        Mact_hist(i,n) = u16(3);
        Tb_fL_hist(i,n) = u16(4);
        Tb_fR_hist(i,n) = u16(5);
        Tb_rL_hist(i,n) = u16(6);
        Tb_rR_hist(i,n) = u16(7);
        Tb_sL_hist(i,n) = u16(8);
        Tb_sR_hist(i,n) = u16(9);

        Tbrake_total_hist(i,n) = ...
            u16(4)+u16(5)+u16(6)+ ...
            u16(7)+u16(8)+u16(9);

        brake_asym_front(i,n) = u16(5)-u16(4);
        brake_asym_rear(i,n)  = u16(7)-u16(6);

        if printControlSignals && ...
                mod(n-1,print_interval_steps)==0

            fprintf(['t=%6.2fs | Veh %d | delta=%7.3f deg | ' ...
                     'Tdrv=%9.2f | Mact=%9.2f | ' ...
                     'fL=%8.2f fR=%8.2f rL=%8.2f rR=%8.2f ' ...
                     'sL=%8.2f sR=%8.2f | a=%6.3f | egap=%7.3f\n'], ...
                t,i,rad2deg(u16(1)),u16(2),u16(3), ...
                u16(4),u16(5),u16(6),u16(7),u16(8),u16(9), ...
                Ui,e_i);
        end

        %% 16-DOF plant
        [dxdt,yplant,Fz] = plant_16dof_onefile( ...
            x_prev,u16,p);

        ax_actual_hist(i,n) = dxdt(1);

        % External disturbances.
        dxdt(8) = dxdt(8)+Dv;
        dxdt(1) = dxdt(1)+Da;

        dxdt(~isfinite(dxdt)) = 0;

        x_next = x_prev+dxdt*Tstep;

        x_next(1) = max(x_next(1),0.1);

        % Keep wheel speeds finite and non-negative.
        x_next(9:14) = max(x_next(9:14),0);

        x16(i,n,:) = x_next;

        u_actual(i,n) = dxdt(1);

        X_hist(i,n) = x_next(8);
        Y_hist(i,n) = x_next(7);

        u16_prev(i,:) = u16(:).';

        % Physical LTR from wheel loads.
        LTR_f_hist(i,n) = yplant(6);
        LTR_r_hist(i,n) = yplant(7);

        Fzf_L_hist(i,n) = Fz(1);
        Fzf_R_hist(i,n) = Fz(2);
        Fzr_L_hist(i,n) = Fz(3);
        Fzr_R_hist(i,n) = Fz(4);
        Fzs_L_hist(i,n) = Fz(5);
        Fzs_R_hist(i,n) = Fz(6);

    end
end

%% 12. POST-PROCESSING
vx_16 = squeeze(x16(:,:,1));
vy_16 = squeeze(x16(:,:,2));
r_16 = squeeze(x16(:,:,3));
gamma_16 = squeeze(x16(:,:,5));
Y_16 = squeeze(x16(:,:,7));
X_16 = squeeze(x16(:,:,8));
phi_16 = squeeze(x16(:,:,15));

velocity_error_to_leader = zeros(Nveh,Nstep);

for i = 1:Nveh
    velocity_error_to_leader(i,:) = vx_16(i,:) - v0_hist;
end

% Desired DLC reference for plotting.
Xref = linspace(0,180,1801);
Yref = zeros(size(Xref));

for k = 1:numel(Xref)
    [Yref(k),~,~,~,~] = dlc_reference( ...
        Xref(k),x_lane_start,x_lane_end,A_lane,k_lane);
end

Xrel = zeros(Nveh,Nstep);

for i = 1:Nveh
    Xrel(i,:) = X_16(i,:) - ...
        (X_leader_initial-i*d_initial);
end

%% Performance metrics
rms_gap_error = zeros(Nveh,1);
max_gap_error = zeros(Nveh,1);
rms_velocity_error = zeros(Nveh,1);
max_velocity_error = zeros(Nveh,1);
rms_lat_error = zeros(Nveh,1);
max_lat_error = zeros(Nveh,1);
max_steering = zeros(Nveh,1);
max_roll = zeros(Nveh,1);
max_gamma = zeros(Nveh,1);
packet_loss_rate = zeros(Nveh,1);
rms_obs_v = zeros(Nveh,1);
rms_obs_a = zeros(Nveh,1);

max_LTR_f = zeros(Nveh,1);
max_LTR_r = zeros(Nveh,1);
rms_LTR_f = zeros(Nveh,1);
rms_LTR_r = zeros(Nveh,1);

peak_Tdrive = zeros(Nveh,1);
peak_Tbrake = zeros(Nveh,1);
peak_Mact = zeros(Nveh,1);

for i = 1:Nveh

    rms_gap_error(i) = sqrt(mean(e_gap(i,:).^2));
    max_gap_error(i) = max(abs(e_gap(i,:)));

    rms_velocity_error(i) = ...
        sqrt(mean(velocity_error_to_leader(i,:).^2));

    max_velocity_error(i) = ...
        max(abs(velocity_error_to_leader(i,:)));

    rms_lat_error(i) = sqrt(mean(ey_hist(i,:).^2));
    max_lat_error(i) = max(abs(ey_hist(i,:)));

    max_steering(i) = max(abs(rad2deg(delta_hist(i,:))));
    max_roll(i) = max(abs(rad2deg(phi_16(i,:))));
    max_gamma(i) = max(abs(rad2deg(gamma_16(i,:))));

    packet_loss_rate(i) = 100*mean(packet_lost(i,:));

    if useFTDO
        rms_obs_v(i) = sqrt(mean((D_v_true(i,:)-D_v_hat(i,:)).^2));
        rms_obs_a(i) = sqrt(mean((D_a_true(i,:)-D_a_hat(i,:)).^2));
    else
        rms_obs_v(i) = NaN;
        rms_obs_a(i) = NaN;
    end

    max_LTR_f(i) = max(abs(LTR_f_hist(i,:)));
    max_LTR_r(i) = max(abs(LTR_r_hist(i,:)));

    rms_LTR_f(i) = sqrt(mean(LTR_f_hist(i,:).^2));
    rms_LTR_r(i) = sqrt(mean(LTR_r_hist(i,:).^2));

    peak_Tdrive(i) = max(abs(Tdrive_hist(i,:)));
    peak_Tbrake(i) = max(abs(Tbrake_total_hist(i,:)));
    peak_Mact(i) = max(abs(Mact_hist(i,:)));

end

idx_ss = T >= 5;

string_rms_ratio = zeros(1,Nveh);

ref_rms = sqrt(mean( ...
    a0(idx_ss).^2));

if ref_rms < 1e-8
    ref_rms = sqrt(mean( ...
        (v0_hist(idx_ss)-v0_hist(find(idx_ss,1))).^2));
end

for i = 1:Nveh
    if i == 1
        num = sqrt(mean( ...
            (vx_16(i,idx_ss)-v0_hist(idx_ss)).^2));
        den = sqrt(mean( ...
            (v0_hist(idx_ss)-v0_hist(find(idx_ss,1))).^2));
    else
        num = sqrt(mean( ...
            (vx_16(i,idx_ss)-vx_16(i-1,idx_ss)).^2));
        den = sqrt(mean( ...
            (vx_16(i-1,idx_ss)-v0_hist(idx_ss)).^2));
    end

    if den < 1e-8
        string_rms_ratio(i) = NaN;
    else
        string_rms_ratio(i) = num/den;
    end
end

%% 13. JOURNAL FIGURES
if doPic

    colors = lines(Nveh);

    %% Figure 1 - longitudinal position
    figure(1); clf; hold on;
    for i = 1:Nveh
        plot(T,X_16(i,:),'LineWidth',1.5, ...
            'Color',colors(i,:), ...
            'DisplayName',sprintf('Vehicle %d',i));
    end
    plot(T,X0,'k--','LineWidth',2,'DisplayName','Leader');
    grid on; box on;
    xlabel('Time (s)'); ylabel('X (m)');
    title('Longitudinal Position');
    legend('Location','best');

    %% Figure 2 - velocity
    figure(2); clf; hold on;
    for i = 1:Nveh
        plot(T,vx_16(i,:),'LineWidth',1.6, ...
            'Color',colors(i,:), ...
            'DisplayName',sprintf('Vehicle %d',i));
    end
    plot(T,v0_hist,'k--','LineWidth',2,'DisplayName','Leader');
    grid on; box on;
    xlabel('Time (s)'); ylabel('v_x (m/s)');
    title('Longitudinal Velocity Tracking');
    legend('Location','best');

    %% Figure 3 - velocity error
    figure(3); clf; hold on;
    for i = 1:Nveh
        plot(T,velocity_error_to_leader(i,:), ...
            'LineWidth',1.4,'Color',colors(i,:), ...
            'DisplayName',sprintf('Vehicle %d',i));
    end
    yline(0,'k--');
    grid on; box on;
    xlabel('Time (s)'); ylabel('v_i-v_0 (m/s)');
    title('Velocity Error to Leader');
    legend('Location','best');

    %% Figure 4 - gap error
    figure(4); clf; hold on;
    for i = 1:Nveh
        plot(T,e_gap(i,:),'LineWidth',1.4, ...
            'Color',colors(i,:), ...
            'DisplayName',sprintf('Vehicle %d',i));
    end
    yline(0,'k--');
    grid on; box on;
    xlabel('Time (s)'); ylabel('e_d (m)');
    title('CTH Gap Error');
    legend('Location','best');

    %% Figure 5 - actual acceleration
    figure(5); clf; hold on;
    for i = 1:Nveh
        plot(T,u_actual(i,:),'LineWidth',1.4, ...
            'Color',colors(i,:), ...
            'DisplayName',sprintf('Vehicle %d',i));
    end
    yline(aM,'k--','DisplayName','a_{max}');
    yline(am,'k-.','DisplayName','a_{min}');
    grid on; box on;
    xlabel('Time (s)'); ylabel('a_x (m/s^2)');
    title('Actual Longitudinal Acceleration');
    legend('Location','best');

    %% Figure 6 - time headway
    figure(6); clf; hold on;
    for i = 1:Nveh
        plot(T,h_hist(i,:),'LineWidth',1.4, ...
            'Color',colors(i,:), ...
            'DisplayName',sprintf('Vehicle %d',i));
    end
    yline(h0,'k--','DisplayName','h_0');
    yline(h_min,'k:','DisplayName','h_{min}');
    yline(h_max,'k-.','DisplayName','h_{max}');
    grid on; box on;
    xlabel('Time (s)'); ylabel('h_i (s)');
    title('Variable Time Headway');
    legend('Location','best');

    %% Figure 7 - DLC trajectory
    figure(7); clf; hold on;
    plot(Xref,Yref,'k--','LineWidth',2.4, ...
        'DisplayName','Desired Path');
    for i = 1:Nveh
        plot(Xrel(i,:),Y_16(i,:),'LineWidth',1.6, ...
            'Color',colors(i,:), ...
            'DisplayName',sprintf('Vehicle %d',i));
    end
    grid on; box on;
    xlabel('Relative X (m)'); ylabel('Y (m)');
    title('Double Lane Change Tracking');
    xlim([0 180]);
    legend('Location','best');

    %% Figure 8 - lateral error
    figure(8); clf; hold on;
    for i = 1:Nveh
        plot(T,ey_hist(i,:),'LineWidth',1.4, ...
            'Color',colors(i,:), ...
            'DisplayName',sprintf('Vehicle %d',i));
    end
    yline(0,'k--');
    grid on; box on;
    xlabel('Time (s)'); ylabel('e_y (m)');
    title('Lateral Tracking Error');
    legend('Location','best');

    %% Figure 9 - steering
    figure(9); clf; hold on;
    for i = 1:Nveh
        plot(T,rad2deg(delta_hist(i,:)), ...
            'LineWidth',1.4,'Color',colors(i,:), ...
            'DisplayName',sprintf('Vehicle %d',i));
    end
    yline(rad2deg(delta_limit),'k--','DisplayName','+ limit');
    yline(-rad2deg(delta_limit),'k-.','DisplayName','- limit');
    grid on; box on;
    xlabel('Time (s)'); ylabel('\delta_f (deg)');
    title('Front Steering');
    legend('Location','best');

    %% Figure 10 - yaw rate
    figure(10); clf; hold on;
    for i = 1:Nveh
        plot(T,rad2deg(r_16(i,:)), ...
            'LineWidth',1.4,'Color',colors(i,:), ...
            'DisplayName',sprintf('Vehicle %d',i));
    end
    grid on; box on;
    xlabel('Time (s)'); ylabel('r_t (deg/s)');
    title('Tractor Yaw Rate');
    legend('Location','best');

    %% Figure 11 - articulation
    figure(11); clf; hold on;
    for i = 1:Nveh
        plot(T,rad2deg(gamma_16(i,:)), ...
            'LineWidth',1.4,'Color',colors(i,:), ...
            'DisplayName',sprintf('Vehicle %d',i));
    end
    yline(0,'k--');
    yline(rad2deg(p.gamma_max),'k:','DisplayName','\gamma limit');
    yline(-rad2deg(p.gamma_max),'k-.');
    grid on; box on;
    xlabel('Time (s)'); ylabel('\gamma (deg)');
    title('Articulation Angle');
    legend('Location','best');

    %% Figure 12 - roll
    figure(12); clf; hold on;
    for i = 1:Nveh
        plot(T,rad2deg(phi_16(i,:)), ...
            'LineWidth',1.4,'Color',colors(i,:), ...
            'DisplayName',sprintf('Vehicle %d',i));
    end
    yline(0,'k--');
    grid on; box on;
    xlabel('Time (s)'); ylabel('\phi_t (deg)');
    title('Tractor Roll Angle');
    legend('Location','best');

    %% Figure 13 - physical LTR
    figure(13); clf; hold on;
    for i = 1:Nveh
        plot(T,LTR_f_hist(i,:), ...
            'LineWidth',1.5,'Color',colors(i,:), ...
            'DisplayName',sprintf('V%d front axle',i));
        plot(T,LTR_r_hist(i,:), ...
            '--','LineWidth',1.1,'Color',colors(i,:), ...
            'HandleVisibility','off');
    end
    yline(p.LTR_max,'k--','LineWidth',1.5, ...
        'DisplayName','LTR limit');
    yline(-p.LTR_max,'k:','HandleVisibility','off');
    grid on; box on;
    xlabel('Time (s)'); ylabel('LTR');
    title('Physical Load Transfer Ratio');
    legend('Location','best');

    %% Figure 14 - LTR envelopes
    figure(14); clf; hold on;
    plot(T,max(abs(LTR_f_hist),abs(LTR_r_hist)), ...
        'LineWidth',1.7,'DisplayName','max(|LTR_f|,|LTR_r|)');
    yline(p.LTR_max,'k--','LineWidth',1.5, ...
        'DisplayName','LTR limit');
    grid on; box on;
    xlabel('Time (s)'); ylabel('|LTR|');
    title('Maximum Axle LTR Envelope');
    legend('Location','best');

    %% Figure 15 - FTDO dv
    figure(15); clf; hold on;
    plot(T,D_v_true(1,:),'k--','LineWidth',1.7, ...
        'DisplayName','True d_v');
    plot(T,D_v_hat(1,:),'LineWidth',1.4, ...
        'DisplayName','Estimated d_v');
    grid on; box on;
    xlabel('Time (s)'); ylabel('d_v');
    title('FTDO - d_v');
    legend('Location','best');

    %% Figure 16 - FTDO da
    figure(16); clf; hold on;
    plot(T,D_a_true(1,:),'k--','LineWidth',1.7, ...
        'DisplayName','True d_a');
    plot(T,D_a_hat(1,:),'LineWidth',1.4, ...
        'DisplayName','Estimated d_a');
    grid on; box on;
    xlabel('Time (s)'); ylabel('d_a');
    title('FTDO - d_a');
    legend('Location','best');

    %% Figure 17 - packet loss map
    figure(17); clf;
    imagesc(T,1:Nveh,packet_lost);
    axis xy;
    grid on; box on;
    xlabel('Time (s)'); ylabel('Vehicle');
    title('V2V Packet Loss Map');
    colorbar;

    %% Figure 18 - communication age
    figure(18); clf; hold on;
    for i = 1:Nveh
        plot(T,comm_delay_hist(i,:), ...
            'LineWidth',1.3,'Color',colors(i,:), ...
            'DisplayName',sprintf('Vehicle %d',i));
    end
    yline(tau_comm,'k--','DisplayName','Nominal delay');
    grid on; box on;
    xlabel('Time (s)'); ylabel('Age (s)');
    title('V2V Information Age');
    legend('Location','best');

    %% Figure 19 - drive torque
    figure(19); clf; hold on;
    for i = 1:Nveh
        plot(T,Tdrive_hist(i,:), ...
            'LineWidth',1.5,'Color',colors(i,:), ...
            'DisplayName',sprintf('Vehicle %d',i));
    end
    grid on; box on;
    xlabel('Time (s)'); ylabel('T_{drive} (Nm)');
    title('Front-Axle Drive Torque');
    legend('Location','best');

    %% Figure 20 - front brake torques
    figure(20); clf; hold on;
    plot(T,Tb_fL_hist(1,:),'LineWidth',1.5, ...
        'DisplayName','V1 FL');
    plot(T,Tb_fR_hist(1,:),'--','LineWidth',1.5, ...
        'DisplayName','V1 FR');
    grid on; box on;
    xlabel('Time (s)'); ylabel('Brake torque (Nm)');
    title('V1 Front Wheel Brake Torques');
    legend('Location','best');

    %% Figure 21 - tractor rear brake torques
    figure(21); clf; hold on;
    plot(T,Tb_rL_hist(1,:),'LineWidth',1.5, ...
        'DisplayName','V1 RL');
    plot(T,Tb_rR_hist(1,:),'--','LineWidth',1.5, ...
        'DisplayName','V1 RR');
    grid on; box on;
    xlabel('Time (s)'); ylabel('Brake torque (Nm)');
    title('V1 Tractor Rear Brake Torques');
    legend('Location','best');

    %% Figure 22 - semi-trailer brake torques
    figure(22); clf; hold on;
    plot(T,Tb_sL_hist(1,:),'LineWidth',1.5, ...
        'DisplayName','V1 SL');
    plot(T,Tb_sR_hist(1,:),'--','LineWidth',1.5, ...
        'DisplayName','V1 SR');
    grid on; box on;
    xlabel('Time (s)'); ylabel('Brake torque (Nm)');
    title('V1 Semi-Trailer Brake Torques');
    legend('Location','best');

    %% Figure 23 - active anti-roll moment
    figure(23); clf; hold on;
    for i = 1:Nveh
        plot(T,Mact_hist(i,:), ...
            'LineWidth',1.4,'Color',colors(i,:), ...
            'DisplayName',sprintf('Vehicle %d',i));
    end
    yline(p.M_act_max,'k--','DisplayName','+M_{act,max}');
    yline(-p.M_act_max,'k-.','DisplayName','-M_{act,max}');
    grid on; box on;
    xlabel('Time (s)'); ylabel('M_{act} (Nm)');
    title('Active Anti-Roll Moment');
    legend('Location','best');

    %% Figure 24 - total brake torque
    figure(24); clf; hold on;
    for i = 1:Nveh
        plot(T,Tbrake_total_hist(i,:), ...
            'LineWidth',1.4,'Color',colors(i,:), ...
            'DisplayName',sprintf('Vehicle %d',i));
    end
    grid on; box on;
    xlabel('Time (s)'); ylabel('Total brake torque (Nm)');
    title('Total Brake Torque');
    legend('Location','best');

    %% Figure 25 - front differential braking
    figure(25); clf; hold on;
    plot(T,brake_asym_front(1,:), ...
        'LineWidth',1.5,'DisplayName','FL-FR');
    plot(T,brake_asym_rear(1,:), ...
        '--','LineWidth',1.5,'DisplayName','RL-RR');
    yline(0,'k:');
    grid on; box on;
    xlabel('Time (s)'); ylabel('\Delta T_b (Nm)');
    title('V1 Differential-Braking Torque Asymmetry');
    legend('Location','best');

    %% Figure 26 - wheel vertical loads
    figure(26); clf; hold on;
    plot(T,Fzf_L_hist(1,:),'LineWidth',1.3,'DisplayName','Fz fL');
    plot(T,Fzf_R_hist(1,:),'--','LineWidth',1.3,'DisplayName','Fz fR');
    plot(T,Fzr_L_hist(1,:),'LineWidth',1.3,'DisplayName','Fz rL');
    plot(T,Fzr_R_hist(1,:),'--','LineWidth',1.3,'DisplayName','Fz rR');
    plot(T,Fzs_L_hist(1,:),'LineWidth',1.3,'DisplayName','Fz sL');
    plot(T,Fzs_R_hist(1,:),'--','LineWidth',1.3,'DisplayName','Fz sR');
    grid on; box on;
    xlabel('Time (s)'); ylabel('F_z (N)');
    title('V1 Wheel Vertical Loads');
    legend('Location','best');

    %% Figure 27 - V1 longitudinal controller components
    figure(27); clf;
    tiledlayout(3,1);

    nexttile; hold on;
    plot(T,u_cmd(1,:),'LineWidth',1.4,'DisplayName','U_i');
    plot(T,u_actual(1,:),'LineWidth',1.4,'DisplayName','a_x actual');
    yline(0,'k:');
    grid on; box on;
    ylabel('m/s^2');
    title('V1 Command vs Plant Acceleration');
    legend('Location','best');

    nexttile;
    plot(T,Tdrive_hist(1,:),'LineWidth',1.4);
    grid on; box on;
    ylabel('Nm');
    title('V1 Drive Torque');

    nexttile; hold on;
    plot(T,Ui_af_hist(1,:),'LineWidth',1.1,'DisplayName','pred a');
    plot(T,Ui_af_leader_hist(1,:),'LineWidth',1.1,'DisplayName','leader a');
    plot(T,Ui_kv_hist(1,:),'LineWidth',1.1,'DisplayName','FB');
    plot(T,Ui_gap_hist(1,:),'LineWidth',1.1,'DisplayName','gap');
    plot(T,Ui_dist_hist(1,:),'LineWidth',1.1,'DisplayName','FTDO');
    plot(T,Ui_vx_track_hist(1,:),'LineWidth',1.1,'DisplayName','vx-track');
    yline(0,'k:');
    grid on; box on;
    xlabel('Time (s)'); ylabel('m/s^2');
    title('V1 Longitudinal Controller Components');
    legend('Location','best');

end

%% 14. CONSOLE SUMMARY
fprintf('\n');
fprintf('===============================================================\n');
fprintf(' CASE: %s\n',controller_name);
fprintf('===============================================================\n');

fprintf(['FTDO=%s | delay=%s | loss=%s | varHeadway=%s | ' ...
         'leaderPreview=%s\n'], ...
    onoff(useFTDO),onoff(useCommDelay), ...
    onoff(usePacketLoss),onoff(useVariableHeadway), ...
    onoff(useLeaderAccelPreview));

fprintf('\nController parameters:\n');
fprintf('  a_eq0              = %.6f m/s^2\n',a_eq0);
fprintf('  h0/h_min/h_max     = %.3f / %.3f / %.3f s\n',h0,h_min,h_max);
fprintf('  k_gap              = %.4f\n',k_gap);
fprintf('  k_af               = %.4f\n',k_af);
fprintf('  k_af_leader        = %.4f\n',k_af_leader);
fprintf('  k_slide            = %.4f\n',k_slide);
fprintf('  max_jerk           = %.4f m/s^3\n',max_jerk);
fprintf('  leader t_ramp      = %.3f s\n',1.5);
fprintf('  communication      = %.3f s\n',tau_comm);
fprintf('  packet loss        = %.2f %%\n',100*packet_loss_prob);

fprintf('\nPerformance:\n');

for i = 1:Nveh
    fprintf(['V%d: RMS vx err=%8.4f | MAX vx err=%8.4f | ' ...
             'RMS gap=%8.4f | RMS lat=%8.4f | loss=%6.2f %%\n'], ...
        i,rms_velocity_error(i),max_velocity_error(i), ...
        rms_gap_error(i),rms_lat_error(i),packet_loss_rate(i));

    fprintf(['    max|gamma|=%7.3f deg | max|roll|=%7.3f deg | ' ...
             'max|LTRf|=%6.3f | max|LTRr|=%6.3f\n'], ...
        max_gamma(i),max_roll(i),max_LTR_f(i),max_LTR_r(i));

    fprintf(['    peak Tdrive=%9.2f Nm | peak Tbrake=%9.2f Nm | ' ...
             'peak Mact=%9.2f Nm\n'], ...
        peak_Tdrive(i),peak_Tbrake(i),peak_Mact(i));
end

fprintf('\nString RMS ratios:\n');
fprintf('  ');
fprintf('%.4f ',string_rms_ratio);
fprintf('\n');

fprintf('===============================================================\n');

%% 15. EXPORT
% Build the results struct regardless of exportResults so the comparison
% driver can always collect the case results in memory.
results = struct();
results.meta.case_id = caseCfg.id;
results.meta.case_tag = caseCfg.tag;
results.meta.controller_name = controller_name;

if exportResults

    if ~exist(export_folder,'dir')
        mkdir(export_folder);
    end

    timestamp = datestr(now,'yyyymmdd_HHMMSS');

    results.meta.Nveh = Nveh;
    results.meta.Tstep = Tstep;
    results.meta.SimTime = SimTime;
    results.meta.tau_comm = tau_comm;
    results.meta.packet_loss_prob = packet_loss_prob;
    results.meta.h0 = h0;
    results.meta.h_min = h_min;
    results.meta.h_max = h_max;
    results.meta.d0 = d0;
    results.meta.ctrl_channel_names = ctrl_channel_names;
    results.meta.LTR_max = p.LTR_max;
    results.meta.case_id = caseCfg.id;
    results.meta.case_tag = caseCfg.tag;
    results.meta.controller_name = controller_name;

    results.time.T = T;

    results.leader.X = X0;
    results.leader.v = v0_hist;
    results.leader.a = a0;
    results.leader.Y = Y0;
    results.leader.psi = psi0;

    results.state.X = X_16;
    results.state.Y = Y_16;
    results.state.vx = vx_16;
    results.state.vy = vy_16;
    results.state.r = r_16;
    results.state.gamma = gamma_16;
    results.state.phi = phi_16;

    results.control.u_cmd = u_cmd;
    results.control.u_actual = u_actual;
    results.control.u16 = u16_hist;

    results.control_breakdown.Ui_predecessor_af = Ui_af_hist;
    results.control_breakdown.Ui_leader_preview = Ui_af_leader_hist;
    results.control_breakdown.Ui_feedback = Ui_kv_hist;
    results.control_breakdown.Ui_gap = Ui_gap_hist;
    results.control_breakdown.Ui_ftdo_comp = Ui_dist_hist;
    results.control_breakdown.Ui_vx_track = Ui_vx_track_hist;

    results.control_breakdown.Tdrive = Tdrive_hist;
    results.control_breakdown.Mact = Mact_hist;
    results.control_breakdown.Tb_fL = Tb_fL_hist;
    results.control_breakdown.Tb_fR = Tb_fR_hist;
    results.control_breakdown.Tb_rL = Tb_rL_hist;
    results.control_breakdown.Tb_rR = Tb_rR_hist;
    results.control_breakdown.Tb_sL = Tb_sL_hist;
    results.control_breakdown.Tb_sR = Tb_sR_hist;
    results.control_breakdown.Tbrake_total = Tbrake_total_hist;
    results.control_breakdown.brake_asym_front = brake_asym_front;
    results.control_breakdown.brake_asym_rear = brake_asym_rear;

    results.errors.e_gap = e_gap;
    results.errors.e_v = e_v;
    results.errors.ey = ey_hist;
    results.errors.epsi = epsi_hist;
    results.errors.delta = delta_hist;
    results.errors.vel_err_to_leader = velocity_error_to_leader;

    results.headway.h = h_hist;
    results.headway.d_star = dstar_hist;

    results.ftdo.Dv_true = D_v_true;
    results.ftdo.Da_true = D_a_true;
    results.ftdo.Dv_hat = D_v_hat;
    results.ftdo.Da_hat = D_a_hat;

    results.comm.packet_received = packet_received;
    results.comm.packet_lost = packet_lost;
    results.comm.age = comm_delay_hist;

    results.safety.LTR_front = LTR_f_hist;
    results.safety.LTR_rear = LTR_r_hist;
    results.safety.Fz_fL = Fzf_L_hist;
    results.safety.Fz_fR = Fzf_R_hist;
    results.safety.Fz_rL = Fzr_L_hist;
    results.safety.Fz_rR = Fzr_R_hist;
    results.safety.Fz_sL = Fzs_L_hist;
    results.safety.Fz_sR = Fzs_R_hist;

    results.metrics.rms_gap_error = rms_gap_error;
    results.metrics.max_gap_error = max_gap_error;
    results.metrics.rms_velocity_error = rms_velocity_error;
    results.metrics.max_velocity_error = max_velocity_error;
    results.metrics.rms_lat_error = rms_lat_error;
    results.metrics.max_lat_error = max_lat_error;
    results.metrics.max_steering_deg = max_steering;
    results.metrics.max_roll_deg = max_roll;
    results.metrics.max_gamma_deg = max_gamma;
    results.metrics.packet_loss_rate = packet_loss_rate;
    results.metrics.rms_obs_v = rms_obs_v;
    results.metrics.rms_obs_a = rms_obs_a;
    results.metrics.string_rms_ratio = string_rms_ratio;
    results.metrics.max_LTR_front = max_LTR_f;
    results.metrics.max_LTR_rear = max_LTR_r;
    results.metrics.rms_LTR_front = rms_LTR_f;
    results.metrics.rms_LTR_rear = rms_LTR_r;
    results.metrics.peak_drive_torque = peak_Tdrive;
    results.metrics.peak_brake_torque = peak_Tbrake;
    results.metrics.peak_Mact = peak_Mact;

    mat_path = fullfile(export_folder, ...
        sprintf('platoon_results_%s.mat',timestamp));

    save(mat_path,'results');

    fprintf('\n[Export] MAT saved:\n  %s\n',mat_path);

    %% Per-vehicle time-series CSV
    for i = 1:Nveh

        Ti = table();

        Ti.Time_s = T(:);
        Ti.X_m = X_16(i,:).';
        Ti.Y_m = Y_16(i,:).';
        Ti.vx_mps = vx_16(i,:).';
        Ti.vy_mps = vy_16(i,:).';
        Ti.ax_cmd_mps2 = u_cmd(i,:).';
        Ti.ax_actual_mps2 = u_actual(i,:).';
        Ti.gap_error_m = e_gap(i,:).';
        Ti.vel_error_leader_mps = velocity_error_to_leader(i,:).';
        Ti.headway_s = h_hist(i,:).';
        Ti.lateral_error_m = ey_hist(i,:).';
        Ti.heading_error_rad = epsi_hist(i,:).';
        Ti.steering_deg = rad2deg(delta_hist(i,:)).';
        Ti.yaw_rate_dps = rad2deg(r_16(i,:)).';
        Ti.articulation_deg = rad2deg(gamma_16(i,:)).';
        Ti.roll_deg = rad2deg(phi_16(i,:)).';

        Ti.LTR_front = LTR_f_hist(i,:).';
        Ti.LTR_rear = LTR_r_hist(i,:).';

        Ti.Fz_fL_N = Fzf_L_hist(i,:).';
        Ti.Fz_fR_N = Fzf_R_hist(i,:).';
        Ti.Fz_rL_N = Fzr_L_hist(i,:).';
        Ti.Fz_rR_N = Fzr_R_hist(i,:).';
        Ti.Fz_sL_N = Fzs_L_hist(i,:).';
        Ti.Fz_sR_N = Fzs_R_hist(i,:).';

        Ti.Dv_true = D_v_true(i,:).';
        Ti.Dv_hat = D_v_hat(i,:).';
        Ti.Da_true = D_a_true(i,:).';
        Ti.Da_hat = D_a_hat(i,:).';

        Ti.packet_lost = packet_lost(i,:).';
        Ti.comm_age_s = comm_delay_hist(i,:).';

        Ti.T_drive_Nm = Tdrive_hist(i,:).';
        Ti.M_act_Nm = Mact_hist(i,:).';
        Ti.Tb_fL_Nm = Tb_fL_hist(i,:).';
        Ti.Tb_fR_Nm = Tb_fR_hist(i,:).';
        Ti.Tb_rL_Nm = Tb_rL_hist(i,:).';
        Ti.Tb_rR_Nm = Tb_rR_hist(i,:).';
        Ti.Tb_sL_Nm = Tb_sL_hist(i,:).';
        Ti.Tb_sR_Nm = Tb_sR_hist(i,:).';
        Ti.Tb_total_Nm = Tbrake_total_hist(i,:).';
        Ti.deltaTb_front_Nm = brake_asym_front(i,:).';
        Ti.deltaTb_rear_Nm = brake_asym_rear(i,:).';

        csv_path = fullfile(export_folder, ...
            sprintf('platoon_veh%d_timeseries_%s.csv',i,timestamp));

        writetable(Ti,csv_path);

    end

    %% Tracking summary
    Tsum = table();

    Tsum.Vehicle = (1:Nveh).';
    Tsum.RMS_gap_error_m = rms_gap_error;
    Tsum.Max_gap_error_m = max_gap_error;
    Tsum.RMS_vel_error_mps = rms_velocity_error;
    Tsum.Max_vel_error_mps = max_velocity_error;
    Tsum.RMS_lat_error_m = rms_lat_error;
    Tsum.Max_lat_error_m = max_lat_error;
    Tsum.Max_steering_deg = max_steering;
    Tsum.Max_roll_deg = max_roll;
    Tsum.Max_articulation_deg = max_gamma;
    Tsum.Packet_loss_pct = packet_loss_rate;
    Tsum.RMS_FTDO_dv = rms_obs_v;
    Tsum.RMS_FTDO_da = rms_obs_a;
    Tsum.Max_LTR_front = max_LTR_f;
    Tsum.Max_LTR_rear = max_LTR_r;
    Tsum.RMS_LTR_front = rms_LTR_f;
    Tsum.RMS_LTR_rear = rms_LTR_r;
    Tsum.Peak_drive_torque_Nm = peak_Tdrive;
    Tsum.Peak_brake_torque_Nm = peak_Tbrake;
    Tsum.Peak_Mact_Nm = peak_Mact;

    summary_path = fullfile(export_folder, ...
        sprintf('platoon_summary_metrics_%s.csv',timestamp));

    writetable(Tsum,summary_path);

    %% Control effort summary
    VehCol = [];
    ChannelCol = {};
    MeanCol = [];
    RMSCol = [];
    MaxCol = [];
    MinCol = [];

    for i = 1:Nveh
        for k = 1:numel(ctrl_channel_names)

            sig_k = squeeze(u16_hist(i,:,k));

            VehCol(end+1,1) = i; %#ok<SAGROW>
            ChannelCol{end+1,1} = ctrl_channel_names{k}; %#ok<SAGROW>
            MeanCol(end+1,1) = mean(sig_k); %#ok<SAGROW>
            RMSCol(end+1,1) = sqrt(mean(sig_k.^2)); %#ok<SAGROW>
            MaxCol(end+1,1) = max(sig_k); %#ok<SAGROW>
            MinCol(end+1,1) = min(sig_k); %#ok<SAGROW>

        end
    end

    Tctrl = table();

    Tctrl.Vehicle = VehCol;
    Tctrl.Channel = ChannelCol;
    Tctrl.Mean = MeanCol;
    Tctrl.RMS = RMSCol;
    Tctrl.Max = MaxCol;
    Tctrl.Min = MinCol;

    ctrl_summary_path = fullfile(export_folder, ...
        sprintf('platoon_control_effort_%s.csv',timestamp));

    writetable(Tctrl,ctrl_summary_path);

    fprintf('[Export] CSV files written to:\n  %s\n',export_folder);

end

fprintf('\nSimulation completed.\n');

end % run_platoon_case

%% =========================================================================
% LOCAL FUNCTION 1 - VEHICLE PARAMETERS
%% =========================================================================
function p = vehicle_params_onefile()

p.m_t = 5760;
p.m_s = 6500;
p.m_tot = p.m_t+p.m_s;
p.g = 9.81;

p.I_zt = 34823;
p.I_zs = 179992;
p.I_xxt = 25000;

p.I_wf = 40;
p.I_wr = 40;
p.I_ws = 50;

p.L_ft = 1.100;
p.L_rt = 2.390;
p.L_wt = 1.100;
p.L_fs = 5.210;
p.L_rs = 3.280;
p.L_t = p.L_ft+p.L_rt;

p.T_f = 1.535;
p.T_r = 1.535;
p.T_s = 1.535;

p.R_f = 0.51;
p.R_r = 0.51;
p.R_s = 0.51;

p.C_sigma_f = 125000;
p.C_sigma_r = 150000;
p.C_sigma_s = 150000;

p.C_alpha_f = 164090;
p.C_alpha_r = 164090;
p.C_alpha_s = 164090;

p.mu = 0.85;

p.C_D = 0.30;
p.A_a = 2.0;
p.rho_a = 1.225;

p.a_f = 0.002;
p.a_r = 0.002;
p.a_s = 0.002;

p.h_t = 1.20;
p.h_s = 1.50;

p.K_phi_f = 1.8e5;
p.K_phi_r = 2.2e5;
p.C_phi_f = 1.8e4;
p.C_phi_r = 2.2e4;

p.K_phi = p.K_phi_f+p.K_phi_r;
p.C_phi = p.C_phi_f+p.C_phi_r;

p.F_zf_static = p.m_t*p.g*p.L_rt/p.L_t;
p.F_zr_static = p.m_t*p.g*p.L_ft/p.L_t;
p.F_zs_static = p.m_s*p.g;

p.gamma_max = deg2rad(35);
p.LTR_max = 0.70;
p.delta_max = deg2rad(30);
p.ddelta_max = deg2rad(60);

p.Fx_cmd_max = 40000;
p.Fx_cmd_min = -60000;

p.M_act_max = 15000;

end

%% =========================================================================
% LOCAL FUNCTION 2 - CONTROL ALLOCATION
%% =========================================================================
function u_input = allocate_control_onefile( ...
    a_des,delta_f,v_x,r,roll_angle,p)

u_input = zeros(9,1);

u_input(1) = max(min(delta_f,p.delta_max),-p.delta_max);

epsilon = 1e-3;

if abs(v_x) < epsilon
    v_x_eff = epsilon;
else
    v_x_eff = v_x;
end

F_a = 0.5*p.C_D*p.A_a*p.rho_a*v_x_eff^2;

F_roll = ...
    (p.F_zf_static*p.a_f + ...
     p.F_zr_static*p.a_r + ...
     p.F_zs_static*p.a_s)/p.R_f;

F_req = p.m_tot*a_des + F_a + F_roll;

F_req = max(min(F_req,p.Fx_cmd_max),p.Fx_cmd_min);

if F_req >= 0

    T_drive = F_req*p.R_f;

    u_input(2) = max(min( ...
        T_drive,p.Fx_cmd_max*p.R_f),0);

    u_input(3) = 0;

else

    T_brake_total = abs(F_req)*p.R_f;

    % Differential braking yaw-moment term.
    delta_T = 2000*r;

    Tb_base_f = (0.3*T_brake_total)/2;
    Tb_base_r = (0.3*T_brake_total)/2;
    Tb_base_s = (0.4*T_brake_total)/2;

    u_input(4) = max(Tb_base_f-delta_T,0);
    u_input(5) = max(Tb_base_f+delta_T,0);

    u_input(6) = max(Tb_base_r-delta_T,0);
    u_input(7) = max(Tb_base_r+delta_T,0);

    u_input(8) = Tb_base_s;
    u_input(9) = Tb_base_s;

end

% Preserve the supplied allocator's roll-angle-based LTR proxy.
LTR_proxy = abs(roll_angle)/0.15;

if LTR_proxy > p.LTR_max
    u_input(3) = sign(roll_angle)*p.M_act_max;
else
    u_input(3) = 0;
end

end

%% =========================================================================
% LOCAL FUNCTION 3 - 16-DOF PLANT
%
% Based directly on the supplied plant_16dof.m.
% Additional output Fz contains:
%   [Fzf_L,Fzf_R,Fzr_L,Fzr_R,Fzs_L,Fzs_R]
%% =========================================================================
function [dxdt,y,Fz] = plant_16dof_onefile(x,u,p)

if numel(x) ~= 16
    error('x phai co 16 phan tu.');
end

if numel(u) ~= 9
    error('u phai co 9 phan tu.');
end

epsilon = 1e-3;

m_t = p.m_t;
m_s = p.m_s;
m_tot = p.m_tot;
g = p.g;

I_zt = p.I_zt;
I_zs = p.I_zs;
I_xxt = p.I_xxt;

I_wf = p.I_wf;
I_wr = p.I_wr;
I_ws = p.I_ws;

L_ft = p.L_ft;
L_rt = p.L_rt;
L_wt = p.L_wt;
L_fs = p.L_fs;
L_rs = p.L_rs;
L_t = p.L_t;

T_f = p.T_f;
T_r = p.T_r;
T_s = p.T_s;

R_f = p.R_f;
R_r = p.R_r;
R_s = p.R_s;

C_sigma_f = p.C_sigma_f;
C_sigma_r = p.C_sigma_r;
C_sigma_s = p.C_sigma_s;

C_alpha_f = p.C_alpha_f;
C_alpha_r = p.C_alpha_r;
C_alpha_s = p.C_alpha_s;

mu = p.mu;

C_D = p.C_D;
A_a = p.A_a;
rho_a = p.rho_a;

a_f = p.a_f;
a_r = p.a_r;
a_s = p.a_s;

h_t = p.h_t;

K_phi = p.K_phi;
C_phi = p.C_phi;

F_zf_static = p.F_zf_static;
F_zr_static = p.F_zr_static;
F_zs_static = p.F_zs_static;

%% States
v_x = x(1);
v_y = x(2);
r_t = x(3);
r_s = x(4);
gamma = x(5);
psi = x(6);
Y = x(7); %#ok<NASGU>
X = x(8); %#ok<NASGU>

omega_fL = x(9);
omega_fR = x(10);
omega_rL = x(11);
omega_rR = x(12);
omega_sL = x(13);
omega_sR = x(14);

phi_t = x(15);
dphi_t = x(16);

%% Inputs
delta_f = u(1);
T_f_drive = u(2);
M_act = u(3);

Tb_fL = u(4);
Tb_fR = u(5);
Tb_rL = u(6);
Tb_rR = u(7);
Tb_sL = u(8);
Tb_sR = u(9);

if abs(v_x) < epsilon
    v_x_eff = epsilon; %#ok<NASGU>
else
    v_x_eff = v_x; %#ok<NASGU>
end

%% Local axle velocities
v_yf = v_y + L_ft*r_t;
v_yr = v_y - L_rt*r_t;

v_ys = v_y - L_wt*r_t - r_s*(L_fs+L_rs);
v_xs = v_x*cos(gamma);

%% Wheel longitudinal velocities
v_xfL = v_x - T_f/2*r_t;
v_xfR = v_x + T_f/2*r_t;

v_xrL = v_x - T_r/2*r_t;
v_xrR = v_x + T_r/2*r_t;

v_xsL = v_xs - T_s/2*r_s;
v_xsR = v_xs + T_s/2*r_s;

%% Lateral slip
alpha_f = delta_f - ...
    atan2(v_yf,abs(v_x)+epsilon);

alpha_r = -atan2(v_yr,abs(v_x)+epsilon);

alpha_s = -atan2(v_ys,abs(v_xs)+epsilon);

%% Longitudinal slip
sigma_fL = (R_f*omega_fL-v_xfL)/ ...
    (abs(v_xfL)+epsilon);

sigma_fR = (R_f*omega_fR-v_xfR)/ ...
    (abs(v_xfR)+epsilon);

sigma_rL = (R_r*omega_rL-v_xrL)/ ...
    (abs(v_xrL)+epsilon);

sigma_rR = (R_r*omega_rR-v_xrR)/ ...
    (abs(v_xrR)+epsilon);

sigma_sL = (R_s*omega_sL-v_xsL)/ ...
    (abs(v_xsL)+epsilon);

sigma_sR = (R_s*omega_sR-v_xsR)/ ...
    (abs(v_xsR)+epsilon);

sat = @(s) max(min(s,0.99),-0.99);

sigma_fL = sat(sigma_fL);
sigma_fR = sat(sigma_fR);
sigma_rL = sat(sigma_rL);
sigma_rR = sat(sigma_rR);
sigma_sL = sat(sigma_sL);
sigma_sR = sat(sigma_sR);

%% Aerodynamic drag
F_a = 0.5*C_D*A_a*rho_a*v_x^2*sign(v_x+1e-9);

%% Approximate longitudinal acceleration for load transfer
F_x_guess = ...
    T_f_drive/R_f ...
    - (Tb_fL+Tb_fR+Tb_rL+Tb_rR)/R_f ...
    - F_a;

a_x_est = F_x_guess/m_tot;
a_y_est = v_x*r_t;

%% Longitudinal load transfer
Delta_Fz_long = ...
    m_t*h_t/L_t*a_x_est;

F_zf_long = F_zf_static-Delta_Fz_long;
F_zr_long = F_zr_static+Delta_Fz_long;

%% Roll dynamics
M_lat_roll = m_t*h_t*a_y_est;
M_gravity = m_t*g*h_t*sin(phi_t);
M_suspension = K_phi*phi_t + C_phi*dphi_t;

ddphi_t = ...
    (M_lat_roll-M_gravity-M_suspension+M_act)/I_xxt;

M_roll_total = M_lat_roll-M_gravity;

Delta_Fz_lat_f = 0.50*M_roll_total/T_f;
Delta_Fz_lat_r = 0.50*M_roll_total/T_r;

%% Wheel vertical loads
F_zf_L = max(0.5*F_zf_long+Delta_Fz_lat_f,0);
F_zf_R = max(0.5*F_zf_long-Delta_Fz_lat_f,0);

F_zr_L = max(0.5*F_zr_long+Delta_Fz_lat_r,0);
F_zr_R = max(0.5*F_zr_long-Delta_Fz_lat_r,0);

F_zs_L = 0.5*F_zs_static;
F_zs_R = 0.5*F_zs_static;

%% Tire forces - Dugoff
[Fx_fL,Fy_fL] = calcDugoff( ...
    alpha_f,sigma_fL,F_zf_L,C_sigma_f,C_alpha_f,mu);

[Fx_fR,Fy_fR] = calcDugoff( ...
    alpha_f,sigma_fR,F_zf_R,C_sigma_f,C_alpha_f,mu);

[Fx_rL,Fy_rL] = calcDugoff( ...
    alpha_r,sigma_rL,F_zr_L,C_sigma_r,C_alpha_r,mu);

[Fx_rR,Fy_rR] = calcDugoff( ...
    alpha_r,sigma_rR,F_zr_R,C_sigma_r,C_alpha_r,mu);

[Fx_sL,Fy_sL] = calcDugoff( ...
    alpha_s,sigma_sL,F_zs_L,C_sigma_s,C_alpha_s,mu);

[Fx_sR,Fy_sR] = calcDugoff( ...
    alpha_s,sigma_sR,F_zs_R,C_sigma_s,C_alpha_s,mu);

%% Rolling resistance
Fx_fL = Fx_fL-F_zf_L*a_f/R_f;
Fx_fR = Fx_fR-F_zf_R*a_f/R_f;

Fx_rL = Fx_rL-F_zr_L*a_r/R_r;
Fx_rR = Fx_rR-F_zr_R*a_r/R_r;

Fx_sL = Fx_sL-F_zs_L*a_s/R_s;
Fx_sR = Fx_sR-F_zs_R*a_s/R_s;

%% Axle force sums
Fx_f = Fx_fL+Fx_fR;
Fy_f = Fy_fL+Fy_fR;

Fx_r = Fx_rL+Fx_rR;
Fy_r = Fy_rL+Fy_rR;

Fx_s = Fx_sL+Fx_sR;
Fy_s = Fy_sL+Fy_sR;

%% Body-frame transformations
Fx_f_body = ...
    Fx_f*cos(delta_f)-Fy_f*sin(delta_f);

Fy_f_body = ...
    Fx_f*sin(delta_f)+Fy_f*cos(delta_f);

Fx_s_body = ...
    Fx_s*cos(gamma)+Fy_s*sin(gamma);

Fy_s_body = ...
    Fy_s*cos(gamma)-Fx_s*sin(gamma);

Fx_total = Fx_f_body+Fx_r+Fx_s_body-F_a;
Fy_total = Fy_f_body+Fy_r+Fy_s_body;

%% Yaw moments
M_z_t = ...
    Fy_f_body*L_ft ...
    - Fy_r*L_rt ...
    + (Fx_fR-Fx_fL)*T_f/2 ...
    + (Fx_rR-Fx_rL)*T_r/2;

M_z_s = ...
    -Fy_s_body*L_rs ...
    + (Fx_sR-Fx_sL)*T_s/2;

%% Vehicle dynamics
dv_x = Fx_total/m_tot+v_y*r_t;
dv_y = Fy_total/m_tot-v_x*r_t;

dr_t = M_z_t/I_zt;
dr_s = M_z_s/I_zs;

dgamma = r_t-r_s;
dpsi = r_t;

dY = v_x*sin(psi)+v_y*cos(psi);
dX = v_x*cos(psi)-v_y*sin(psi);

%% Wheel rotational dynamics
domega_fL = ...
    (T_f_drive/2-Tb_fL-Fx_fL*R_f)/I_wf;

domega_fR = ...
    (T_f_drive/2-Tb_fR-Fx_fR*R_f)/I_wf;

domega_rL = ...
    (-Tb_rL-Fx_rL*R_r)/I_wr;

domega_rR = ...
    (-Tb_rR-Fx_rR*R_r)/I_wr;

domega_sL = ...
    (-Tb_sL-Fx_sL*R_s)/I_ws;

domega_sR = ...
    (-Tb_sR-Fx_sR*R_s)/I_ws;

%% State derivative
dxdt = zeros(16,1);

dxdt(1) = dv_x;
dxdt(2) = dv_y;
dxdt(3) = dr_t;
dxdt(4) = dr_s;
dxdt(5) = dgamma;
dxdt(6) = dpsi;
dxdt(7) = dY;
dxdt(8) = dX;

dxdt(9) = domega_fL;
dxdt(10) = domega_fR;
dxdt(11) = domega_rL;
dxdt(12) = domega_rR;
dxdt(13) = domega_sL;
dxdt(14) = domega_sR;

dxdt(15) = dphi_t;
dxdt(16) = ddphi_t;

%% Physical LTR
LTR_f = ...
    (F_zf_L-F_zf_R)/max(F_zf_L+F_zf_R,1);

LTR_r = ...
    (F_zr_L-F_zr_R)/max(F_zr_L+F_zr_R,1);

y = [v_x;r_t;Y;gamma;phi_t;LTR_f;LTR_r];

Fz = [ ...
    F_zf_L; ...
    F_zf_R; ...
    F_zr_L; ...
    F_zr_R; ...
    F_zs_L; ...
    F_zs_R];

end

%% =========================================================================
% LOCAL FUNCTION 4 - DUGOFF TIRE
%% =========================================================================
function [Fx,Fy] = calcDugoff( ...
    alpha,sigma,Fz,C_sigma,C_alpha,mu)

epsilon = 1e-8;

Fz = max(Fz,0);
sigma = max(min(sigma,0.99),-0.99);

denom = 2*sqrt( ...
    (C_sigma*sigma)^2 + ...
    (C_alpha*tan(alpha))^2);

if denom < epsilon
    lambda = 1;
else
    lambda = ...
        mu*Fz*(1-sigma)/denom;
end

if lambda < 1
    f_lambda = lambda*(2-lambda);
else
    f_lambda = 1;
end

Fx = ...
    C_sigma*sigma/(1-sigma)*f_lambda;

Fy = ...
    C_alpha*tan(alpha)/(1-sigma)*f_lambda;

F_total = sqrt(Fx^2+Fy^2);
F_limit = mu*Fz;

if F_total > F_limit && F_total > epsilon
    scale = F_limit/F_total;
    Fx = Fx*scale;
    Fy = Fy*scale;
end

end

%% =========================================================================
% LOCAL FUNCTION 5 - SIGNED POWER
%% =========================================================================
function y = sigpow(x,a)

if x == 0
    y = 0;
else
    y = abs(x)^a*sign(x);
end

end

%% =========================================================================
% LOCAL FUNCTION 6 - LEADER ACCELERATION PROFILE
%% =========================================================================
function a = leader_accel_profile(t,t_ramp)

a_hold = 0.15;
t1 = 2;
t2 = 15;

if t_ramp <= 0

    if t <= t1
        a = 0;
    elseif t <= t2
        a = a_hold;
    else
        a = 0;
    end

    return;
end

if t <= t1

    a = 0;

elseif t <= t1+t_ramp

    frac = (t-t1)/t_ramp;
    a = a_hold*0.5*(1-cos(pi*frac));

elseif t <= t2

    a = a_hold;

elseif t <= t2+t_ramp

    frac = (t-t2)/t_ramp;
    a = a_hold*0.5*(1+cos(pi*frac));

else

    a = 0;

end

end

%% =========================================================================
% LOCAL FUNCTION 7 - DLC REFERENCE
%% =========================================================================
function [y,dy_dx,d2y_dx2,psi_ref,kappa_ref] = ...
    dlc_reference(X,X_start,X_end,A,k)

z1 = k*(X-X_start);
z2 = k*(X-X_end);

t1 = tanh(z1);
t2 = tanh(z2);

s1 = 1-t1^2;
s2 = 1-t2^2;

y = A*(t1-t2);

dy_dx = A*k*(s1-s2);

d2y_dx2 = ...
    A*k^2*(-2*t1*s1+2*t2*s2);

psi_ref = atan2(dy_dx,1);

denominator = ...
    max((1+dy_dx^2)^(3/2),1e-9);

kappa_ref = d2y_dx2/denominator;

end

%% =========================================================================
% LOCAL FUNCTION 8 - VEHICLE INITIAL X
%% =========================================================================
function X0v = X0_vehicle_start( ...
    i,X_leader_initial,d_initial)

X0v = X_leader_initial-i*d_initial;

end

%% =========================================================================
% LOCAL FUNCTION 9 - ON/OFF
%% =========================================================================
function s = onoff(flag)

if flag
    s = 'ON';
else
    s = 'OFF';
end

end
