clear; clc; close all;

%% ========================================================================
% PLOT_FROM_MAT_EXPORT
%
% Doc lai file .mat da duoc xuat boi script chinh (bien "results")
% va ve lai toan bo cac hinh, sau do xuat ra PNG/JPG do phan giai cao
% (300 dpi) de chen vao LaTeX.
%
% CACH DUNG:
%   1. Sua bien "mat_file" ben duoi tro toi file .mat can ve
%      (vi du: export_journal/platoon_results_20260904_120000.mat)
%   2. Sua "out_folder" neu muon luu anh o noi khac
%   3. Chon dinh dang anh: 'png' (khuyen nghi cho LaTeX) hoac 'jpg'
%   4. Chay script
%% ========================================================================

%% ---- CAU HINH ----------------------------------------------------------

mat_file   = 'export_journal/platoon_results_XXXXXXXX_XXXXXX.mat'; % <-- SUA O DAY
out_folder = fullfile(pwd,'figures_latex');
img_format = 'png';     % 'png' (khuyen nghi, khong mat du lieu) hoac 'jpg'
img_dpi    = 300;       % do phan giai xuat anh

if ~exist(out_folder,'dir')
    mkdir(out_folder);
end

%% ---- DOC FILE .MAT -------------------------------------------------

if ~isfile(mat_file)
    error(['Khong tim thay file: %s\n' ...
           'Hay sua bien "mat_file" o dau script tro dung duong dan .mat.'], mat_file);
end

S = load(mat_file,'results');
R = S.results;

%% ---- LAY LAI CAC BIEN TU STRUCT -------------------------------------

Nveh   = R.meta.Nveh;
h0     = R.meta.h0;
ctrl_channel_names = R.meta.ctrl_channel_names;

T = R.time.T;

X0      = R.leader.X;
v0_hist = R.leader.v;
a0      = R.leader.a;

X_16     = R.state.X;
Y_16     = R.state.Y;
vx_16    = R.state.vx;
r_16     = R.state.r;
gamma_16 = R.state.gamma;
phi_16   = R.state.phi;

u_cmd    = R.control.u_cmd;
u_actual = R.control.u_actual;
u16_hist = R.control.u16;          % [Nveh x Nstep x 9]

Ui_af_hist        = R.control_breakdown.Ui_predecessor_af;
Ui_af_leader_hist = R.control_breakdown.Ui_leader_preview;
Ui_kv_hist        = R.control_breakdown.Ui_feedback;
Ui_gap_hist       = R.control_breakdown.Ui_gap;
Ui_dist_hist      = R.control_breakdown.Ui_ftdo_comp;
Ui_vx_track_hist  = R.control_breakdown.Ui_vx_track;

e_gap = R.errors.e_gap;
ey_hist    = R.errors.ey;
delta_hist = R.errors.delta;
velocity_error_to_leader = R.errors.vel_err_to_leader;

h_hist = R.headway.h;

D_v_true = R.ftdo.Dv_true;
D_a_true = R.ftdo.Da_true;
D_v_hat  = R.ftdo.Dv_hat;
D_a_hat  = R.ftdo.Da_hat;

packet_lost = R.comm.packet_lost;

% Gioi han gia toc dung de ve duong nam ngang tham chieu
% (khong duoc luu trong struct nen khai bao lai giong file goc)
aM = 3.0;
am = -3.0;
delta_limit = deg2rad(5.5);

%% ---- MAU SAC ---------------------------------------------------------

colors = lines(Nveh);

%% ========================================================================
% HAM TIEN ICH: luu figure hien tai ra file anh chat luong cao
%% ========================================================================

save_fig = @(fig_handle,name) export_current_fig(fig_handle,out_folder,name,img_format,img_dpi);

%% ---------------------------------------------------------------
% Figure 1 - X
%% ---------------------------------------------------------------

f1 = figure('Name','Fig1_Position');
clf; hold on;
for i = 1:Nveh
    plot(T,X_16(i,:),'LineWidth',1.6,'Color',colors(i,:), ...
        'DisplayName',sprintf('Vehicle %d',i));
end
plot(T,X0,'k--','LineWidth',2.0,'DisplayName','Leader');
grid on; box on;
xlabel('Time (s)'); ylabel('X (m)');
title('Longitudinal Position');
legend('Location','best');
save_fig(f1,'fig01_position_X');

%% ---------------------------------------------------------------
% Figure 2 - vx
%% ---------------------------------------------------------------

f2 = figure('Name','Fig2_Velocity');
clf; hold on;
for i = 1:Nveh
    plot(T,vx_16(i,:),'LineWidth',1.8,'Color',colors(i,:), ...
        'DisplayName',sprintf('Vehicle %d',i));
end
plot(T,v0_hist,'k--','LineWidth',2.4,'DisplayName','Leader');
grid on; box on;
xlabel('Time (s)'); ylabel('v_x (m/s)');
title('Longitudinal Velocity Tracking');
legend('Location','best');
save_fig(f2,'fig02_velocity_tracking');

%% ---------------------------------------------------------------
% Figure 3 - velocity error
%% ---------------------------------------------------------------

f3 = figure('Name','Fig3_VelError');
clf; hold on;
for i = 1:Nveh
    plot(T,velocity_error_to_leader(i,:),'LineWidth',1.5,'Color',colors(i,:), ...
        'DisplayName',sprintf('Vehicle %d',i));
end
yline(0,'k--');
grid on; box on;
xlabel('Time (s)'); ylabel('v_i-v_0 (m/s)');
title('Velocity Error to Leader');
legend('Location','best');
save_fig(f3,'fig03_velocity_error');

%% ---------------------------------------------------------------
% Figure 4 - gap error
%% ---------------------------------------------------------------

f4 = figure('Name','Fig4_GapError');
clf; hold on;
for i = 1:Nveh
    plot(T,e_gap(i,:),'LineWidth',1.5,'Color',colors(i,:), ...
        'DisplayName',sprintf('Vehicle %d',i));
end
yline(0,'k--');
grid on; box on;
xlabel('Time (s)'); ylabel('e_d (m)');
title('CTH Gap Error');
legend('Location','best');
save_fig(f4,'fig04_gap_error');

%% ---------------------------------------------------------------
% Figure 5 - actual longitudinal acceleration
%% ---------------------------------------------------------------

f5 = figure('Name','Fig5_Accel');
clf; hold on;
for i = 1:Nveh
    plot(T,u_actual(i,:),'LineWidth',1.4,'Color',colors(i,:), ...
        'DisplayName',sprintf('Vehicle %d',i));
end
yline(aM,'k--','DisplayName','a_{max}');
yline(am,'k-.','DisplayName','a_{min}');
grid on; box on;
xlabel('Time (s)'); ylabel('a_x (m/s^2)');
title('Actual Longitudinal Acceleration');
legend('Location','best');
save_fig(f5,'fig05_actual_acceleration');

%% ---------------------------------------------------------------
% Figure 6 - time headway
%% ---------------------------------------------------------------

f6 = figure('Name','Fig6_Headway');
clf; hold on;
for i = 1:Nveh
    plot(T,h_hist(i,:),'LineWidth',1.4,'Color',colors(i,:), ...
        'DisplayName',sprintf('Vehicle %d',i));
end
yline(h0,'k--','DisplayName','h_0');
grid on; box on;
xlabel('Time (s)'); ylabel('h_i (s)');
title('Time Headway');
legend('Location','best');
save_fig(f6,'fig06_time_headway');

%% ---------------------------------------------------------------
% Figure 7 - DLC (khong co duong tham chieu ly tuong vi khong duoc
% luu trong .mat; chi ve quy dao thuc te cua tung xe theo X tuong doi)
%% ---------------------------------------------------------------

f7 = figure('Name','Fig7_DLC');
clf; hold on;
for i = 1:Nveh
    Xrel_i = X_16(i,:) - X_16(i,1);
    plot(Xrel_i,Y_16(i,:),'LineWidth',1.7,'Color',colors(i,:), ...
        'DisplayName',sprintf('Vehicle %d',i));
end
grid on; box on;
xlabel('Relative X (m)'); ylabel('Y (m)');
title('Double Lane Change Tracking');
xlim([0 180]); ylim([-1 5]);
legend('Location','best');
save_fig(f7,'fig07_double_lane_change');

%% ---------------------------------------------------------------
% Figure 8 - lateral error
%% ---------------------------------------------------------------

f8 = figure('Name','Fig8_LatError');
clf; hold on;
for i = 1:Nveh
    plot(T,ey_hist(i,:),'LineWidth',1.5,'Color',colors(i,:), ...
        'DisplayName',sprintf('Vehicle %d',i));
end
yline(0,'k--');
grid on; box on;
xlabel('Time (s)'); ylabel('e_y (m)');
title('Lateral Tracking Error');
legend('Location','best');
save_fig(f8,'fig08_lateral_error');

%% ---------------------------------------------------------------
% Figure 9 - steering
%% ---------------------------------------------------------------

f9 = figure('Name','Fig9_Steering');
clf; hold on;
for i = 1:Nveh
    plot(T,rad2deg(delta_hist(i,:)),'LineWidth',1.5,'Color',colors(i,:), ...
        'DisplayName',sprintf('Vehicle %d',i));
end
yline(rad2deg(delta_limit),'k--','DisplayName','+ limit');
yline(-rad2deg(delta_limit),'k-.','DisplayName','- limit');
grid on; box on;
xlabel('Time (s)'); ylabel('\delta_f (deg)');
title('Front Steering');
legend('Location','best');
save_fig(f9,'fig09_steering');

%% ---------------------------------------------------------------
% Figure 10 - yaw rate
%% ---------------------------------------------------------------

f10 = figure('Name','Fig10_YawRate');
clf; hold on;
for i = 1:Nveh
    plot(T,rad2deg(r_16(i,:)),'LineWidth',1.4,'Color',colors(i,:), ...
        'DisplayName',sprintf('Vehicle %d',i));
end
grid on; box on;
xlabel('Time (s)'); ylabel('r_t (deg/s)');
title('Tractor Yaw Rate');
legend('Location','best');
save_fig(f10,'fig10_yaw_rate');

%% ---------------------------------------------------------------
% Figure 11 - articulation
%% ---------------------------------------------------------------

f11 = figure('Name','Fig11_Articulation');
clf; hold on;
for i = 1:Nveh
    plot(T,rad2deg(gamma_16(i,:)),'LineWidth',1.4,'Color',colors(i,:), ...
        'DisplayName',sprintf('Vehicle %d',i));
end
yline(0,'k--');
grid on; box on;
xlabel('Time (s)'); ylabel('\gamma (deg)');
title('Articulation Angle');
legend('Location','best');
save_fig(f11,'fig11_articulation_angle');

%% ---------------------------------------------------------------
% Figure 12 - roll
%% ---------------------------------------------------------------

f12 = figure('Name','Fig12_Roll');
clf; hold on;
for i = 1:Nveh
    plot(T,rad2deg(phi_16(i,:)),'LineWidth',1.4,'Color',colors(i,:), ...
        'DisplayName',sprintf('Vehicle %d',i));
end
yline(0,'k--');
grid on; box on;
xlabel('Time (s)'); ylabel('\phi_t (deg)');
title('Tractor Roll Angle');
legend('Location','best');
save_fig(f12,'fig12_roll_angle');

%% ---------------------------------------------------------------
% Figure 13 - FTDO dv (chi Vehicle 1, giong ban goc)
%% ---------------------------------------------------------------

f13 = figure('Name','Fig13_FTDO_dv');
clf; hold on;
plot(T,D_v_true(1,:),'k--','LineWidth',1.8,'DisplayName','True d_v');
plot(T,D_v_hat(1,:),'LineWidth',1.4,'DisplayName','Estimated d_v');
grid on; box on;
xlabel('Time (s)'); ylabel('d_v');
title('FTDO - d_v');
legend('Location','best');
save_fig(f13,'fig13_ftdo_dv');

%% ---------------------------------------------------------------
% Figure 14 - FTDO da (chi Vehicle 1)
%% ---------------------------------------------------------------

f14 = figure('Name','Fig14_FTDO_da');
clf; hold on;
plot(T,D_a_true(1,:),'k--','LineWidth',1.8,'DisplayName','True d_a');
plot(T,D_a_hat(1,:),'LineWidth',1.4,'DisplayName','Estimated d_a');
grid on; box on;
xlabel('Time (s)'); ylabel('d_a');
title('FTDO - d_a');
legend('Location','best');
save_fig(f14,'fig14_ftdo_da');

%% ---------------------------------------------------------------
% Figure 15 - packet loss map
%% ---------------------------------------------------------------

f15 = figure('Name','Fig15_PacketLoss');
clf;
imagesc(T,1:Nveh,packet_lost);
axis xy;
grid on; box on;
xlabel('Time (s)'); ylabel('Vehicle');
title('V2V Packet Loss Map');
colorbar;
save_fig(f15,'fig15_packet_loss_map');

%% ---------------------------------------------------------------
% Figure 101 - longitudinal diagnostics for V1 (subplot)
%% ---------------------------------------------------------------

idx800 = 1:min(800,numel(T));

f101 = figure('Name','Fig101_V1_Diagnostics');
clf;

subplot(3,1,1); hold on;
plot(T(idx800),u_cmd(1,idx800),'LineWidth',1.5,'DisplayName','U_i');
plot(T(idx800),u_actual(1,idx800),'LineWidth',1.5,'DisplayName','a_x actual');
yline(0,'k:');
grid on;
title('V1 Command vs Plant Acceleration');
legend('Location','best');

subplot(3,1,2);
Tdrive1 = squeeze(u16_hist(1,:,2));
plot(T(idx800),Tdrive1(idx800),'LineWidth',1.5);
grid on;
title('V1 Drive Torque');
ylabel('Nm');

subplot(3,1,3); hold on;
plot(T(idx800),Ui_af_hist(1,idx800),'LineWidth',1.2,'DisplayName','pred a');
plot(T(idx800),Ui_af_leader_hist(1,idx800),'LineWidth',1.2,'DisplayName','leader a');
plot(T(idx800),Ui_kv_hist(1,idx800),'LineWidth',1.2,'DisplayName','FB');
plot(T(idx800),Ui_gap_hist(1,idx800),'LineWidth',1.2,'DisplayName','gap');
plot(T(idx800),Ui_dist_hist(1,idx800),'LineWidth',1.2,'DisplayName','FTDO');
plot(T(idx800),Ui_vx_track_hist(1,idx800),'LineWidth',1.2,'DisplayName','vx-track');
yline(0,'k:');
grid on;
title('V1 Longitudinal Controller Components');
legend('Location','best');

save_fig(f101,'fig101_v1_diagnostics');

fprintf('\nDa xuat toan bo hinh anh (%s, %d dpi) vao thu muc:\n  %s\n', ...
    upper(img_format),img_dpi,out_folder);

%% ========================================================================
% LOCAL FUNCTION
%% ========================================================================

function export_current_fig(fig_handle,out_folder,name,img_format,img_dpi)
    % Dat kich thuoc chuan de anh dep khi chen vao LaTeX
    fig_handle.Units = 'inches';
    fig_handle.Position(3:4) = [6 4];

    out_path = fullfile(out_folder,[name '.' img_format]);

    % exportgraphics cho chat luong cao, nen dung neu co (R2020a tro len)
    if exist('exportgraphics','file')
        exportgraphics(fig_handle,out_path,'Resolution',img_dpi);
    else
        print(fig_handle,out_path,['-d' img_format],['-r' num2str(img_dpi)]);
    end

    fprintf('  Saved: %s\n',out_path);
end
