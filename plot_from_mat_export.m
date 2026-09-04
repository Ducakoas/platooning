clear; clc; close all;

%% ========================================================================
% EXPORT_PAPER_ASSETS
%
% Doc lai file .mat da xuat boi script mo phong chinh ("results" struct)
% va tao ra TOAN BO tai nguyen ma file LaTeX (paper) can, dat dung ten
% file de ban chi viec upload ca thu muc "figures_latex/" len la dung
% duoc ngay, khong can sua tay duong dan anh hay go lai so lieu:
%
%   figures_latex/fig_vx_tracking.png   -> Fig. vx  (Section V.A)
%   figures_latex/fig_gap_error.png     -> Fig. gap (Section V.B)
%   figures_latex/fig_dlc_path.png      -> Fig. dlc (Section V.C)
%   figures_latex/fig_gamma_roll.png    -> Fig. gamma_roll (Section V.C)
%   figures_latex/table_metrics.tex     -> Table III (Section V.B), tu
%                                          dong dien so lieu that
%
% Ngoai ra van xuat bo hinh chan doan day du (fig01..fig15, fig101)
% giong script truoc, phong khi can them hinh phu.
%
% CACH DUNG:
%   1. Sua "mat_file" tro toi file .mat can dung
%   2. Chay script trong MATLAB
%   3. Copy nguyen thu muc "figures_latex/" vao cung cap voi file .tex
%      (hoac day len Overleaf) roi bien dich binh thuong
%% ========================================================================

%% ---- CAU HINH ----------------------------------------------------------

%% ---- CAU HINH ----------------------------------------------------------

% Thư mục chứa dữ liệu xuất (dùng đường dẫn tương đối để máy nào cũng chạy được)
%% ---- CAU HINH ----------------------------------------------------------
% Sửa lại đường dẫn này trỏ đúng đến thư mục thực tế đang chứa file .mat
folder = 'C:\Users\ducnm103\Downloads\export_journal\04_Proposed_Full'; % Hoặc đường dẫn thư mục export của bạn

% Ưu tiên tìm file kết quả mới nhất vừa chạy
mat_file = fullfile(folder, 'latest_platoon_results.mat');

% Nếu không có file latest, tự động quét lấy file .mat mới nhất theo thời gian tạo
if ~exist(mat_file, 'file')
    files = dir(fullfile(folder, 'platoon_results_*.mat'));
    if isempty(files)
        error('Khong tim thay bat ky file .mat nao trong thu muc: %s', folder);
    end
    [~, idx] = max([files.datenum]);
    mat_file = fullfile(folder, files(idx).name);
end

disp(['--> Dang load du lieu tu: ', mat_file]);

out_folder = fullfile(pwd, 'figures_latex');
img_format = 'png';
img_dpi    = 300;

if ~exist(out_folder, 'dir')
    mkdir(out_folder);
end

S = load(mat_file, 'results');
R = S.results;

%% ---- LAY LAI CAC BIEN TU STRUCT -------------------------------------

Nveh = R.meta.Nveh;
h0   = R.meta.h0;

T = R.time.T;

X0      = R.leader.X;9
v0_hist = R.leader.v;

X_16     = R.state.X;
Y_16     = R.state.Y;9
vx_16    = R.state.vx;
r_16     = R.state.r;
gamma_16 = R.state.gamma;
phi_16   = R.state.phi;

u_cmd    = R.control.u_cmd;
u_actual = R.control.u_actual;
u16_hist = R.control.u16;

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

% Hang so mo phong khong duoc luu trong struct -> khai bao lai giong
% file kich ban goc (day la cac thong so THIET KE co dinh, khong phai
% du lieu do, nen an toan khi hardcode lai o day)
aM = 3.0;  am = -3.0;
delta_limit = deg2rad(5.5);

lane_width   = 3.5;  A_lane = lane_width/2;
k_lane       = 0.092;
x_lane_start = 50;   x_lane_end = 120;

colors = lines(Nveh);

save_fig = @(fig_handle,name) export_current_fig(fig_handle,out_folder,name,img_format,img_dpi);

%% ========================================================================
% CAC HINH DANH RIENG CHO PAPER (dung ten file LaTeX can)
%% ========================================================================

%% ---- fig_vx_tracking.png ------------------------------------------

fv = figure('Name','fig_vx_tracking');
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
save_fig(fv,'fig_vx_tracking');

%% ---- fig_gap_error.png ----------------------------------------------

fg = figure('Name','fig_gap_error');
clf; hold on;
for i = 1:Nveh
    plot(T,e_gap(i,:),'LineWidth',1.5,'Color',colors(i,:), ...
        'DisplayName',sprintf('Vehicle %d',i));
end
yline(0,'k--');
grid on; box on;
xlabel('Time (s)'); ylabel('e_i (m)');
title('Variable-Time-Headway Spacing Error');
legend('Location','best');
save_fig(fg,'fig_gap_error');

%% ---- fig_dlc_path.png (voi duong tham chieu ly tuong) ---------------

Xref = linspace(0,180,2000);
Yref = A_lane*(tanh(k_lane*(Xref-x_lane_start)) - tanh(k_lane*(Xref-x_lane_end)));

fd = figure('Name','fig_dlc_path');
clf; hold on;
plot(Xref,Yref,'k--','LineWidth',2.4,'DisplayName','Desired Path');
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
save_fig(fd,'fig_dlc_path');

%% ---- fig_gamma_roll.png (2 subplot: articulation + roll) ------------

fgr = figure('Name','fig_gamma_roll');
clf;

subplot(2,1,1); hold on;
for i = 1:Nveh
    plot(T,rad2deg(gamma_16(i,:)),'LineWidth',1.4,'Color',colors(i,:), ...
        'DisplayName',sprintf('Vehicle %d',i));
end
yline(0,'k--');
grid on; box on;
ylabel('\gamma (deg)');
title('Articulation Angle');
legend('Location','best');

subplot(2,1,2); hold on;
for i = 1:Nveh
    plot(T,rad2deg(phi_16(i,:)),'LineWidth',1.4,'Color',colors(i,:), ...
        'DisplayName',sprintf('Vehicle %d',i));
end
yline(0,'k--');
grid on; box on;
xlabel('Time (s)'); ylabel('\phi_t (deg)');
title('Tractor Roll Angle');
legend('Location','best');

fgr.Position(3:4) = [700 500]; % hinh doi cao hon vi co 2 subplot
save_fig(fgr,'fig_gamma_roll');

%% ========================================================================
% BANG SO LIEU TU DONG: table_metrics.tex (thay the Table III cua paper)
%% ========================================================================

tex_path = fullfile(out_folder,'table_metrics.tex');
fid = fopen(tex_path,'w');

wl = @(s) fprintf(fid,'%s\n',s);   % ghi 1 dong y nguyen, khong bi fprintf
                                    % dien giai % hay \ vi truyen qua %s

wl('% Auto-generated by export_paper_assets.m -- DO NOT EDIT BY HAND');
wl('% Regenerate this file any time by re-running export_paper_assets.m');
wl('\begin{table}[!t]');
wl('\centering');
wl('\caption{Summary Performance Metrics}');
wl('\label{tab:results}');
wl('\begin{tabular}{@{}lcccc@{}}');
wl('\toprule');
wl('Vehicle & RMS $v_x$ err. (m/s) & RMS gap err. (m) & RMS $e_y$ (m) & $R_i$ \\');
wl('\midrule');

for i = 1:Nveh
    if i == 1
        Ri_str = '--';
    else
        Ri_str = sprintf('%.3f',R.metrics.string_rms_ratio(i-1));
    end

    row = sprintf('%d & %.4f & %.4f & %.4f & %s \\\\', ...
        i, ...
        R.metrics.rms_velocity_error(i), ...
        R.metrics.rms_gap_error(i), ...
        R.metrics.rms_lat_error(i), ...
        Ri_str);

    wl(row);
end

wl('\bottomrule');
wl('\end{tabular}');
wl('\end{table}');

fclose(fid);

fprintf('  Saved: %s\n',tex_path);

%% ========================================================================
% BO HINH CHAN DOAN DAY DU (giong script truoc, de tham khao them)
%% ========================================================================

f1 = figure('Name','Fig1_Position'); clf; hold on;
for i=1:Nveh, plot(T,X_16(i,:),'LineWidth',1.6,'Color',colors(i,:),'DisplayName',sprintf('Vehicle %d',i)); end
plot(T,X0,'k--','LineWidth',2.0,'DisplayName','Leader');
grid on; box on; xlabel('Time (s)'); ylabel('X (m)'); title('Longitudinal Position'); legend('Location','best');
save_fig(f1,'fig01_position_X');

f3 = figure('Name','Fig3_VelError'); clf; hold on;
for i=1:Nveh, plot(T,velocity_error_to_leader(i,:),'LineWidth',1.5,'Color',colors(i,:),'DisplayName',sprintf('Vehicle %d',i)); end
yline(0,'k--'); grid on; box on; xlabel('Time (s)'); ylabel('v_i-v_0 (m/s)'); title('Velocity Error to Leader'); legend('Location','best');
save_fig(f3,'fig03_velocity_error');

f5 = figure('Name','Fig5_Accel'); clf; hold on;
for i=1:Nveh, plot(T,u_actual(i,:),'LineWidth',1.4,'Color',colors(i,:),'DisplayName',sprintf('Vehicle %d',i)); end
yline(aM,'k--','DisplayName','a_{max}'); yline(am,'k-.','DisplayName','a_{min}');
grid on; box on; xlabel('Time (s)'); ylabel('a_x (m/s^2)'); title('Actual Longitudinal Acceleration'); legend('Location','best');
save_fig(f5,'fig05_actual_acceleration');

f6 = figure('Name','Fig6_Headway'); clf; hold on;
for i=1:Nveh, plot(T,h_hist(i,:),'LineWidth',1.4,'Color',colors(i,:),'DisplayName',sprintf('Vehicle %d',i)); end
yline(h0,'k--','DisplayName','h_0'); grid on; box on; xlabel('Time (s)'); ylabel('h_i (s)'); title('Time Headway'); legend('Location','best');
save_fig(f6,'fig06_time_headway');

f8 = figure('Name','Fig8_LatError'); clf; hold on;
for i=1:Nveh, plot(T,ey_hist(i,:),'LineWidth',1.5,'Color',colors(i,:),'DisplayName',sprintf('Vehicle %d',i)); end
yline(0,'k--'); grid on; box on; xlabel('Time (s)'); ylabel('e_y (m)'); title('Lateral Tracking Error'); legend('Location','best');
save_fig(f8,'fig08_lateral_error');

f9 = figure('Name','Fig9_Steering'); clf; hold on;
for i=1:Nveh, plot(T,rad2deg(delta_hist(i,:)),'LineWidth',1.5,'Color',colors(i,:),'DisplayName',sprintf('Vehicle %d',i)); end
yline(rad2deg(delta_limit),'k--','DisplayName','+ limit'); yline(-rad2deg(delta_limit),'k-.','DisplayName','- limit');
grid on; box on; xlabel('Time (s)'); ylabel('\delta_f (deg)'); title('Front Steering'); legend('Location','best');
save_fig(f9,'fig09_steering');

f10 = figure('Name','Fig10_YawRate'); clf; hold on;
for i=1:Nveh, plot(T,rad2deg(r_16(i,:)),'LineWidth',1.4,'Color',colors(i,:),'DisplayName',sprintf('Vehicle %d',i)); end
grid on; box on; xlabel('Time (s)'); ylabel('r_t (deg/s)'); title('Tractor Yaw Rate'); legend('Location','best');
save_fig(f10,'fig10_yaw_rate');

f13 = figure('Name','Fig13_FTDO_dv'); clf; hold on;
plot(T,D_v_true(1,:),'k--','LineWidth',1.8,'DisplayName','True d_v');
plot(T,D_v_hat(1,:),'LineWidth',1.4,'DisplayName','Estimated d_v');
grid on; box on; xlabel('Time (s)'); ylabel('d_v'); title('FTDO - d_v'); legend('Location','best');
save_fig(f13,'fig13_ftdo_dv');

f14 = figure('Name','Fig14_FTDO_da'); clf; hold on;
plot(T,D_a_true(1,:),'k--','LineWidth',1.8,'DisplayName','True d_a');
plot(T,D_a_hat(1,:),'LineWidth',1.4,'DisplayName','Estimated d_a');
grid on; box on; xlabel('Time (s)'); ylabel('d_a'); title('FTDO - d_a'); legend('Location','best');
save_fig(f14,'fig14_ftdo_da');

f15 = figure('Name','Fig15_PacketLoss'); clf;
imagesc(T,1:Nveh,packet_lost); axis xy; grid on; box on;
xlabel('Time (s)'); ylabel('Vehicle'); title('V2V Packet Loss Map'); colorbar;
save_fig(f15,'fig15_packet_loss_map');

idx800 = 1:min(800,numel(T));
f101 = figure('Name','Fig101_V1_Diagnostics'); clf;
subplot(3,1,1); hold on;
plot(T(idx800),u_cmd(1,idx800),'LineWidth',1.5,'DisplayName','U_i');
plot(T(idx800),u_actual(1,idx800),'LineWidth',1.5,'DisplayName','a_x actual');
yline(0,'k:'); grid on; title('V1 Command vs Plant Acceleration'); legend('Location','best');
subplot(3,1,2);
Tdrive1 = squeeze(u16_hist(1,:,2));
plot(T(idx800),Tdrive1(idx800),'LineWidth',1.5); grid on; title('V1 Drive Torque'); ylabel('Nm');
subplot(3,1,3); hold on;
plot(T(idx800),Ui_af_hist(1,idx800),'LineWidth',1.2,'DisplayName','pred a');
plot(T(idx800),Ui_af_leader_hist(1,idx800),'LineWidth',1.2,'DisplayName','leader a');
plot(T(idx800),Ui_kv_hist(1,idx800),'LineWidth',1.2,'DisplayName','FB');
plot(T(idx800),Ui_gap_hist(1,idx800),'LineWidth',1.2,'DisplayName','gap');
plot(T(idx800),Ui_dist_hist(1,idx800),'LineWidth',1.2,'DisplayName','FTDO');
plot(T(idx800),Ui_vx_track_hist(1,idx800),'LineWidth',1.2,'DisplayName','vx-track');
yline(0,'k:'); grid on; title('V1 Longitudinal Controller Components'); legend('Location','best');
save_fig(f101,'fig101_v1_diagnostics');

fprintf('\nHoan tat. Thu muc san sang de dung cho LaTeX:\n  %s\n',out_folder);
fprintf('Chi can copy ca thu muc nay vao cung cap voi file .tex va bien dich.\n');

%% ========================================================================
% LOCAL FUNCTION
%% ========================================================================

function export_current_fig(fig_handle,out_folder,name,img_format,img_dpi)
    fig_handle.Units = 'inches';
    if isempty(fig_handle.Position) || fig_handle.Position(3) < 1
        fig_handle.Position(3:4) = [6 4];
    end

    out_path = fullfile(out_folder,[name '.' img_format]);

    if exist('exportgraphics','file')
        exportgraphics(fig_handle,out_path,'Resolution',img_dpi);
    else
        print(fig_handle,out_path,['-d' img_format],['-r' num2str(img_dpi)]);
    end

    fprintf('  Saved: %s\n',out_path);
end