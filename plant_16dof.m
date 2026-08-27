function [dxdt, y] = plant_16dof(x, u, p)
% =========================================================================
% plant_16dof
%
% MO RONG cua dof_StateFcn3 (13-DOF) thanh 16-DOF:
%   - Tach omega_f, omega_r, omega_s (axle-level) thanh omega_fL/fR,
%     omega_rL/rR, omega_sL/sR (wheel-level) -> cho phep PHANH VI SAI
%     (differential braking) tao moment yaw truc tiep, dung cho
%     brake_allocation.m
%   - Them dau vao M_act: moment chu dong chong lat tu bo dieu khien
%     treo (lqr_suspension.m), cong truc tiep vao phuong trinh roll.
%   - Moment yaw M_z_t, M_z_s duoc bo sung so hang (Fx_R - Fx_L)*T/2,
%     day la co che vat ly de phanh vi sai tao yaw moment.
%
% States (16):
%  x(1)=v_x   x(2)=v_y   x(3)=r_t   x(4)=r_s   x(5)=gamma  x(6)=psi
%  x(7)=Y     x(8)=X
%  x(9)=omega_fL  x(10)=omega_fR
%  x(11)=omega_rL x(12)=omega_rR
%  x(13)=omega_sL x(14)=omega_sR
%  x(15)=phi_t    x(16)=dphi_t
%
% Inputs (9):
%  u(1)=delta_f     : goc lai banh truoc (rad)
%  u(2)=T_f_drive   : tong moment keo truc truoc, chia deu 2 ben (Nm)
%  u(3)=M_act       : moment chu dong chong lat tu suspension controller (Nm)
%  u(4..9)=Tb_fL,Tb_fR,Tb_rL,Tb_rR,Tb_sL,Tb_sR : moment phanh tung banh (Nm, >=0)
%
% Outputs: y = [v_x; r_t; Y; gamma; phi_t; LTR_f; LTR_r]
% =========================================================================

if numel(x) ~= 16, error('x phai co 16 phan tu.'); end
if numel(u) ~= 9,  error('u phai co 9 phan tu.'); end

epsilon = 1e-3;

%% Trich tham so
m_t=p.m_t; m_s=p.m_s; m_tot=p.m_tot; g=p.g;
I_zt=p.I_zt; I_zs=p.I_zs; I_xxt=p.I_xxt;
I_wf=p.I_wf; I_wr=p.I_wr; I_ws=p.I_ws;
L_ft=p.L_ft; L_rt=p.L_rt; L_wt=p.L_wt; L_fs=p.L_fs; L_rs=p.L_rs; L_t=p.L_t;
T_f=p.T_f; T_r=p.T_r; T_s=p.T_s;
R_f=p.R_f; R_r=p.R_r; R_s=p.R_s;
C_sigma_f=p.C_sigma_f; C_sigma_r=p.C_sigma_r; C_sigma_s=p.C_sigma_s;
C_alpha_f=p.C_alpha_f; C_alpha_r=p.C_alpha_r; C_alpha_s=p.C_alpha_s;
mu=p.mu;
C_D=p.C_D; A_a=p.A_a; rho_a=p.rho_a;
a_f=p.a_f; a_r=p.a_r; a_s=p.a_s;
h_t=p.h_t;
K_phi=p.K_phi; C_phi=p.C_phi;
F_zf_static=p.F_zf_static; F_zr_static=p.F_zr_static; F_zs_static=p.F_zs_static;

%% States
v_x=x(1); v_y=x(2); r_t=x(3); r_s=x(4); gamma=x(5); psi=x(6);
Y=x(7); X=x(8); %#ok<NASGU>
omega_fL=x(9); omega_fR=x(10);
omega_rL=x(11); omega_rR=x(12);
omega_sL=x(13); omega_sR=x(14);
phi_t=x(15); dphi_t=x(16);

%% Inputs
delta_f = u(1);
T_f_drive = u(2);
M_act = u(3);
Tb_fL=u(4); Tb_fR=u(5); Tb_rL=u(6); Tb_rR=u(7); Tb_sL=u(8); Tb_sR=u(9);

if abs(v_x) < epsilon, v_x_eff = epsilon; else, v_x_eff = v_x; end %#ok<NASGU>

%% Van toc cuc bo truc (dung chung goc truot ngang cho ca 2 ben - xap xi)
v_yf = v_y + L_ft*r_t;
v_yr = v_y - L_rt*r_t;
v_ys = v_y - L_wt*r_t - r_s*(L_fs+L_rs);
v_xs = v_x*cos(gamma);

% Van toc doc truc rieng cho tung banh (anh huong cua yaw rate qua vet banh)
v_xfL = v_x - T_f/2*r_t;  v_xfR = v_x + T_f/2*r_t;
v_xrL = v_x - T_r/2*r_t;  v_xrR = v_x + T_r/2*r_t;
v_xsL = v_xs - T_s/2*r_s; v_xsR = v_xs + T_s/2*r_s;

%% Goc truot ngang (dung chung truc, xap xi)
alpha_f = delta_f - atan2(v_yf, abs(v_x)+epsilon);
alpha_r = -atan2(v_yr, abs(v_x)+epsilon);
alpha_s = -atan2(v_ys, abs(v_xs)+epsilon);

%% Truot doc rieng tung banh
sigma_fL = (R_f*omega_fL - v_xfL)/(abs(v_xfL)+epsilon);
sigma_fR = (R_f*omega_fR - v_xfR)/(abs(v_xfR)+epsilon);
sigma_rL = (R_r*omega_rL - v_xrL)/(abs(v_xrL)+epsilon);
sigma_rR = (R_r*omega_rR - v_xrR)/(abs(v_xrR)+epsilon);
sigma_sL = (R_s*omega_sL - v_xsL)/(abs(v_xsL)+epsilon);
sigma_sR = (R_s*omega_sR - v_xsR)/(abs(v_xsR)+epsilon);

sat = @(s) max(min(s,0.99),-0.99);
sigma_fL=sat(sigma_fL); sigma_fR=sat(sigma_fR);
sigma_rL=sat(sigma_rL); sigma_rR=sat(sigma_rR);
sigma_sL=sat(sigma_sL); sigma_sR=sat(sigma_sR);

%% Can khong khi
F_a = 0.5*C_D*A_a*rho_a*v_x^2*sign(v_x+1e-9);

%% Uoc luong gia toc de tinh chuyen tai (dung gia tri buoc truoc)
F_x_guess = T_f_drive/R_f - (Tb_fL+Tb_fR+Tb_rL+Tb_rR)/R_f - F_a;
a_x_est = F_x_guess/m_tot;
a_y_est = v_x*r_t;

%% Chuyen tai doc truc
Delta_Fz_long = m_t*h_t/L_t*a_x_est;
F_zf_long = F_zf_static - Delta_Fz_long;
F_zr_long = F_zr_static + Delta_Fz_long;

%% Dong luc hoc roll (co them M_act)
M_lat_roll   = m_t*h_t*a_y_est;
M_gravity    = m_t*g*h_t*sin(phi_t);
M_suspension = K_phi*phi_t + C_phi*dphi_t;
ddphi_t = (M_lat_roll - M_gravity - M_suspension + M_act)/I_xxt;

M_roll_total = M_lat_roll - M_gravity;
Delta_Fz_lat_f = 0.50*M_roll_total/T_f;
Delta_Fz_lat_r = 0.50*M_roll_total/T_r;

%% Tai banh
F_zf_L = max(0.5*F_zf_long + Delta_Fz_lat_f, 0);
F_zf_R = max(0.5*F_zf_long - Delta_Fz_lat_f, 0);
F_zr_L = max(0.5*F_zr_long + Delta_Fz_lat_r, 0);
F_zr_R = max(0.5*F_zr_long - Delta_Fz_lat_r, 0);
F_zs_L = 0.5*F_zs_static;
F_zs_R = 0.5*F_zs_static;

%% Luc lop (Dugoff) - moi banh rieng
[Fx_fL,Fy_fL] = calcDugoff(alpha_f, sigma_fL, F_zf_L, C_sigma_f, C_alpha_f, mu);
[Fx_fR,Fy_fR] = calcDugoff(alpha_f, sigma_fR, F_zf_R, C_sigma_f, C_alpha_f, mu);
[Fx_rL,Fy_rL] = calcDugoff(alpha_r, sigma_rL, F_zr_L, C_sigma_r, C_alpha_r, mu);
[Fx_rR,Fy_rR] = calcDugoff(alpha_r, sigma_rR, F_zr_R, C_sigma_r, C_alpha_r, mu);
[Fx_sL,Fy_sL] = calcDugoff(alpha_s, sigma_sL, F_zs_L, C_sigma_s, C_alpha_s, mu);
[Fx_sR,Fy_sR] = calcDugoff(alpha_s, sigma_sR, F_zs_R, C_sigma_s, C_alpha_s, mu);

%% Can lan
Fx_fL = Fx_fL - F_zf_L*a_f/R_f;  Fx_fR = Fx_fR - F_zf_R*a_f/R_f;
Fx_rL = Fx_rL - F_zr_L*a_r/R_r;  Fx_rR = Fx_rR - F_zr_R*a_r/R_r;
Fx_sL = Fx_sL - F_zs_L*a_s/R_s;  Fx_sR = Fx_sR - F_zs_R*a_s/R_s;

%% Tong hop luc truc
Fx_f = Fx_fL+Fx_fR;  Fy_f = Fy_fL+Fy_fR;
Fx_r = Fx_rL+Fx_rR;  Fy_r = Fy_rL+Fy_rR;
Fx_s = Fx_sL+Fx_sR;  Fy_s = Fy_sL+Fy_sR;

%% Chuyen he truc lai + moc keo
Fx_f_body = Fx_f*cos(delta_f) - Fy_f*sin(delta_f);
Fy_f_body = Fx_f*sin(delta_f) + Fy_f*cos(delta_f);

Fx_s_body = Fx_s*cos(gamma) + Fy_s*sin(gamma);
Fy_s_body = Fy_s*cos(gamma) - Fx_s*sin(gamma);

Fx_total = Fx_f_body + Fx_r + Fx_s_body - F_a;
Fy_total = Fy_f_body + Fy_r + Fy_s_body;

%% Moment yaw - CONG THEM so hang phanh vi sai (Fx_R - Fx_L)*T/2
M_z_t = Fy_f_body*L_ft - Fy_r*L_rt ...
      + (Fx_fR-Fx_fL)*T_f/2 + (Fx_rR-Fx_rL)*T_r/2;

M_z_s = -Fy_s_body*L_rs + (Fx_sR-Fx_sL)*T_s/2;

%% Dong luc hoc than xe
dv_x = Fx_total/m_tot + v_y*r_t;
dv_y = Fy_total/m_tot - v_x*r_t;
dr_t = M_z_t/I_zt;
dr_s = M_z_s/I_zs;
dgamma = r_t - r_s;
dpsi = r_t;
dY = v_x*sin(psi) + v_y*cos(psi);
dX = v_x*cos(psi) - v_y*sin(psi);

%% Dong luc hoc banh xe (moment keo tru moment phanh tru moment lop)
domega_fL = (T_f_drive/2 - Tb_fL - Fx_fL*R_f)/I_wf;
domega_fR = (T_f_drive/2 - Tb_fR - Fx_fR*R_f)/I_wf;
domega_rL = (-Tb_rL - Fx_rL*R_r)/I_wr;
domega_rR = (-Tb_rR - Fx_rR*R_r)/I_wr;
domega_sL = (-Tb_sL - Fx_sL*R_s)/I_ws;
domega_sR = (-Tb_sR - Fx_sR*R_s)/I_ws;

%% Ghep dao ham
dxdt = zeros(16,1);
dxdt(1)=dv_x;  dxdt(2)=dv_y;  dxdt(3)=dr_t;  dxdt(4)=dr_s;
dxdt(5)=dgamma; dxdt(6)=dpsi;
dxdt(7)=dY; dxdt(8)=dX;
dxdt(9)=domega_fL; dxdt(10)=domega_fR;
dxdt(11)=domega_rL; dxdt(12)=domega_rR;
dxdt(13)=domega_sL; dxdt(14)=domega_sR;
dxdt(15)=dphi_t; dxdt(16)=ddphi_t;

%% Output - them LTR de giam sat/rang buoc
LTR_f = (F_zf_L-F_zf_R)/max(F_zf_L+F_zf_R,1);
LTR_r = (F_zr_L-F_zr_R)/max(F_zr_L+F_zr_R,1);

y = [v_x; r_t; Y; gamma; phi_t; LTR_f; LTR_r];

end


%% ========================================================================
function [Fx,Fy] = calcDugoff(alpha, sigma, Fz, C_sigma, C_alpha, mu)
epsilon = 1e-8;
Fz = max(Fz,0);
sigma = max(min(sigma,0.99),-0.99);

denom = 2*sqrt((C_sigma*sigma)^2 + (C_alpha*tan(alpha))^2);
if denom < epsilon
    lambda = 1;
else
    lambda = mu*Fz*(1-sigma)/denom;
end

if lambda < 1
    f_lambda = lambda*(2-lambda);
else
    f_lambda = 1;
end

Fx = C_sigma*sigma/(1-sigma)*f_lambda;
Fy = C_alpha*tan(alpha)/(1-sigma)*f_lambda;

F_total = sqrt(Fx^2+Fy^2);
F_limit = mu*Fz;
if F_total > F_limit
    scale = F_limit/F_total;
    Fx = Fx*scale; Fy = Fy*scale;
end
end
