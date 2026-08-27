function p = vehicle_params()
% =========================================================================
% vehicle_params
%
% Tham so xe dung chung cho: plant_16dof, reduced_model_nmpc,
% lqr_suspension, brake_allocation, main_sim.
%
% Lay tu dof_StateFcn3.m goc, giu nguyen gia tri de dam bao mo hinh
% reduced-order (NMPC) va mo hinh plant (16-DOF) nhat quan voi nhau.
% =========================================================================

%% Khoi luong / quan tinh
p.m_t   = 5760;
p.m_s   = 6500;
p.m_tot = p.m_t + p.m_s;
p.g     = 9.81;

p.I_zt  = 34823;
p.I_zs  = 179992;
p.I_xxt = 25000;

p.I_wf = 40;
p.I_wr = 40;
p.I_ws = 50;

%% Hinh hoc
p.L_ft = 1.100;
p.L_rt = 2.390;
p.L_wt = 1.100;
p.L_fs = 5.210;
p.L_rs = 3.280;
p.L_t  = p.L_ft + p.L_rt;

%% Vet banh xe
p.T_f = 1.535;
p.T_r = 1.535;
p.T_s = 1.535;

%% Ban kinh banh
p.R_f = 0.51;
p.R_r = 0.51;
p.R_s = 0.51;

%% Lop (Dugoff)
p.C_sigma_f = 125000;
p.C_sigma_r = 150000;
p.C_sigma_s = 150000;

p.C_alpha_f = 164090;
p.C_alpha_r = 164090;
p.C_alpha_s = 164090;

p.mu = 0.85;

%% Khi dong hoc
p.C_D   = 0.30;
p.A_a   = 2.0;
p.rho_a = 1.225;

%% Can lan
p.a_f = 0.002;
p.a_r = 0.002;
p.a_s = 0.002;

%% Chieu cao trong tam
p.h_t = 1.20;
p.h_s = 1.50;

%% Treo / roll
p.K_phi_f = 1.8e5;
p.K_phi_r = 2.2e5;
p.C_phi_f = 1.8e4;
p.C_phi_r = 2.2e4;
p.K_phi   = p.K_phi_f + p.K_phi_r;
p.C_phi   = p.C_phi_f + p.C_phi_r;

%% Tai tinh
p.F_zf_static = p.m_t*p.g*p.L_rt/p.L_t;
p.F_zr_static = p.m_t*p.g*p.L_ft/p.L_t;
p.F_zs_static = p.m_s*p.g;

%% Gioi han an toan (dung cho NMPC / brake allocation)
p.gamma_max   = deg2rad(35);   % nguong jackknife (can hieu chinh theo hinh hoc thuc te)
p.LTR_max     = 0.70;          % nguong canh bao lat (Load Transfer Ratio)
p.delta_max   = deg2rad(30);
p.ddelta_max  = deg2rad(60);   % toc do quay vo lang, rad/s
p.Fx_cmd_max  =  40000;        % N, luc doc truc mong muon toi da (keo)
p.Fx_cmd_min  = -60000;        % N, phanh toi da (am)
p.M_act_max   = 15000;         % Nm, gioi han moment chu dong chong lat

end
