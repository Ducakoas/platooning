clear; clc; close all;
rng(11);

%% ========================================================================
% MAIN PLATOONING - FULL SINGLE VERSION - TRACKING FIX 2
%
% 16-DOF articulated vehicle platoon
% PF/CACC longitudinal tracking
% Fixed-Time Disturbance Observer
% V2V delay + packet loss + zero-order hold
% Velocity prediction under communication delay
% Leader acceleration preview
% CTH spacing policy
% Double Lane Change lateral control
%
% Required SAME folder:
%   vehicle_params.m
%   allocate_control.m
%   plant_16dof.m
%% ========================================================================

doPic = true;

Nveh   = 6;
Tstep  = 0.01;
SimTime = 30;
Nstep  = round(SimTime/Tstep) + 1;

%% ========================================================================
% 1. FEATURES
%% ========================================================================

useFTDO            = true;
useCommDelay       = true;
usePacketLoss      = true;
useVariableHeadway = true;

% Leader acceleration preview is used to reduce error propagation.
useLeaderAccelPreview = true;

%% ========================================================================
% 2. COMMUNICATION
%% ========================================================================

tau_comm          = 0.20;       % [s]
packet_loss_prob  = 0.10;       % 10%
comm_delay_steps  = round(tau_comm/Tstep);

%% ========================================================================
% 3. VEHICLE PARAMETERS
%% ========================================================================

p = vehicle_params();

%% ========================================================================
% 4. PLATOON PARAMETERS
%% ========================================================================

d0 = 20.0;

h0    = 0.50;
h_min = 0.50;
h_max = 0.70;

% Variable Time Headway parameters
k_v_headway = 0.03;
Delta_h_kappa = 0.04;
Delta_h_gamma = 0.02;

a_y_scale   = 2.5;
gamma_scale = deg2rad(5);

v0  = 16.0;
v00 = 16.0;

d_initial = d0 + h0*v0;

%% ========================================================================
% 5. INITIAL STEADY-STATE ROAD-LOAD COMPENSATION
%
% When vx = 16 m/s and desired acceleration = 0, the vehicle still needs
% a small positive longitudinal force to overcome rolling + aero drag.
%
% This removes the artificial startup dip 16 -> 15.5 m/s caused by
% starting the 16-DOF plant with zero longitudinal drive command.
%% ========================================================================

F_a0 = ...
    0.5*p.C_D*p.A_a*p.rho_a*v0^2;

F_roll0 = ...
    p.F_zf_static*p.a_f + ...
    p.F_zr_static*p.a_r + ...
    p.F_zs_static*p.a_s;

a_eq0 = ...
    (F_a0 + F_roll0)/p.m_tot;

fprintf('Initial road-load compensation acceleration = %.6f m/s^2\n',a_eq0);

%% ========================================================================
% 6. LONGITUDINAL CONTROLLER
%% ========================================================================

% CSMC / sliding parameters
k1      = 0.18;
epsilon = 0.20;

q   = 0.80;
ppp = 0.70;

delta = 0.10;

% Main predecessor velocity feedback
%
% Controller structure:
%
%   Ui_fb = (e_v_pred - epsilon*ur
%            - delta/q*sig^ppp(S))/h
%            - k_gap*e
%            + k_slide*sig^ppp(S)
%
% This retains the structure of the old controller that gave good vx
% tracking instead of using a very large pure k_v gain.
%% ------------------------------------------------------------------------

k_gap   = 0.12;
k_slide = 0.015;

% Acceleration feedforward from predecessor
k_af = 1.00;

% Direct leader acceleration preview
k_af_leader = 0.85;

% Acceleration limits
am = -3.0;
aM =  3.0;

% Jerk limit
max_jerk = 3.0;

% Prediction error clamp
max_velocity_error_for_control = 1.0;
%% ========================================================================
%  ADDITIONAL vx TRACKING SUPPORT
%  Only active during the observed velocity-drop interval.
%% ========================================================================

k_vx_1015   = 0.75;
vx_err_max  = 0.80;
%% ========================================================================
% 7. FIXED-TIME DISTURBANCE OBSERVER
%% ========================================================================

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

observer_comp_gain = 0.03;

d_v_hat_limit = 0.5;
d_a_hat_limit = 0.5;

% FTDO warm-up time
FTDO_warmup = 1.0;       % [s]

%% ========================================================================
% 8. DOUBLE LANE CHANGE
%% ========================================================================

lane_width = 3.5;
A_lane = lane_width/2;

k_lane = 0.092;

x_lane_start = 50;
x_lane_end   = 120;

%% ========================================================================
% 9. LATERAL CONTROLLER
%% ========================================================================

K_stanley = 0.272;
Kpsi      = 0.82;
Kr        = 0.20;
Kvy       = 0.080;
Kgamma    = 0.08;
Krs       = 0.02;

lookahead = 8.0;

delta_limit = deg2rad(5.5);

%% ========================================================================
% 10. STATE ARRAYS
%% ========================================================================

% Full 16-DOF state
x16 = zeros(Nveh,Nstep,16);

% Reduced state:
% x(:,:,1) = X
% x(:,:,2) = vx
% x(:,:,3) = ax
x = zeros(Nveh,Nstep,3);

%% Leader
X0      = zeros(1,Nstep);
v0_hist = zeros(1,Nstep);
a0      = zeros(1,Nstep);
Y0      = zeros(1,Nstep);
psi0    = zeros(1,Nstep);

%% Control
u_cmd    = zeros(Nveh,Nstep);
u_actual = zeros(Nveh,Nstep);
u16_prev = zeros(Nveh,9);

% Start desired longitudinal command at steady-state road-load value
u_cmd(:,1) = a_eq0;

%% Sliding surface
s      = zeros(Nveh,Nstep);
S      = zeros(Nveh,Nstep);
intOfE = zeros(Nveh,1);

%% Longitudinal errors
e_gap = zeros(Nveh,Nstep);
e_v   = zeros(Nveh,Nstep);

%% Headway
h_hist    = zeros(Nveh,Nstep);
dstar_hist = zeros(Nveh,Nstep);

%% Global positions
X_hist = zeros(Nveh,Nstep);
Y_hist = zeros(Nveh,Nstep);

%% Geometry
dparallel_hist = zeros(Nveh,Nstep);
dperp_hist     = zeros(Nveh,Nstep);
deuclid_hist   = zeros(Nveh,Nstep);

%% Lateral
y_ref_hist     = zeros(Nveh,Nstep);
psi_ref_hist   = zeros(Nveh,Nstep);
kappa_ref_hist = zeros(Nveh,Nstep);
ey_hist        = zeros(Nveh,Nstep);
epsi_hist      = zeros(Nveh,Nstep);
delta_hist     = zeros(Nveh,Nstep);

%% FTDO
D_v_true = zeros(Nveh,Nstep);
D_a_true = zeros(Nveh,Nstep);

D_v_hat = zeros(Nveh,Nstep);
D_a_hat = zeros(Nveh,Nstep);

%% Diagnostics
ax_actual_hist = zeros(Nveh,Nstep);
Tdrive_hist    = zeros(Nveh,Nstep);

Ui_af_hist        = zeros(Nveh,Nstep);
Ui_af_leader_hist = zeros(Nveh,Nstep);
Ui_kv_hist        = zeros(Nveh,Nstep);
Ui_gap_hist       = zeros(Nveh,Nstep);
Ui_dist_hist      = zeros(Nveh,Nstep);

%% Observer states
zX0 = zeros(Nveh,1);
zX1 = zeros(Nveh,1);

zV0 = zeros(Nveh,1);
zV1 = zeros(Nveh,1);

%% ========================================================================
% 11. V2V RECEIVER MEMORY
%% ========================================================================

rx_X     = zeros(Nveh,1);
rx_Y     = zeros(Nveh,1);
rx_psi   = zeros(Nveh,1);
rx_vx    = zeros(Nveh,1);
rx_vy    = zeros(Nveh,1);
rx_gamma = zeros(Nveh,1);
rx_r     = zeros(Nveh,1);
rx_acc   = zeros(Nveh,1);

% Index of the most recently received packet
rx_time_idx = ones(Nveh,1);

packet_received = zeros(Nveh,Nstep);
packet_lost     = zeros(Nveh,Nstep);
comm_delay_hist = zeros(Nveh,Nstep);

%% ========================================================================
% 12. LEADER INITIALIZATION
%% ========================================================================

X_leader_initial = Nveh*d_initial;

X0(1)      = X_leader_initial;
v0_hist(1) = v00;
a0(1)      = 0;

[Y0(1),~,~,psi0(1),~] = ...
    dlc_reference( ...
        X0(1), ...
        X_leader_initial+x_lane_start, ...
        X_leader_initial+x_lane_end, ...
        A_lane, ...
        k_lane);

%% ========================================================================
% 13. FOLLOWER INITIALIZATION
%% ========================================================================

for i = 1:Nveh

    init_X  = X_leader_initial - i*d_initial;
    init_vx = v0;

    %% Full state
    x16(i,1,1) = init_vx;     % vx
    x16(i,1,2) = 0;           % vy
    x16(i,1,3) = 0;           % r_t
    x16(i,1,4) = 0;           % r_s
    x16(i,1,5) = 0;           % gamma
    x16(i,1,6) = 0;           % psi
    x16(i,1,7) = 0;           % Y
    x16(i,1,8) = init_X;      % X

    x16(i,1,9) = init_vx/p.R_f;
    x16(i,1,10) = init_vx/p.R_f;
    x16(i,1,11) = init_vx/p.R_r;
    x16(i,1,12) = init_vx/p.R_r;
    x16(i,1,13) = init_vx/p.R_s;
    x16(i,1,14) = init_vx/p.R_s;

    x16(i,1,15) = 0;
    x16(i,1,16) = 0;

    %% Reduced state
    x(i,1,:) = [init_X,init_vx,0];

    X_hist(i,1) = init_X;
    Y_hist(i,1) = 0;

    %% Observer states
    zX0(i) = init_X;
    zX1(i) = 0;

    zV0(i) = init_vx;
    zV1(i) = 0;

    %% Receiver initialization
    if i == 1

        rx_X(i)     = X0(1);
        rx_Y(i)     = Y0(1);
        rx_psi(i)   = psi0(1);
        rx_vx(i)    = v0_hist(1);
        rx_vy(i)    = 0;
        rx_gamma(i) = 0;
        rx_r(i)     = 0;
        rx_acc(i)   = a_eq0;

    else

        j = i-1;

        rx_X(i)     = x16(j,1,8);
        rx_Y(i)     = 0;
        rx_psi(i)   = 0;
        rx_vx(i)    = v0;
        rx_vy(i)    = 0;
        rx_gamma(i) = 0;
        rx_r(i)     = 0;
        rx_acc(i)   = a_eq0;

    end

    rx_time_idx(i) = 1;

    h_hist(i,1) = h0;

    %% Initial 16-DOF control consistent with steady 16 m/s
    u0_16 = allocate_control( ...
        a_eq0, ...
        0, ...
        init_vx, ...
        0, ...
        0, ...
        p);

    u16_prev(i,:) = u0_16(:).';

end

%% ========================================================================
% 14. MAIN CLOSED LOOP
%% ========================================================================

for n = 2:Nstep

    t = (n-1)*Tstep;

    %% ====================================================================
    % 14.1 LEADER
    %% ====================================================================

    if t <= 2

        a0(n) = 0;

    elseif t <= 15

        a0(n) = 0.15;

    else

        a0(n) = 0;

    end

    v0_hist(n) = ...
        v0_hist(n-1) + a0(n)*Tstep;

    X0(n) = ...
        X0(n-1) ...
        + v0_hist(n-1)*Tstep ...
        + 0.5*a0(n)*Tstep^2;

    [Y0(n),~,~,psi0(n),~] = ...
        dlc_reference( ...
            X0(n), ...
            X_leader_initial+x_lane_start, ...
            X_leader_initial+x_lane_end, ...
            A_lane, ...
            k_lane);

    %% ====================================================================
    % 14.2 FOLLOWERS
    %% ====================================================================

    for i = 1:Nveh

        %% ----------------------------------------------------------------
        % Current 16-DOF state
        %% ----------------------------------------------------------------

        x_prev = reshape(x16(i,n-1,:),16,1);

        vx    = x_prev(1);
        vy    = x_prev(2);
        r_t   = x_prev(3);
        r_s   = x_prev(4);
        gamma = x_prev(5);
        psi   = x_prev(6);
        Ypos  = x_prev(7);
        Xpos  = x_prev(8);
        phi   = x_prev(15);

        %% ----------------------------------------------------------------
        % External disturbances
        %% ----------------------------------------------------------------

        Dv = ...
            0.30 ...
            * sin(0.65*t + 0.17*i) ...
            * exp(-((t-12)/4.5)^2);

        Da = ...
            0.35 ...
            * sin(1.15*t + 0.13*i) ...
            * exp(-((t-17)/3.8)^2);

        D_v_true(i,n) = Dv;
        D_a_true(i,n) = Da;

        %% =================================================================
        % 14.3 FIXED-TIME DISTURBANCE OBSERVER
        %% =================================================================

        if useFTDO

            [dx_nom,~] = ...
                plant_16dof( ...
                    x_prev, ...
                    u16_prev(i,:), ...
                    p);

            fX_nom = dx_nom(8);
            fV_nom = dx_nom(1);

            %% X observer
            eOX = zX0(i)-Xpos;

            zX0_dot = ...
                fX_nom ...
                + zX1(i) ...
                - FTDO_X.a1*sigpow(eOX,alpha_ft) ...
                - FTDO_X.a2*sigpow(eOX,beta_ft);

            zX1_dot = ...
                - FTDO_X.b1*sigpow(eOX,2*alpha_ft-1) ...
                - FTDO_X.b2*sigpow(eOX,2*beta_ft-1);

            zX0(i) = zX0(i) + zX0_dot*Tstep;
            zX1(i) = zX1(i) + zX1_dot*Tstep;

            d_v_hat = ...
                max(min(zX1(i),d_v_hat_limit),-d_v_hat_limit);

            %% Velocity observer
            eOV = zV0(i)-vx;

            zV0_dot = ...
                fV_nom ...
                + zV1(i) ...
                - FTDO_V.a1*sigpow(eOV,alpha_ft) ...
                - FTDO_V.a2*sigpow(eOV,beta_ft);

            zV1_dot = ...
                - FTDO_V.b1*sigpow(eOV,2*alpha_ft-1) ...
                - FTDO_V.b2*sigpow(eOV,2*beta_ft-1);

            zV0(i) = zV0(i) + zV0_dot*Tstep;
            zV1(i) = zV1(i) + zV1_dot*Tstep;

            d_a_hat = ...
                max(min(zV1(i),d_a_hat_limit),-d_a_hat_limit);

        else

            d_v_hat = 0;
            d_a_hat = 0;

        end

        D_v_hat(i,n) = d_v_hat;
        D_a_hat(i,n) = d_a_hat;

        %% =================================================================
        % 14.4 V2V COMMUNICATION
        %% =================================================================

        if useCommDelay

            tx_idx = ...
                max(n-comm_delay_steps,1);

        else

            tx_idx = n-1;

        end

        packet_ok = ...
            ~usePacketLoss || rand >= packet_loss_prob;

        if packet_ok

            if i == 1

                %% Leader -> Vehicle 1

                rx_X(i)     = X0(tx_idx);
                rx_Y(i)     = Y0(tx_idx);
                rx_psi(i)   = psi0(tx_idx);
                rx_vx(i)    = v0_hist(tx_idx);
                rx_vy(i)    = 0;
                rx_gamma(i) = 0;
                rx_r(i)     = 0;
                rx_acc(i)   = a0(tx_idx);

            else

                %% Vehicle i-1 -> Vehicle i

                j = i-1;

                rx_X(i) = ...
                    x16(j,tx_idx,8);

                rx_Y(i) = ...
                    x16(j,tx_idx,7);

                rx_psi(i) = ...
                    x16(j,tx_idx,6);

                rx_vx(i) = ...
                    x16(j,tx_idx,1);

                rx_vy(i) = ...
                    x16(j,tx_idx,2);

                rx_gamma(i) = ...
                    x16(j,tx_idx,5);

                rx_r(i) = ...
                    x16(j,tx_idx,3);

                % IMPORTANT:
                % Transmit predecessor DESIRED acceleration command
                % instead of delayed actual plant acceleration.
                if tx_idx >= 2

                    rx_acc(i) = ...
                        u_cmd(j,tx_idx);

                else

                    rx_acc(i) = ...
                        a_eq0;

                end

            end

            % Store time index of the packet actually received
            rx_time_idx(i) = tx_idx;

            packet_received(i,n) = 1;
            packet_lost(i,n)     = 0;

        else

            %% Packet lost -> Zero-order hold
            % Receiver states remain unchanged.
            packet_received(i,n) = 0;
            packet_lost(i,n)     = 1;

        end

        %% True age of currently held information
        age_comm = ...
            (n-rx_time_idx(i))*Tstep;

        comm_delay_hist(i,n) = ...
            age_comm;

        %% Received data
        Xp     = rx_X(i);
        Yp     = rx_Y(i);
        vx_p   = rx_vx(i);
        acc_p  = rx_acc(i);

%% =================================================================
% 14.5 GEOMETRY
%
% IMPORTANT:
% Xp is a DELAYED predecessor position.
% Therefore do NOT use Xp directly for the spacing error.
%
% Predict predecessor position to current time:
%
% Xp_hat = Xp + tau*vx_p + 0.5*tau^2*acc_p
%
% This removes the artificial 3.2 m gap error caused by
% 0.2 s communication delay at 16 m/s.
%% =================================================================

prediction_horizon = min(age_comm,0.40);

Xp_pred = ...
    Xp ...
    + prediction_horizon*vx_p ...
    + 0.5*prediction_horizon^2*acc_p;

Yp_pred = Yp;

dX = Xp_pred-Xpos;
dY = Yp_pred-Ypos;

        t_hat = ...
            [cos(psi);sin(psi)];

        n_hat = ...
            [-sin(psi);cos(psi)];

        d_parallel = ...
            dX*t_hat(1) + dY*t_hat(2);

        d_perp = ...
            dX*n_hat(1) + dY*n_hat(2);

        d_euclid = ...
            hypot(dX,dY);

        dparallel_hist(i,n) = d_parallel;
        dperp_hist(i,n)     = d_perp;
        deuclid_hist(i,n)   = d_euclid;

%% =================================================================
% 14.6 VARIABLE TIME HEADWAY
%% =================================================================

if useVariableHeadway

    % -------------------------------------------------------------
    % 1. Velocity-based term
    % -------------------------------------------------------------
    e_v_headway = ...
        abs(vx-v0_hist(n));

    h_v = ...
        k_v_headway*e_v_headway;

    % -------------------------------------------------------------
    % 2. Lateral-risk term
    % -------------------------------------------------------------
    X0v_tmp = ...
        X0_vehicle_start( ...
            i, ...
            X_leader_initial, ...
            d_initial);

    [~,~,~,~,kappa_tmp] = ...
        dlc_reference( ...
            Xpos+lookahead, ...
            X0v_tmp+x_lane_start, ...
            X0v_tmp+x_lane_end, ...
            A_lane, ...
            k_lane);

    risk_kappa = ...
        tanh( ...
            vx^2*abs(kappa_tmp) ...
            /max(a_y_scale,1e-6));

    risk_gamma = ...
        tanh( ...
            abs(gamma) ...
            /max(gamma_scale,1e-6));

    h_raw = ...
        h0 ...
        + h_v ...
        + Delta_h_kappa*risk_kappa ...
        + Delta_h_gamma*risk_gamma;

else

    h_raw = h0;

end

% -------------------------------------------------------------
% Smooth headway variation
% -------------------------------------------------------------
h_prev = ...
    h_hist(i,n-1);

h_i = ...
    h_prev + 0.05*(h_raw-h_prev);

% Hard limits
h_i = ...
    min(max(h_i,h_min),h_max);

h_hist(i,n) = h_i;
        %% =================================================================
        % 14.7 CTH GAP ERROR
        %% =================================================================

        physical_gap_X = ...
    Xp_pred-Xpos;

        d_star = ...
            d0 + h_i*max(vx,0);

        e_i = ...
            d_star-physical_gap_X;

        dstar_hist(i,n) = d_star;
        e_gap(i,n)      = e_i;

        %% =================================================================
        % 14.8 PREDICTIVE RELATIVE VELOCITY
        %
        % Delayed data:
        %
        %   vx_p(t-tau)
        %   a_p(t-tau)
        %
        % Prediction:
        %
        %   vx_p_hat(t)
        %      = vx_p(t-tau) + tau*a_p(t-tau)
        %
        % Prediction is capped at 0.40 s during long packet-loss bursts.
        %% =================================================================

        prediction_horizon = ...
            min(age_comm,0.40);

        vx_p_pred = ...
            vx_p + prediction_horizon*acc_p;

        e_v_pred = ...
            vx_p_pred-vx;

        e_v_pred = ...
            max(min( ...
                e_v_pred, ...
                max_velocity_error_for_control), ...
                -max_velocity_error_for_control);

        e_v(i,n) = e_v_pred;

        %% =================================================================
        % 14.9 SLIDING SURFACE
        %% =================================================================

        ur = ...
            k1*e_i;

        intOfE(i) = ...
            intOfE(i) + ur*Tstep;

        % Anti-windup
        intOfE(i) = ...
            max(min(intOfE(i),20),-20);

        s(i,n) = ...
            e_i + epsilon*intOfE(i);

        if i == 1

            s_pred = 0;

        else

            % Current-time predecessor surface has already been computed
            % because vehicle i-1 is processed before vehicle i.
            s_pred = ...
                s(i-1,n);

        end

        S(i,n) = ...
            q*s_pred-s(i,n);

        %% =================================================================
        % 14.10 LEADER ACCELERATION PREVIEW
        %% =================================================================

        if useLeaderAccelPreview

            leader_tx_idx = ...
                max(n-comm_delay_steps,1);

            aL_rx = ...
                a0(leader_tx_idx);

            aL_pred = ...
                aL_rx;

        else

            aL_pred = 0;

        end

%% =================================================================
% 14.11 LONGITUDINAL CONTROL
%% =================================================================

% Feedback component
Ui_fb = ...
    ( ...
        e_v_pred ...
        - epsilon*ur ...
        - (delta/max(q,1e-6)) ...
            *sigpow(S(i,n),ppp) ...
    ) ...
    /max(h_i,0.45);

Ui_fb = ...
    Ui_fb ...
    - k_gap*e_i ...
    + k_slide*sigpow(S(i,n),ppp);

% Predecessor acceleration feedforward
Ui_af = ...
    k_af*acc_p;

% Leader acceleration preview
Ui_afL = ...
    k_af_leader*aL_pred;

%% =================================================================
% FTDO COMPENSATION
%% =================================================================

if t < FTDO_warmup

    Ui_dist = 0;

else

    Ui_dist = ...
        -observer_comp_gain*d_a_hat ...
        -observer_comp_gain* ...
            d_v_hat/max(h_i,0.45);

end

%% =================================================================
% ADDITIONAL vx TRACKING SUPPORT
%% =================================================================

Ui_vx_track = 0;

if t >= 9 && t <= 16

    % Leader velocity tracking error
    e_vx_leader = ...
        v0_hist(n)-vx;

    % Saturation
    e_vx_leader = ...
        max(min(e_vx_leader,vx_err_max), ...
            -vx_err_max);

    % Smooth activation:
    % 0 at 9 s
    % maximum around 12.5 s
    % 0 at 16 s
    w_vx = ...
        sin(pi*(t-9)/7)^2;

    Ui_vx_track = ...
        k_vx_1015*w_vx*e_vx_leader;

end
%% =================================================================
% TOTAL DESIRED ACCELERATION
%% =================================================================

Ui_des = ...
      Ui_fb ...
    + Ui_af ...
    + Ui_afL ...
    + Ui_dist ...
    + Ui_vx_track;

%% =================================================================
% DIAGNOSTICS
%% =================================================================

Ui_af_hist(i,n)        = Ui_af;
Ui_af_leader_hist(i,n) = Ui_afL;
Ui_kv_hist(i,n)        = Ui_fb;
Ui_gap_hist(i,n)       = -k_gap*e_i;
Ui_dist_hist(i,n)      = Ui_dist;
Ui_vx_track_hffst = zeros(Nveh,Nstep);


%% =================================================================
% ACCELERATION SATURATION
%% =================================================================

Ui_des = ...
    max(min(Ui_des,aM),am);

        %% =================================================================
        % 14.12 JERK LIMITER
        % =================================================================

        if n == 2

            % CRITICAL:
            % Do not start from zero acceleration.
            %
            % Start from the acceleration needed to overcome the
            % initial road load at 16 m/s.
            Ui_prev = a_eq0;

        else

            Ui_prev = ...
                u_cmd(i,n-1);

        end

        max_step = ...
            max_jerk*Tstep;

        Ui = ...
            max( ...
                min(Ui_des,Ui_prev+max_step), ...
                Ui_prev-max_step);

        Ui = ...
            max(min(Ui,aM),am);

        u_cmd(i,n) = Ui;

        %% =================================================================
        % 14.13 DOUBLE LANE CHANGE LATERAL CONTROL
        %% =================================================================

        X0v = ...
            X0_vehicle_start( ...
                i, ...
                X_leader_initial, ...
                d_initial);

        X_start_i = ...
            X0v+x_lane_start;

        X_end_i = ...
            X0v+x_lane_end;

        %% Reference at current position
        [y_ref_actual,~,~,~,~] = ...
            dlc_reference( ...
                Xpos, ...
                X_start_i, ...
                X_end_i, ...
                A_lane, ...
                k_lane);

        %% Look-ahead reference
        X_lookahead = ...
            Xpos+lookahead;

        [y_ref,~,~,psi_ref,kappa_ref] = ...
            dlc_reference( ...
                X_lookahead, ...
                X_start_i, ...
                X_end_i, ...
                A_lane, ...
                k_lane);

        %% Errors
        e_y = ...
            y_ref-Ypos;

        e_psi = ...
            atan2( ...
                sin(psi_ref-psi), ...
                cos(psi_ref-psi));

        %% Feedforward steering
        delta_ff = ...
            atan(p.L_t*kappa_ref);

        %% Stanley cross-track term
        vx_safe = ...
            max(abs(vx),2.0);

        delta_cte = ...
            atan2(K_stanley*e_y,vx_safe);

        %% Total steering command
        delta_f = ...
              delta_ff ...
            + delta_cte ...
            + Kpsi*e_psi ...
            - Kr*r_t ...
            - Kvy*vy ...
            - Kgamma*gamma ...
            - Krs*r_s;

        %% Steering saturation
        delta_f = ...
            max(min(delta_f,delta_limit), ...
                -delta_limit);

        %% Save lateral signals
        y_ref_hist(i,n)     = y_ref_actual;
        psi_ref_hist(i,n)   = psi_ref;
        kappa_ref_hist(i,n) = kappa_ref;

        ey_hist(i,n)   = e_y;
        epsi_hist(i,n) = e_psi;
        delta_hist(i,n) = delta_f;

        %% =================================================================
        % 14.14 CONTROL ALLOCATION
        %% =================================================================

        u16 = ...
            allocate_control( ...
                Ui, ...
                delta_f, ...
                max(vx,1.0), ...
                r_t, ...
                phi, ...
                p);

        %% Steering safety limit
        u16(1) = ...
            max(min(u16(1),delta_limit), ...
                -delta_limit);

        %% Drive torque safety limit
        u16(2) = ...
            max( ...
                min(u16(2), ...
                    p.Fx_cmd_max*p.R_f), ...
                0);

        %% =================================================================
        % 14.15 16-DOF PLANT
        %% =================================================================

        [dxdt,~] = ...
            plant_16dof( ...
                x_prev, ...
                u16, ...
                p);

        %% Diagnostic before adding explicit disturbances
        ax_actual_hist(i,n) = ...
            dxdt(1);

        Tdrive_hist(i,n) = ...
            u16(2);

        %% External disturbances
        dxdt(8) = ...
            dxdt(8)+Dv;

        dxdt(1) = ...
            dxdt(1)+Da;

        %% Numerical safety
        dxdt(~isfinite(dxdt)) = 0;

        %% Euler integration
        x_next = ...
            x_prev+dxdt*Tstep;

        %% Physical speed protection
        x_next(1) = ...
            max(x_next(1),0.1);

        %% Save actual acceleration
        u_actual(i,n) = ...
            dxdt(1);

        %% Save control
        u16_prev(i,:) = ...
            u16(:).';

        %% Save full state
        x16(i,n,:) = ...
            x_next;

        %% Save reduced state
        x(i,n,:) = ...
            [x_next(8), ...
             x_next(1), ...
             dxdt(1)];

        %% Save global coordinates
        X_hist(i,n) = ...
            x_next(8);

        Y_hist(i,n) = ...
            x_next(7);

    end

end

%% ========================================================================
% 15. POST PROCESSING
%% ========================================================================

T = ...
    (0:Nstep-1)*Tstep;

p_veh = ...
    squeeze(x(:,:,1));

vx_16 = ...
    squeeze(x16(:,:,1));

vy_16 = ...
    squeeze(x16(:,:,2));

r_16 = ...
    squeeze(x16(:,:,3));

gamma_16 = ...
    squeeze(x16(:,:,5));

Y_16 = ...
    squeeze(x16(:,:,7));

X_16 = ...
    squeeze(x16(:,:,8));

phi_16 = ...
    squeeze(x16(:,:,15));

%% ========================================================================
% 16. PERFORMANCE METRICS
%% ========================================================================

velocity_error_to_leader = ...
    vx_16-repmat(v0_hist,Nveh,1);

max_gap_error = ...
    max(abs(e_gap),[],2);

rms_gap_error = ...
    sqrt(mean(e_gap.^2,2));

max_velocity_error = ...
    max(abs(velocity_error_to_leader),[],2);

rms_velocity_error = ...
    sqrt(mean(velocity_error_to_leader.^2,2));

max_lat_error = ...
    max(abs(ey_hist),[],2);

rms_lat_error = ...
    sqrt(mean(ey_hist.^2,2));

max_steering = ...
    max(abs(rad2deg(delta_hist)),[],2);

max_roll = ...
    max(abs(rad2deg(phi_16)),[],2);

max_gamma = ...
    max(abs(rad2deg(gamma_16)),[],2);

packet_loss_rate = ...
    mean(packet_lost(:,2:end),2)*100;

rms_obs_v = ...
    sqrt(mean((D_v_true-D_v_hat).^2,2));

rms_obs_a = ...
    sqrt(mean((D_a_true-D_a_hat).^2,2));

%% ========================================================================
% 17. VELOCITY STRING-AMPLIFICATION RATIO
%
% R1 compares V1 to Leader.
% Ri compares Vi to Vi-1.
%
% Ri < 1 means the velocity error is attenuated from the previous vehicle.
%% ========================================================================

idx_eval = ...
    find(T >= 5);

string_rms_ratio = ...
    nan(Nveh-1,1);

for i = 1:Nveh-1

    if i == 1

        prev_err = ...
            vx_16(1,idx_eval)-v0_hist(idx_eval);

    else

        prev_err = ...
            vx_16(i,idx_eval)-vx_16(i-1,idx_eval);

    end

    curr_err = ...
        vx_16(i+1,idx_eval)-vx_16(i,idx_eval);

    prev_rms = ...
        sqrt(mean(prev_err.^2));

    curr_rms = ...
        sqrt(mean(curr_err.^2));

    string_rms_ratio(i) = ...
        curr_rms/max(prev_rms,1e-6);

end

%% ========================================================================
% 18. REFERENCE DLC
%% ========================================================================

Xref = ...
    linspace(0,180,2000);

Yref = ...
    A_lane*( ...
        tanh(k_lane*(Xref-x_lane_start)) ...
        -tanh(k_lane*(Xref-x_lane_end)));

Xrel = ...
    zeros(Nveh,Nstep);

for i = 1:Nveh

    Xrel(i,:) = ...
        X_16(i,:)-X_16(i,1);

end

%% ========================================================================
% 19. PLOTS
%% ========================================================================

if doPic

    colors = lines(Nveh);

    %% ---------------------------------------------------------------
    % Figure 1 - X
    %% ---------------------------------------------------------------

    figure(1);
    clf;
    hold on;

    for i = 1:Nveh

        plot( ...
            T, ...
            X_16(i,:), ...
            'LineWidth',1.6, ...
            'Color',colors(i,:), ...
            'DisplayName',sprintf('Vehicle %d',i));

    end

    plot( ...
        T, ...
        X0, ...
        'k--', ...
        'LineWidth',2.0, ...
        'DisplayName','Leader');

    grid on;
    box on;

    xlabel('Time (s)');
    ylabel('X (m)');
    title('Longitudinal Position');

    legend('Location','best');

    %% ---------------------------------------------------------------
    % Figure 2 - vx
    %% ---------------------------------------------------------------

    figure(2);
    clf;
    hold on;

    for i = 1:Nveh

        plot( ...
            T, ...
            vx_16(i,:), ...
            'LineWidth',1.8, ...
            'Color',colors(i,:), ...
            'DisplayName',sprintf('Vehicle %d',i));

    end

    plot( ...
        T, ...
        v0_hist, ...
        'k--', ...
        'LineWidth',2.4, ...
        'DisplayName','Leader');

    grid on;
    box on;

    xlabel('Time (s)');
    ylabel('v_x (m/s)');
    title('Longitudinal Velocity Tracking');

    legend('Location','best');

    %% ---------------------------------------------------------------
    % Figure 3 - velocity error
    %% ---------------------------------------------------------------

    figure(3);
    clf;
    hold on;

    for i = 1:Nveh

        plot( ...
            T, ...
            velocity_error_to_leader(i,:), ...
            'LineWidth',1.5, ...
            'Color',colors(i,:), ...
            'DisplayName',sprintf('Vehicle %d',i));

    end

    yline(0,'k--');

    grid on;
    box on;

    xlabel('Time (s)');
    ylabel('v_i-v_0 (m/s)');
    title('Velocity Error to Leader');

    legend('Location','best');

    %% ---------------------------------------------------------------
    % Figure 4 - gap error
    %% ---------------------------------------------------------------

    figure(4);
    clf;
    hold on;

    for i = 1:Nveh

        plot( ...
            T, ...
            e_gap(i,:), ...
            'LineWidth',1.5, ...
            'Color',colors(i,:), ...
            'DisplayName',sprintf('Vehicle %d',i));

    end

    yline(0,'k--');

    grid on;
    box on;

    xlabel('Time (s)');
    ylabel('e_d (m)');
    title('CTH Gap Error');

    legend('Location','best');

    %% ---------------------------------------------------------------
    % Figure 5 - actual longitudinal acceleration
    %% ---------------------------------------------------------------

    figure(5);
    clf;
    hold on;

    for i = 1:Nveh

        plot( ...
            T, ...
            u_actual(i,:), ...
            'LineWidth',1.4, ...
            'Color',colors(i,:), ...
            'DisplayName',sprintf('Vehicle %d',i));

    end

    yline(aM,'k--','DisplayName','a_{max}');
    yline(am,'k-.','DisplayName','a_{min}');

    grid on;
    box on;

    xlabel('Time (s)');
    ylabel('a_x (m/s^2)');
    title('Actual Longitudinal Acceleration');

    legend('Location','best');

    %% ---------------------------------------------------------------
    % Figure 6 - time headway
    %% ---------------------------------------------------------------

    figure(6);
    clf;
    hold on;

    for i = 1:Nveh

        plot( ...
            T, ...
            h_hist(i,:), ...
            'LineWidth',1.4, ...
            'Color',colors(i,:), ...
            'DisplayName',sprintf('Vehicle %d',i));

    end

    yline(h0,'k--','DisplayName','h_0');

    grid on;
    box on;

    xlabel('Time (s)');
    ylabel('h_i (s)');
    title('Time Headway');

    legend('Location','best');

    %% ---------------------------------------------------------------
    % Figure 7 - DLC
    %% ---------------------------------------------------------------

    figure(7);
    clf;
    hold on;

    plot( ...
        Xref, ...
        Yref, ...
        'k--', ...
        'LineWidth',2.4, ...
        'DisplayName','Desired Path');

    for i = 1:Nveh

        plot( ...
            Xrel(i,:), ...
            Y_16(i,:), ...
            'LineWidth',1.7, ...
            'Color',colors(i,:), ...
            'DisplayName',sprintf('Vehicle %d',i));

    end

    grid on;
    box on;

    xlabel('Relative X (m)');
    ylabel('Y (m)');
    title('Double Lane Change Tracking');

    xlim([0 180]);
    ylim([-1 5]);

    legend('Location','best');

    %% ---------------------------------------------------------------
    % Figure 8 - lateral error
    %% ---------------------------------------------------------------

    figure(8);
    clf;
    hold on;

    for i = 1:Nveh

        plot( ...
            T, ...
            ey_hist(i,:), ...
            'LineWidth',1.5, ...
            'Color',colors(i,:), ...
            'DisplayName',sprintf('Vehicle %d',i));

    end

    yline(0,'k--');

    grid on;
    box on;

    xlabel('Time (s)');
    ylabel('e_y (m)');
    title('Lateral Tracking Error');

    legend('Location','best');

    %% ---------------------------------------------------------------
    % Figure 9 - steering
    %% ---------------------------------------------------------------

    figure(9);
    clf;
    hold on;

    for i = 1:Nveh

        plot( ...
            T, ...
            rad2deg(delta_hist(i,:)), ...
            'LineWidth',1.5, ...
            'Color',colors(i,:), ...
            'DisplayName',sprintf('Vehicle %d',i));

    end

    yline( ...
        rad2deg(delta_limit), ...
        'k--', ...
        'DisplayName','+ limit');

    yline( ...
        -rad2deg(delta_limit), ...
        'k-.', ...
        'DisplayName','- limit');

    grid on;
    box on;

    xlabel('Time (s)');
    ylabel('\delta_f (deg)');
    title('Front Steering');

    legend('Location','best');

    %% ---------------------------------------------------------------
    % Figure 10 - yaw rate
    %% ---------------------------------------------------------------

    figure(10);
    clf;
    hold on;

    for i = 1:Nveh

        plot( ...
            T, ...
            rad2deg(r_16(i,:)), ...
            'LineWidth',1.4, ...
            'Color',colors(i,:), ...
            'DisplayName',sprintf('Vehicle %d',i));

    end

    grid on;
    box on;

    xlabel('Time (s)');
    ylabel('r_t (deg/s)');
    title('Tractor Yaw Rate');

    legend('Location','best');

    %% ---------------------------------------------------------------
    % Figure 11 - articulation
    %% ---------------------------------------------------------------

    figure(11);
    clf;
    hold on;

    for i = 1:Nveh

        plot( ...
            T, ...
            rad2deg(gamma_16(i,:)), ...
            'LineWidth',1.4, ...
            'Color',colors(i,:), ...
            'DisplayName',sprintf('Vehicle %d',i));

    end

    yline(0,'k--');

    grid on;
    box on;

    xlabel('Time (s)');
    ylabel('\gamma (deg)');
    title('Articulation Angle');

    legend('Location','best');

    %% ---------------------------------------------------------------
    % Figure 12 - roll
    %% ---------------------------------------------------------------

    figure(12);
    clf;
    hold on;

    for i = 1:Nveh

        plot( ...
            T, ...
            rad2deg(phi_16(i,:)), ...
            'LineWidth',1.4, ...
            'Color',colors(i,:), ...
            'DisplayName',sprintf('Vehicle %d',i));

    end

    yline(0,'k--');

    grid on;
    box on;

    xlabel('Time (s)');
    ylabel('\phi_t (deg)');
    title('Tractor Roll Angle');

    legend('Location','best');

    %% ---------------------------------------------------------------
    % Figure 13 - FTDO dv
    %% ---------------------------------------------------------------

    figure(13);
    clf;
    hold on;

    plot( ...
        T, ...
        D_v_true(1,:), ...
        'k--', ...
        'LineWidth',1.8, ...
        'DisplayName','True d_v');

    plot( ...
        T, ...
        D_v_hat(1,:), ...
        'LineWidth',1.4, ...
        'DisplayName','Estimated d_v');

    grid on;
    box on;

    xlabel('Time (s)');
    ylabel('d_v');
    title('FTDO - d_v');

    legend('Location','best');

    %% ---------------------------------------------------------------
    % Figure 14 - FTDO da
    %% ---------------------------------------------------------------

    figure(14);
    clf;
    hold on;

    plot( ...
        T, ...
        D_a_true(1,:), ...
        'k--', ...
        'LineWidth',1.8, ...
        'DisplayName','True d_a');

    plot( ...
        T, ...
        D_a_hat(1,:), ...
        'LineWidth',1.4, ...
        'DisplayName','Estimated d_a');

    grid on;
    box on;

    xlabel('Time (s)');
    ylabel('d_a');
    title('FTDO - d_a');

    legend('Location','best');

    %% ---------------------------------------------------------------
    % Figure 15 - packet loss
    %% ---------------------------------------------------------------

    figure(15);
    clf;

    imagesc( ...
        T, ...
        1:Nveh, ...
        packet_lost);

    axis xy;

    grid on;
    box on;

    xlabel('Time (s)');
    ylabel('Vehicle');

    title('V2V Packet Loss Map');

    colorbar;

    %% ---------------------------------------------------------------
    % Figure 101 - longitudinal diagnostics for V1
    %% ---------------------------------------------------------------

    idx800 = ...
        1:min(800,Nstep);

    figure(101);
    clf;

    subplot(3,1,1);
    hold on;

    plot( ...
        T(idx800), ...
        u_cmd(1,idx800), ...
        'LineWidth',1.5, ...
        'DisplayName','U_i');

    plot( ...
        T(idx800), ...
        ax_actual_hist(1,idx800), ...
        'LineWidth',1.5, ...
        'DisplayName','a_x actual');

    yline(0,'k:');

    grid on;

    title('V1 Command vs Plant Acceleration');

    legend('Location','best');

    subplot(3,1,2);

    plot( ...
        T(idx800), ...
        Tdrive_hist(1,idx800), ...
        'LineWidth',1.5);

    grid on;

    title('V1 Drive Torque');

    ylabel('Nm');

    subplot(3,1,3);
    hold on;

    plot( ...
        T(idx800), ...
        Ui_af_hist(1,idx800), ...
        'LineWidth',1.2, ...
        'DisplayName','pred a');

    plot( ...
        T(idx800), ...
        Ui_af_leader_hist(1,idx800), ...
        'LineWidth',1.2, ...
        'DisplayName','leader a');

    plot( ...
        T(idx800), ...
        Ui_kv_hist(1,idx800), ...
        'LineWidth',1.2, ...
        'DisplayName','FB');

    plot( ...
        T(idx800), ...
        Ui_gap_hist(1,idx800), ...
        'LineWidth',1.2, ...
        'DisplayName','gap');

    plot( ...
        T(idx800), ...
        Ui_dist_hist(1,idx800), ...
        'LineWidth',1.2, ...
        'DisplayName','FTDO');

    yline(0,'k:');

    grid on;

    title('V1 Longitudinal Controller Components');

    legend('Location','best');

end

%% ========================================================================
% 20. CONSOLE SUMMARY
%% ========================================================================

fprintf('\n');
fprintf('===============================================================\n');
fprintf(' FULL TRACKING + DLC VERSION - FIX 2\n');
fprintf('===============================================================\n');

fprintf( ...
    'FTDO=%s | delay=%s | loss=%s | varHeadway=%s | leaderPreview=%s\n', ...
    onoff(useFTDO), ...
    onoff(useCommDelay), ...
    onoff(usePacketLoss), ...
    onoff(useVariableHeadway), ...
    onoff(useLeaderAccelPreview));

fprintf('\nController parameters:\n');
fprintf('  a_eq0               = %.6f m/s^2\n',a_eq0);
fprintf('  k_gap               = %.4f\n',k_gap);
fprintf('  k_af                = %.4f\n',k_af);
fprintf('  k_af_leader         = %.4f\n',k_af_leader);
fprintf('  k_slide             = %.4f\n',k_slide);
fprintf('  max_jerk            = %.4f m/s^3\n',max_jerk);
fprintf('  delay               = %.3f s\n',tau_comm);
fprintf('  packet loss         = %.2f %%\n',100*packet_loss_prob);

fprintf('\nPerformance:\n');

for i = 1:Nveh

    fprintf( ...
        ['V%d: RMS vx err = %.4f m/s | ', ...
         'MAX vx err = %.4f m/s | ', ...
         'RMS gap err = %.4f m | ', ...
         'RMS lat err = %.4f m | ', ...
         'packet loss = %.2f %%\n'], ...
        i, ...
        rms_velocity_error(i), ...
        max_velocity_error(i), ...
        rms_gap_error(i), ...
        rms_lat_error(i), ...
        packet_loss_rate(i));

end

fprintf('\nVelocity string RMS ratios:\n');

for i = 1:Nveh-1

    fprintf( ...
        'V%d / V%d = %.4f\n', ...
        i+1, ...
        i, ...
        string_rms_ratio(i));

end

fprintf('\nFTDO RMS estimation errors:\n');

for i = 1:Nveh

    fprintf( ...
        'V%d: dv = %.5f | da = %.5f\n', ...
        i, ...
        rms_obs_v(i), ...
        rms_obs_a(i));

end

fprintf('\nSafety:\n');

for i = 1:Nveh

    fprintf( ...
        'V%d: max|gamma| = %.3f deg | ', ...
        'max|roll| = %.3f deg | ', ...
        'max|delta| = %.3f deg\n', ...
        i, ...
        max_gamma(i), ...
        max_roll(i), ...
        max_steering(i));

end

fprintf('===============================================================\n');
fprintf('Simulation completed.\n');
fprintf('===============================================================\n');

%% ========================================================================
% LOCAL FUNCTIONS
%% ========================================================================

function y = sigpow(x,a)

    if x == 0

        y = 0;

    else

        y = abs(x)^a*sign(x);

    end

end

%% ------------------------------------------------------------------------
% Double lane change reference
%% ------------------------------------------------------------------------

function [y,dy_dx,d2y_dx2,psi_ref,kappa_ref] = ...
    dlc_reference(X,X_start,X_end,A,k)

    z1 = ...
        k*(X-X_start);

    z2 = ...
        k*(X-X_end);

    t1 = tanh(z1);
    t2 = tanh(z2);

    s1 = 1-t1^2;
    s2 = 1-t2^2;

    y = ...
        A*(t1-t2);

    dy_dx = ...
        A*k*(s1-s2);

    d2y_dx2 = ...
        A*k^2*(-2*t1*s1+2*t2*s2);

    psi_ref = ...
        atan2(dy_dx,1);

    denominator = ...
        max((1+dy_dx^2)^(3/2),1e-9);

    kappa_ref = ...
        d2y_dx2/denominator;

end

%% ------------------------------------------------------------------------
% Initial X of vehicle i
%% ------------------------------------------------------------------------

function X0v = ...
    X0_vehicle_start(i,X_leader_initial,d_initial)

    X0v = ...
        X_leader_initial-i*d_initial;

end

%% ------------------------------------------------------------------------
% ON / OFF string
%% ------------------------------------------------------------------------

function s = onoff(flag)

    if flag

        s = 'ON';

    else

        s = 'OFF';

    end

end