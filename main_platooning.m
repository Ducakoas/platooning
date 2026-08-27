clear; clc; close all;

%% ========================================================================
% MAIN PLATOONING SIMULATION
% 6-vehicle platoon
% 16-DOF articulated vehicle
% Double Lane Change tracking
%
% Required files:
%   1) vehicle_params.m
%   2) plant_16dof.m
%   3) allocate_control.m
%
% Main structure:
%   Longitudinal:
%       CSMC-like platooning controller
%
%   Lateral:
%       Spatial Double Lane Change reference
%       + look-ahead
%       + curvature feedforward
%       + Stanley-like cross-track correction
%       + heading/yaw-rate damping
%
% IMPORTANT 16-DOF state mapping:
%   x(1)  = vx
%   x(2)  = vy
%   x(3)  = r_t
%   x(4)  = r_s
%   x(5)  = gamma
%   x(6)  = psi
%   x(7)  = Y
%   x(8)  = X
%   x(15) = phi_t
%   x(16) = dphi_t
%% ========================================================================


%% ========================================================================
%  TOPOLOGY / PROCESS CONTROL
%% ========================================================================

doPic = true;

isDelayed = true;       % One-step delay for predecessor acceleration
IndexTopo = 1;          % 1 = PF

flagCtrlSat = 1;
flagFinitefTimeCtrl = 1;


%% ========================================================================
%  GENERAL PARAMETERS
%% ========================================================================

% Desired physical inter-vehicle spacing excluding time headway
ddes = 20;                  % [m]

% Initial velocity
v0  = 16;                   % follower [m/s]
v00 = 16;                   % leader [m/s]

% Constant Time Headway
h = 0.5;                    % [s]

% Initial physical spacing according to CTH
% desired gap = ddes + h*v
d_initial = ddes + h*v0;    % 28 m


%% ========================================================================
%  LONGITUDINAL CSMC PARAMETERS
%% ========================================================================

k1      = 0.30;
epsilon = 0.50;

delta = 0.20;
q     = 0.80;
ppp   = 0.30;

% Acceleration limits
am = -4;                    % [m/s^2]
aM =  4;                    % [m/s^2]


%% ========================================================================
%  SIMULATION
%% ========================================================================

tstart = 0;

Nveh = 6;

Tstep = 0.01;               % [s]
deltaT = Tstep;

SimTime = 30;               % [s]
Nstep = round(SimTime/Tstep) + tstart;


%% ========================================================================
%  VEHICLE PARAMETERS
%% ========================================================================

p = vehicle_params();


%% ========================================================================
%  ARRAYS
%% ========================================================================

% Reduced state:
%   x(:,:,1) = actual X position
%   x(:,:,2) = actual vx
%   x(:,:,3) = longitudinal command acceleration
x = zeros(Nveh,Nstep,3);

% Full 16-DOF state
x_16dof = zeros(Nveh,Nstep,16);

% Leader
x0 = zeros(3,Nstep);

% Disturbance placeholders
dd = zeros(Nveh,Nstep,2);

% Sliding mode variables
s = zeros(Nveh,Nstep);
S = zeros(Nveh,Nstep);

% Platoon errors
% e(:,:,1) = spacing error
% e(:,:,2) = velocity error
e = zeros(Nveh,Nstep,2);

% Neighbor velocity
Vi_sum = zeros(Nveh,Nstep);

% Control histories
u = zeros(Nveh,Nstep);
u_control_history = zeros(Nveh,Nstep);


%% ========================================================================
%  INTEGRAL TERM
%% ========================================================================

intOfUr = zeros(Nveh,1);


%% ========================================================================
%  ACTUAL GLOBAL STATES
%% ========================================================================

x_global = zeros(Nveh,Nstep);     % actual X
y_global = zeros(Nveh,Nstep);     % actual Y


%% ========================================================================
%  LANE CHANGE REFERENCES
%% ========================================================================

y_ref_history   = zeros(Nveh,Nstep);
psi_ref_history = zeros(Nveh,Nstep);
delta_history   = zeros(Nveh,Nstep);

e_y_history   = zeros(Nveh,Nstep);
e_psi_history = zeros(Nveh,Nstep);


%% ========================================================================
%  DOUBLE LANE CHANGE PARAMETERS
%
%  Desired:
%
%       Y = 0
%            ________
%           /
%          /
%         /
%        \________
%
%  Maximum lateral displacement = 3.5 m
%% ========================================================================

lane_width = 3.5;

% Because:
% tanh(...) - tanh(...) -> 2
% choose A = 1.75 so maximum Y = 3.5 m
A_lane = lane_width/2;

% Smaller than previous 0.10 to make transition smoother
k_lane = 0.08;

% Start and return positions measured from each vehicle initial X
x_lane_start = 50;            % [m]
x_lane_end   = 120;           % [m]


%% ========================================================================
%  LATERAL CONTROLLER
%% ========================================================================

% Stanley-like cross-track gain
K_stanley = .0;

% Heading error gain
Kpsi = 1.20;

% Tractor yaw-rate damping
Kr = 0.20;

% Lateral velocity damping
Kvy = 0.015;

% Look-ahead distance
lookahead = 6.0;              % [m]

% Steering limit
delta_limit = deg2rad(6);


%% ========================================================================
%  COMMUNICATION TOPOLOGY
%% ========================================================================

M = zeros(Nveh,Nveh);
P = zeros(Nveh,Nveh);

switch IndexTopo

    case 1
        % ------------------------------------------------------------
        % PF: predecessor following
        %
        % vehicle 1 <-- leader
        % vehicle 2 <-- vehicle 1
        % vehicle 3 <-- vehicle 2
        % ...
        % ------------------------------------------------------------

        M(2:Nveh+1:Nveh^2) = 1;
        P(1,1) = 1;

    case 2
        % PLF
        M(2:Nveh+1:Nveh^2) = 1;
        P = eye(Nveh);

    case 3
        % TPF
        M(2:Nveh+1:Nveh^2) = 1;
        M(3:Nveh+1:Nveh^2-Nveh) = 1;
        P(1,1) = 1;
        P(2,2) = 1;

    case 4
        % TPLF
        M(2:Nveh+1:Nveh^2) = 1;
        M(3:Nveh+1:Nveh^2-Nveh) = 1;
        M(2,1) = 1;
        P = eye(Nveh);

    otherwise
        error('Unsupported topology.');

end

MP = M + P;

% Number of information sources
DP = ones(Nveh,1);


%% ========================================================================
%  PAPER-STYLE TOPOLOGY WEIGHTING FUNCTION
%% ========================================================================

f = @(i,j) (i-j) + i*(i==j);


%% ========================================================================
%  LEADER INITIALIZATION
%
%  Leader position is chosen so that the first follower starts at:
%
%       X_leader - (ddes + h*v0)
%
%  This gives approximately zero initial CTH spacing error.
%% ========================================================================

x0(:,1) = [ ...
    Nveh*d_initial; ...
    v00; ...
    0];


%% ========================================================================
%  FOLLOWER INITIALIZATION
%% ========================================================================

for i = 1:Nveh

    %% ------------------------------------------------------------
    % Initial global position
    %% ------------------------------------------------------------

    init_X = x0(1,1) - i*d_initial;

    init_vx = v0;


    %% ------------------------------------------------------------
    % Reduced longitudinal state
    %% ------------------------------------------------------------

    x(i,1,1) = init_X;
    x(i,1,2) = init_vx;
    x(i,1,3) = 0;


    %% ------------------------------------------------------------
    % 16-DOF state initialization
    %% ------------------------------------------------------------

    x_16dof(i,1,:) = zeros(1,1,16);

    % 1: vx
    x_16dof(i,1,1) = init_vx;

    % 2: vy
    x_16dof(i,1,2) = 0;

    % 3: tractor yaw rate
    x_16dof(i,1,3) = 0;

    % 4: semitrailer yaw rate
    x_16dof(i,1,4) = 0;

    % 5: articulation angle
    x_16dof(i,1,5) = 0;

    % 6: tractor heading
    x_16dof(i,1,6) = 0;

    % 7: global Y
    x_16dof(i,1,7) = 0;

    % 8: global X
    x_16dof(i,1,8) = init_X;

    % 9-10: front wheel angular velocities
    x_16dof(i,1,9)  = init_vx/p.R_f;
    x_16dof(i,1,10) = init_vx/p.R_f;

    % 11-12: rear wheel angular velocities
    x_16dof(i,1,11) = init_vx/p.R_r;
    x_16dof(i,1,12) = init_vx/p.R_r;

    % 13-14: semitrailer wheel angular velocities
    x_16dof(i,1,13) = init_vx/p.R_s;
    x_16dof(i,1,14) = init_vx/p.R_s;

    % 15-16: tractor roll
    x_16dof(i,1,15) = 0;
    x_16dof(i,1,16) = 0;


    %% ------------------------------------------------------------
    % Store actual global positions
    %% ------------------------------------------------------------

    x_global(i,1) = init_X;
    y_global(i,1) = 0;

end


%% ========================================================================
%  MAIN CLOSED-LOOP SIMULATION
%% ========================================================================

for n = 2:Nstep

    t_current = n*Tstep;


    %% ====================================================================
    % LEADER DYNAMICS
    %% ====================================================================

    x0(:,n) = x0(:,n-1);


    % ------------------------------------------------------------
    % Leader acceleration profile
    %
    % 0 - 2 s       : 0
    % 2 - 15 s      : +0.15
    % 15 - 30 s     : 0
    % ------------------------------------------------------------

    if n <= 2/Tstep + tstart

        x0(3,n) = 0;

    elseif n <= 15/Tstep + tstart

        x0(3,n) = 0.15;

    else

        x0(3,n) = 0;

    end


    % ------------------------------------------------------------
    % Proper constant-acceleration integration
    % ------------------------------------------------------------

    x0(1,n) = ...
        x0(1,n-1) ...
        + x0(2,n-1)*Tstep ...
        + 0.5*x0(3,n)*Tstep^2;

    x0(2,n) = ...
        x0(2,n-1) ...
        + x0(3,n)*Tstep;


    %% ====================================================================
    % FOLLOWING VEHICLES
    %% ====================================================================

    for i = 1:Nveh


        %% ----------------------------------------------------------------
        % Previous 16-DOF state
        %% ----------------------------------------------------------------

        x_prev = reshape( ...
            x_16dof(i,n-1,:), ...
            16,1);


        %% ----------------------------------------------------------------
        % Previous reduced state
        %% ----------------------------------------------------------------

        Xi = reshape( ...
            x(i,n-1,:), ...
            3,1);


        %% ----------------------------------------------------------------
        % Reset neighbor quantities
        %% ----------------------------------------------------------------

        Xi_error_sum = zeros(3,1);

        Vi_sum(i,n) = 0;

        Ui_sum = 0;


        %% =================================================================
        % INFORMATION EXCHANGE
        %% =================================================================

        for j = 1:i

            if MP(i,j) == 0
                continue;
            end


            %% =============================================================
            % LEADER INFORMATION
            %% =============================================================

            if i == j

                Xi_neighbor = x0(:,n);


                % CTH reference:
                %
                % p_ref = p_leader - h*v_i
                %

                Xi_neighbor(1) = ...
                    Xi_neighbor(1) ...
                    - h*(x_prev(1) + dd(i,n,1));


                Vi_neighbor = x0(2,n);

                Ui_neighbor = x0(3,n);


            %% =============================================================
            % PREDECESSOR INFORMATION
            %% =============================================================

            else

                Xi_neighbor = reshape( ...
                    x(j,n-1,:), ...
                    3,1);


                % CTH reference:
                %
                % p_ref = p_predecessor - h*v_i
                %

                Xi_neighbor(1) = ...
                    Xi_neighbor(1) ...
                    - h*(x_prev(1) + dd(i,n,1));


                Vi_neighbor = x(j,n-1,2);

                Ui_neighbor = u(j,n-1);

            end


            %% ------------------------------------------------------------
            % Position / velocity error accumulation
            %% ------------------------------------------------------------

            Xi_error_sum = ...
                Xi_error_sum ...
                + Xi ...
                - Xi_neighbor ...
                + D0_function(i,j,ddes);


            Vi_sum(i,n) = ...
                Vi_sum(i,n) ...
                + Vi_neighbor;


            %% ------------------------------------------------------------
            % Neighbor acceleration
            %% ------------------------------------------------------------

            if isDelayed

                Ui_sum = Ui_sum + Ui_neighbor;

            else

                Ui_sum = Ui_sum + Ui_neighbor;

            end

        end


        %% =================================================================
        % PLATOON ERRORS
        %% =================================================================

        e(i,n,1) = Xi_error_sum(1);

        e(i,n,2) = Xi_error_sum(2);


        %% =================================================================
        % LONGITUDINAL CSMC
        %% =================================================================

        if flagFinitefTimeCtrl


            % Linear convergence component
            ur = k1*e(i,n,1);


            % Integral term
            intOfUr(i) = ...
                intOfUr(i) ...
                + ur*deltaT;


            % Sliding variable
            s(i,n) = ...
                e(i,n,1) ...
                + epsilon*intOfUr(i);


            % Coupled sliding variable
            if i < Nveh

                S(i,n) = ...
                    q*s(i,n) ...
                    - s(i+1,n-1);

            else

                S(i,n) = q*s(i,n);

            end


            %% ------------------------------------------------------------
            % Longitudinal control
            %% ------------------------------------------------------------

            Ui = ...
                ( ...
                  Vi_sum(i,n) ...
                  - DP(i)*x_prev(1) ...
                  - epsilon*ur ...
                  - (delta/q)*sign(S(i,n))*abs(S(i,n))^ppp ...
                ) ...
                /(DP(i)*h);


        else

            %% ------------------------------------------------------------
            % Linear fallback
            %% ------------------------------------------------------------

            Ui = -k1*e(i,n,1);

        end


        %% ----------------------------------------------------------------
        % Longitudinal saturation
        %% ----------------------------------------------------------------

        if flagCtrlSat

            Ui = max(min(Ui,aM),am);

        end


        %% ----------------------------------------------------------------
        % Save longitudinal control
        %% ----------------------------------------------------------------

        u(i,n) = Ui;

        u_control_history(i,n) = Ui;


        %% =================================================================
        % ACTUAL 16-DOF STATES
        %% =================================================================

        vx_actual   = x_prev(1);
        vy_actual   = x_prev(2);

        r_t_actual  = x_prev(3);

        psi_actual  = x_prev(6);

        Y_actual    = x_prev(7);
        X_actual    = x_prev(8);

        phi_actual  = x_prev(15);


        %% =================================================================
        % DOUBLE LANE CHANGE REFERENCE
        %% =================================================================

        % Initial X of this vehicle
        X0_vehicle = x_16dof(i,1,8);


        %% ----------------------------------------------------------------
        % Reference at ACTUAL X
        % Used only for plotting
        %% ----------------------------------------------------------------

        x_actual_ref = ...
            X_actual - X0_vehicle;


        za1 = k_lane*(x_actual_ref - x_lane_start);
        za2 = k_lane*(x_actual_ref - x_lane_end);

        y_ref_actual = ...
            A_lane*(tanh(za1)-tanh(za2));


        %% ----------------------------------------------------------------
        % LOOK-AHEAD REFERENCE
        %% ----------------------------------------------------------------

        X_lookahead = ...
            X_actual + lookahead;


        x_la = ...
            X_lookahead - X0_vehicle;


        %% ----------------------------------------------------------------
        % Smooth S-curve
        %% ----------------------------------------------------------------

        z1 = k_lane*(x_la - x_lane_start);
        z2 = k_lane*(x_la - x_lane_end);

        tanh1 = tanh(z1);
        tanh2 = tanh(z2);

        sech1_sq = 1 - tanh1^2;
        sech2_sq = 1 - tanh2^2;


        %% ----------------------------------------------------------------
        % Desired lateral position
        %% ----------------------------------------------------------------

        y_ref = ...
            A_lane*(tanh1 - tanh2);


        %% ----------------------------------------------------------------
        % First derivative
        %% ----------------------------------------------------------------

        dy_dx = ...
            A_lane*k_lane*( ...
                sech1_sq ...
                - sech2_sq);


        %% ----------------------------------------------------------------
        % Second derivative
        %% ----------------------------------------------------------------

        d2y_dx2 = ...
            A_lane*k_lane^2*( ...
                -2*tanh1*sech1_sq ...
                +2*tanh2*sech2_sq);


        %% ----------------------------------------------------------------
        % Desired heading
        %% ----------------------------------------------------------------

        psi_ref = ...
            atan2(dy_dx,1);


        %% ----------------------------------------------------------------
        % Desired curvature
        %% ----------------------------------------------------------------

        den_curvature = ...
            max((1 + dy_dx^2)^(3/2),1e-6);

        kappa_ref = ...
            d2y_dx2/den_curvature;


        %% ----------------------------------------------------------------
        % Curvature feedforward
        %
        % Vehicle wheelbase approximation = p.L_t
        %% ----------------------------------------------------------------

        delta_ff = ...
            atan(p.L_t*kappa_ref);


        %% =================================================================
        % LATERAL TRACKING ERRORS
        %% =================================================================

        % Cross-track error
        e_y = ...
            y_ref - Y_actual;


        % Heading error wrapped to [-pi, pi]
        e_psi = atan2( ...
            sin(psi_ref - psi_actual), ...
            cos(psi_ref - psi_actual));


        %% =================================================================
        % STANLEY-LIKE LATERAL CONTROLLER
        %% =================================================================

        vx_safe = max(abs(vx_actual),1.0);


        % Cross-track correction
        delta_cte = ...
            atan2(K_stanley*e_y,vx_safe);


        % Complete steering law
        delta_f_cmd = ...
              delta_ff ...
            + delta_cte ...
            + Kpsi*e_psi ...
            - Kr*r_t_actual ...
            - Kvy*vy_actual;


        %% =================================================================
        % STEERING RATE / COMMAND LIMIT
        %% =================================================================

        delta_f_cmd = ...
            max( ...
                min(delta_f_cmd,delta_limit), ...
                -delta_limit);


        %% =================================================================
        % CONTROL ALLOCATION
        %% =================================================================

        % NOTE:
        % allocate_control(a_des, delta_f, vx, r, roll_angle, p)
        %
        % Correct 16-DOF states:
        %   yaw rate   = x_prev(3)
        %   roll angle = x_prev(15)

        u_16dof = allocate_control( ...
            Ui, ...
            delta_f_cmd, ...
            max(vx_actual,1.0), ...
            r_t_actual, ...
            phi_actual, ...
            p);


        %% ----------------------------------------------------------------
        % Additional steering protection
        %% ----------------------------------------------------------------

        u_16dof(1) = ...
            max( ...
                min(u_16dof(1),delta_limit), ...
                -delta_limit);


        %% ----------------------------------------------------------------
        % Drive torque protection
        %% ----------------------------------------------------------------

        u_16dof(2) = ...
            max( ...
                min(u_16dof(2),15000), ...
                1000);


        %% =================================================================
        % 16-DOF VEHICLE PLANT
        %% =================================================================

        [dxdt,~] = ...
            plant_16dof( ...
                x_prev, ...
                u_16dof, ...
                p);


        %% ----------------------------------------------------------------
        % Numerical protection
        %% ----------------------------------------------------------------

        dxdt(isnan(dxdt)) = 0;
        dxdt(isinf(dxdt)) = 0;


        %% ----------------------------------------------------------------
        % Euler integration
        %% ----------------------------------------------------------------

        x_next = ...
            x_prev + dxdt*Tstep;


        %% ----------------------------------------------------------------
        % Physical longitudinal speed floor
        %% ----------------------------------------------------------------

        x_next(1) = ...
            max(x_next(1),0.1);


        %% =================================================================
        % STORE FULL 16-DOF STATE
        %% =================================================================

        x_16dof(i,n,:) = x_next;


        %% =================================================================
        % STORE ACTUAL GLOBAL POSITION
        %% =================================================================

        x_global(i,n) = x_next(8);

        y_global(i,n) = x_next(7);


        %% =================================================================
        % STORE LANE REFERENCES / ERRORS
        %% =================================================================

        y_ref_history(i,n)   = y_ref_actual;
        psi_ref_history(i,n) = psi_ref;

        delta_history(i,n) = delta_f_cmd;

        e_y_history(i,n) = e_y;
        e_psi_history(i,n) = e_psi;


        %% =================================================================
        % STORE REDUCED STATES
        %% =================================================================

        % Actual X
        x(i,n,1) = x_next(8);

        % Actual vx
        x(i,n,2) = x_next(1);

        % Applied longitudinal acceleration command
        x(i,n,3) = Ui;


    end

end


%% ========================================================================
% POST PROCESSING
%% ========================================================================

p0 = x0(1,:);
v0_plot = x0(2,:);
a0 = x0(3,:);

p_veh = reshape(x(:,:,1),Nveh,Nstep);
v_veh = reshape(x(:,:,2),Nveh,Nstep);

space = reshape(e(:,:,1),Nveh,Nstep);
velocity_error = reshape(e(:,:,2),Nveh,Nstep);

t = (1:Nstep)*Tstep;


%% ========================================================================
% EXTRACT FULL 16-DOF STATES
%% ========================================================================

vx_16  = zeros(Nveh,Nstep);
vy_16  = zeros(Nveh,Nstep);

r_16   = zeros(Nveh,Nstep);
rs_16  = zeros(Nveh,Nstep);

gamma_16 = zeros(Nveh,Nstep);
psi_16   = zeros(Nveh,Nstep);

Y_16 = zeros(Nveh,Nstep);
X_16 = zeros(Nveh,Nstep);

phi_16 = zeros(Nveh,Nstep);


for i = 1:Nveh

    vx_16(i,:) = ...
        squeeze(x_16dof(i,:,1));

    vy_16(i,:) = ...
        squeeze(x_16dof(i,:,2));

    r_16(i,:) = ...
        squeeze(x_16dof(i,:,3));

    rs_16(i,:) = ...
        squeeze(x_16dof(i,:,4));

    gamma_16(i,:) = ...
        squeeze(x_16dof(i,:,5));

    psi_16(i,:) = ...
        squeeze(x_16dof(i,:,6));

    Y_16(i,:) = ...
        squeeze(x_16dof(i,:,7));

    X_16(i,:) = ...
        squeeze(x_16dof(i,:,8));

    phi_16(i,:) = ...
        squeeze(x_16dof(i,:,15));

end


%% ========================================================================
% LEADER VELOCITY ERROR
%% ========================================================================

leader_velocity_error = zeros(Nveh,Nstep);

for i = 1:Nveh

    leader_velocity_error(i,:) = ...
        vx_16(i,:) - v0_plot;

end


%% ========================================================================
% CREATE DESIRED PATH FOR PLOT
%% ========================================================================

X_ref_plot = linspace(0,180,2000);

Y_ref_plot = ...
    A_lane*( ...
        tanh(k_lane*(X_ref_plot-x_lane_start)) ...
        - ...
        tanh(k_lane*(X_ref_plot-x_lane_end)) );


%% ========================================================================
% PLOTTING
%% ========================================================================

if doPic

    set(0,'DefaultFigureColor','white');

    set(0, ...
        'DefaultAxesFontName','Times New Roman');

    set(0, ...
        'DefaultAxesFontSize',12);

    colors = lines(Nveh);


    %% ====================================================================
    % FIGURE 1
    % Longitudinal positions
    %% ====================================================================

    figure(1);
    clf;
    hold on;

    for i = 1:Nveh

        plot( ...
            t, ...
            p_veh(i,:), ...
            'LineWidth',1.8, ...
            'Color',colors(i,:), ...
            'DisplayName',sprintf('Vehicle %d',i));

    end

    plot( ...
        t,p0, ...
        'k--', ...
        'LineWidth',2.3, ...
        'DisplayName','Leader');

    xlabel('Time (s)');
    ylabel('X Position (m)');
    title('Longitudinal Vehicle Position');

    legend('Location','best');
    grid on;
    box on;


    %% ====================================================================
    % FIGURE 2
    % Velocity
    %% ====================================================================

    figure(2);
    clf;
    hold on;

    for i = 1:Nveh

        plot( ...
            t, ...
            vx_16(i,:), ...
            'LineWidth',1.8, ...
            'Color',colors(i,:), ...
            'DisplayName',sprintf('Vehicle %d',i));

    end

    plot( ...
        t,v0_plot, ...
        'k--', ...
        'LineWidth',2.3, ...
        'DisplayName','Leader');

    xlabel('Time (s)');
    ylabel('Velocity v_x (m/s)');
    title('Vehicle Velocity Profiles');

    legend('Location','best');
    grid on;
    box on;


    %% ====================================================================
    % FIGURE 3
    % Spacing error
    %% ====================================================================

    figure(3);
    clf;
    hold on;

    for i = 2:Nveh

        plot( ...
            t, ...
            space(i,:), ...
            'LineWidth',1.8, ...
            'Color',colors(i,:), ...
            'DisplayName',sprintf('Vehicle %d',i));

    end

    yline(0,'k--','LineWidth',1.1);

    xlabel('Time (s)');
    ylabel('Spacing Error e_i (m)');
    title('Vehicle Spacing Errors');

    legend('Location','best');
    grid on;
    box on;


    %% ====================================================================
    % FIGURE 4
    % Longitudinal control
    %% ====================================================================

    figure(4);
    clf;
    hold on;

    for i = 1:Nveh

        plot( ...
            t, ...
            u_control_history(i,:), ...
            'LineWidth',1.7, ...
            'Color',colors(i,:), ...
            'DisplayName',sprintf('Vehicle %d',i));

    end

    yline(aM, ...
        'k--', ...
        'LineWidth',1.1, ...
        'DisplayName','Upper Limit');

    yline(am, ...
        'k-.', ...
        'LineWidth',1.1, ...
        'DisplayName','Lower Limit');

    xlabel('Time (s)');
    ylabel('Longitudinal Control (m/s^2)');
    title('Platoon Longitudinal Control');

    legend('Location','best');
    grid on;
    box on;


    %% ====================================================================
    % FIGURE 5
    % ACTUAL XY TRAJECTORIES
    %
    % This is the most important plot.
    %% ====================================================================

    figure(5);
    clf;
    hold on;

    % Desired S-curve
    plot( ...
        X_ref_plot, ...
        Y_ref_plot, ...
        'k--', ...
        'LineWidth',2.8, ...
        'DisplayName','Desired Path');


    % Actual vehicle trajectories
    for i = 1:Nveh

        Xplot = ...
            X_16(i,:) - X_16(i,1);

        Yplot = ...
            Y_16(i,:);

        plot( ...
            Xplot, ...
            Yplot, ...
            'LineWidth',1.8, ...
            'Color',colors(i,:), ...
            'DisplayName',sprintf('Vehicle %d',i));

    end

    xlabel('Relative X Position (m)');
    ylabel('Global Y Position (m)');
    title('Double Lane Change Tracking');

    xlim([0 180]);
    ylim([-1 5]);

    legend('Location','best');
    grid on;
    box on;


    %% ====================================================================
    % FIGURE 6
    % Lateral tracking error
    %% ====================================================================

    figure(6);
    clf;
    hold on;

    for i = 1:Nveh

        plot( ...
            t, ...
            e_y_history(i,:), ...
            'LineWidth',1.7, ...
            'Color',colors(i,:), ...
            'DisplayName',sprintf('Vehicle %d',i));

    end

    yline(0, ...
        'k--', ...
        'LineWidth',1.1);

    xlabel('Time (s)');
    ylabel('Lateral Tracking Error e_y (m)');
    title('Lateral Tracking Error');

    legend('Location','best');
    grid on;
    box on;


    %% ====================================================================
    % FIGURE 7
    % Heading error
    %% ====================================================================

    figure(7);
    clf;
    hold on;

    for i = 1:Nveh

        plot( ...
            t, ...
            rad2deg(e_psi_history(i,:)), ...
            'LineWidth',1.7, ...
            'Color',colors(i,:), ...
            'DisplayName',sprintf('Vehicle %d',i));

    end

    yline(0, ...
        'k--', ...
        'LineWidth',1.1);

    xlabel('Time (s)');
    ylabel('Heading Error (deg)');
    title('Heading Tracking Error');

    legend('Location','best');
    grid on;
    box on;


    %% ====================================================================
    % FIGURE 8
    % Steering angle
    %% ====================================================================

    figure(8);
    clf;
    hold on;

    for i = 1:Nveh

        plot( ...
            t, ...
            rad2deg(delta_history(i,:)), ...
            'LineWidth',1.7, ...
            'Color',colors(i,:), ...
            'DisplayName',sprintf('Vehicle %d',i));

    end

    yline( ...
        rad2deg(delta_limit), ...
        'k--', ...
        'LineWidth',1.1, ...
        'DisplayName','Steering Limit');

    yline( ...
        -rad2deg(delta_limit), ...
        'k-.', ...
        'LineWidth',1.1, ...
        'HandleVisibility','off');

    xlabel('Time (s)');
    ylabel('\delta_f (deg)');
    title('Front Steering Commands');

    legend('Location','best');
    grid on;
    box on;


    %% ====================================================================
    % FIGURE 9
    % Tractor yaw rate
    %% ====================================================================

    figure(9);
    clf;
    hold on;

    for i = 1:Nveh

        plot( ...
            t, ...
            rad2deg(r_16(i,:)), ...
            'LineWidth',1.7, ...
            'Color',colors(i,:), ...
            'DisplayName',sprintf('Vehicle %d',i));

    end

    xlabel('Time (s)');
    ylabel('Tractor Yaw Rate r_t (deg/s)');
    title('Tractor Yaw Rate');

    legend('Location','best');
    grid on;
    box on;


    %% ====================================================================
    % FIGURE 10
    % Articulation angle
    %% ====================================================================

    figure(10);
    clf;
    hold on;

    for i = 1:Nveh

        plot( ...
            t, ...
            rad2deg(gamma_16(i,:)), ...
            'LineWidth',1.7, ...
            'Color',colors(i,:), ...
            'DisplayName',sprintf('Vehicle %d',i));

    end

    yline(0,'k--','LineWidth',1.0);

    xlabel('Time (s)');
    ylabel('Articulation Angle \gamma (deg)');
    title('Tractor-Semitrailer Articulation Angle');

    legend('Location','best');
    grid on;
    box on;


    %% ====================================================================
    % FIGURE 11
    % Roll angle
    %% ====================================================================

    figure(11);
    clf;
    hold on;

    for i = 1:Nveh

        plot( ...
            t, ...
            rad2deg(phi_16(i,:)), ...
            'LineWidth',1.7, ...
            'Color',colors(i,:), ...
            'DisplayName',sprintf('Vehicle %d',i));

    end

    xlabel('Time (s)');
    ylabel('Roll Angle \phi_t (deg)');
    title('Tractor Roll Angle');

    legend('Location','best');
    grid on;
    box on;


    %% ====================================================================
    % FIGURE 12
    % Lateral position versus time
    %% ====================================================================

    figure(12);
    clf;
    hold on;

    for i = 1:Nveh

        plot( ...
            t, ...
            Y_16(i,:), ...
            'LineWidth',1.7, ...
            'Color',colors(i,:), ...
            'DisplayName',sprintf('Vehicle %d',i));

    end

    xlabel('Time (s)');
    ylabel('Y Position (m)');
    title('Lateral Position vs Time');

    legend('Location','best');
    grid on;
    box on;


    %% ====================================================================
    % FIGURE 13
    % Velocity error
    %% ====================================================================

    figure(13);
    clf;
    hold on;

    for i = 1:Nveh

        plot( ...
            t, ...
            leader_velocity_error(i,:), ...
            'LineWidth',1.7, ...
            'Color',colors(i,:), ...
            'DisplayName',sprintf('Vehicle %d',i));

    end

    yline(0,'k--','LineWidth',1.0);

    xlabel('Time (s)');
    ylabel('Velocity Error (m/s)');
    title('Leader-Follower Velocity Error');

    legend('Location','best');
    grid on;
    box on;

end


%% ========================================================================
% NUMERICAL PERFORMANCE SUMMARY
%% ========================================================================

max_spacing_error = zeros(Nveh,1);

rms_lateral_error = zeros(Nveh,1);

max_lateral_error = zeros(Nveh,1);

max_heading_error = zeros(Nveh,1);

max_steering = zeros(Nveh,1);

max_roll = zeros(Nveh,1);

max_articulation = zeros(Nveh,1);


for i = 1:Nveh

    max_spacing_error(i) = ...
        max(abs(space(i,:)));

    rms_lateral_error(i) = ...
        sqrt(mean(e_y_history(i,:).^2));

    max_lateral_error(i) = ...
        max(abs(e_y_history(i,:)));

    max_heading_error(i) = ...
        max(abs(rad2deg(e_psi_history(i,:))));

    max_steering(i) = ...
        max(abs(rad2deg(delta_history(i,:))));

    max_roll(i) = ...
        max(abs(rad2deg(phi_16(i,:))));

    max_articulation(i) = ...
        max(abs(rad2deg(gamma_16(i,:))));

end


%% ========================================================================
% PRINT RESULTS
%% ========================================================================

fprintf('\n');
fprintf('============================================================\n');
fprintf('     PLATOON + 16-DOF + DOUBLE LANE CHANGE\n');
fprintf('============================================================\n');

fprintf('\n');

fprintf('Maximum spacing error:\n');

for i = 1:Nveh

    fprintf( ...
        'Vehicle %d : %.4f m\n', ...
        i,max_spacing_error(i));

end


fprintf('\n');

fprintf('RMS lateral tracking error:\n');

for i = 1:Nveh

    fprintf( ...
        'Vehicle %d : %.4f m\n', ...
        i,rms_lateral_error(i));

end


fprintf('\n');

fprintf('Maximum lateral tracking error:\n');

for i = 1:Nveh

    fprintf( ...
        'Vehicle %d : %.4f m\n', ...
        i,max_lateral_error(i));

end


fprintf('\n');

fprintf('Maximum heading error:\n');

for i = 1:Nveh

    fprintf( ...
        'Vehicle %d : %.4f deg\n', ...
        i,max_heading_error(i));

end


fprintf('\n');

fprintf('Maximum steering angle:\n');

for i = 1:Nveh

    fprintf( ...
        'Vehicle %d : %.4f deg\n', ...
        i,max_steering(i));

end


fprintf('\n');

fprintf('Maximum roll angle:\n');

for i = 1:Nveh

    fprintf( ...
        'Vehicle %d : %.4f deg\n', ...
        i,max_roll(i));

end


fprintf('\n');

fprintf('Maximum articulation angle:\n');

for i = 1:Nveh

    fprintf( ...
        'Vehicle %d : %.4f deg\n', ...
        i,max_articulation(i));

end


fprintf('\n');
fprintf('============================================================\n');
fprintf('Simulation completed.\n');
fprintf('============================================================\n');


%% ========================================================================
% LOCAL FUNCTION
%
% Reproduces:
%     f(i,j) = (i-j) + i*(i==j)
%
% and multiplies by D0 = [ddes; 0; 0]
%
% This is used in the spacing-error formulation.
%% ========================================================================

function Dterm = D0_function(i,j,ddes)

    f_ij = (i-j) + i*(i==j);

    Dterm = [ ...
        ddes*f_ij;
        0;
        0];

end