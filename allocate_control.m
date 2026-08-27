function u_input = allocate_control(a_des, delta_f, v_x, r, roll_angle, p)
% allocate_control: Phân bổ điều khiển toàn diện cho mô hình 16-DOF
    u_input = zeros(9, 1);
    
    % 1. Góc lái vô lăng (Chuyển làn) giới hạn theo thông số xe
    u_input(1) = max(min(delta_f, p.delta_max), -p.delta_max); 
    
    epsilon = 1e-3;
    if abs(v_x) < epsilon, v_x_eff = epsilon; else, v_x_eff = v_x; end
    
    % 2. Lực cản không khí và Lực dọc yêu cầu từ Platooning
    F_a = 0.5 * p.C_D * p.A_a * p.rho_a * v_x_eff^2;
    F_req = p.m_tot * a_des + F_a;
    F_req = max(min(F_req, p.Fx_cmd_max), p.Fx_cmd_min);
    
    if F_req >= 0
        % Trạng thái tăng tốc/kéo
        T_drive = F_req * p.R_f;
        u_input(2) = max(min(T_drive, p.Fx_cmd_max * p.R_f), 0);
        u_input(3) = 0; 
    else
        % Trạng thái phanh: Phân bổ phanh vi sai tạo Yaw Moment chống văng đuôi
        T_brake_total = abs(F_req) * p.R_f;
        delta_T = 2000 * r; 
        
        Tb_base_f = (0.3 * T_brake_total) / 2;
        Tb_base_r = (0.3 * T_brake_total) / 2;
        Tb_base_s = (0.4 * T_brake_total) / 2;
        
        u_input(4) = max(Tb_base_f - delta_T, 0); % Tb_fL
        u_input(5) = max(Tb_base_f + delta_T, 0); % Tb_fR
        u_input(6) = max(Tb_base_r - delta_T, 0); % Tb_rL
        u_input(7) = max(Tb_base_r + delta_T, 0); % Tb_rR
        u_input(8) = Tb_base_s;                   % Tb_sL
        u_input(9) = Tb_base_s;                   % Tb_sR
    end
    
    % 3. Kiểm soát chống lật thông qua chỉ số LTR và Moment treo chủ động M_act
    LTR = abs(roll_angle) / 0.15; 
    if LTR > p.LTR_max
        u_input(3) = sign(roll_angle) * p.M_act_max;
    else
        u_input(3) = 0;
    end
end