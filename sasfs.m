%% ==============================================================================
%  文件名: Stewart_Ball_Balancer.m
%  描述: 终极优化版 Stewart 平台板球系统 (LQI积分控制 + 向量化IK + 双轴RLS)
%  改进: 
%    1. [控制] 升级 LQR 为 LQI (增加积分项)，消除稳态误差
%    2. [效率] 逆运动学 (IK) 算法全面向量化，移除循环，提升计算速度
%    3. [修复] 修正小球悬空问题：基于几何边心距计算严格的安全半径
%    4. [视觉] 仪表盘显示真实的安全边界
%  作者: Gemini (Assisting User)
% ==============================================================================

clear; clc; close all;

%% ------------------- 1. 机械结构参数 -------------------
% 1.1 关键几何尺寸
L_side_base = 0.032 + 0.040 + 0.032; 
dist_servo  = 0.040;                 

% 计算底座半径与偏置角
r_in_hex = L_side_base * sqrt(3) / 2; 
d_off = dist_servo / 2;
PAR.radius_base = sqrt(r_in_hex^2 + d_off^2);
delta_angle_base = 2 * rad2deg(atan(d_off / r_in_hex));

% 连杆参数
PAR.radius_plat     = 0.095;  % 动平台外接圆半径 (连接点距离)
PAR.len_servo_arm   = 0.045;  
PAR.len_rod         = 0.160;  
PAR.r_ball          = 0.015;  
PAR.m_ball          = 0.050;  

% 1.2 角度布局 (成对分布 Pairwise)
delta_angle_plat = 20; 
PAR.beta_base = [ ...
    30 - delta_angle_base/2, 30 + delta_angle_base/2, ...
    150 - delta_angle_base/2, 150 + delta_angle_base/2, ...
    270 - delta_angle_base/2, 270 + delta_angle_base/2 ];

PAR.beta_plat = [ ...
    30 - delta_angle_plat/2, 30 + delta_angle_plat/2, ...
    150 - delta_angle_plat/2, 150 + delta_angle_plat/2, ...
    270 - delta_angle_plat/2, 270 + delta_angle_plat/2 ];

% [新增] 计算平台几何的内切安全半径 (防止视觉悬空)
max_span_deg = 120 - delta_angle_plat;
PAR.radius_safe = PAR.radius_plat * cos(deg2rad(max_span_deg/2)); 
fprintf('平台几何参数:\n 外接半径: %.3fm\n 安全内切半径: %.3fm\n', PAR.radius_plat, PAR.radius_safe);

% 1.3 初始高度
h_theoretical = sqrt(PAR.len_rod^2 - (PAR.radius_base - PAR.radius_plat)^2);
PAR.z_home = h_theoretical; 

fprintf('=== 系统初始化完成 (LQI Mode) ===\n');


%% ------------------- 2. 控制与物理配置 -------------------
dt = 0.005;          % 采样周期 5ms
T_total = 25.0;      % 仿真时长
time_vec = 0:dt:T_total;

% --- 深度物理参数 ---
g = 9.81;
I_ball = (2/5) * PAR.m_ball * PAR.r_ball^2;
K_dyn_true = PAR.m_ball / (PAR.m_ball + I_ball / PAR.r_ball^2); 
mu_friction = 0.008; 

% --- 延迟模拟 ---
delay_ms = 40;       
delay_steps = round(delay_ms / (dt*1000));
buf_theta = zeros(1, delay_steps + 1);
buf_phi   = zeros(1, delay_steps + 1);

% --- [优化1] LQI 参数 (带积分作用) ---
% 状态向量 x_aug = [积分误差; 位置误差; 速度误差]
% x_dot = A*x + B*u
A_sys = [0, 1, 0;  % 积分误差导数 = 位置误差
         0, 0, 1;  % 位置误差导数 = 速度误差
         0, 0, 0]; % 速度误差导数 = B*u (近似)
     
B_sys = [0; 0; K_dyn_true * g];

% 权重矩阵 Q: [积分权重, 位置权重, 速度权重]
% 增加积分权重可消除稳态误差，但过大会导致超调
Q_lqr = diag([150, 350, 8]); 
R_lqr = 0.8;            

try
    K_lqi = lqr(A_sys, B_sys, Q_lqr, R_lqr);
    fprintf('LQI Gains: Ki=%.3f, Kp=%.3f, Kd=%.3f\n', K_lqi(1), K_lqi(2), K_lqi(3));
catch
    K_lqi = [10.0, 14.0, 3.5]; % 备用
end

% --- RLS 系统辨识 ---
rls_x.theta = 0.5; rls_x.P = 1000; rls_x.lambda = 0.998;
rls_y.theta = 0.5; rls_y.P = 1000; rls_y.lambda = 0.998;

% --- 状态观测器 ---
L_obs = [0.18; 0.5]; 
x_hat = [0; 0]; y_hat = [0; 0];

% 数据预分配
max_steps = length(time_vec);
log.time       = zeros(1, max_steps);
log.pos_true   = zeros(2, max_steps);
log.pos_target = zeros(2, max_steps);
log.angles_cmd = zeros(2, max_steps);
log.u_comp_x   = zeros(2, max_steps);
log.k_est      = zeros(2, max_steps);
log.servos     = zeros(6, max_steps);
log.valid_len  = 0; 


%% ------------------- 3. 仿真计算 -------------------
fprintf('正在执行物理仿真计算...\n');

theta_cmd_prev = 0; phi_cmd_prev = 0;
theta_filtered = 0; phi_filtered = 0;
state_true = [0; 0; 0; 0]; 
log.pos_est_vel_prev_x = 0;
log.pos_est_vel_prev_y = 0;
theta_accum = 0; phi_accum = 0;

% 积分误差累积器
err_int_x = 0;
err_int_y = 0;

R_ball_vis = eye(3); 

for i = 1:max_steps
    t = time_vec(i);
    x_curr = state_true(1); y_curr = state_true(3);
    
    % === Step 1: 物理环境 & 掉球检测 ===
    limit_dist = PAR.radius_safe - PAR.r_ball/2; 
    dist_from_center = sqrt(x_curr^2 + y_curr^2);
    
    if dist_from_center > limit_dist
        fprintf('!!! 警告: 小球在 t=%.2fs 时越界 (%.3fm > %.3fm)，仿真停止 !!!\n', t, dist_from_center, limit_dist);
        log.valid_len = i - 1;
        break; 
    end
    
    noise = 0.001 * randn(2,1); 
    meas_x = x_curr + noise(1);
    meas_y = y_curr + noise(2);
    
    % === Step 2: 状态观测器 ===
    B_hat_x = [0; rls_x.theta * g * dt];
    x_hat_pred = [1, dt; 0, 1] * x_hat + B_hat_x * theta_cmd_prev;
    x_hat = x_hat_pred + L_obs * (meas_x - x_hat_pred(1));
    
    B_hat_y = [0; rls_y.theta * g * dt];
    y_hat_pred = [1, dt; 0, 1] * y_hat + B_hat_y * (-phi_cmd_prev); 
    y_hat = y_hat_pred + L_obs * (meas_y - y_hat_pred(1));
    
    % === Step 3: 双轴 RLS 辨识 ===
    theta_accum = theta_accum + theta_cmd_prev;
    phi_accum   = phi_accum   - phi_cmd_prev; 
    
    if mod(i, 4) == 0 && i > 10 
        % X 轴
        acc_est_x = (x_hat(2) - log.pos_est_vel_prev_x) / (dt*4); 
        ang_avg_x = theta_accum / 4;
        phi_k_x = g * sin(ang_avg_x);
        if abs(phi_k_x) > 0.15 
            K_gain = rls_x.P * phi_k_x / (rls_x.lambda + phi_k_x * rls_x.P * phi_k_x);
            rls_x.theta = rls_x.theta + K_gain * (acc_est_x - phi_k_x * rls_x.theta);
            rls_x.P = (rls_x.P - K_gain * phi_k_x * rls_x.P) / rls_x.lambda;
        end
        rls_x.theta = max(min(rls_x.theta, 0.9), 0.4);
        
        % Y 轴
        acc_est_y = (y_hat(2) - log.pos_est_vel_prev_y) / (dt*4);
        ang_avg_y = phi_accum / 4; 
        phi_k_y = g * sin(ang_avg_y);
        if abs(phi_k_y) > 0.15
            K_gain = rls_y.P * phi_k_y / (rls_y.lambda + phi_k_y * rls_y.P * phi_k_y);
            rls_y.theta = rls_y.theta + K_gain * (acc_est_y - phi_k_y * rls_y.theta);
            rls_y.P = (rls_y.P - K_gain * phi_k_y * rls_y.P) / rls_y.lambda;
        end
        rls_y.theta = max(min(rls_y.theta, 0.9), 0.4);
        
        theta_accum = 0; phi_accum = 0;
        log.pos_est_vel_prev_x = x_hat(2);
        log.pos_est_vel_prev_y = y_hat(2);
    end
    
    % === Step 4: 控制器 (LQI + 前馈) ===
    % 8字形轨迹
    A_x = 0.040; w_x = 1.2;
    A_y = 0.030; w_y = 2.4; 
    
    rx = A_x * sin(w_x * t); 
    vx = A_x * w_x * cos(w_x * t); 
    ax = -A_x * w_x^2 * sin(w_x * t);
    
    ry = A_y * sin(w_y * t); 
    vy = A_y * w_y * cos(w_y * t); 
    ay = -A_y * w_y^2 * sin(w_y * t);
    
    % 计算误差
    ex = x_hat(1) - rx; evx = x_hat(2) - vx;
    ey = y_hat(1) - ry; evy = y_hat(2) - vy;
    
    % 累积积分误差 (带抗饱和)
    err_int_x = max(min(err_int_x + ex * dt, 0.2), -0.2);
    err_int_y = max(min(err_int_y + ey * dt, 0.2), -0.2);
    
    % LQI 控制律: u = -K * [int_e; e; de]
    u_fb_x = -K_lqi(1)*err_int_x - K_lqi(2)*ex - K_lqi(3)*evx;
    u_ff_x = asin(ax / (rls_x.theta * g));
    theta_raw = u_fb_x + u_ff_x;
    
    u_fb_y = -K_lqi(1)*err_int_y - K_lqi(2)*ey - K_lqi(3)*evy;
    u_ff_y = asin(ay / (rls_y.theta * g));
    phi_raw = -(u_fb_y + u_ff_y); 
    
    % 滤波器
    alpha_filter = 0.3; 
    theta_filtered = (1 - alpha_filter) * theta_filtered + alpha_filter * theta_raw;
    phi_filtered   = (1 - alpha_filter) * phi_filtered   + alpha_filter * phi_raw;
    
    % 延迟模拟
    buf_theta = [buf_theta(2:end), theta_filtered];
    buf_phi   = [buf_phi(2:end),   phi_filtered];
    theta_act = buf_theta(1);
    phi_act   = buf_phi(1);
    
    % 限幅
    theta_act = max(min(theta_act, deg2rad(14)), deg2rad(-14));
    phi_act   = max(min(phi_act,   deg2rad(14)), deg2rad(-14));
    
    theta_cmd_prev = theta_filtered; 
    phi_cmd_prev   = phi_filtered;
    
    % === Step 5: 逆运动学 (优化: 向量化计算) ===
    pose_target = [0, 0, PAR.z_home, phi_act, theta_act, 0];
    % [优化2] 使用向量化 IK 函数
    [servo_degs, isValid] = Stewart_IK_Rotary_Vectorized(pose_target, PAR);
    
    if ~isValid
        if i > 1, servo_degs = log.servos(:, i-1); else, servo_degs = zeros(6,1); end
    end
    
    % === Step 6: 物理更新 ===
    % X轴
    acc_x_ideal = K_dyn_true * g * sin(theta_act);
    fric_x = -mu_friction * g * sign(state_true(2));
    if abs(state_true(2))<0.005 && abs(acc_x_ideal)<mu_friction*g, fric_x = -acc_x_ideal; end
    ax_total = acc_x_ideal + fric_x + 0.02*randn();
    
    state_true(2) = state_true(2) + ax_total * dt; 
    state_true(1) = state_true(1) + state_true(2) * dt; 
    
    % Y轴
    acc_y_ideal = -K_dyn_true * g * sin(phi_act); 
    fric_y = -mu_friction * g * sign(state_true(4));
    if abs(state_true(4))<0.005 && abs(acc_y_ideal)<mu_friction*g, fric_y = -acc_y_ideal; end
    ay_total = acc_y_ideal + fric_y + 0.02*randn();
    
    state_true(4) = state_true(4) + ay_total * dt; 
    state_true(3) = state_true(3) + state_true(4) * dt; 
    
    % 更新记录
    log.time(i)       = t;
    log.pos_true(:,i) = [state_true(1); state_true(3)];
    log.pos_target(:,i) = [rx; ry];
    log.angles_cmd(:,i) = [theta_act; phi_act];
    log.u_comp_x(:,i) = [u_fb_x; u_ff_x]; 
    log.k_est(:,i)    = [rls_x.theta; rls_y.theta];
    log.servos(:, i)  = servo_degs;
    log.valid_len     = i;
end

% 截断数据
len = log.valid_len;
if len == 0, len = max_steps; log.valid_len = max_steps; end 
log.time       = log.time(1:len);
log.pos_true   = log.pos_true(:, 1:len);
log.pos_target = log.pos_target(:, 1:len);
log.angles_cmd = log.angles_cmd(:, 1:len);
log.u_comp_x   = log.u_comp_x(:, 1:len);
log.k_est      = log.k_est(:, 1:len);
log.servos     = log.servos(:, 1:len);


%% ------------------- 4. 七合一同步仪表盘 (修复版) -------------------
fig = figure('Name', 'Stewart平台 2D全向平衡仿真', 'Color', 'w', 'Position', [50, 50, 1400, 800]);

% --- 布局定义 (3x3 增强版) ---
ax_3d  = subplot(3, 3, [1, 4, 7]); % 左侧 3D
ax_traj= subplot(3, 3, 2); % 2D 轨迹
ax_pos = subplot(3, 3, 3); % X/Y 误差
ax_ang = subplot(3, 3, 5); % Pitch/Roll
ax_ctrl= subplot(3, 3, 6); % 控制分量
ax_srv = subplot(3, 3, 8); % 舵机
ax_rls = subplot(3, 3, 9); % RLS

% --- 1. 3D 实时仿真 ---
axes(ax_3d);
axis equal; grid on; hold on;
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
view(35, 25); 
xlim([-0.2 0.2]); ylim([-0.2 0.2]); zlim([-0.05 0.35]);
title('1. 3D 实时仿真 (含自旋)');

R_b = PAR.radius_base;
base_x = R_b * cos(deg2rad(PAR.beta_base));
base_y = R_b * sin(deg2rad(PAR.beta_base));
patch(base_x, base_y, zeros(1,6), [0.2 0.2 0.2], 'FaceAlpha', 0.3); 

h_plat = patch('XData', [], 'YData', [], 'ZData', [], 'FaceColor', [0 0.7 1], 'FaceAlpha', 0.8, 'EdgeColor', 'k');
h_legs = plot3(nan, nan, nan, 'k-', 'LineWidth', 1.5); 
h_arms = plot3(nan, nan, nan, 'r-', 'LineWidth', 2);   

[sx, sy, sz] = sphere(16);
sx = sx * PAR.r_ball; sy = sy * PAR.r_ball; sz = sz * PAR.r_ball;
C = zeros(size(sz)); C(8:10,:) = 1; 
h_ball = surf(sx, sy, sz, C, 'EdgeColor', 'none', 'FaceAlpha', 1);
colormap(ax_3d, 'winter'); 
light('Position', [0.5 -0.5 1], 'Style', 'infinite');
lighting gouraud; material shiny;

h_tgt_pt = plot3(nan, nan, nan, 'go', 'MarkerSize', 8, 'LineWidth', 2);

% --- 2. X-Y 平面轨迹图 ---
axes(ax_traj); hold on; grid on; title('2. 平面轨迹 (8字形)');
axis equal; xlim([-0.12 0.12]); ylim([-0.12 0.12]);
xlabel('X (m)'); ylabel('Y (m)');
plot(log.pos_target(1,:), log.pos_target(2,:), 'g--', 'Color', [0.8 0.8 0.8]);
h_trail = plot(nan, nan, 'b-', 'LineWidth', 1.5); 
h_head  = plot(nan, nan, 'bo', 'MarkerFaceColor', 'b'); 
h_tgt_2d= plot(nan, nan, 'gx', 'LineWidth', 2); 
viscircles([0,0], PAR.radius_safe, 'Color', 'r', 'LineStyle', '--'); 

% --- 3. X/Y 位置时间序列 ---
t_end = log.time(end);
axes(ax_pos); hold on; grid on; title('3. 位置追踪 (X & Y)');
xlim([0 t_end]); ylim([-0.12 0.12]);
h_line_x = plot(nan, nan, 'b', 'DisplayName', 'X轴');
h_line_y = plot(nan, nan, 'm', 'DisplayName', 'Y轴');
yline(PAR.radius_safe, 'r--'); yline(-PAR.radius_safe, 'r--'); 
h_cur_3 = xline(0, 'k');
legend([h_line_x, h_line_y], 'Location', 'best', 'Orientation', 'horizontal');

% --- 4. 平台姿态 (Pitch/Roll) ---
axes(ax_ang); hold on; grid on; title('4. 平台姿态 (Pitch/Roll)');
xlim([0 t_end]); ylim([-15 15]);
h_line_pit = plot(nan, nan, 'k', 'DisplayName', 'Pitch (X)');
h_line_rol = plot(nan, nan, 'Color', [0.8 0.5 0], 'DisplayName', 'Roll (Y)');
h_cur_4 = xline(0, 'k');
legend([h_line_pit, h_line_rol], 'Location', 'best', 'Orientation', 'horizontal');

% --- 5. 控制分量 (LQI vs FF) ---
axes(ax_ctrl); hold on; grid on; title('5. 控制分量分解 (X轴)');
xlim([0 t_end]); ylim([-12 12]);
h_line_fb = plot(nan, nan, 'm', 'DisplayName', 'LQI反馈');
h_line_ff = plot(nan, nan, 'c', 'DisplayName', '预瞄前馈');
h_cur_5 = xline(0, 'k');
legend([h_line_fb, h_line_ff], 'Location', 'best');

% --- 6. 舵机角度 ---
axes(ax_srv); hold on; grid on; title('6. 舵机角度');
xlim([0 t_end]);
colors = lines(6);
h_lines_srv = gobjects(6,1);
for k=1:6, h_lines_srv(k) = plot(nan, nan, 'Color', colors(k,:)); end
h_cur_6 = xline(0, 'k');

% --- 7. RLS 双轴辨识 ---
axes(ax_rls); hold on; grid on; title('7. RLS 在线参数辨识 (X & Y)');
xlim([0 t_end]); ylim([0.4 1.0]);
h_line_rls_x = plot(nan, nan, 'r', 'LineWidth', 1.5, 'DisplayName', 'X轴 Est');
h_line_rls_y = plot(nan, nan, 'b', 'LineWidth', 1.5, 'DisplayName', 'Y轴 Est');
yline(K_dyn_true, 'k--', '真实值');
h_cur_7 = xline(0, 'k');
legend([h_line_rls_x, h_line_rls_y], 'Location', 'best');


% --- 动画循环 ---
fprintf('开始回放... 按 Ctrl+C 退出\n');
step_size = 5; 
R_ball_curr = eye(3);

for k = 1:step_size:len
    if ~ishandle(fig), break; end 
    tic;
    
    t_now = log.time(k);
    pos_x = log.pos_true(1, k);
    pos_y = log.pos_true(2, k);
    
    % 更新球旋转 (视觉)
    if k > step_size
        dx = pos_x - log.pos_true(1, k-step_size);
        dy = pos_y - log.pos_true(2, k-step_size);
        d_ang = sqrt(dx^2 + dy^2) / PAR.r_ball;
        axis_rot = [-dy; dx; 0];
        if norm(axis_rot) > 1e-6
            axis_rot = axis_rot / norm(axis_rot);
            R_inc = axang2rotm([axis_rot', d_ang]);
            R_ball_curr = R_inc * R_ball_curr;
        end
    end
    
    % 3D 更新
    T = [0; 0; PAR.z_home];
    theta = log.angles_cmd(1, k);
    phi   = log.angles_cmd(2, k);
    Rx = [1 0 0; 0 cos(phi) -sin(phi); 0 sin(phi) cos(phi)];
    Ry = [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)];
    Rot = Ry * Rx; 
    
    P_pts = zeros(3, 6);
    for i = 1:6
        P_local = [PAR.radius_plat * cos(deg2rad(PAR.beta_plat(i))); PAR.radius_plat * sin(deg2rad(PAR.beta_plat(i))); 0];
        P_pts(:, i) = T + Rot * P_local;
    end
    
    ball_center = T + Rot * [pos_x; pos_y; PAR.r_ball];
    pts_ball = [sx(:), sy(:), sz(:)] * R_ball_curr'; 
    set(h_ball, 'XData', reshape(pts_ball(:,1), size(sx)) + ball_center(1), ...
                'YData', reshape(pts_ball(:,2), size(sx)) + ball_center(2), ...
                'ZData', reshape(pts_ball(:,3), size(sx)) + ball_center(3));
    
    tgt_world = T + Rot * [log.pos_target(1, k); log.pos_target(2, k); PAR.r_ball];
    set(h_tgt_pt, 'XData', tgt_world(1), 'YData', tgt_world(2), 'ZData', tgt_world(3));
    set(h_plat, 'XData', P_pts(1,:), 'YData', P_pts(2,:), 'ZData', P_pts(3,:));
    
    B_pts = zeros(3, 6); A_pts = zeros(3, 6);
    for i = 1:6
        beta_b = deg2rad(PAR.beta_base(i));
        B_pts(:, i) = [R_b * cos(beta_b); R_b * sin(beta_b); 0];
        alpha = deg2rad(log.servos(i, k));
        vec_tan = [-sin(beta_b); cos(beta_b); 0];
        if mod(i, 2) == 1, vec_x = vec_tan; else, vec_x = -vec_tan; end
        vec_y = [0;0;1];
        A_pts(:, i) = B_pts(:, i) + vec_x * (PAR.len_servo_arm * cos(alpha)) + vec_y * (PAR.len_servo_arm * sin(alpha));
    end
    
    lx=[]; ly=[]; lz=[]; ax=[]; ay=[]; az=[];
    for i=1:6
        ax=[ax, B_pts(1,i), A_pts(1,i), nan]; ay=[ay, B_pts(2,i), A_pts(2,i), nan]; az=[az, B_pts(3,i), A_pts(3,i), nan];
        lx=[lx, A_pts(1,i), P_pts(1,i), nan]; ly=[ly, A_pts(2,i), P_pts(2,i), nan]; lz=[lz, A_pts(3,i), P_pts(3,i), nan];
    end
    set(h_arms, 'XData', ax, 'YData', ay, 'ZData', az);
    set(h_legs, 'XData', lx, 'YData', ly, 'ZData', lz);
    
    % 图表更新
    idx = 1:k; t_d = log.time(idx);
    
    set(h_trail, 'XData', log.pos_true(1, max(1, k-200):k), 'YData', log.pos_true(2, max(1, k-200):k)); 
    set(h_head, 'XData', pos_x, 'YData', pos_y);
    set(h_tgt_2d, 'XData', log.pos_target(1, k), 'YData', log.pos_target(2, k));
    
    set(h_line_x, 'XData', t_d, 'YData', log.pos_true(1, idx));
    set(h_line_y, 'XData', t_d, 'YData', log.pos_true(2, idx));
    
    set(h_line_pit, 'XData', t_d, 'YData', rad2deg(log.angles_cmd(1, idx)));
    set(h_line_rol, 'XData', t_d, 'YData', rad2deg(log.angles_cmd(2, idx)));
    
    set(h_line_fb, 'XData', t_d, 'YData', rad2deg(log.u_comp_x(1, idx)));
    set(h_line_ff, 'XData', t_d, 'YData', rad2deg(log.u_comp_x(2, idx)));
    
    set(h_line_rls_x, 'XData', t_d, 'YData', log.k_est(1, idx));
    set(h_line_rls_y, 'XData', t_d, 'YData', log.k_est(2, idx));
    
    for s=1:6, set(h_lines_srv(s), 'XData', t_d, 'YData', log.servos(s, idx)); end
    
    % 光标
    set([h_cur_3, h_cur_4, h_cur_5, h_cur_6, h_cur_7], 'Value', t_now);
    
    drawnow limitrate;
    pause(max(0.001, dt*step_size - toc)); 
end
fprintf('仿真演示结束。\n');


%% ------------------- 5. 附录: 向量化逆运动学函数 -------------------
function [servo_angles, isValid] = Stewart_IK_Rotary_Vectorized(pose, PAR)
    % [优化] 向量化计算 6-RSS 逆运动学
    R_b = PAR.radius_base; R_p = PAR.radius_plat; a = PAR.len_servo_arm; s = PAR.len_rod;
    
    T = pose(1:3)'; 
    phi = pose(4); theta = pose(5); psi = pose(6);
    cph=cos(phi); sph=sin(phi); cth=cos(theta); sth=sin(theta); cps=cos(psi); sps=sin(psi);
    Rot = [cps*cth, cps*sth*sph - sps*cph, cps*sth*cph + sps*sph;
           sps*cth, sps*sth*sph + cps*cph, sps*sth*cph - cps*sph;
           -sth,    cth*sph,               cth*cph];
           
    % 1. 批量计算世界坐标系下的点
    beta_p = deg2rad(PAR.beta_plat); % 1x6
    beta_b = deg2rad(PAR.beta_base); % 1x6
    
    P_local = [R_p * cos(beta_p); R_p * sin(beta_p); zeros(1,6)]; % 3x6
    q_all = T + Rot * P_local; % 3x6 (动平台锚点世界坐标)
    B_all = [R_b * cos(beta_b); R_b * sin(beta_b); zeros(1,6)]; % 3x6 (基座舵机轴心)
    
    l_vec = q_all - B_all; % 3x6 向量
    
    % 2. 批量定义局部坐标系向量
    vec_radial = [cos(beta_b); sin(beta_b); zeros(1,6)]; % 3x6
    vec_tangent = [-sin(beta_b); cos(beta_b); zeros(1,6)]; % 3x6
    
    % vec_x 规则: 奇数 CCW (tangent), 偶数 CW (-tangent)
    % mask: 奇数为1, 偶数为-1
    mask = ones(1,6); mask(2:2:6) = -1;
    vec_x = vec_tangent .* mask; % 3x6
    vec_y = repmat([0;0;1], 1, 6); % 3x6 (Z轴)
    
    % 3. 批量投影求解
    % dot(A, B) 对于矩阵列向量点积: sum(A.*B, 1)
    z_off = sum(l_vec .* vec_radial, 1);
    x_t = sum(l_vec .* vec_x, 1);
    y_t = sum(l_vec .* vec_y, 1);
    
    s_eff_sq = s^2 - z_off.^2;
    
    isValid = all(s_eff_sq >= 0);
    if ~isValid, servo_angles = zeros(6,1); return; end
    
    L = -2 * a * x_t;
    M = -2 * a * y_t;
    N = s_eff_sq - a^2 - x_t.^2 - y_t.^2;
    
    rho = sqrt(L.^2 + M.^2);
    
    isValid = all(abs(N) <= rho);
    if ~isValid, servo_angles = zeros(6,1); return; end
    
    phi_bias = atan2(M, L);
    alpha = acos(N ./ rho) - phi_bias;
    
    servo_angles = rad2deg(alpha'); % 6x1 输出
end