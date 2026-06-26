clear all; close all; clc;
import casadi.*

%% =========================================================
% 1. KHỞI TẠO ROS2 & ĐỒNG BỘ PHÒNG MẠNG VỚI PI (DOMAIN 30)
% =========================================================
% Ép ĐỒNG BỘ mạng của MATLAB vào chung phòng 30 với các máy tính khác và Pi
setenv('ROS_DOMAIN_ID', '30'); 

node = ros2node("/matlab_nmpc_node");

% Bộ phát lệnh vận tốc xuống xe thật qua Wi-Fi
cmdPub = ros2publisher(node, "/cmd_vel", "geometry_msgs/Twist");

% Bộ thu nhận tọa độ định vị tuyệt đối từ thuật toán AMCL trên Pi
poseSub = ros2subscriber(node, "/amcl_pose", "geometry_msgs/PoseWithCovarianceStamped");

cmdMsg = ros2message(cmdPub);

disp('Đang đợi tín hiệu định vị AMCL từ Raspberry Pi...');
pose_msg = receive(poseSub, 15); % Đợi tối đa 15 giây để nhận tin nhắn mồi đầu tiên
disp('Đã kết nối phần cứng thành công. Đang khởi tạo NMPC...');

%% =========================================================
% 2. THÔNG SỐ HỆ THỐNG & ROBOT THỰC TẾ
% =========================================================
L_robot = 0.25;                % Chiều dài xe: 0.25m
W_robot = 0.20;                % Chiều rộng xe: 0.20m
r_wheel = 0.05;                % Bán kính bánh xe (m)
B_width = W_robot;             % Khoảng cách giữa 2 cụm bánh xe (m)
N = 15;                        % Prediction Horizon (Số bước dự báo)
dt = 0.2; T_sim = 500; 

G_r = [1, 0; -1, 0; 0, 1; 0, -1];
g_r = [L_robot/2; L_robot/2; W_robot/2; W_robot/2];

%% =========================================================
% 3. THIẾT LẬP VẬT CẢN & CÁC BIÊN TƯỜNG MỚI (Hệ tọa độ Map)
% =========================================================
% Vật cản 1 duy nhất: (0.63 0) (0.63 -0.15) (0.78 -0.15) (0.78 0)
obs(1).b = [0.78; -0.63; 0.00; 0.15];  

% Cấu hình các biên mút tường bãi đỗ phục vụ thuật toán kiểm tra tính khả thi
obs(2).b = [1.10; 0.00; 0.86; 0.00];  
obs(3).b = [2.09; 0.00; -0.61; 0.00];  
obs(4).b = [1.10; 0.00; 0.86; 0.00];   
obs(5).b = [2.00; -1.48; 0.86; 0.00];  
M = length(obs);

% Tọa độ vị trí đích đỗ xe lý tưởng trong túi đỗ hướng lên (giữ nguyên theo mã của bạn)
WP_Park = [1.29; 1.10; -pi/2]; 

%% === THUẬT TOÁN NHẬN DẠNG: KIỂM TRA CHIỀU RỘNG & VẬT CẢN ===
is_spot_valid = true;
wall_left_x = obs(4).b(1); 
wall_right_x = -obs(5).b(2);
actual_spot_width = wall_right_x - wall_left_x;
min_required_width = W_robot + 0.15; 

if actual_spot_width < min_required_width
    is_spot_valid = false;
    fprintf('CẢNH BÁO: Bãi quá hẹp (%.2fm). Cần tối thiểu %.2fm.\n', actual_spot_width, min_required_width);
end

maneuver_zone = [wall_left_x, wall_right_x, -0.61, 1.33]; 
for m = 1:1 
    ob_xmin = -obs(m).b(2); ob_xmax = obs(m).b(1);
    ob_ymin = -obs(m).b(4); ob_ymax = obs(m).b(3);
    if ~(ob_xmax < maneuver_zone(1) || ob_xmin > maneuver_zone(2) || ...
         ob_ymax < maneuver_zone(3) || ob_ymin > maneuver_zone(4))
        is_spot_valid = false;
        fprintf('CẢNH BÁO: Vật cản %d chặn lối vào.\n', m);
    end
end

%% =========================================================
% 4. THIẾT LẬP CONVENTIONAL NMPC SOLVER
% =========================================================
opti = casadi.Opti();
X = opti.variable(3, N+1); U = opti.variable(2, N);    

x_init = opti.parameter(3, 1); x_target = opti.parameter(3, 1); 
v_min = opti.parameter(1, 1);  

Q_run  = diag([100, 80, 20]); Q_term = diag([1e4, 1e4, 2e4]); 
R = diag([0.5, 1.0]); R_smooth = diag([800, 1200]); 

obj = (X(:, N+1)-x_target)'*Q_term*(X(:, N+1)-x_target);
for k = 1:N
    obj = obj + (X(:,k)-x_target)'*Q_run*(X(:,k)-x_target) + U(:,k)'*R*U(:,k);
    if k > 1, obj = obj + (U(:,k)-U(:,k-1))'*R_smooth*(U(:,k)-U(:,k-1)); end
end
opti.minimize(obj);

for k = 1:N
    opti.subject_to(X(:, k+1) == X(:, k) + [U(1,k)*cos(X(3,k)); U(1,k)*sin(X(3,k)); U(2,k)]*dt);
    opti.subject_to(v_min <= U(1,k) <= 0.4);  
    opti.subject_to(-1.2 <= U(2,k) <= 1.2);
    
    % Ràng buộc khoảng cách hình tròn truyền thống (Vật cản 1)
    R_safe_sq = 0.35^2; 
    opti.subject_to(sumsqr(X(1:2, k+1) - [0.705; -0.075]) >= R_safe_sq);
    
    % Biên tường tĩnh bằng hàm trơn smooth bounds
    opti.subject_to(X(2, k+1) >= -0.61 + 0.11); 
    
    smooth_left = 0.5 * (1 + tanh(20 * (X(1, k+1) - 1.10)));
    smooth_right = 0.5 * (1 + tanh(20 * (X(1, k+1) - 1.48)));
    pulse_zone = smooth_left - smooth_right;
    y_ceiling = 0.86 + (1.33 - 0.86) * pulse_zone; 
    
    opti.subject_to(X(2, k+1) <= y_ceiling - 0.11); 
end
opti.subject_to(X(:, 1) == x_init);
opti.solver('ipopt', struct('ipopt', struct('max_iter', 200, 'print_level', 0), 'print_time', 0));

%% =========================================================
% 5. KHỞI TẠO ĐỒ HỌA THEO DÕI REAL-TIME TRÊN LỚP MẠNG
% =========================================================
current_phase = 1; 
v_hist = []; w_hist = []; wL_hist = []; wR_hist = []; time_hist = [];
solver_time_hist = []; settling_time = 0; 

[gx, gy] = meshgrid(0.0:0.1:2.2, -0.5:0.05:0.8); 
candidates = [gx(:), gy(:)]; 

h_fig1 = figure('Color','w', 'Name', 'NMPC Hardware Monitor', 'Position', [100, 100, 850, 550]);
hold on; grid on; axis equal; 
xlim([-0.2 2.4]); ylim([-1.0 1.7]);
xlabel('X (m)'); ylabel('Y (m)');

% Đổ màu các vùng tường
patch([0 2.09 2.09 0], [-1.0 -1.0 -0.61 -0.61], [0.5 0.5 0.5], 'FaceAlpha', 0.15, 'EdgeColor', 'none'); 
patch([0 1.10 1.10 0], [0.86 0.86 1.70 1.70], [0.5 0.5 0.5], 'FaceAlpha', 0.15, 'EdgeColor', 'none');  
patch([1.48 2.40 2.40 1.48], [0.86 0.86 1.70 1.70], [0.5 0.5 0.5], 'FaceAlpha', 0.15, 'EdgeColor', 'none');          

% Vẽ vật cản dạng hộp đen
patch([0.63 0.78 0.78 0.63], [-0.15 -0.15 0.00 0.00], 'k', 'FaceAlpha', 0.9);
h_obs_black = patch(NaN, NaN, 'k', 'DisplayName', 'Obstacles'); 

parking_x = [0    1.10 1.10 1.48 1.48 2.00];
parking_y = [0.86 0.86 1.33 1.33 0.86 0.86];
plot(parking_x, parking_y, 'k', 'LineWidth', 4);
plot([0 2.09], [-0.61 -0.61], 'k', 'LineWidth', 4);

plot(WP_Park(1), WP_Park(2), 'ro', 'MarkerSize', 10, 'LineWidth', 1.5);
h_path = plot(NaN, NaN, 'b', 'LineWidth', 1.5, 'DisplayName', 'Real Path'); 
h_env_l = plot(NaN, NaN, 'g--', 'LineWidth', 0.8, 'HandleVisibility', 'off'); 
h_env_r = plot(NaN, NaN, 'g--', 'LineWidth', 0.8, 'DisplayName', 'Trajectory Envelope'); 
h_pred = plot(NaN, NaN, 'r--', 'LineWidth', 1.2, 'DisplayName', 'NMPC Prediction'); 
h_robot = patch(NaN, NaN, 'g', 'FaceAlpha', 0.8, 'DisplayName', 'Robot'); 
h_wp = plot(0, 0, 'rx', 'MarkerSize', 12, 'LineWidth', 2, 'DisplayName', 'Current Target'); 
legend([h_robot, h_path, h_env_r, h_pred, h_wp, h_obs_black], 'Location', 'northeastoutside');

% Đọc giá trị ban đầu để khởi tạo mảng lịch sử tọa độ
init_msg = poseSub.LatestMessage;
x_history = [init_msg.pose.pose.position.x; init_msg.pose.pose.position.y; 0];

%% =========================================================
% 6. VÒNG LẶP ĐIỀU KHIỂN PHẦN CỨNG THỜI GIAN THỰC
% =========================================================
for t = 1:T_sim
    %% --- ĐỌC TOÀN BỘ TỌA ĐỘ ĐÃ SỬA SAI TỪ AMCL (RÀ QUÉT QUA WI-FI) ---
    pose_msg = poseSub.LatestMessage;
    if isempty(pose_msg)
        pause(0.05); % Chờ gói tin tiếp theo nếu mạng bị delay cục bộ
        continue;
    end
    
    px = pose_msg.pose.pose.position.x;
    py = pose_msg.pose.pose.position.y;
    q  = pose_msg.pose.pose.orientation;
    
    % Biến đổi hệ Quaternion về góc Yaw góc đơn phẳng phục vụ CasADi Matrix
    eul = quat2eul([q.w q.x q.y q.z]);
    yaw = eul(1);
    
    x_curr = [px; py; yaw];
    x_history = [x_history, x_curr];
    
    %% --- ĐỊNH TUYẾN PHASE ĐIỀU KHIỂN ---
    if current_phase == 1
        best_cost = inf; gold_x = 1.75; gold_y = 0.15; 
        for i = 1:size(candidates, 1)
            Pi = candidates(i, :)';
            d_obs1 = norm(Pi - [0.705; -0.075]);
            if d_obs1 < 0.35 || Pi(2) < -0.45 || Pi(2) > 0.70, continue; end
            if Pi(1) > x_curr(1) + 0.05 && norm(Pi - x_curr(1:2)) < 1.2
                cost = 60.0 * (Pi(1) - gold_x)^2 + 20.0 * (Pi(2) - gold_y)^2; 
                if cost < best_cost, best_cost = cost; chosen_wp = [Pi; 0]; end
            end
        end
        opti.set_value(x_target, chosen_wp);
        opti.set_value(v_min, -0.05); 
        
        if x_curr(1) > 1.52 
            if is_spot_valid, current_phase = 2; else, current_phase = 3; end
        end
    elseif current_phase == 2
        opti.set_value(x_target, WP_Park);
        opti.set_value(v_min, -0.25); 
    elseif current_phase == 3
        opti.set_value(x_target, [2.2; 0.1; 0]);
        opti.set_value(v_min, 0.1); 
    end
    
    %% --- GIẢI HỆ TOÁN TỐI ƯU HÓA NMPC ---
    opti.set_value(x_init, x_curr);
    tic; 
    try
        sol = opti.solve();
        u_apply = sol.value(U(:, 1)); x_pred_data = sol.value(X); 
        opti.set_initial(sol.value_variables()); 
    catch
        u_apply = opti.debug.value(U(:, 1)); x_pred_data = opti.debug.value(X);
    end
    solver_time_hist(end+1) = toc;
    
    v = u_apply(1); w = u_apply(2);
    time_hist(end+1) = (t-1)*dt; v_hist(end+1) = v; w_hist(end+1) = w;
    wL_hist(end+1) = (v - (w * B_width / 2)) / r_wheel;
    wR_hist(end+1) = (v + (w * B_width / 2)) / r_wheel;
    
    %% --- BẮN LỆNH VẬN TỐC THỰC TẾ XUỐNG CHO PI ĐIỀU KHIỂN STM32 ---
    cmdMsg.linear.x  = v;
    cmdMsg.angular.z = w;
    send(cmdPub, cmdMsg);
    
    dist_err = norm(x_curr(1:2) - WP_Park(1:2));
    if current_phase == 2 && dist_err < 0.04 && settling_time == 0, settling_time = t*dt; end
    
    %% --- CẬP NHẬT ĐỒ HỌA MONITOR ---
    if isvalid(h_fig1)
        set(h_path, 'XData', x_history(1,:), 'YData', x_history(2,:));
        env_l_x = x_history(1,:) - (W_robot/2) * sin(x_history(3,:));
        env_l_y = x_history(2,:) + (W_robot/2) * cos(x_history(3,:));
        env_r_x = x_history(1,:) + (W_robot/2) * sin(x_history(3,:));
        env_r_y = x_history(2,:) - (W_robot/2) * cos(x_history(3,:));
        set(h_env_l, 'XData', env_l_x, 'YData', env_l_y);
        set(h_env_r, 'XData', env_r_x, 'YData', env_r_y);
        set(h_pred, 'XData', x_pred_data(1,:), 'YData', x_pred_data(2,:)); 
        set(h_wp, 'XData', opti.debug.value(x_target(1)), 'YData', opti.debug.value(x_target(2)));
        R_mat = [cos(x_curr(3)) -sin(x_curr(3)); sin(x_curr(3)) cos(x_curr(3))];
        p = R_mat * ([-L_robot/2, L_robot/2, L_robot/2, -L_robot/2; ...
                      -W_robot/2, -W_robot/2, W_robot/2, W_robot/2]) + x_curr(1:2);
        set(h_robot, 'XData', p(1,:), 'YData', p(2,:));
        drawnow limitrate;
    end
    
    % Kiểm tra điều kiện dừng thực tế
    if (current_phase == 2 && dist_err < 0.04) || (current_phase == 3 && x_curr(1) > 2.1), break; end
    
    % Khung chống kẹt bảo vệ động cơ xe thật
    if t > 50 && mean(abs(v_hist(end-20:end))) < 0.005 && current_phase == 1
        fprintf('HỆ THỐNG BỊ KẸT: Robot không thể di chuyển tiến lên do vướng biên tròn an toàn!\n');
        break; 
    end
    pause(0.01);
end

%% =========================================================
% 7. PHANH KHẨN CẤP ĐỂ DỪNG ROBOT NGOÀI ĐỜI THỰC
% =========================================================
cmdMsg.linear.x  = 0;
cmdMsg.angular.z = 0;
send(cmdPub, cmdMsg);
disp('Robot đã lùi chuồng hoàn tất và phanh khẩn cấp thành công.');

%% =========================================================
% 8. XUẤT ĐỒ THỊ VÀ BẢNG SỐ LIỆU ĐÁNH GIÁ THỰC TẾ
% =========================================================
figure('Color','w', 'Name', 'Hardware Velocities');
subplot(2,1,1); plot(time_hist, v_hist, 'b', 'LineWidth', 2); grid on; ylabel('v (m/s)'); title('Real Linear Velocity');
subplot(2,1,2); plot(time_hist, w_hist, 'r', 'LineWidth', 2); grid on; ylabel('w (rad/s)'); xlabel('Time (s)'); title('Real Angular Velocity');

figure('Color','w', 'Name', 'Hardware Analysis Metrics', 'Position', [100, 100, 1000, 350]);
subplot(1,3,1); plot(time_hist, sqrt(sum((x_history(1:2, 1:length(time_hist)) - WP_Park(1:2)).^2, 1)), 'LineWidth', 2);
grid on; title('Pos Error (m)');
subplot(1,3,2); plot(time_hist, solver_time_hist * 1000, 'Color', [0.85 0.32 0.1], 'LineWidth', 1.5);
hold on; yline(dt*1000, 'r--', 'Ts'); grid on; title('Solver Time (ms)');
subplot(1,3,3); plot(time_hist, abs(rad2deg(wrapToPi(x_history(3, 1:length(time_hist)) - WP_Park(3)))), 'Color', [0.46 0.67 0.18], 'LineWidth', 2);
grid on; title('Heading Error (deg)');

terminal_pos_error = norm(x_history(1:2, end) - WP_Park(1:2));
terminal_heading_error = abs(rad2deg(wrapToPi(x_history(3, end) - WP_Park(3))));
min_clearance = inf;
for i = 1:size(x_history, 2)
    d1 = norm(x_history(1:2, i) - [0.705; -0.075]) - 0.11; 
    min_clearance = min([min_clearance, d1]);
end
fprintf('\n======= SỐ LIỆU ĐỂ ĐIỀN BẢNG THỰC TẾ (DOMAIN 30) =======\n');
fprintf('Terminal position error : %.4f (m)\n', terminal_pos_error);
fprintf('Heading error           : %.2f (deg)\n', terminal_heading_error);
fprintf('Min clearance           : %.4f (m)\n', min_clearance);
fprintf('Average solve time      : %.2f (ms)\n', mean(solver_time_hist)*1000);
fprintf('Max solve time          : %.2f (ms)\n', max(solver_time_hist)*1000);
fprintf('=====================================================\n');