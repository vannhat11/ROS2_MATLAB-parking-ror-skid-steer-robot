clear all; close all; clc;
import casadi.*

%% =========================================================
% 1. KHỞI TẠO ROS 2 & ĐỒNG BỘ PHÒNG MẠNG VỚI PI (DOMAIN 30)
% =========================================================
% Ép đồng bộ mạng của MATLAB vào chung Domain 30 với con Pi trên xe thật
setenv('ROS_DOMAIN_ID', '30'); 
try
    node = ros2node("/matlab_nmpc_node");
catch ME
    error('LỖI: Không thể khởi tạo Node ROS 2. Hãy kiểm tra cài đặt ROS 2 Toolbox trên MATLAB!');
end

% Bộ phát lệnh vận tốc xuống xe thật qua mạng Wi-Fi
cmdPub = ros2publisher(node, "/cmd_vel", "geometry_msgs/Twist");
cmdMsg = ros2message(cmdPub);

% KHỞI TẠO SUBSCRIBER HỨNG TỌA ĐỘ LỌC TỪ NODE BRIDGE CỦA PI
disp('Đang lắng nghe Topic tọa độ lọc từ Pi (/robot_pose_filtered)...');
poseSub = ros2subscriber(node, "/robot_pose_filtered", "geometry_msgs/Point");

% Chờ tối đa 15 giây để bắt gói tin mồi đầu tiên từ Pi xem mạng thông suốt chưa
tic;
msg_init = [];
while true
    [msg_init, status] = receive(poseSub, 1.0); % Chờ tối đa 1s mỗi vòng quét
    if status
        break;
    end
    if toc > 15
        error('LỖI: Quá thời gian chờ dữ liệu! Hãy kiểm tra xem đã bật node bridge trên Pi chưa.');
    end
    disp('Đang đợi tín hiệu định vị truyền từ Raspberry Pi...');
end
disp('Đã bắt tay kết nối phần cứng thành công. Đang khởi tạo bộ giải NMPC...');

%% =========================================================
% 2. THÔNG SỐ HỆ THỐNG & ROBOT THỰC TẾ
% =========================================================
L_robot = 0.25;                % Chiều dài xe: 0.25m
W_robot = 0.20;                % Chiều rộng xe: 0.20m
r_wheel = 0.05;                % Bán kính bánh xe (m)
B_width = 0.190;               % Khoảng cách giữa 2 cụm bánh xe (Khớp với wheel_base bên Pi)
N = 15;                        % Prediction Horizon (Số bước dự báo)
dt = 0.2; T_sim = 500; 

%% =========================================================
% 3. THIẾT LẬP VẬT CẢN & CÁC BIÊN TƯỜNG (Ô đỗ rộng 0.35m)
% =========================================================
obs(1).b = [0.75; -0.65; 0.07; 0.08];  
obs(2).b = [1.20; 0.00; 0.91; 0.00];  
obs(3).b = [2.10; 0.00; -0.53; 0.00];  
obs(4).b = [1.20; 0.00; 0.91; 0.00];   
obs(5).b = [2.10; -1.55; 0.91; 0.00];  
M = length(obs);

% Tọa độ vị trí đích đỗ xe lý tưởng trong chuồng
WP_Park = [1.; 1.25; -pi/2]; 

%% === THUẬT TOÁN NHẬN DẠNG: KIỂM TRA CHIỀU RỘNG & VẬT CẢN ===
is_spot_valid = true;
wall_left_x = obs(4).b(1); 
wall_right_x = -obs(5).b(2);
actual_spot_width = wall_right_x - wall_left_x;
min_required_width = 0.25; 
if actual_spot_width < min_required_width
    is_spot_valid = false;
    fprintf('CẢNH BÁO: Bãi quá hẹp (%.2fm). Cần tối thiểu %.2fm.\n', actual_spot_width, min_required_width);
end
maneuver_zone = [wall_left_x, wall_right_x, -0.53, 1.47]; 
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
% 4. THIẾT LẬP TOÁN NMPC (CONVENTIONAL NMPC)
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
    % Động học xe 4 bánh Steer-Skid dạng Unicycle xấp xỉ
    opti.subject_to(X(:, k+1) == X(:, k) + [U(1,k)*cos(X(3,k)); U(1,k)*sin(X(3,k)); U(2,k)]*dt);
    
    % Giới hạn vận tốc của mô hình xe thật
    opti.subject_to(v_min <= U(1,k) <= 0.22);  
    opti.subject_to(-1.0 <= U(2,k) <= 1.0);
    
    % Ràng buộc khoảng cách an toàn tránh vật cản tĩnh
    R_safe_sq = 0.25^2; 
    opti.subject_to(sumsqr(X(1:2, k+1) - [0.70; -0.005]) >= R_safe_sq);
    opti.subject_to(sumsqr(X(1:2, k+1) - [1.71; 0.615]) >= R_safe_sq);
    
    opti.subject_to(X(2, k+1) >= -0.53 + 0.11); 
    
    % Hàm mượt tanh giới hạn hành lang biên chuồng
    smooth_left = 0.5 * (1 + tanh(20 * (X(1, k+1) - 1.20)));
    smooth_right = 0.5 * (1 + tanh(20 * (X(1, k+1) - 1.55)));
    pulse_zone = smooth_left - smooth_right;
    y_ceiling = 0.91 + (1.47 - 0.91) * pulse_zone; 
    
    opti.subject_to(X(2, k+1) <= y_ceiling - 0.11); 
end
opti.subject_to(X(:, 1) == x_init);
opti.solver('ipopt', struct('ipopt', struct('max_iter', 200, 'print_level', 0), 'print_time', 0));

%% =========================================================
% 5. KHỞI TẠO BIẾN LƯU TRỮ & ĐỒ HỌA THEO DÕI THỜI GIAN THỰC
% =========================================================
current_phase = 1; 
v_hist = []; w_hist = []; time_hist = []; solver_time_hist = []; 
[gx, gy] = meshgrid(0.0:0.1:2.2, -0.5:0.05:0.8); 
candidates = [gx(:), gy(:)]; 

h_fig1 = figure('Color','w', 'Name', 'NMPC Hardware Monitor', 'Position', [100, 100, 850, 550]);
hold on; grid on; axis equal; 
xlim([-0.2 2.4]); ylim([-0.9 1.8]);
xlabel('X (m)'); ylabel('Y (m)');
% Vẽ bãi đỗ xe và vật cản hình học
patch([0 2.10 2.10 0], [-1.0 -1.0 -0.53 -0.53], [0.5 0.5 0.5], 'FaceAlpha', 0.15, 'EdgeColor', 'none'); 
patch([0 1.20 1.20 0], [0.91 0.91 1.70 1.70], [0.5 0.5 0.5], 'FaceAlpha', 0.15, 'EdgeColor', 'none');  
patch([1.55 2.40 2.40 1.55], [0.91 0.91 1.70 1.70], [0.5 0.5 0.5], 'FaceAlpha', 0.15, 'EdgeColor', 'none');          
patch([0.65 0.75 0.75 0.65], [-0.08 -0.08 0.07 0.07], 'k', 'FaceAlpha', 0.9);
patch([1.67 1.75 1.75 1.67], [0.53 0.53 0.70 0.70], 'k', 'FaceAlpha', 0.9);
h_obs_black = patch(NaN, NaN, 'k', 'DisplayName', 'Obstacles'); 
parking_x = [0    1.20 1.20 1.55 1.55 2.10];
parking_y = [0.91 0.91 1.47 1.47 0.91 0.91];
plot(parking_x, parking_y, 'k', 'LineWidth', 4);
plot([0 2.10], [-0.53 -0.53], 'k', 'LineWidth', 4);
plot(WP_Park(1), WP_Park(2), 'ro', 'MarkerSize', 10, 'LineWidth', 1.5);
h_path = plot(NaN, NaN, 'b', 'LineWidth', 1.5, 'DisplayName', 'Real Path'); 
h_env_l = plot(NaN, NaN, 'g--', 'LineWidth', 0.8, 'HandleVisibility', 'off'); 
h_env_r = plot(NaN, NaN, 'g--', 'LineWidth', 0.8, 'DisplayName', 'Trajectory Envelope'); 
h_pred = plot(NaN, NaN, 'r--', 'LineWidth', 1.2, 'DisplayName', 'Prediction'); 
h_robot = patch(NaN, NaN, 'g', 'FaceAlpha', 0.8, 'DisplayName', 'Robot'); 
h_wp = plot(0, 0, 'rx', 'MarkerSize', 12, 'LineWidth', 2, 'DisplayName', 'Waypoint'); 
legend([h_robot, h_path, h_env_r, h_pred, h_wp, h_obs_black], 'Location', 'northeastoutside');

% --- ĐÃ FIXED CÚ PHÁP: Chuyển đổi sang chữ viết thường chuẩn ROS 2 ---
x_history = [msg_init.x; msg_init.y; msg_init.z];

%% =========================================================
% 6. VÒNG LẶP ĐIỀU KHIỂN VÀ GIẢI TỐI ƯU THỜI GIAN THỰC
% =========================================================
for t = 1:T_sim
    % ĐỌC TỌA ĐỘ MỚI NHẤT TỪ TOPIC TRUNG CHUYỂN CỦA PI
    msg_curr = poseSub.LatestMessage;
    if isempty(msg_curr)
        pause(0.01);
        continue;
    end
    
    % --- ĐÃ FIXED CÚ PHÁP: Chuyển đổi sang chữ viết thường chuẩn ROS 2 ---
    x_curr = [msg_curr.x; msg_curr.y; msg_curr.z];
    x_history = [x_history, x_curr];
    
    %% --- ĐỊNH TUYẾN PHASE ĐIỀU KHIỂN CHẬM MỊN AN TOÀN ---
    if current_phase == 1
        best_cost = inf; gold_x = 1.75; gold_y = 0.15; 
        for i = 1:size(candidates, 1)
            Pi = candidates(i, :)';
            d_obs1 = norm(Pi - [0.70; -0.005]);
            d_obs2 = norm(Pi - [1.71; 0.615]);
            if d_obs1 < 0.25 || d_obs2 < 0.25 || Pi(2) < -0.42 || Pi(2) > 0.80, continue; end
            
            if Pi(1) > x_curr(1) + 0.05 && norm(Pi - x_curr(1:2)) < 1.2
                cost = 60.0 * (Pi(1) - gold_x)^2 + 20.0 * (Pi(2) - gold_y)^2; 
                if cost < best_cost, best_cost = cost; chosen_wp = [Pi; 0]; end
            end
        end
        opti.set_value(x_target, chosen_wp);
        opti.set_value(v_min, 0.0); 
        
        if x_curr(1) > 1.55 
            if is_spot_valid, current_phase = 2; else, current_phase = 3; end
        end
    elseif current_phase == 2
        opti.set_value(x_target, WP_Park);
        opti.set_value(v_min, -0.10); % Ép đi lùi chậm mượt khi vào chuồng
    elseif current_phase == 3
        opti.set_value(x_target, [2.2; 0.1; 0]);
        opti.set_value(v_min, 0.05); 
    end
    
    %% --- GIẢI HỆ TOÁN TỐI ƯU HÓA NMPC BẰNG IPOPT ---
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
    
    %% --- BẮN LỆNH ĐIỀU KHIỂN VẬN TỐC THỜI GIAN THỰC XUỐNG PI ---
    cmdMsg.linear.x  = v;
    cmdMsg.angular.z = w;
    send(cmdPub, cmdMsg);
    
    dist_err = norm(x_curr(1:2) - WP_Park(1:2));
    
    %% --- CẬP NHẬT ĐỒ HỌA ĐỒ THỊ MONITOR ---
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
    
    % Kiểm tra điều kiện dừng xe
    if (current_phase == 2 && dist_err < 0.04) || (current_phase == 3 && x_curr(1) > 2.1), break; end
    
    pause(0.01);
end

%% =========================================================
% 7. PHANH KHẨN CẤP ĐỂ DỪNG ROBOT AN TOÀN
% =========================================================
cmdMsg.linear.x  = 0;
cmdMsg.angular.z = 0;
send(cmdPub, cmdMsg);
disp('Robot đã hoàn tất quỹ đạo đỗ. Đã phát lệnh phanh khẩn cấp.');

%% =========================================================
% 8. XUẤT ĐỒ THỊ VÀ BẢNG SỐ LIỆU ĐÁNH GIÁ ĐỂ ĐIỀN ĐỒ ÁN
% =========================================================
figure('Color','w', 'Name', 'Robot Velocities');
subplot(2,1,1); plot(time_hist, v_hist, 'b', 'LineWidth', 2); grid on; ylabel('v (m/s)'); title('Linear Velocity');
subplot(2,1,2); plot(time_hist, w_hist, 'r', 'LineWidth', 2); grid on; ylabel('w (rad/s)'); xlabel('Time (s)'); title('Angular Velocity');
figure('Color','w', 'Name', 'Analysis Metrics', 'Position', [100, 100, 1000, 350]);
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
    d1 = norm(x_history(1:2, i) - [0.70; -0.005]) - 0.11; 
    min_clearance = min([min_clearance, d1]);
end

fprintf('\n======= SỐ LIỆU THỰC TẾ ĐỂ ĐIỀN BẢNG ĐỒ ÁN =======\n');
fprintf('Terminal position error : %.4f (m)\n', terminal_pos_error);
fprintf('Heading error           : %.2f (deg)\n', terminal_heading_error);
fprintf('Min clearance           : %.4f (m)\n', min_clearance);
fprintf('Average solve time      : %.2f (ms)\n', mean(solver_time_hist)*1000);
fprintf('Max solve time          : %.2f (ms)\n', max(solver_time_hist)*1000);
if (terminal_pos_error < 0.04) && (min_clearance > 0)
    fprintf('Success                 : Yes (1)\n');
else
    fprintf('Success                 : No (0)\n');
end
fprintf('=====================================================\n');