clear all; close all; clc;
import casadi.*

%% 1. THÔNG SỐ HỆ THỐNG & ROBOT (SKID-STEER)
L_robot = 0.5; W_robot = 0.4;
r_wheel = 0.05;      % Bán kính bánh xe (m)
B_width = W_robot;   % Khoảng cách giữa 2 cụm bánh xe (m)
N = 15;              % Prediction Horizon (Số bước dự báo)
dt = 0.2; 
T_sim = 500;

% Các ma trận phục vụ mô hình hóa đa diện lồi của Robot
G_r = [1, 0; -1, 0; 0, 1; 0, -1];
g_r = [L_robot/2; L_robot/2; W_robot/2; W_robot/2];

%% 2. THIẾT LẬP VẬT CẢN (TỌA ĐỘ CẬP NHẬT THEO YÊU CẦU)
% Vật cản 1: Tâm (0.5, 1.7) với kích thước 0.4x0.4
% x_min = 0.3, x_max = 0.7 | y_min = 1.5, y_max = 1.9
obs(1).b = [0.7; -0.3; 1.9; -1.5];

% Vật cản 2: Tâm (2.7, 1.3) với kích thước 0.4x0.4
% x_min = 2.5, x_max = 2.9 | y_min = 1.1, y_max = 1.5
obs(2).b = [2.9; -2.5; 1.5; -1.1];

% Các biên tường tĩnh bao quanh khu vực sa bàn (Chuyển đổi chỉ số sang 3, 4, 5)
obs(3).b = [4.0; 3.0; 4.0; -3.0];
obs(4).b = [1.7; 3.0; 0.8; 0.0];
obs(5).b = [4.0; -2.3; 0.8; 0.0];
M = length(obs);

% Vị trí tư thế mục tiêu ô đỗ xe cuối cùng (Target Pose)
WP_Park = [2.0; 0.4; pi/2];

%% === THUẬT TOÁN NHẬN DẠNG: KIỂM TRA CHIỀU RỘNG & VẬT CẢN ===
is_spot_valid = true;

% 1. Kiểm tra chiều rộng thực tế của bãi đỗ (Giữa tường tĩnh số 4 và số 5)
wall_left_x = obs(4).b(1);
wall_right_x = -obs(5).b(2);
actual_spot_width = wall_right_x - wall_left_x;

% Robot rộng 0.4m, cần tối thiểu không gian 0.55m để lùi chuồng an toàn
min_required_width = W_robot + 0.15;
if actual_spot_width < min_required_width
    is_spot_valid = false;
    fprintf('CẢNH BÁO: Bãi quá hẹp (%.2fm). Cần tối thiểu %.2fm.\n', actual_spot_width, min_required_width);
end

% 2. Kiểm tra vùng xoay trở cơ động (Maneuver Zone: X[1.7-2.3], Y[0-1.5])
maneuver_zone = [wall_left_x, wall_right_x, 0.0, 1.5];
for m = 1:2
    ob_xmin = -obs(m).b(2); ob_xmax = obs(m).b(1);
    ob_ymin = -obs(m).b(4); ob_ymax = obs(m).b(3);
    if ~(ob_xmax < maneuver_zone(1) || ob_xmin > maneuver_zone(2) || ob_ymax < maneuver_zone(3) || ob_ymin > maneuver_zone(4))
        is_spot_valid = false;
        fprintf('CẢNH BÁO: Vật cản %d chặn lối vào bãi đỗ.\n', m);
    end
end

%% 3. THIẾT LẬP CẤU HÌNH BÀI TOÁN NMPC (CasADi)
opti = casadi.Opti();
X = opti.variable(3, N+1); 
U = opti.variable(2, N);

% Khai báo các biến đối ngẫu nhân tử Lagrange phục vụ ràng buộc OBCA né vật cản & biên tường
lam = cell(M, 1); mu = cell(M, 1);
for m = 1:M
    lam{m} = opti.variable(4, N+1); 
    mu{m} = opti.variable(4, N+1);
end

% Định nghĩa các tham số Parameter biến thiên theo thời gian thực
x_init = opti.parameter(3, 1); 
x_target = opti.parameter(3, 1);
v_min = opti.parameter(1, 1); 
safe_margin = opti.parameter(1, 1);

% Thiết lập các ma trận trọng số chi phí phạt (Cost Matrices)
Q_run = diag([100, 80, 20]); 
Q_term = diag([1e4, 1e4, 2e4]);
R = diag([0.5, 1.0]); 
R_smooth = diag([800, 1200]);

% Xây dựng hàm mục tiêu chi phí bình phương tối thiểu toàn cục (Objective Function)
obj = (X(:, N+1)-x_target)'*Q_term*(X(:, N+1)-x_target);
for k = 1:N
    obj = obj + (X(:,k)-x_target)'*Q_run*(X(:,k)-x_target) + U(:,k)'*R*U(:,k);
    if k > 1
        obj = obj + (U(:,k)-U(:,k-1))'*R_smooth*(U(:,k)-U(:,k-1)); 
    end
end
opti.minimize(obj);

% Áp đặt hệ thống các ràng buộc hình học và động học phương tiện
for k = 1:N
    % Ràng buộc mô hình động học vi phân xe lái trượt
    opti.subject_to(X(:, k+1) == X(:, k) + [U(1,k)*cos(X(3,k)); U(1,k)*sin(X(3,k)); U(2,k)]*dt);
    opti.subject_to(v_min <= U(1,k) <= 0.5);
    opti.subject_to(-1.0 <= U(2,k) <= 1.0);
    
    R_th = [cos(X(3,k)) -sin(X(3,k)); sin(X(3,k)) cos(X(3,k))];
    for m = 1:M
        Am = [1, 0; -1, 0; 0, 1; 0, -1]; 
        bm = obs(m).b;
        
        % Ràng buộc phi tuyến khoảng cách an toàn OBCA (Khoảng cách không âm)
        opti.subject_to(lam{m}(:,k) >= 0); 
        opti.subject_to(mu{m}(:,k) >= 0);
        opti.subject_to(-g_r'*lam{m}(:,k) + (Am*X(1:2,k)-bm)'*mu{m}(:,k) >= safe_margin);
        opti.subject_to(G_r'*lam{m}(:,k) + R_th'*Am'*mu{m}(:,k) == 0);
        opti.subject_to(sumsqr(Am'*mu{m}(:,k)) <= 1);
    end
end
opti.subject_to(X(:, 1) == x_init);

% Khởi tạo cấu hình bộ giải toán tối ưu phi tuyến IPOPT
opti.solver('ipopt', struct('ipopt', struct('max_iter', 200, 'print_level', 0), 'print_time', 0));

%% 4. KHỞI TẠO BIẾN LƯU TRỮ VÀ GIAO DIỆN ĐỒ HỌA MÔ PHỎNG
x_curr = [-3.0; 1.5; 0.0];
current_phase = 1; 
x_history = x_curr;
v_hist = []; w_hist = []; wL_hist = []; wR_hist = []; time_hist = [];
solver_time_hist = []; settling_time = 0;

% Khởi tạo lưới không gian phân bố các điểm Waypoint ứng viên (Candidates)
[gx, gy] = meshgrid(-3.0:0.15:4.0, 0.9:0.05:2.9);
candidates = [gx(:), gy(:)];

h_fig1 = figure('Color','w', 'Name', 'NMPC OBCA Dual-Phase Simulation', 'Position', [100, 100, 900, 600]);
hold on; grid on; axis equal; xlim([-3.0 4.2]); ylim([0.0 4.0]);
xlabel('X (m)'); ylabel('Y (m)');

% Trực quan hóa các khối tường biên tĩnh trên sa bàn
patch([-3 1.7 1.7 -3], [0 0 0.8 0.8], [0.5 0.5 0.5], 'FaceAlpha', 0.2, 'EdgeColor', 'none');
patch([wall_right_x 4 4 wall_right_x], [0 0 0.8 0.8], [0.5 0.5 0.5], 'FaceAlpha', 0.2, 'EdgeColor', 'none');
patch([-3 4 4 -3], [3 3 4 4], [0.5 0.5 0.5], 'FaceAlpha', 0.2, 'EdgeColor', 'none');

% Trực quan hóa 2 khối vật cản màu đen hình học thực tế (Theo tọa độ mới)
for m = 1:2
    ob_xmin = -obs(m).b(2); ob_xmax = obs(m).b(1);
    ob_ymin = -obs(m).b(4); ob_ymax = obs(m).b(3);
    patch([ob_xmin ob_xmax ob_xmax ob_xmin], [ob_ymin ob_ymin ob_ymax ob_ymax], 'k', 'FaceAlpha', 0.9);
end
h_obs_black = patch(NaN, NaN, 'k', 'DisplayName', 'Obstacles');

plot(WP_Park(1), WP_Park(2), 'ro', 'MarkerSize', 10, 'LineWidth', 1.5);
h_path = plot(NaN, NaN, 'b', 'LineWidth', 1.5, 'DisplayName', 'Actual Path');

% Cấu hình hiển thị đường bao quét biên an toàn phương tiện (Envelope)
h_env_l = plot(NaN, NaN, 'g--', 'LineWidth', 0.8, 'HandleVisibility', 'off');
h_env_r = plot(NaN, NaN, 'g--', 'LineWidth', 0.8, 'DisplayName', 'Trajectory Envelope');
h_pred = plot(NaN, NaN, 'r--', 'LineWidth', 1.2, 'DisplayName', 'NMPC Prediction Horizon');
h_robot = patch(NaN, NaN, 'g', 'FaceAlpha', 0.8, 'DisplayName', 'Robot Body');
h_wp = plot(0, 0, 'rx', 'MarkerSize', 12, 'LineWidth', 2, 'DisplayName', 'Active Waypoint');
legend([h_robot, h_path, h_env_r, h_pred, h_wp, h_obs_black], 'Location', 'northeastoutside');

% Định nghĩa ngưỡng bán kính vòng chuyển pha chiến lược dựa trên khoảng cách Euclid
d_sw = 1.3; 

%% 5. VÒNG LẶP MÔ PHỎNG THỜI GIAN THỰC (CORE LOOP)
for t = 1:T_sim
    if current_phase == 1
        % GIAI ĐOẠN 1: TỐI ƯU HÓA HÌNH HỌC TÌM ĐÍCH TRUNG GIAN S_K_INT
        best_cost = inf; gold_x = 2.5; gold_y = 1.5;
        for i = 1:size(candidates, 1)
            Pi = candidates(i, :)';
            % Tính toán khoảng cách an toàn cục bộ đến tâm 2 vật cản mới cập nhật
            d_obs1 = norm(Pi - [0.5; 1.7]); 
            d_obs2 = norm(Pi - [2.7; 1.3]);
            
            if d_obs1 < 0.55 || d_obs2 < 0.55 || Pi(2) < 0.95 || Pi(2) > 2.85, continue; end
            if Pi(1) > x_curr(1) + 0.1 && norm(Pi - x_curr(1:2)) < 2.0
                cost = 60.0 * (Pi(1) - gold_x)^2 + 20.0 * (Pi(2) - gold_y)^2;
                if cost < best_cost
                    best_cost = cost; 
                    chosen_wp = [Pi; 0]; 
                end
            end
        end
        opti.set_value(x_target, chosen_wp);
        opti.set_value(safe_margin, 0.08); 
        opti.set_value(v_min, -0.1); % Cấm lùi ngặt nghèo ở pha tiếp cận
        
        % Đánh giá khoảng cách Euclid thực tế kết hợp điều kiện vị trí X vượt qua chuồng
        d_k = norm(x_curr(1:2) - WP_Park(1:2)); 
        if  x_curr(1) >= 2.1
            if is_spot_valid
                current_phase = 2; 
            else
                current_phase = 3; 
            end
        end
        
    elseif current_phase == 2
        % GIAI ĐOẠN 2: ĐỖ XE CHÍNH XÁC VÀO TÂM CHUỒNG Z_REF
        opti.set_value(x_target, WP_Park);
        opti.set_value(safe_margin, 0.05); % Thu hẹp biên an toàn trong không gian hẹp
        opti.set_value(v_min, -0.3);       % Nới lỏng ràng buộc cho phép xe chạy lùi chuồng
        
    elseif current_phase == 3
        % GIAI ĐOẠN BYPASS: CHẠY QUA LUÔN KHI PHÁT HIỆN BÃI ĐỖ KHÔNG AN TOÀN
        opti.set_value(x_target, [4.0; 1.5; 0]);
        opti.set_value(safe_margin, 0.1); 
        opti.set_value(v_min, 0.1);
    end
    
    opti.set_value(x_init, x_curr);
    tic;
    try
        sol = opti.solve();
        u_apply = sol.value(U(:, 1)); 
        x_pred_data = sol.value(X);
        
        % =========================================================================
        % CƠ CHẾ SHIFT-BASED WARM-START THEO ĐÚNG HỆ PHƯƠNG TRÌNH TOÁN LUẬN VĂN
        % =========================================================================
        x_old = sol.value(X);
        u_old = sol.value(U);
        
        x_init_guess = [x_old(:, 2:end), x_old(:, end)]; 
        opti.set_initial(X, x_init_guess);
        
        u_init_guess = [u_old(:, 2:end), u_old(:, end)];
        opti.set_initial(U, u_init_guess);
        
        for m = 1:M
            lam_old = sol.value(lam{m});
            mu_old = sol.value(mu{m});
            opti.set_initial(lam{m}, [lam_old(:, 2:end), lam_old(:, end)]);
            opti.set_initial(mu{m}, [mu_old(:, 2:end), mu_old(:, end)]);
        end
        % =========================================================================
        
    catch
        u_apply = opti.debug.value(U(:, 1)); 
        x_pred_data = opti.debug.value(X);
    end
    solver_time_hist(end+1) = toc;
    
    % Tính toán quy đổi vận tốc dài và góc
    v = u_apply(1); w = u_apply(2);
    time_hist(end+1) = (t-1)*dt; v_hist(end+1) = v; w_hist(end+1) = w;
    
    % Mô hình động học ngược tính toán vận tốc riêng biệt cho bánh xe bên Trái/Phải
    wL_hist(end+1) = (v - (w * B_width / 2)) / r_wheel;
    wR_hist(end+1) = (v + (w * B_width / 2)) / r_wheel;
    
    % Cập nhật trạng thái chu kỳ kế tiếp tích phân Euler
    x_curr = x_curr + [v*cos(x_curr(3)); v*sin(x_curr(3)); w] * dt;
    x_history = [x_history, x_curr];
    dist_err = norm(x_curr(1:2) - WP_Park(1:2));
    
    if current_phase == 2 && dist_err < 0.03 && settling_time == 0
        settling_time = t*dt; 
    end
    
    % Cập nhật giao diện hình ảnh trực quan hóa thời gian thực
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
    
    % Điều kiện dừng vòng lặp lớn khép kín hành trình
    if (current_phase == 2 && dist_err < 0.03) || (current_phase == 3 && x_curr(1) > 3.9)
        break; 
    end
    
    % Cơ chế phát hiện hệ thống bị kẹt
    if t > 50 && mean(abs(v_hist(end-20:end))) < 0.005 && current_phase == 1
        fprintf('HỆ THỐNG BỊ KẸT: Bộ giải không tìm thấy lối đi khả thi!\n');
        time_hist = time_hist(1:length(v_hist));
        solver_time_hist = solver_time_hist(1:length(v_hist));
        break;
    end
end

%% 6. VẼ ĐỒ THỊ VẬN TỐC TỔNG THỂ ROBOT
figure('Color','w', 'Name', 'Robot Velocities');
subplot(2,1,1); plot(time_hist, v_hist, 'b', 'LineWidth', 2); grid on; ylabel('v (m/s)'); title('Linear Velocity');
subplot(2,1,2); plot(time_hist, w_hist, 'r', 'LineWidth', 2); grid on; ylabel('w (rad/s)'); xlabel('Time (s)'); title('Angular Velocity');

%% 7. VẼ ĐỒ THỊ VẬN TỐC QUAY CỦA BÁNH XE SKID-STEER
figure('Color','w', 'Name', 'Wheel Velocities');
plot(time_hist, wL_hist, 'm', 'LineWidth', 1.5, 'DisplayName', '\omega_L');
hold on; plot(time_hist, wR_hist, 'k--', 'LineWidth', 1.5, 'DisplayName', '\omega_R');
grid on; xlabel('Time (s)'); ylabel('rad/s'); title('Skid-Steer Wheel Velocities'); legend show;

%% 8. VẼ BIỂU ĐỒ CHỈ TIÊU ĐÁNH GIÁ CHẤT LƯỢNG ĐIỀU KHIỂN
figure('Color','w', 'Name', 'Analysis Metrics', 'Position', [100, 100, 1000, 350]);
subplot(1,3,1); plot(time_hist, sqrt(sum((x_history(1:2, 1:length(time_hist)) - WP_Park(1:2)).^2, 1)), 'LineWidth', 2);
grid on; title('Pos Error (m)');
subplot(1,3,2); plot(time_hist, solver_time_hist * 1000, 'Color', [0.85 0.32 0.1], 'LineWidth', 1.5);
hold on; yline(dt*1000, 'r--', 'Ts'); grid on; title('Solver Time (ms)');
subplot(1,3,3); plot(time_hist, abs(rad2deg(wrapToPi(x_history(3, 1:length(time_hist)) - WP_Park(3)))), 'Color', [0.46 0.67 0.18], 'LineWidth', 2);
grid on; title('Heading Error (deg)');

% --- IN KẾT QUẢ ĐÁNH GIÁ ĐẦU RA RA TERMINAL ---
fprintf('\n--- KẾT QUẢ ĐÁNH GIÁ CHẤT LƯỢNG HỆ THỐNG ---\n');
if current_phase == 2
    fprintf('Kết quả hành trình: Đã đỗ xe thành công.\n');
else
    fprintf('Kết quả hành trình: Đã chạy vượt qua luôn do không gian bãi đỗ không an toàn.\n'); 
end
avg_solve_time = mean(solver_time_hist) * 1000;
max_solve_time = max(solver_time_hist) * 1000;
terminal_pos_error = norm(x_history(1:2, end) - WP_Park(1:2));
fprintf('Thời gian giải bài toán trung bình : %.2f ms\n', avg_solve_time);
fprintf('Thời gian giải bài toán cực đại   : %.2f ms\n', max_solve_time);
fprintf('Sai số vị trí điểm cuối hành trình: %.4f (m)\n', terminal_pos_error);