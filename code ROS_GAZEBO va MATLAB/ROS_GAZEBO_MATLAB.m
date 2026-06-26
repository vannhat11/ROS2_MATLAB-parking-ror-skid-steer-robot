clear; close all; clc;
import casadi.*

%% 1. ROS2 INIT
node = ros2node("/matlab_nmpc_node");
cmdPub = ros2publisher(node, "/cmd_vel", "geometry_msgs/Twist");
odomSub = ros2subscriber(node, "/odom", "nav_msgs/Odometry");
cmdMsg = ros2message(cmdPub);

disp('Waiting for odom...');
odom_msg = receive(odomSub, 10);
disp('Odom received. Initializing NMPC...');

%% 2. ROBOT & SIM PARAMETERS
L_robot = 0.38; W_robot = 0.27;
r_wheel = 0.05; B_width = W_robot; % Thông số bánh xe
N = 15; dt = 0.2; T_sim = 500;

G_r = [1 0; -1 0; 0 1; 0 -1];
g_r = [L_robot/2; L_robot/2; W_robot/2; W_robot/2];

%% 3. OBSTACLES & TARGET
% Giữ nguyên các thông số Obstacle của bạn
obs(1).b = [2.55; -2.06; 2.00; -1.50];
obs(2).b = [5.50; -5.00; 1.24; -0.75];
obs(3).b = [4.12; -0.12; 0.47; -0.01];
obs(4).b = [5.76; -4.58; 0.47; -0.01];
obs(5).b = [5.80; -0.12; 2.46; -2.36];
M = length(obs);

WP_Park = [4.35; 0.25; pi/2]; 

%% 4. NMPC SETUP (CasADi)
opti = casadi.Opti();
X = opti.variable(3, N+1); U = opti.variable(2, N);
lam = cell(M, 1); mu = cell(M, 1);
for m = 1:M
    lam{m} = opti.variable(4, N+1); mu{m} = opti.variable(4, N+1);
end

x_init = opti.parameter(3, 1); x_target = opti.parameter(3, 1);
v_min = opti.parameter(1, 1); safe_margin = opti.parameter(1, 1);

% Cost Function
Q_run = diag([100 80 20]); Q_term = diag([1e4 1e4 2e4]);
R = diag([0.5 1.0]); R_smooth = diag([800 1200]);

obj = (X(:,N+1)-x_target)'*Q_term*(X(:,N+1)-x_target);
for k = 1:N
    obj = obj + (X(:,k)-x_target)'*Q_run*(X(:,k)-x_target) + U(:,k)'*R*U(:,k);
    if k > 1
        obj = obj + (U(:,k)-U(:,k-1))'*R_smooth*(U(:,k)-U(:,k-1));
    end
end
opti.minimize(obj);

% Constraints
for k = 1:N
    opti.subject_to(X(:,k+1) == X(:,k) + [U(1,k)*cos(X(3,k)); U(1,k)*sin(X(3,k)); U(2,k)]*dt);
    opti.subject_to(v_min <= U(1,k) <= 0.5);
    opti.subject_to(-1.0 <= U(2,k) <= 1.0);
    R_th = [cos(X(3,k)) -sin(X(3,k)); sin(X(3,k)) cos(X(3,k))];
    for m = 1:M
        Am = [1 0; -1 0; 0 1; 0 -1]; bm = obs(m).b;
        opti.subject_to(lam{m}(:,k) >= 0); opti.subject_to(mu{m}(:,k) >= 0);
        opti.subject_to(-g_r'*mu{m}(:,k) + (Am*X(1:2,k)-bm)'*lam{m}(:,k) >= safe_margin);
        opti.subject_to(G_r'*mu{m}(:,k) + R_th'*Am'*lam{m}(:,k) == 0);
        opti.subject_to(sumsqr(Am'*lam{m}(:,k)) <= 1);
    end
end
opti.subject_to(X(:,1) == x_init);
opti.solver('ipopt', struct('ipopt', struct('max_iter', 200, 'print_level', 0), 'print_time', 0));

%% 5. GRAPHICS SETUP
h_fig = figure('Color','w', 'Name', 'NMPC ROS2 Reverse Parking'); hold on; grid on; axis equal;
xlim([0 6.2]); ylim([0 2.8]);
% Vẽ vật cản (giữ nguyên các hàm patch của bạn)
patch([2.06 2.55 2.55 2.06], [1.50 1.50 2.00 2.00], 'k');
patch([5.00 5.50 5.50 5.00], [0.75 0.75 1.24 1.24], 'k');
patch([0.12 4.12 4.12 0.12], [0.01 0.01 0.47 0.47], [0.5 0.5 0.5]);
patch([4.58 5.76 5.76 4.58], [0.01 0.01 0.47 0.47], [0.5 0.5 0.5]);
patch([0.12 5.80 5.80 0.12], [2.36 2.36 2.46 2.46], [0.5 0.5 0.5]);
plot(WP_Park(1), WP_Park(2), 'ro', 'MarkerSize', 10, 'LineWidth', 2);

h_path = plot(NaN,NaN,'b','LineWidth',2);
h_env_l = plot(NaN,NaN,'g--','LineWidth',0.5); % Envelope Left
h_env_r = plot(NaN,NaN,'g--','LineWidth',0.5); % Envelope Right
h_pred = plot(NaN,NaN,'r--','LineWidth',1.5);
h_robot = patch(NaN,NaN,'g','FaceAlpha',0.8);
h_wp = plot(0, 0, 'rx', 'MarkerSize', 12, 'LineWidth', 2);

%% 6. CONTROL LOOP
[gx,gy] = meshgrid(0:0.15:6.0, 0:0.05:2.5);
candidates = [gx(:), gy(:)];
current_phase = 1;
x_hist = []; wL_hist = []; wR_hist = [];

for t = 1:T_sim
    % 6.1 Get Odometry
    odom_msg = odomSub.LatestMessage;
    if isempty(odom_msg), pause(0.01); continue; end
    
    px = odom_msg.pose.pose.position.x;
    py = odom_msg.pose.pose.position.y;
    q = odom_msg.pose.pose.orientation;
    eul = quat2eul([q.w q.x q.y q.z]);
    yaw = eul(1);
    x_curr = [px; py; yaw];
    x_hist(:, end+1) = x_curr;

    % 6.2 Decision Logic
    if current_phase == 1
        gold_x = 5.10; gold_y = 1.70; best_cost = inf;
        chosen_wp = [x_curr(1)+0.5; x_curr(2); 0];
        
        for i = 1:size(candidates,1)
            Pi = candidates(i,:)';
            d_obs1 = norm(Pi - [2.305; 1.75]);
            d_obs2 = norm(Pi - [5.25; 0.995]);
            if d_obs1 < 0.5 || d_obs2 < 0.5 || Pi(2) < 0.15 || Pi(2) > 2.25, continue; end
            
            if Pi(1) > x_curr(1)
                cost = 100*(Pi(1)-gold_x)^2 + 50*(Pi(2)-gold_y)^2;
                if cost < best_cost
                    best_cost = cost; chosen_wp = [Pi; 0];
                end
            end
        end
        opti.set_value(x_target, chosen_wp);
        opti.set_value(safe_margin, 0.12);
        opti.set_value(v_min, 0.05);
        
        % Chuyển giai đoạn khi x > 4.8 như bạn yêu cầu
        if x_curr(1) > 4.8, current_phase = 2; end
    else
        opti.set_value(x_target, WP_Park);
        opti.set_value(safe_margin, 0.05);
        opti.set_value(v_min, -0.35);
    end

    % 6.3 Solve
    opti.set_value(x_init, x_curr);
    try
        sol = opti.solve();
        u_apply = sol.value(U(:,1));
        x_pred_data = sol.value(X);
        opti.set_initial(sol.value_variables());
    catch
        u_apply = opti.debug.value(U(:,1));
        x_pred_data = opti.debug.value(X);
    end

    % 6.4 Calculate Wheel Velocities (Skid-Steer)
    v = u_apply(1); w = u_apply(2);
    wL_hist(end+1) = (v - (w * B_width / 2)) / r_wheel;
    wR_hist(end+1) = (v + (w * B_width / 2)) / r_wheel;

    % 6.5 Send ROS2 Message
    cmdMsg.linear.x = v;
    cmdMsg.angular.z = w;
    send(cmdPub, cmdMsg);

    % 6.6 Graphics Update
    if isvalid(h_fig)
        set(h_path, 'XData', x_hist(1,:), 'YData', x_hist(2,:));
        set(h_pred, 'XData', x_pred_data(1,:), 'YData', x_pred_data(2,:));
        set(h_wp, 'XData', opti.debug.value(x_target(1)), 'YData', opti.debug.value(x_target(2)));
        
        % Envelope calculation
        env_l_x = x_hist(1,:) - (W_robot/2)*sin(x_hist(3,:));
        env_l_y = x_hist(2,:) + (W_robot/2)*cos(x_hist(3,:));
        env_r_x = x_hist(1,:) + (W_robot/2)*sin(x_hist(3,:));
        env_r_y = x_hist(2,:) - (W_robot/2)*cos(x_hist(3,:));
        set(h_env_l, 'XData', env_l_x, 'YData', env_l_y);
        set(h_env_r, 'XData', env_r_x, 'YData', env_r_y);

        R_mat = [cos(yaw) -sin(yaw); sin(yaw) cos(yaw)];
        body = R_mat * ([-L_robot/2 L_robot/2 L_robot/2 -L_robot/2; -W_robot/2 -W_robot/2 W_robot/2 W_robot/2]) + x_curr(1:2);
        set(h_robot, 'XData', body(1,:), 'YData', body(2,:));
        drawnow limitrate;
    end

    % Stop condition
    if current_phase == 2 && norm(x_curr(1:2)-WP_Park(1:2)) < 0.05 && abs(rad2deg(wrapToPi(x_curr(3)-WP_Park(3)))) < 3
        disp('Parking Success!'); break;
    end
    pause(0.01); % Giảm pause để ROS2 mượt hơn
end

% Stop Robot
cmdMsg.linear.x = 0; cmdMsg.angular.z = 0;
send(cmdPub, cmdMsg);
disp('Robot stopped.');