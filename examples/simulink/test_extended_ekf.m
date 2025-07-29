%% Test Extended EKF with Tire Force Estimation
% 测试扩展的EKF，包含四轮横向力估计

clear; clc; close all;

%% 模拟车辆运动数据
dt = 0.01;  % 时间步长
t_end = 15; % 仿真时间
t = 0:dt:t_end;
N = length(t);

% 真实车辆状态 [v_x, v_y, gamma, Fy_fl, Fy_fr, Fy_rl, Fy_rr]
vx_true = 20 + 5*sin(0.1*t);           % 变化的纵向速度 15-25 m/s
vy_true = 3 * sin(0.3*t);              % 正弦变化的横向速度，最大3m/s
gamma_true = 0.2 * sin(0.5*t);         % 正弦变化的横摆角速度

% 车辆参数
m = 1500.0;
lf = 1.2; lr = 1.4;
track_f = 1.6; track_r = 1.6;
Cf = 50000.0; Cr = 50000.0;

% 模拟控制输入
delta_true = 0.1 * sin(0.5*t);         % 前轮转角 ±0.1弧度 (≈±6度)
Fx_fl_true = 1000 + 500*sin(0.2*t);    % 前左纵向力
Fx_fr_true = 1000 + 500*cos(0.2*t);    % 前右纵向力  
Fx_rl_true = 800 + 300*sin(0.15*t);    % 后左纵向力
Fx_rr_true = 800 + 300*cos(0.15*t);    % 后右纵向力

% 真实轮胎横向力（基于简化轮胎模型）
Fy_fl_true = zeros(1, N);
Fy_fr_true = zeros(1, N);
Fy_rl_true = zeros(1, N);
Fy_rr_true = zeros(1, N);

for i = 1:N
    % 计算各轮的滑移角（简化计算）
    cos_delta = cos(delta_true(i));
    sin_delta = sin(delta_true(i));
    
    % 各轮在车辆坐标系下的速度
    vx_fl = vx_true(i) + gamma_true(i) * track_f/2;
    vy_fl = vy_true(i) + gamma_true(i) * lf;
    vx_fr = vx_true(i) - gamma_true(i) * track_f/2;
    vy_fr = vy_true(i) + gamma_true(i) * lf;
    vx_rl = vx_true(i) + gamma_true(i) * track_r/2;
    vy_rl = vy_true(i) - gamma_true(i) * lr;
    vx_rr = vx_true(i) - gamma_true(i) * track_r/2;
    vy_rr = vy_true(i) - gamma_true(i) * lr;
    
    % 转换到轮胎坐标系（前轮考虑转向）
    vx_wheel_fl = vx_fl * cos_delta + vy_fl * sin_delta;
    vy_wheel_fl = -vx_fl * sin_delta + vy_fl * cos_delta;
    vx_wheel_fr = vx_fr * cos_delta + vy_fr * sin_delta;
    vy_wheel_fr = -vx_fr * sin_delta + vy_fr * cos_delta;
    
    % 计算滑移角
    if abs(vx_wheel_fl) > 0.1
        alpha_fl = atan2(vy_wheel_fl, vx_wheel_fl);
    else
        alpha_fl = 0;
    end
    
    if abs(vx_wheel_fr) > 0.1
        alpha_fr = atan2(vy_wheel_fr, vx_wheel_fr);
    else
        alpha_fr = 0;
    end
    
    if abs(vx_rl) > 0.1
        alpha_rl = atan2(vy_rl, vx_rl);
    else
        alpha_rl = 0;
    end
    
    if abs(vx_rr) > 0.1
        alpha_rr = atan2(vy_rr, vx_rr);
    else
        alpha_rr = 0;
    end
    
    % 计算真实轮胎横向力
    Fy_fl_true(i) = -Cf * max(-0.3, min(0.3, alpha_fl));
    Fy_fr_true(i) = -Cf * max(-0.3, min(0.3, alpha_fr));
    Fy_rl_true(i) = -Cr * max(-0.3, min(0.3, alpha_rl));
    Fy_rr_true(i) = -Cr * max(-0.3, min(0.3, alpha_rr));
end

%% 生成模拟测量数据
measurements = zeros(7, N);
control_inputs = zeros(5, N);

for i = 1:N
    % 计算真实加速度
    % 转换轮胎力到车辆坐标系
    cos_delta = cos(delta_true(i));
    sin_delta = sin(delta_true(i));
    
    Fx_veh_fl = Fx_fl_true(i) * cos_delta - Fy_fl_true(i) * sin_delta;
    Fy_veh_fl = Fx_fl_true(i) * sin_delta + Fy_fl_true(i) * cos_delta;
    Fx_veh_fr = Fx_fr_true(i) * cos_delta - Fy_fr_true(i) * sin_delta;
    Fy_veh_fr = Fx_fr_true(i) * sin_delta + Fy_fr_true(i) * cos_delta;
    
    Fx_total = Fx_veh_fl + Fx_veh_fr + Fx_rl_true(i) + Fx_rr_true(i);
    Fy_total = Fy_veh_fl + Fy_veh_fr + Fy_rl_true(i) + Fy_rr_true(i);
    
    ax_true = Fx_total / m + vy_true(i) * gamma_true(i);
    ay_true = Fy_total / m - vx_true(i) * gamma_true(i);
    
    % 计算真实轮速
    v_fl_true = vx_true(i) + gamma_true(i) * track_f/2;
    v_fr_true = vx_true(i) - gamma_true(i) * track_f/2;
    v_rl_true = vx_true(i) + gamma_true(i) * track_r/2;
    v_rr_true = vx_true(i) - gamma_true(i) * track_r/2;
    
    % 添加测量噪声
    measurements(:, i) = [
        ax_true + 0.3*randn;              % ax with noise
        ay_true + 0.2*randn;              % ay with noise  
        gamma_true(i) + 0.005*randn;      % gamma with noise
        v_fl_true + 0.1*randn;            % v_fl with noise
        v_fr_true + 0.1*randn;            % v_fr with noise
        v_rl_true + 0.1*randn;            % v_rl with noise
        v_rr_true + 0.1*randn;            % v_rr with noise
    ];
    
    % 控制输入（已知）
    control_inputs(:, i) = [
        delta_true(i);
        Fx_fl_true(i);
        Fx_fr_true(i);
        Fx_rl_true(i);
        Fx_rr_true(i);
    ];
end

%% 运行扩展EKF
fprintf('Running Extended EKF with tire force estimation...\n');

% 预分配输出数组
vx_est = zeros(1, N);
vy_est = zeros(1, N);
gamma_est = zeros(1, N);
Fy_fl_est = zeros(1, N);
Fy_fr_est = zeros(1, N);
Fy_rl_est = zeros(1, N);
Fy_rr_est = zeros(1, N);
cov_trace = zeros(1, N);

for i = 1:N
    [state_est, cov_flat] = ekf_simulink_function(measurements(:, i), control_inputs(:, i));
    
    vx_est(i) = state_est(1);
    vy_est(i) = state_est(2);
    gamma_est(i) = state_est(3);
    Fy_fl_est(i) = state_est(4);
    Fy_fr_est(i) = state_est(5);
    Fy_rl_est(i) = state_est(6);
    Fy_rr_est(i) = state_est(7);
    
    % 计算协方差矩阵的迹
    P = reshape(cov_flat, 7, 7);
    cov_trace(i) = trace(P);
    
    if mod(i, 200) == 0
        fprintf('Progress: %.1f%%\n', i/N*100);
    end
end

%% 计算估计误差
vx_error = vx_est - vx_true;
vy_error = vy_est - vy_true;
gamma_error = gamma_est - gamma_true;
Fy_fl_error = Fy_fl_est - Fy_fl_true;
Fy_fr_error = Fy_fr_est - Fy_fr_true;
Fy_rl_error = Fy_rl_est - Fy_rl_true;
Fy_rr_error = Fy_rr_est - Fy_rr_true;

% 计算RMSE
vx_rmse = sqrt(mean(vx_error.^2));
vy_rmse = sqrt(mean(vy_error.^2));
gamma_rmse = sqrt(mean(gamma_error.^2));
Fy_fl_rmse = sqrt(mean(Fy_fl_error.^2));
Fy_fr_rmse = sqrt(mean(Fy_fr_error.^2));
Fy_rl_rmse = sqrt(mean(Fy_rl_error.^2));
Fy_rr_rmse = sqrt(mean(Fy_rr_error.^2));

fprintf('\n=== 扩展EKF估计性能分析 ===\n');
fprintf('纵向速度 RMSE: %.3f m/s\n', vx_rmse);
fprintf('横向速度 RMSE: %.3f m/s\n', vy_rmse);
fprintf('横摆角速度 RMSE: %.3f rad/s\n', gamma_rmse);
fprintf('前左轮胎力 RMSE: %.1f N\n', Fy_fl_rmse);
fprintf('前右轮胎力 RMSE: %.1f N\n', Fy_fr_rmse);
fprintf('后左轮胎力 RMSE: %.1f N\n', Fy_rl_rmse);
fprintf('后右轮胎力 RMSE: %.1f N\n', Fy_rr_rmse);

%% 绘制结果
figure('Name', '扩展EKF性能分析 - 车辆状态', 'Position', [100, 100, 1400, 900]);

% 纵向速度
subplot(3, 3, 1);
plot(t, vx_true, 'b-', 'LineWidth', 2); hold on;
plot(t, vx_est, 'r--', 'LineWidth', 1.5);
xlabel('时间 (s)'); ylabel('纵向速度 (m/s)');
title('纵向速度估计');
legend('真实值', '估计值', 'Location', 'best');
grid on;

% 横向速度
subplot(3, 3, 2);
plot(t, vy_true, 'b-', 'LineWidth', 2); hold on;
plot(t, vy_est, 'r--', 'LineWidth', 1.5);
xlabel('时间 (s)'); ylabel('横向速度 (m/s)');
title('横向速度估计');
legend('真实值', '估计值', 'Location', 'best');
grid on;

% 横摆角速度
subplot(3, 3, 3);
plot(t, gamma_true, 'b-', 'LineWidth', 2); hold on;
plot(t, gamma_est, 'r--', 'LineWidth', 1.5);
xlabel('时间 (s)'); ylabel('横摆角速度 (rad/s)');
title('横摆角速度估计');
legend('真实值', '估计值', 'Location', 'best');
grid on;

% 纵向速度误差
subplot(3, 3, 4);
plot(t, vx_error, 'k-', 'LineWidth', 1);
xlabel('时间 (s)'); ylabel('误差 (m/s)');
title(['纵向速度误差 (RMSE=' num2str(vx_rmse, '%.3f') 'm/s)']);
grid on;

% 横向速度误差
subplot(3, 3, 5);
plot(t, vy_error, 'k-', 'LineWidth', 1);
xlabel('时间 (s)'); ylabel('误差 (m/s)');
title(['横向速度误差 (RMSE=' num2str(vy_rmse, '%.3f') 'm/s)']);
grid on;

% 横摆角速度误差
subplot(3, 3, 6);
plot(t, gamma_error, 'k-', 'LineWidth', 1);
xlabel('时间 (s)'); ylabel('误差 (rad/s)');
title(['横摆角速度误差 (RMSE=' num2str(gamma_rmse, '%.3f') 'rad/s)']);
grid on;

% 控制输入
subplot(3, 3, 7);
plot(t, delta_true*180/pi, 'g-', 'LineWidth', 1.5);
xlabel('时间 (s)'); ylabel('前轮转角 (度)');
title('前轮转角输入');
grid on;

% 纵向力输入
subplot(3, 3, 8);
plot(t, Fx_fl_true, 'b-', 'LineWidth', 1); hold on;
plot(t, Fx_fr_true, 'r-', 'LineWidth', 1);
plot(t, Fx_rl_true, 'g-', 'LineWidth', 1);
plot(t, Fx_rr_true, 'm-', 'LineWidth', 1);
xlabel('时间 (s)'); ylabel('纵向力 (N)');
title('四轮纵向力输入');
legend('FL', 'FR', 'RL', 'RR', 'Location', 'best');
grid on;

% 协方差矩阵迹
subplot(3, 3, 9);
plot(t, cov_trace, 'c-', 'LineWidth', 1.5);
xlabel('时间 (s)'); ylabel('协方差矩阵迹');
title('状态估计不确定性');
grid on;

%% 轮胎力估计结果
figure('Name', '扩展EKF性能分析 - 轮胎横向力', 'Position', [200, 200, 1400, 900]);

% 前左轮胎力
subplot(2, 4, 1);
plot(t, Fy_fl_true, 'b-', 'LineWidth', 2); hold on;
plot(t, Fy_fl_est, 'r--', 'LineWidth', 1.5);
xlabel('时间 (s)'); ylabel('横向力 (N)');
title('前左轮胎横向力');
legend('真实值', '估计值', 'Location', 'best');
grid on;

% 前右轮胎力
subplot(2, 4, 2);
plot(t, Fy_fr_true, 'b-', 'LineWidth', 2); hold on;
plot(t, Fy_fr_est, 'r--', 'LineWidth', 1.5);
xlabel('时间 (s)'); ylabel('横向力 (N)');
title('前右轮胎横向力');
legend('真实值', '估计值', 'Location', 'best');
grid on;

% 后左轮胎力
subplot(2, 4, 3);
plot(t, Fy_rl_true, 'b-', 'LineWidth', 2); hold on;
plot(t, Fy_rl_est, 'r--', 'LineWidth', 1.5);
xlabel('时间 (s)'); ylabel('横向力 (N)');
title('后左轮胎横向力');
legend('真实值', '估计值', 'Location', 'best');
grid on;

% 后右轮胎力
subplot(2, 4, 4);
plot(t, Fy_rr_true, 'b-', 'LineWidth', 2); hold on;
plot(t, Fy_rr_est, 'r--', 'LineWidth', 1.5);
xlabel('时间 (s)'); ylabel('横向力 (N)');
title('后右轮胎横向力');
legend('真实值', '估计值', 'Location', 'best');
grid on;

% 轮胎力误差
subplot(2, 4, 5);
plot(t, Fy_fl_error, 'k-', 'LineWidth', 1);
xlabel('时间 (s)'); ylabel('误差 (N)');
title(['前左轮胎力误差 (RMSE=' num2str(Fy_fl_rmse, '%.0f') 'N)']);
grid on;

subplot(2, 4, 6);
plot(t, Fy_fr_error, 'k-', 'LineWidth', 1);
xlabel('时间 (s)'); ylabel('误差 (N)');
title(['前右轮胎力误差 (RMSE=' num2str(Fy_fr_rmse, '%.0f') 'N)']);
grid on;

subplot(2, 4, 7);
plot(t, Fy_rl_error, 'k-', 'LineWidth', 1);
xlabel('时间 (s)'); ylabel('误差 (N)');
title(['后左轮胎力误差 (RMSE=' num2str(Fy_rl_rmse, '%.0f') 'N)']);
grid on;

subplot(2, 4, 8);
plot(t, Fy_rr_error, 'k-', 'LineWidth', 1);
xlabel('时间 (s)'); ylabel('误差 (N)');
title(['后右轮胎力误差 (RMSE=' num2str(Fy_rr_rmse, '%.0f') 'N)']);
grid on;

%% 性能总结
fprintf('\n=== 扩展EKF性能总结 ===\n');

% 车辆状态估计评价
if vy_rmse < 0.5
    fprintf('✓ 横向速度估计精度优秀 (RMSE < 0.5 m/s)\n');
elseif vy_rmse < 1.0
    fprintf('⚠ 横向速度估计精度良好 (0.5 < RMSE < 1.0 m/s)\n');
else
    fprintf('✗ 横向速度估计精度需改进 (RMSE > 1.0 m/s)\n');
end

% 轮胎力估计评价
avg_tire_force_rmse = mean([Fy_fl_rmse, Fy_fr_rmse, Fy_rl_rmse, Fy_rr_rmse]);
if avg_tire_force_rmse < 1000
    fprintf('✓ 轮胎力估计精度优秀 (平均RMSE < 1kN)\n');
elseif avg_tire_force_rmse < 2000
    fprintf('⚠ 轮胎力估计精度良好 (1kN < 平均RMSE < 2kN)\n');
else
    fprintf('✗ 轮胎力估计精度需改进 (平均RMSE > 2kN)\n');
end

fprintf('\n扩展EKF测试完成！\n');
fprintf('新特性：\n');
fprintf('- 状态向量扩展到7维，包含四轮横向力估计\n');
fprintf('- 考虑前轮转角和四轮纵向力作为控制输入\n');
fprintf('- 更精确的单轮动力学模型\n');
fprintf('- 改进的轮胎力动态特性建模\n');
