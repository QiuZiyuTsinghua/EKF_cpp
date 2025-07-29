%% Test Improved EKF Vehicle Velocity Estimator
% 测试改进后的EKF横向速度估计性能

clear; clc; close all;

%% 模拟车辆运动数据
dt = 0.01;  % 时间步长
t_end = 10; % 仿真时间
t = 0:dt:t_end;
N = length(t);

% 真实车辆状态 [v_x, v_y, gamma]
vx_true = 15 * ones(1, N);  % 恒定纵向速度15m/s
vy_true = 2 * sin(0.5*t);   % 正弦变化的横向速度，最大2m/s
gamma_true = 0.1 * sin(t);  % 正弦变化的横摆角速度

% 车辆参数
m = 1500.0;
track = 1.6;

% 生成模拟测量数据
measurements = zeros(7, N);
for i = 1:N
    % 真实加速度
    ax_true = vy_true(i) * gamma_true(i);
    ay_true = -vx_true(i) * gamma_true(i) + 1.0*cos(0.5*t(i)); % 添加一些横向动力学
    
    % 真实轮速
    v_fl_true = vx_true(i) - gamma_true(i) * track/2;
    v_fr_true = vx_true(i) + gamma_true(i) * track/2;
    v_rl_true = vx_true(i) - gamma_true(i) * track/2;
    v_rr_true = vx_true(i) + gamma_true(i) * track/2;
    
    % 添加测量噪声
    measurements(:, i) = [
        ax_true + 0.3*randn;      % ax with noise
        ay_true + 0.2*randn;      % ay with noise  
        gamma_true(i) + 0.005*randn; % gamma with noise
        v_fl_true + 0.1*randn;    % v_fl with noise
        v_fr_true + 0.1*randn;    % v_fr with noise
        v_rl_true + 0.1*randn;    % v_rl with noise
        v_rr_true + 0.1*randn;    % v_rr with noise
    ];
end

%% 运行改进的EKF
vx_est = zeros(1, N);
vy_est = zeros(1, N);
gamma_est = zeros(1, N);
cov_trace = zeros(1, N);

fprintf('Running improved EKF...\n');
for i = 1:N
    [state_est, cov_flat] = ekf_simulink_function(measurements(:, i));
    
    vx_est(i) = state_est(1);
    vy_est(i) = state_est(2);
    gamma_est(i) = state_est(3);
    
    % 计算协方差矩阵的迹
    P = reshape(cov_flat, 3, 3);
    cov_trace(i) = trace(P);
    
    if mod(i, 100) == 0
        fprintf('Progress: %.1f%%\n', i/N*100);
    end
end

%% 计算估计误差
vx_error = vx_est - vx_true;
vy_error = vy_est - vy_true;
gamma_error = gamma_est - gamma_true;

% 计算RMSE
vx_rmse = sqrt(mean(vx_error.^2));
vy_rmse = sqrt(mean(vy_error.^2));
gamma_rmse = sqrt(mean(gamma_error.^2));

fprintf('\n=== 估计性能分析 ===\n');
fprintf('纵向速度 RMSE: %.3f m/s\n', vx_rmse);
fprintf('横向速度 RMSE: %.3f m/s\n', vy_rmse);
fprintf('横摆角速度 RMSE: %.3f rad/s\n', gamma_rmse);

% 计算横向速度的最大误差
vy_max_error = max(abs(vy_error));
fprintf('横向速度最大误差: %.3f m/s\n', vy_max_error);

%% 绘制结果
figure('Name', '改进EKF性能分析', 'Position', [100, 100, 1200, 800]);

% 纵向速度
subplot(3, 2, 1);
plot(t, vx_true, 'b-', 'LineWidth', 2); hold on;
plot(t, vx_est, 'r--', 'LineWidth', 1.5);
xlabel('时间 (s)'); ylabel('纵向速度 (m/s)');
title('纵向速度估计');
legend('真实值', '估计值', 'Location', 'best');
grid on;

subplot(3, 2, 2);
plot(t, vx_error, 'k-', 'LineWidth', 1);
xlabel('时间 (s)'); ylabel('误差 (m/s)');
title(['纵向速度误差 (RMSE=' num2str(vx_rmse, '%.3f') 'm/s)']);
grid on;

% 横向速度
subplot(3, 2, 3);
plot(t, vy_true, 'b-', 'LineWidth', 2); hold on;
plot(t, vy_est, 'r--', 'LineWidth', 1.5);
xlabel('时间 (s)'); ylabel('横向速度 (m/s)');
title('横向速度估计');
legend('真实值', '估计值', 'Location', 'best');
grid on;

subplot(3, 2, 4);
plot(t, vy_error, 'k-', 'LineWidth', 1);
xlabel('时间 (s)'); ylabel('误差 (m/s)');
title(['横向速度误差 (RMSE=' num2str(vy_rmse, '%.3f') 'm/s)']);
grid on;

% 横摆角速度
subplot(3, 2, 5);
plot(t, gamma_true, 'b-', 'LineWidth', 2); hold on;
plot(t, gamma_est, 'r--', 'LineWidth', 1.5);
xlabel('时间 (s)'); ylabel('横摆角速度 (rad/s)');
title('横摆角速度估计');
legend('真实值', '估计值', 'Location', 'best');
grid on;

subplot(3, 2, 6);
plot(t, gamma_error, 'k-', 'LineWidth', 1);
xlabel('时间 (s)'); ylabel('误差 (rad/s)');
title(['横摆角速度误差 (RMSE=' num2str(gamma_rmse, '%.3f') 'rad/s)']);
grid on;

%% 分析横向速度估计质量
figure('Name', '横向速度估计详细分析', 'Position', [200, 200, 1000, 600]);

subplot(2, 2, 1);
plot(t, vy_true, 'b-', 'LineWidth', 2); hold on;
plot(t, vy_est, 'r--', 'LineWidth', 1.5);
plot(t, vy_true + 2*sqrt(cov_trace), 'g:', 'LineWidth', 1);
plot(t, vy_true - 2*sqrt(cov_trace), 'g:', 'LineWidth', 1);
xlabel('时间 (s)'); ylabel('横向速度 (m/s)');
title('横向速度估计与置信区间');
legend('真实值', '估计值', '±2σ置信区间', 'Location', 'best');
grid on;

subplot(2, 2, 2);
plot(t, abs(vy_error), 'r-', 'LineWidth', 1.5);
xlabel('时间 (s)'); ylabel('绝对误差 (m/s)');
title('横向速度绝对误差');
grid on;

subplot(2, 2, 3);
histogram(vy_error, 30, 'Normalization', 'probability');
xlabel('横向速度误差 (m/s)'); ylabel('概率');
title('横向速度误差分布');
grid on;

subplot(2, 2, 4);
plot(t, cov_trace, 'g-', 'LineWidth', 1.5);
xlabel('时间 (s)'); ylabel('协方差矩阵迹');
title('状态估计不确定性');
grid on;

%% 输出改进建议
fprintf('\n=== 横向速度估计分析 ===\n');
if vy_rmse < 0.5
    fprintf('✓ 横向速度估计精度良好 (RMSE < 0.5 m/s)\n');
elseif vy_rmse < 1.0
    fprintf('⚠ 横向速度估计精度一般 (0.5 < RMSE < 1.0 m/s)\n');
    fprintf('建议: 调整过程噪声或测量噪声参数\n');
else
    fprintf('✗ 横向速度估计精度较差 (RMSE > 1.0 m/s)\n');
    fprintf('建议: 检查车辆模型参数或轮胎模型\n');
end

if vy_max_error > 3.0
    fprintf('⚠ 存在较大的瞬时误差，可能需要:\n');
    fprintf('  - 增加更多传感器信息\n');
    fprintf('  - 改进非线性模型\n');
    fprintf('  - 调整卡尔曼增益\n');
end

fprintf('\n测试完成！\n');
