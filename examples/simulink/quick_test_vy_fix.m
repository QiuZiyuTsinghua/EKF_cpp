%% Quick Test for Lateral Velocity Estimation Fix
% 快速测试横向速度估计修复效果

clear; clc;

%% 简单测试场景
dt = 0.01;
t = 0:dt:10;
N = length(t);

% 简单的测试信号
vx_true = 15;
vy_true = 0.3 * sin(0.5*t);  % 简单正弦，最大±0.3m/s
gamma_true = 0.05 * cos(0.3*t);

% 简单的测量信号
measurements = zeros(7, N);
control_inputs = zeros(5, N);

for i = 1:N
    % 基本测量
    ax = vy_true(i) * gamma_true(i);
    ay = -vx_true * gamma_true(i);
    
    measurements(:, i) = [
        ax + 0.05*randn;
        ay + 0.03*randn;
        gamma_true(i) + 0.001*randn;
        vx_true + 0.02*randn;
        vx_true + 0.02*randn;
        vx_true + 0.02*randn;
        vx_true + 0.02*randn;
    ];
    
    % 基本控制输入
    control_inputs(:, i) = [0.01; 500; 500; 400; 400];
end

%% 运行EKF
fprintf('运行快速测试...\n');

clear ekf_simulink_function;  % 重置EKF状态

vy_est = zeros(1, N);
for i = 1:N
    [states, ~] = ekf_simulink_function(measurements(:, i), control_inputs(:, i));
    vy_est(i) = states(2);
end

%% 显示结果
vy_error = vy_est - vy_true;
vy_rmse = sqrt(mean(vy_error.^2));
vy_max = max(abs(vy_est));

fprintf('\n=== 快速测试结果 ===\n');
fprintf('横向速度真值范围: [%.3f, %.3f] m/s\n', min(vy_true), max(vy_true));
fprintf('横向速度估计范围: [%.3f, %.3f] m/s\n', min(vy_est), max(vy_est));
fprintf('横向速度 RMSE: %.4f m/s\n', vy_rmse);
fprintf('横向速度最大估计值: %.3f m/s\n', vy_max);

if vy_max < 1.0
    fprintf('✓ 估计值在合理范围内\n');
else
    fprintf('✗ 估计值仍然过大\n');
end

if vy_rmse < 0.1
    fprintf('✓ 估计精度良好\n');
else
    fprintf('⚠ 估计精度需要进一步改进\n');
end

% 简单绘图
figure('Position', [300, 300, 800, 400]);
subplot(1, 2, 1);
plot(t, vy_true, 'b-', 'LineWidth', 2); hold on;
plot(t, vy_est, 'r--', 'LineWidth', 1.5);
xlabel('时间 (s)'); ylabel('横向速度 (m/s)');
title('横向速度对比');
legend('真值', '估计值');
grid on;

subplot(1, 2, 2);
plot(t, vy_error, 'k-', 'LineWidth', 1);
xlabel('时间 (s)'); ylabel('误差 (m/s)');
title(['误差 (RMSE=' num2str(vy_rmse, '%.4f') ')']);
grid on;

fprintf('\n快速测试完成！\n');
