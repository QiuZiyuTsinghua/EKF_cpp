%% Test Lateral Velocity Estimation Performance
% 专门测试横向速度估计性能的脚本

clear; clc; close all;

%% 模拟真实的车辆运动场景
dt = 0.01;  % 时间步长
t_end = 20; % 仿真时间
t = 0:dt:t_end;
N = length(t);

% 真实车辆状态 - 设计更真实的横向速度变化
vx_true = 15 * ones(1, N);                    % 恒定纵向速度15m/s
vy_true = 0.3 * sin(0.2*t) .* cos(0.1*t);    % 小幅横向速度变化，最大±0.3m/s
gamma_true = 0.05 * sin(0.3*t);              % 小幅横摆角速度，最大±0.05rad/s

% 添加一些典型的驾驶场景
for i = 1:N
    % 车道变更场景 (t=5-7s)
    if t(i) >= 5 && t(i) <= 7
        lane_change_factor = sin(pi * (t(i) - 5) / 2);
        vy_true(i) = vy_true(i) + 0.4 * lane_change_factor;
        gamma_true(i) = gamma_true(i) + 0.08 * cos(pi * (t(i) - 5) / 2);
    end
    
    % 转弯场景 (t=12-16s)
    if t(i) >= 12 && t(i) <= 16
        turn_factor = 0.5;
        vy_true(i) = vy_true(i) + turn_factor * 0.3;
        gamma_true(i) = gamma_true(i) + turn_factor * 0.1;
    end
end

% 确保横向速度在合理范围内
vy_true = max(-0.8, min(0.8, vy_true));
gamma_true = max(-0.15, min(0.15, gamma_true));

%% 车辆参数
m = 1500.0;
lf = 1.2; lr = 1.4;
track = 1.6;
Cf = 50000.0; Cr = 50000.0;

%% 模拟真实的轮胎横向力
Fy_fl_true = zeros(1, N);
Fy_fr_true = zeros(1, N);
Fy_rl_true = zeros(1, N);
Fy_rr_true = zeros(1, N);

for i = 1:N
    % 基于简化的单轨模型计算轮胎力
    if abs(vx_true(i)) > 0.1
        beta = atan2(vy_true(i), vx_true(i));
        alpha_f = beta - lf * gamma_true(i) / vx_true(i);
        alpha_r = beta + lr * gamma_true(i) / vx_true(i);
    else
        alpha_f = 0;
        alpha_r = 0;
    end
    
    % 限制侧偏角
    alpha_f = max(-0.1, min(0.1, alpha_f));
    alpha_r = max(-0.1, min(0.1, alpha_r));
    
    % 计算前后轴总横向力
    Fyf_total = -Cf * alpha_f;
    Fyr_total = -Cr * alpha_r;
    
    % 分配到各轮（简化假设左右轮均分）
    Fy_fl_true(i) = Fyf_total / 2;
    Fy_fr_true(i) = Fyf_total / 2;
    Fy_rl_true(i) = Fyr_total / 2;
    Fy_rr_true(i) = Fyr_total / 2;
end

%% 生成测量数据
measurements = zeros(7, N);
control_inputs = zeros(5, N);

for i = 1:N
    % 计算真实加速度
    Fx_total = 0;  % 假设无纵向驱动力净值
    Fy_total = Fy_fl_true(i) + Fy_fr_true(i) + Fy_rl_true(i) + Fy_rr_true(i);
    
    ax_true = vy_true(i) * gamma_true(i);  % 向心加速度
    ay_true = Fy_total / m - vx_true(i) * gamma_true(i);  % 横向加速度
    
    % 计算真实轮速
    v_fl_true = vx_true(i) + gamma_true(i) * track/2;
    v_fr_true = vx_true(i) - gamma_true(i) * track/2;
    v_rl_true = vx_true(i) + gamma_true(i) * track/2;
    v_rr_true = vx_true(i) - gamma_true(i) * track/2;
    
    % 添加现实的测量噪声
    measurements(:, i) = [
        ax_true + 0.1*randn;              % ax - 减小噪声
        ay_true + 0.05*randn;             % ay - 减小噪声
        gamma_true(i) + 0.002*randn;      % gamma - 减小噪声
        v_fl_true + 0.05*randn;           % v_fl - 减小噪声
        v_fr_true + 0.05*randn;           % v_fr
        v_rl_true + 0.05*randn;           % v_rl
        v_rr_true + 0.05*randn;           % v_rr
    ];
    
    % 控制输入（更现实的值）
    control_inputs(:, i) = [
        0.02 * sin(0.3*t(i));            % 小幅前轮转角
        500 + 200*sin(0.1*t(i));         % 前左纵向力
        500 + 200*cos(0.1*t(i));         % 前右纵向力
        400;                              % 后左纵向力
        400;                              % 后右纵向力
    ];
end

%% 运行改进的EKF
fprintf('测试改进的横向速度估计性能...\n');

% 预分配输出数组
vx_est = zeros(1, N);
vy_est = zeros(1, N);
gamma_est = zeros(1, N);
Fy_fl_est = zeros(1, N);
Fy_fr_est = zeros(1, N);
Fy_rl_est = zeros(1, N);
Fy_rr_est = zeros(1, N);

% 重置EKF状态（清除persistent变量）
clear ekf_simulink_function;

for i = 1:N
    [state_est, ~] = ekf_simulink_function(measurements(:, i), control_inputs(:, i));
    
    vx_est(i) = state_est(1);
    vy_est(i) = state_est(2);
    gamma_est(i) = state_est(3);
    Fy_fl_est(i) = state_est(4);
    Fy_fr_est(i) = state_est(5);
    Fy_rl_est(i) = state_est(6);
    Fy_rr_est(i) = state_est(7);
    
    if mod(i, 500) == 0
        fprintf('进度: %.1f%%, 当前vy估计: %.3f m/s, 真值: %.3f m/s\n', ...
            i/N*100, vy_est(i), vy_true(i));
    end
end

%% 性能分析
vx_error = vx_est - vx_true;
vy_error = vy_est - vy_true;
gamma_error = gamma_est - gamma_true;

% 计算统计指标
vx_rmse = sqrt(mean(vx_error.^2));
vy_rmse = sqrt(mean(vy_error.^2));
gamma_rmse = sqrt(mean(gamma_error.^2));

vx_max_error = max(abs(vx_error));
vy_max_error = max(abs(vy_error));
gamma_max_error = max(abs(gamma_error));

vy_mean_abs_error = mean(abs(vy_error));

fprintf('\n=== 横向速度估计性能分析 ===\n');
fprintf('横向速度真值范围: [%.3f, %.3f] m/s\n', min(vy_true), max(vy_true));
fprintf('横向速度估计范围: [%.3f, %.3f] m/s\n', min(vy_est), max(vy_est));
fprintf('横向速度 RMSE: %.4f m/s\n', vy_rmse);
fprintf('横向速度 平均绝对误差: %.4f m/s\n', vy_mean_abs_error);
fprintf('横向速度 最大误差: %.4f m/s\n', vy_max_error);
fprintf('纵向速度 RMSE: %.3f m/s\n', vx_rmse);
fprintf('横摆角速度 RMSE: %.4f rad/s\n', gamma_rmse);

%% 绘制详细的分析结果
figure('Name', '横向速度估计性能分析', 'Position', [100, 100, 1400, 1000]);

% 横向速度对比
subplot(3, 3, 1);
plot(t, vy_true, 'b-', 'LineWidth', 2); hold on;
plot(t, vy_est, 'r--', 'LineWidth', 1.5);
xlabel('时间 (s)'); ylabel('横向速度 (m/s)');
title('横向速度估计对比');
legend('真实值', '估计值', 'Location', 'best');
grid on;
ylim([-1.5, 1.5]);

% 横向速度误差
subplot(3, 3, 2);
plot(t, vy_error, 'k-', 'LineWidth', 1);
xlabel('时间 (s)'); ylabel('误差 (m/s)');
title(['横向速度误差 (RMSE=' num2str(vy_rmse, '%.4f') 'm/s)']);
grid on;

% 误差分布
subplot(3, 3, 3);
histogram(vy_error, 50, 'Normalization', 'probability');
xlabel('横向速度误差 (m/s)'); ylabel('概率');
title('横向速度误差分布');
grid on;

% 纵向速度对比
subplot(3, 3, 4);
plot(t, vx_true, 'b-', 'LineWidth', 2); hold on;
plot(t, vx_est, 'r--', 'LineWidth', 1.5);
xlabel('时间 (s)'); ylabel('纵向速度 (m/s)');
title('纵向速度估计');
legend('真实值', '估计值', 'Location', 'best');
grid on;

% 横摆角速度对比
subplot(3, 3, 5);
plot(t, gamma_true, 'b-', 'LineWidth', 2); hold on;
plot(t, gamma_est, 'r--', 'LineWidth', 1.5);
xlabel('时间 (s)'); ylabel('横摆角速度 (rad/s)');
title('横摆角速度估计');
legend('真实值', '估计值', 'Location', 'best');
grid on;

% 轮胎力估计 - 前轮
subplot(3, 3, 6);
plot(t, Fy_fl_true, 'b-', 'LineWidth', 1.5); hold on;
plot(t, Fy_fl_est, 'r--', 'LineWidth', 1.5);
plot(t, Fy_fr_true, 'g-', 'LineWidth', 1.5);
plot(t, Fy_fr_est, 'm--', 'LineWidth', 1.5);
xlabel('时间 (s)'); ylabel('横向力 (N)');
title('前轮横向力估计');
legend('FL真值', 'FL估计', 'FR真值', 'FR估计', 'Location', 'best');
grid on;

% 加速度对比
subplot(3, 3, 7);
ax_meas = measurements(1, :);
ay_meas = measurements(2, :);
plot(t, ax_meas, 'b-', 'LineWidth', 1); hold on;
plot(t, ay_meas, 'r-', 'LineWidth', 1);
xlabel('时间 (s)'); ylabel('加速度 (m/s²)');
title('测量的加速度');
legend('纵向加速度', '横向加速度', 'Location', 'best');
grid on;

% 横向速度估计精度随时间变化
subplot(3, 3, 8);
vy_abs_error = abs(vy_error);
plot(t, vy_abs_error, 'r-', 'LineWidth', 1.5);
xlabel('时间 (s)'); ylabel('绝对误差 (m/s)');
title('横向速度绝对误差');
grid on;

% 控制输入
subplot(3, 3, 9);
plot(t, control_inputs(1, :)*180/pi, 'g-', 'LineWidth', 1.5);
xlabel('时间 (s)'); ylabel('前轮转角 (度)');
title('前轮转角输入');
grid on;

%% 性能评估
fprintf('\n=== 性能评估结果 ===\n');

% 横向速度估计质量评估
if vy_rmse < 0.05
    fprintf('✓ 横向速度估计精度优秀 (RMSE < 0.05 m/s)\n');
elseif vy_rmse < 0.1
    fprintf('⚠ 横向速度估计精度良好 (0.05 < RMSE < 0.1 m/s)\n');
elseif vy_rmse < 0.2
    fprintf('⚠ 横向速度估计精度一般 (0.1 < RMSE < 0.2 m/s)\n');
else
    fprintf('✗ 横向速度估计精度需改进 (RMSE > 0.2 m/s)\n');
end

% 检查是否还有过大的估计值
if max(abs(vy_est)) > 2.0
    fprintf('⚠ 警告：横向速度估计值仍存在超过±2m/s的情况\n');
    fprintf('  最大估计值: %.3f m/s\n', max(vy_est));
    fprintf('  最小估计值: %.3f m/s\n', min(vy_est));
else
    fprintf('✓ 横向速度估计值在合理范围内 (±2m/s)\n');
end

% 收敛性分析
initial_phase = 1:min(500, N);  % 前5秒
steady_phase = 500:N;           % 稳态阶段

if length(steady_phase) > 100
    vy_rmse_steady = sqrt(mean(vy_error(steady_phase).^2));
    fprintf('稳态阶段横向速度 RMSE: %.4f m/s\n', vy_rmse_steady);
    
    if vy_rmse_steady < vy_rmse * 0.8
        fprintf('✓ 估计器显示良好的收敛特性\n');
    end
end

fprintf('\n改进效果总结:\n');
fprintf('- 大幅降低了过程噪声，特别是横向速度的噪声\n');
fprintf('- 增加了轮胎力时间常数，减缓了轮胎力变化\n');
fprintf('- 严格限制了横向速度范围在±2m/s\n');
fprintf('- 减少了轮胎力与车辆状态的耦合强度\n');
fprintf('- 提高了传感器测量的可信度\n');

fprintf('\n测试完成！\n');
