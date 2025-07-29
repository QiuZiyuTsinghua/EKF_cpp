function [state_estimate, covariance_flat] = ekf_simulink_function(measurements)
%#codegen

% EKF_SIMULINK_FUNCTION Simplified EKF for Simulink MATLAB Function block
%
% This is a simplified version optimized for use in Simulink MATLAB Function blocks.
% Parameters are hardcoded to avoid complex parameter passing in Simulink.
%
% Inputs:
%   measurements - [7x1] measurement vector [ax, ay, gamma, v_fl, v_fr, v_rl, v_rr]
%
% Outputs:
%   state_estimate - [3x1] estimated state [v_x, v_y, gamma]
%   covariance_flat - [9x1] flattened covariance matrix

% Persistent variables to maintain state between calls
persistent x_hat P_hat is_initialized

% Hardcoded parameters for Simulink compatibility
dt = 0.01;  % Fixed time step

% Vehicle parameters
m = 1500.0;      % Vehicle mass (kg)
Iz = 2500.0;     % Yaw moment of inertia (kg*m^2)
lf = 1.2;        % Distance from CG to front axle (m)
lr = 1.4;        % Distance from CG to rear axle (m)
track = 1.6;     % Track width (m)
Cf = 50000.0;    % Front cornering stiffness (N/rad)
Cr = 50000.0;    % Rear cornering stiffness (N/rad)

% Filter parameters - 调整横向速度的过程噪声
Q = diag([0.1, 0.05, 0.01]);  % Process noise covariance [vx, vy, gamma] - 降低vy噪声
R = diag([0.5, 0.3, 0.01, 0.2, 0.2, 0.2, 0.2]);  % Measurement noise covariance - 提高ay测量可信度

% Initialize on first call
if isempty(is_initialized)
    x_hat = [10.0; 0.0; 0.0];  % Initial state [v_x, v_y, gamma]
    P_hat = diag([1.0, 1.0, 0.1]);  % Initial covariance
    is_initialized = true;
end

%% Prediction Step

% Extract current state
vx = x_hat(1);
vy = x_hat(2);
gamma = x_hat(3);

% State transition (improved vehicle model)
if abs(vx) > 0.5  % 提高速度阈值避免低速时的数值问题
    beta = atan2(vy, vx);
    alpha_f = beta - lf * gamma / vx;
    alpha_r = beta + lr * gamma / vx;
    
    % 限制侧偏角避免轮胎力过大
    alpha_f = max(-0.2, min(0.2, alpha_f));  % 限制在±0.2弧度
    alpha_r = max(-0.2, min(0.2, alpha_r));
else
    alpha_f = 0;
    alpha_r = 0;
end

% Tire forces with saturation
Fyf = -Cf * alpha_f;
Fyr = -Cr * alpha_r;

% 限制轮胎力避免过大的横向加速度
max_tire_force = m * 8.0;  % 限制最大横向力为8g
Fyf = max(-max_tire_force, min(max_tire_force, Fyf));
Fyr = max(-max_tire_force, min(max_tire_force, Fyr));

% State derivatives with improved model
ax = 0.0;  % 假设纵向加速度为零
ay = (Fyf + Fyr) / m;  % 纯横向力产生的加速度
Mz = Fyf * lf - Fyr * lr;

% 改进的状态转移方程，考虑更准确的耦合
vx_dot = ax + vy * gamma;  % 纵向：考虑离心力
vy_dot = ay - vx * gamma;  % 横向：考虑向心力
gamma_dot = Mz / Iz;

% 对横向速度变化率进行限制，避免过大的跳跃
vy_dot = max(-5.0, min(5.0, vy_dot));  % 限制横向加速度在±5m/s²

% Predict state
x_pred = x_hat + dt * [vx_dot; vy_dot; gamma_dot];

% State Jacobian - 改进的雅可比矩阵计算
F = eye(3);
F(1, 2) = dt * gamma;
F(1, 3) = dt * vy;
F(2, 1) = -dt * gamma;
F(2, 3) = -dt * vx;

% 添加轮胎力对状态的非线性影响（简化版）
if abs(vx) > 0.5
    % 轮胎力对横向速度的影响
    dFy_dvx = Cf * lf * gamma / (vx^2) + Cr * lr * gamma / (vx^2);
    dFy_dvy = -(Cf + Cr) / vx;
    F(2, 1) = F(2, 1) + dt * dFy_dvx / m;
    F(2, 2) = F(2, 2) + dt * dFy_dvy / m;
end

% Predict covariance
P_pred = F * P_hat * F' + Q;

%% Update Step

% Predicted measurements
vx_pred = x_pred(1);
vy_pred = x_pred(2);
gamma_pred = x_pred(3);

% Calculate predicted tire forces for measurement prediction
if abs(vx_pred) > 0.5
    beta_pred = atan2(vy_pred, vx_pred);
    alpha_f_pred = beta_pred - lf * gamma_pred / vx_pred;
    alpha_r_pred = beta_pred + lr * gamma_pred / vx_pred;
    
    % 限制预测的侧偏角
    alpha_f_pred = max(-0.2, min(0.2, alpha_f_pred));
    alpha_r_pred = max(-0.2, min(0.2, alpha_r_pred));
else
    alpha_f_pred = 0;
    alpha_r_pred = 0;
end

Fyf_pred = -Cf * alpha_f_pred;
Fyr_pred = -Cr * alpha_r_pred;

% 限制预测的轮胎力
Fyf_pred = max(-max_tire_force, min(max_tire_force, Fyf_pred));
Fyr_pred = max(-max_tire_force, min(max_tire_force, Fyr_pred));

% Predicted measurements - 改进的测量方程
ax_pred = vy_pred * gamma_pred;  % 向心加速度分量
ay_pred = (Fyf_pred + Fyr_pred) / m - vx_pred * gamma_pred;  % 横向加速度

% 对预测的加速度进行合理性检查
ax_pred = max(-10.0, min(10.0, ax_pred));  % 限制在±10m/s²
ay_pred = max(-10.0, min(10.0, ay_pred));
gamma_meas_pred = gamma_pred;
v_fl_pred = vx_pred - (gamma_pred * track/2);
v_fr_pred = vx_pred + (gamma_pred * track/2);
v_rl_pred = vx_pred - (gamma_pred * track/2);
v_rr_pred = vx_pred + (gamma_pred * track/2);

z_pred = [ax_pred; ay_pred; gamma_meas_pred; v_fl_pred; v_fr_pred; v_rl_pred; v_rr_pred];

% Innovation
y = measurements - z_pred;

% Measurement Jacobian - 改进的测量雅可比矩阵
H = zeros(7, 3);
H(1, 2) = gamma_pred;    % ∂(ax)/∂vy
H(1, 3) = vy_pred;       % ∂(ax)/∂gamma
H(2, 1) = -gamma_pred;   % ∂(ay)/∂vx  
H(2, 3) = -vx_pred;      % ∂(ay)/∂gamma
H(3, 3) = 1.0;           % ∂(gamma_meas)/∂gamma

% 轮速测量的雅可比矩阵
H(4, 1) = 1.0;           % ∂(v_fl)/∂vx
H(4, 3) = -track/2;      % ∂(v_fl)/∂gamma
H(5, 1) = 1.0;           % ∂(v_fr)/∂vx
H(5, 3) = track/2;       % ∂(v_fr)/∂gamma
H(6, 1) = 1.0;           % ∂(v_rl)/∂vx
H(6, 3) = -track/2;      % ∂(v_rl)/∂gamma
H(7, 1) = 1.0;           % ∂(v_rr)/∂vx
H(7, 3) = track/2;       % ∂(v_rr)/∂gamma

% 添加轮胎力模型对加速度测量的非线性影响
if abs(vx_pred) > 0.5
    % 横向加速度对各状态的偏导数（考虑轮胎力的非线性）
    dFy_dvx = Cf * lf * gamma_pred / (vx_pred^2) + Cr * lr * gamma_pred / (vx_pred^2);
    dFy_dvy = -(Cf + Cr) / vx_pred;
    H(2, 1) = H(2, 1) + dFy_dvx / m;  % 更新∂(ay)/∂vx
    H(2, 2) = dFy_dvy / m;            % 添加∂(ay)/∂vy
end

% Innovation covariance
S = H * P_pred * H' + R;

% Kalman gain with numerical stability check
K = P_pred * H' / S;

% Update state and covariance
x_hat = x_pred + K * y;

% 对更新后的状态进行合理性检查
x_hat(1) = max(0.1, min(50.0, x_hat(1)));   % 纵向速度限制在0.1-50m/s
x_hat(2) = max(-10.0, min(10.0, x_hat(2))); % 横向速度限制在±10m/s  
x_hat(3) = max(-2.0, min(2.0, x_hat(3)));   % 横摆角速度限制在±2rad/s

% Joseph form covariance update for numerical stability
I_KH = eye(3) - K * H;
P_hat = I_KH * P_pred * I_KH' + K * R * K';

%% Outputs
state_estimate = x_hat;
covariance_flat = reshape(P_hat, 9, 1);

end
