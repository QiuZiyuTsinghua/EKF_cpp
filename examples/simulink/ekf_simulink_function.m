function [state_estimate, covariance_flat] = ekf_simulink_function(measurements, control_inputs)
%#codegen

% EKF_SIMULINK_FUNCTION Extended EKF for vehicle dynamics with tire force estimation
%
% This version estimates vehicle states and individual tire lateral forces.
%
% Inputs:
%   measurements - [7x1] measurement vector [ax, ay, gamma, v_fl, v_fr, v_rl, v_rr]
%   control_inputs - [5x1] control vector [delta, Fx_fl, Fx_fr, Fx_rl, Fx_rr]
%                    delta: front wheel steering angle (rad)
%                    Fx_*: longitudinal force on each wheel (N)
%
% Outputs:
%   state_estimate - [7x1] estimated state [v_x, v_y, gamma, Fy_fl, Fy_fr, Fy_rl, Fy_rr]
%   covariance_flat - [49x1] flattened covariance matrix (7x7 -> 49x1)

% Persistent variables to maintain state between calls
persistent x_hat P_hat is_initialized

% Hardcoded parameters for Simulink compatibility
dt = 0.01;  % Fixed time step

% Vehicle parameters
m = 1500.0;      % Vehicle mass (kg)
Iz = 2500.0;     % Yaw moment of inertia (kg*m^2)
lf = 1.2;        % Distance from CG to front axle (m)
lr = 1.4;        % Distance from CG to rear axle (m)
track_f = 1.6;   % Front track width (m)
track_r = 1.6;   % Rear track width (m)
Cf = 50000.0;    % Front cornering stiffness (N/rad)
Cr = 50000.0;    % Rear cornering stiffness (N/rad)

% Tire force dynamics time constants
tau_fy = 0.2;   % 增大横向力时间常数，减缓轮胎力变化 (s)

% Filter parameters - 大幅降低横向速度的过程噪声
Q = diag([0.05, 0.01, 0.005, 500, 500, 500, 500]);  % 显著降低vy和轮胎力的过程噪声
R = diag([0.3, 0.2, 0.005, 0.1, 0.1, 0.1, 0.1]);   % 提高传感器测量可信度

% Initialize on first call
if isempty(is_initialized)
    x_hat = [10.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0];  % 轮胎力初值设为0
    P_hat = diag([0.5, 0.1, 0.05, 1000, 1000, 1000, 1000]);  % 降低初始不确定性
    is_initialized = true;
end

%% Prediction Step

% Extract current state [vx, vy, gamma, Fy_fl, Fy_fr, Fy_rl, Fy_rr]
vx = x_hat(1);
vy = x_hat(2);
gamma = x_hat(3);
Fy_fl = x_hat(4);  % Front-left lateral force
Fy_fr = x_hat(5);  % Front-right lateral force
Fy_rl = x_hat(6);  % Rear-left lateral force
Fy_rr = x_hat(7);  % Rear-right lateral force

% Extract control inputs [delta, Fx_fl, Fx_fr, Fx_rl, Fx_rr]
delta = control_inputs(1);    % Front wheel steering angle
Fx_fl = control_inputs(2);    % Front-left longitudinal force
Fx_fr = control_inputs(3);    % Front-right longitudinal force
Fx_rl = control_inputs(4);    % Rear-left longitudinal force
Fx_rr = control_inputs(5);    % Rear-right longitudinal force

% Calculate individual wheel positions relative to CG
% Front wheels
x_fl = lf; y_fl = -track_f/2;  % Front-left wheel position
x_fr = lf; y_fr = track_f/2;   % Front-right wheel position
% Rear wheels  
x_rl = -lr; y_rl = -track_r/2; % Rear-left wheel position
x_rr = -lr; y_rr = track_r/2;  % Rear-right wheel position

% Calculate wheel velocities in wheel coordinate system
% Front wheels (with steering)
cos_delta = cos(delta);
sin_delta = sin(delta);

% Wheel velocities in vehicle frame
vx_fl = vx - gamma * y_fl;  vy_fl = vy + gamma * x_fl;
vx_fr = vx - gamma * y_fr;  vy_fr = vy + gamma * x_fr;
vx_rl = vx - gamma * y_rl;  vy_rl = vy + gamma * x_rl;
vx_rr = vx - gamma * y_rr;  vy_rr = vy + gamma * x_rr;

% Transform front wheel velocities to wheel coordinate system (considering steering)
vx_wheel_fl = vx_fl * cos_delta + vy_fl * sin_delta;
vy_wheel_fl = -vx_fl * sin_delta + vy_fl * cos_delta;
vx_wheel_fr = vx_fr * cos_delta + vy_fr * sin_delta;
vy_wheel_fr = -vx_fr * sin_delta + vy_fr * cos_delta;

% Rear wheels (no steering)
vx_wheel_rl = vx_rl;
vy_wheel_rl = vy_rl;
vx_wheel_rr = vx_rr;
vy_wheel_rr = vy_rr;

% Calculate tire slip angles
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

if abs(vx_wheel_rl) > 0.1
    alpha_rl = atan2(vy_wheel_rl, vx_wheel_rl);
else
    alpha_rl = 0;
end

if abs(vx_wheel_rr) > 0.1
    alpha_rr = atan2(vy_wheel_rr, vx_wheel_rr);
else
    alpha_rr = 0;
end

% Limit slip angles to avoid unrealistic values
alpha_fl = max(-0.3, min(0.3, alpha_fl));
alpha_fr = max(-0.3, min(0.3, alpha_fr));
alpha_rl = max(-0.3, min(0.3, alpha_rl));
alpha_rr = max(-0.3, min(0.3, alpha_rr));

% Calculate desired lateral forces based on tire model (steady-state)
Fy_des_fl = -Cf * alpha_fl;
Fy_des_fr = -Cf * alpha_fr;
Fy_des_rl = -Cr * alpha_rl;
Fy_des_rr = -Cr * alpha_rr;

% Limit desired forces - 大幅减小轮胎力限制
max_tire_force_per_wheel = 3000;  % 单轮最大横向力3kN (约0.2g每轮)
Fy_des_fl = max(-max_tire_force_per_wheel, min(max_tire_force_per_wheel, Fy_des_fl));
Fy_des_fr = max(-max_tire_force_per_wheel, min(max_tire_force_per_wheel, Fy_des_fr));
Fy_des_rl = max(-max_tire_force_per_wheel, min(max_tire_force_per_wheel, Fy_des_rl));
Fy_des_rr = max(-max_tire_force_per_wheel, min(max_tire_force_per_wheel, Fy_des_rr));

% Transform tire forces back to vehicle coordinate system
% Front tire forces (considering steering angle)
Fx_veh_fl = Fx_fl * cos_delta - Fy_fl * sin_delta;
Fy_veh_fl = Fx_fl * sin_delta + Fy_fl * cos_delta;
Fx_veh_fr = Fx_fr * cos_delta - Fy_fr * sin_delta;
Fy_veh_fr = Fx_fr * sin_delta + Fy_fr * cos_delta;

% Rear tire forces (no transformation needed)
Fx_veh_rl = Fx_rl;
Fy_veh_rl = Fy_rl;
Fx_veh_rr = Fx_rr;
Fy_veh_rr = Fy_rr;

% Calculate total forces and moments
Fx_total = Fx_veh_fl + Fx_veh_fr + Fx_veh_rl + Fx_veh_rr;
Fy_total = Fy_veh_fl + Fy_veh_fr + Fy_veh_rl + Fy_veh_rr;
Mz_total = (Fx_veh_fl * y_fl - Fy_veh_fl * x_fl) + ...
           (Fx_veh_fr * y_fr - Fy_veh_fr * x_fr) + ...
           (Fx_veh_rl * y_rl - Fy_veh_rl * x_rl) + ...
           (Fx_veh_rr * y_rr - Fy_veh_rr * x_rr);

% State derivatives - 修正加速度计算
ax = Fx_total / m + vy * gamma;        % 纵向加速度（包含向心力）
ay = Fy_total / m - vx * gamma;        % 横向加速度（包含离心力）
gamma_dot = Mz_total / Iz;             % 横摆角加速度

% Tire force dynamics (first-order lag) - 增加阻尼
Fy_fl_dot = (Fy_des_fl - Fy_fl) / tau_fy;
Fy_fr_dot = (Fy_des_fr - Fy_fr) / tau_fy;
Fy_rl_dot = (Fy_des_rl - Fy_rl) / tau_fy;
Fy_rr_dot = (Fy_des_rr - Fy_rr) / tau_fy;

% 限制轮胎力变化率，防止过快变化
max_fy_rate = 10000;  % 最大轮胎力变化率 N/s
Fy_fl_dot = max(-max_fy_rate, min(max_fy_rate, Fy_fl_dot));
Fy_fr_dot = max(-max_fy_rate, min(max_fy_rate, Fy_fr_dot));
Fy_rl_dot = max(-max_fy_rate, min(max_fy_rate, Fy_rl_dot));
Fy_rr_dot = max(-max_fy_rate, min(max_fy_rate, Fy_rr_dot));

% State vector derivatives
vx_dot = ax;
vy_dot = ay;

% 大幅限制速度变化率，特别是横向速度
vx_dot = max(-5.0, min(5.0, vx_dot));   % 限制纵向加速度
vy_dot = max(-2.0, min(2.0, vy_dot));   % 严格限制横向加速度在±2m/s²
gamma_dot = max(-2.0, min(2.0, gamma_dot));

% Predict state
x_pred = x_hat + dt * [vx_dot; vy_dot; gamma_dot; Fy_fl_dot; Fy_fr_dot; Fy_rl_dot; Fy_rr_dot];

% State Jacobian F (7x7 matrix) - 减小轮胎力耦合强度
F = eye(7);

% Vehicle dynamics Jacobian - 保持原有的车辆动力学耦合
F(1, 2) = dt * gamma;           % ∂(vx)/∂(vy)
F(1, 3) = dt * vy;              % ∂(vx)/∂(gamma)
F(2, 1) = -dt * gamma;          % ∂(vy)/∂(vx)  
F(2, 3) = -dt * vx;             % ∂(vy)/∂(gamma)

% Force contributions to accelerations - 减小轮胎力的影响系数
force_coupling_factor = 0.5;  % 减少轮胎力耦合强度
F(1, 4) = dt * sin_delta / m * force_coupling_factor;   % ∂(vx)/∂(Fy_fl)
F(1, 5) = dt * sin_delta / m * force_coupling_factor;   % ∂(vx)/∂(Fy_fr)
F(2, 4) = dt * cos_delta / m * force_coupling_factor;   % ∂(vy)/∂(Fy_fl)
F(2, 5) = dt * cos_delta / m * force_coupling_factor;   % ∂(vy)/∂(Fy_fr)
F(2, 6) = dt / m * force_coupling_factor;               % ∂(vy)/∂(Fy_rl)
F(2, 7) = dt / m * force_coupling_factor;               % ∂(vy)/∂(Fy_rr)

% Yaw moment contributions - 减小横摆力矩的影响
moment_coupling_factor = 0.3;  % 进一步减少横摆力矩耦合
F(3, 4) = dt * (-x_fl * cos_delta - y_fl * sin_delta) / Iz * moment_coupling_factor;
F(3, 5) = dt * (-x_fr * cos_delta - y_fr * sin_delta) / Iz * moment_coupling_factor;
F(3, 6) = dt * (-x_rl) / Iz * moment_coupling_factor;
F(3, 7) = dt * (-x_rr) / Iz * moment_coupling_factor;

% Tire force dynamics (diagonal terms) - 增大阻尼
tire_damping = 1 - dt / tau_fy;
F(4, 4) = tire_damping;      % ∂(Fy_fl)/∂(Fy_fl)
F(5, 5) = tire_damping;      % ∂(Fy_fr)/∂(Fy_fr)
F(6, 6) = tire_damping;      % ∂(Fy_rl)/∂(Fy_rl)
F(7, 7) = tire_damping;      % ∂(Fy_rr)/∂(Fy_rr)

% Predict covariance
P_pred = F * P_hat * F' + Q;

%% Update Step

% Predicted measurements based on current state
vx_pred = x_pred(1);
vy_pred = x_pred(2);
gamma_pred = x_pred(3);
Fy_fl_pred = x_pred(4);
Fy_fr_pred = x_pred(5);
Fy_rl_pred = x_pred(6);
Fy_rr_pred = x_pred(7);

% Calculate predicted wheel velocities
vx_fl_pred = vx_pred - gamma_pred * y_fl;
vx_fr_pred = vx_pred - gamma_pred * y_fr;  
vx_rl_pred = vx_pred - gamma_pred * y_rl;
vx_rr_pred = vx_pred - gamma_pred * y_rr;

% Transform forces to vehicle frame for acceleration calculation
Fx_veh_fl_pred = Fx_fl * cos_delta - Fy_fl_pred * sin_delta;
Fy_veh_fl_pred = Fx_fl * sin_delta + Fy_fl_pred * cos_delta;
Fx_veh_fr_pred = Fx_fr * cos_delta - Fy_fr_pred * sin_delta;
Fy_veh_fr_pred = Fx_fr * sin_delta + Fy_fr_pred * cos_delta;

% Predicted accelerations
Fx_total_pred = Fx_veh_fl_pred + Fx_veh_fr_pred + Fx_rl + Fx_rr;
Fy_total_pred = Fy_veh_fl_pred + Fy_veh_fr_pred + Fy_rl_pred + Fy_rr_pred;

ax_pred = Fx_total_pred / m + vy_pred * gamma_pred;
ay_pred = Fy_total_pred / m - vx_pred * gamma_pred;

% Limit predicted accelerations
ax_pred = max(-10.0, min(10.0, ax_pred));
ay_pred = max(-10.0, min(10.0, ay_pred));

% Predicted measurement vector
z_pred = [ax_pred; ay_pred; gamma_pred; vx_fl_pred; vx_fr_pred; vx_rl_pred; vx_rr_pred];

% Innovation
y = measurements - z_pred;

% Measurement Jacobian H (7x7 matrix)
H = zeros(7, 7);

% Acceleration measurements
% ∂(ax)/∂(states)
H(1, 1) = 0;                    % ∂(ax)/∂(vx) 
H(1, 2) = gamma_pred;           % ∂(ax)/∂(vy)
H(1, 3) = vy_pred;              % ∂(ax)/∂(gamma)
H(1, 4) = sin_delta / m;        % ∂(ax)/∂(Fy_fl)
H(1, 5) = sin_delta / m;        % ∂(ax)/∂(Fy_fr)
H(1, 6) = 0;                    % ∂(ax)/∂(Fy_rl)
H(1, 7) = 0;                    % ∂(ax)/∂(Fy_rr)

% ∂(ay)/∂(states)  
H(2, 1) = -gamma_pred;          % ∂(ay)/∂(vx)
H(2, 2) = 0;                    % ∂(ay)/∂(vy)
H(2, 3) = -vx_pred;             % ∂(ay)/∂(gamma)
H(2, 4) = cos_delta / m;        % ∂(ay)/∂(Fy_fl)
H(2, 5) = cos_delta / m;        % ∂(ay)/∂(Fy_fr)
H(2, 6) = 1 / m;                % ∂(ay)/∂(Fy_rl)
H(2, 7) = 1 / m;                % ∂(ay)/∂(Fy_rr)

% ∂(gamma_meas)/∂(states)
H(3, 3) = 1.0;                  % ∂(gamma_meas)/∂(gamma)

% Wheel speed measurements  
% ∂(v_fl)/∂(states)
H(4, 1) = 1.0;                  % ∂(v_fl)/∂(vx)
H(4, 3) = -y_fl;                % ∂(v_fl)/∂(gamma)

% ∂(v_fr)/∂(states)
H(5, 1) = 1.0;                  % ∂(v_fr)/∂(vx)
H(5, 3) = -y_fr;                % ∂(v_fr)/∂(gamma)

% ∂(v_rl)/∂(states)
H(6, 1) = 1.0;                  % ∂(v_rl)/∂(vx)
H(6, 3) = -y_rl;                % ∂(v_rl)/∂(gamma)

% ∂(v_rr)/∂(states)
H(7, 1) = 1.0;                  % ∂(v_rr)/∂(vx)
H(7, 3) = -y_rr;                % ∂(v_rr)/∂(gamma)

% Innovation covariance
S = H * P_pred * H' + R;

% Kalman gain with numerical stability check
K = P_pred * H' / S;

% Update state and covariance
x_hat = x_pred + K * y;

% State constraints - 严格限制横向速度范围
x_hat(1) = max(0.5, min(50.0, x_hat(1)));     % vx: 0.5-50 m/s（避免过低速度）
x_hat(2) = max(-2.0, min(2.0, x_hat(2)));     % vy: ±2 m/s（严格限制横向速度）
x_hat(3) = max(-1.5, min(1.5, x_hat(3)));     % gamma: ±1.5 rad/s
x_hat(4) = max(-5000, min(5000, x_hat(4)));   % Fy_fl: ±5kN
x_hat(5) = max(-5000, min(5000, x_hat(5)));   % Fy_fr: ±5kN  
x_hat(6) = max(-5000, min(5000, x_hat(6)));   % Fy_rl: ±5kN
x_hat(7) = max(-5000, min(5000, x_hat(7)));   % Fy_rr: ±5kN

% Joseph form covariance update for numerical stability
I_KH = eye(7) - K * H;
P_hat = I_KH * P_pred * I_KH' + K * R * K';

%% Outputs
state_estimate = x_hat;
covariance_flat = reshape(P_hat, 49, 1);

end
