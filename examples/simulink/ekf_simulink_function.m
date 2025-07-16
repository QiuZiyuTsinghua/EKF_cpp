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

% Filter parameters
Q = diag([0.1, 0.1, 0.01]);  % Process noise covariance
R = diag([0.5, 0.5, 0.01, 0.2, 0.2, 0.2, 0.2]);  % Measurement noise covariance

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

% State transition (simplified vehicle model)
if abs(vx) > 0.1
    beta = atan2(vy, vx);
    alpha_f = beta - lf * gamma / vx;
    alpha_r = beta + lr * gamma / vx;
else
    alpha_f = 0;
    alpha_r = 0;
end

% Tire forces
Fyf = -Cf * alpha_f;
Fyr = -Cr * alpha_r;

% State derivatives
ax = 0.0;  % Assume zero longitudinal acceleration
ay = (Fyf + Fyr) / m;
Mz = Fyf * lf - Fyr * lr;

vx_dot = ax + vy * gamma;
vy_dot = ay - vx * gamma;
gamma_dot = Mz / Iz;

% Predict state
x_pred = x_hat + dt * [vx_dot; vy_dot; gamma_dot];

% State Jacobian
F = eye(3);
F(1, 2) = dt * gamma;
F(1, 3) = dt * vy;
F(2, 1) = -dt * gamma;
F(2, 3) = -dt * vx;

% Predict covariance
P_pred = F * P_hat * F' + Q;

%% Update Step

% Predicted measurements
vx_pred = x_pred(1);
vy_pred = x_pred(2);
gamma_pred = x_pred(3);

% Calculate predicted tire forces for measurement prediction
if abs(vx_pred) > 0.1
    beta_pred = atan2(vy_pred, vx_pred);
    alpha_f_pred = beta_pred - lf * gamma_pred / vx_pred;
    alpha_r_pred = beta_pred + lr * gamma_pred / vx_pred;
else
    alpha_f_pred = 0;
    alpha_r_pred = 0;
end

Fyf_pred = -Cf * alpha_f_pred;
Fyr_pred = -Cr * alpha_r_pred;

% Predicted measurements
ax_pred = vy_pred * gamma_pred;
ay_pred = (Fyf_pred + Fyr_pred) / m - vx_pred * gamma_pred;
gamma_meas_pred = gamma_pred;
v_fl_pred = vx_pred - (gamma_pred * track/2);
v_fr_pred = vx_pred + (gamma_pred * track/2);
v_rl_pred = vx_pred - (gamma_pred * track/2);
v_rr_pred = vx_pred + (gamma_pred * track/2);

z_pred = [ax_pred; ay_pred; gamma_meas_pred; v_fl_pred; v_fr_pred; v_rl_pred; v_rr_pred];

% Innovation
y = measurements - z_pred;

% Measurement Jacobian
H = zeros(7, 3);
H(1, 2) = gamma_pred;    % ∂(ax)/∂vy
H(1, 3) = vy_pred;       % ∂(ax)/∂gamma
H(2, 1) = -gamma_pred;   % ∂(ay)/∂vx
H(2, 3) = -vx_pred;      % ∂(ay)/∂gamma
H(3, 3) = 1.0;           % ∂(gamma_meas)/∂gamma
H(4, 1) = 1.0;           % ∂(v_fl)/∂vx
H(4, 3) = -track/2;      % ∂(v_fl)/∂gamma
H(5, 1) = 1.0;           % ∂(v_fr)/∂vx
H(5, 3) = track/2;       % ∂(v_fr)/∂gamma
H(6, 1) = 1.0;           % ∂(v_rl)/∂vx
H(6, 3) = -track/2;      % ∂(v_rl)/∂gamma
H(7, 1) = 1.0;           % ∂(v_rr)/∂vx
H(7, 3) = track/2;       % ∂(v_rr)/∂gamma

% Innovation covariance
S = H * P_pred * H' + R;

% Kalman gain
K = P_pred * H' / S;

% Update state and covariance
x_hat = x_pred + K * y;
I_KH = eye(3) - K * H;
P_hat = I_KH * P_pred * I_KH' + K * R * K';

%% Outputs
state_estimate = x_hat;
covariance_flat = reshape(P_hat, 9, 1);

end
