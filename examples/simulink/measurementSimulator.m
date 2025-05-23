function [ax, ay, gamma_meas, v_fl, v_fr, v_rl, v_rr] = measurementSimulator(vx, vy, gamma)
% Simulate sensor measurements from true vehicle states
% 
% Inputs:
%   vx - True longitudinal velocity (m/s)
%   vy - True lateral velocity (m/s)
%   gamma - True yaw rate (rad/s)
%
% Outputs:
%   ax - Measured longitudinal acceleration (m/s^2)
%   ay - Measured lateral acceleration (m/s^2)
%   gamma_meas - Measured yaw rate (rad/s)
%   v_fl - Measured front-left wheel speed (m/s)
%   v_fr - Measured front-right wheel speed (m/s)
%   v_rl - Measured rear-left wheel speed (m/s)
%   v_rr - Measured rear-right wheel speed (m/s)

% Vehicle parameters
track = 1.6;  % Track width (m)

% Calculate true measurements
ax = vy * gamma;  % Centripetal component only (simplified)
ay = -vx * gamma;  % Lateral acceleration from circular motion (simplified)

% Calculate wheel speeds
v_fl = vx - (gamma * track/2);  % Front-left wheel
v_fr = vx + (gamma * track/2);  % Front-right wheel
v_rl = vx - (gamma * track/2);  % Rear-left wheel
v_rr = vx + (gamma * track/2);  % Rear-right wheel

% Add measurement noise (normal distribution)
rng('shuffle');  % Initialize random number generator
ax = ax + 0.1 * randn();  % Acceleration noise (std=0.1 m/s^2)
ay = ay + 0.1 * randn();
gamma_meas = gamma + 0.01 * randn();  % Yaw rate noise (std=0.01 rad/s)
v_fl = v_fl + 0.2 * randn();  % Wheel speed noise (std=0.2 m/s)
v_fr = v_fr + 0.2 * randn();
v_rl = v_rl + 0.2 * randn();
v_rr = v_rr + 0.2 * randn();

end
