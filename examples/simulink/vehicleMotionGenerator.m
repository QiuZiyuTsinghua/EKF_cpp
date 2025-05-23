function [vx, vy, gamma, acceleration, steering] = vehicleMotionGenerator(t)
% Generate realistic vehicle motion profiles for simulation
% 
% Inputs:
%   t - Current simulation time
%
% Outputs:
%   vx - Longitudinal velocity (m/s)
%   vy - Lateral velocity (m/s)
%   gamma - Yaw rate (rad/s)
%   acceleration - Applied longitudinal acceleration (m/s^2)
%   steering - Applied steering angle (rad)

% Initial velocity (m/s)
initial_vx = 10.0;

% Define acceleration profile
if t > 5.0 && t < 7.0
    acceleration = -2.0;  % Braking: -2 m/s^2
else
    acceleration = 0.0;   % Constant speed
end

% Define steering profile
if t > 2.0 && t < 4.0
    steering = 0.1;  % Turn with 0.1 rad steering angle
else
    steering = 0.0;  % Straight driving
end

% Simplified vehicle parameters
m = 1500.0;  % Vehicle mass (kg)
Iz = 2500.0;  % Yaw moment of inertia (kg*m^2)
lf = 1.2;  % Distance from CG to front axle (m)
lr = 1.4;  % Distance from CG to rear axle (m)
Cf = 50000.0;  % Front cornering stiffness (N/rad)
Cr = 50000.0;  % Rear cornering stiffness (N/rad)

% Calculate states (this is approximate; in a real model, these would be 
% calculated using integration over time)
vx = initial_vx + acceleration * min(t, 10.0);
vx = max(vx, 0);  % Ensure velocity doesn't go negative

% Calculate yaw rate based on steering input and velocity
steer_effect = steering * vx * 0.8;  % Simplified steering effect
gamma = steer_effect;

% Calculate lateral velocity (simplified)
vy = gamma * 0.5;  % Simplified lateral velocity

end
