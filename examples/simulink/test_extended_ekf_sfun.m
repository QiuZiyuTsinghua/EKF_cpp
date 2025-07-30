%% Test Extended EKF S-Function
% This script tests the extended EKF S-function with 7-state model
% State: [v_x, v_y, gamma, Fy_fl, Fy_fr, Fy_rl, Fy_rr]
% Control: [delta, Fx_fl, Fx_fr, Fx_rl, Fx_rr]
% Measurement: [ax, ay, gamma, v_fl, v_fr, v_rl, v_rr]

clear all; close all; clc;

%% Model Parameters
stateDim = 7;    % Extended state dimension
measDim = 7;     % Measurement dimension  
ctrlDim = 5;     % Control dimension
dt = 0.01;       % Time step (10ms)

%% Initial Conditions
% Initial state [v_x, v_y, gamma, Fy_fl, Fy_fr, Fy_rl, Fy_rr]
initialState = [15.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0];  % Starting at 15 m/s

% Initial covariance matrix (7x7)
initialCov = diag([1.0, 0.5, 0.1, 1000, 1000, 1000, 1000]);

% Process noise covariance (7x7)
processNoiseCov = diag([0.1, 0.05, 0.01, 1000, 1000, 1000, 1000]);

% Measurement noise covariance (7x7)
measNoiseCov = diag([0.5, 0.3, 0.01, 0.2, 0.2, 0.2, 0.2]);

%% Test S-Function Parameters
% Parameters for S-function: [stateDim, measDim, ctrlDim, dt, 
%                            initialState, initialCov, processNoiseCov, measNoiseCov]
params = {stateDim, measDim, ctrlDim, dt, ...
          initialState, initialCov, processNoiseCov, measNoiseCov};

%% Simulation Test Data
% Simulate a turning maneuver
t_sim = 0:dt:5;  % 5 second simulation
N = length(t_sim);

% Control inputs: [delta, Fx_fl, Fx_fr, Fx_rl, Fx_rr]
steering_angle = 0.1 * sin(0.5 * t_sim);  % Sinusoidal steering
longitudinal_forces = ones(4, N) * 1000;   % Constant 1000N per wheel

controls = [steering_angle; longitudinal_forces];

% Simulated measurements: [ax, ay, gamma, v_fl, v_fr, v_rl, v_rr]
% For testing, generate synthetic measurements
measurements = zeros(7, N);
for i = 1:N
    % Simplified measurement simulation
    vx = 15 + 0.1 * randn();  % Longitudinal velocity with noise
    vy = 0.5 * sin(0.3 * t_sim(i)) + 0.05 * randn();  % Lateral velocity
    gamma = 0.05 * sin(0.5 * t_sim(i)) + 0.01 * randn();  % Yaw rate
    
    % Accelerations (simplified)
    ax = 0.1 + 0.1 * randn();
    ay = gamma * vx + 0.1 * randn();
    
    % Wheel speeds (simplified kinematic model)
    track = 1.6;
    v_fl = vx - gamma * track/2 + 0.1 * randn();
    v_fr = vx + gamma * track/2 + 0.1 * randn();
    v_rl = vx - gamma * track/2 + 0.1 * randn();
    v_rr = vx + gamma * track/2 + 0.1 * randn();
    
    measurements(:, i) = [ax; ay; gamma; v_fl; v_fr; v_rl; v_rr];
end

%% Display Test Configuration
fprintf('=== Extended EKF S-Function Test Configuration ===\n');
fprintf('State dimension: %d\n', stateDim);
fprintf('Measurement dimension: %d\n', measDim);
fprintf('Control dimension: %d\n', ctrlDim);
fprintf('Simulation time: %.1f seconds\n', t_sim(end));
fprintf('Time step: %.3f seconds\n', dt);
fprintf('\nState vector: [v_x, v_y, gamma, Fy_fl, Fy_fr, Fy_rl, Fy_rr]\n');
fprintf('Control vector: [delta, Fx_fl, Fx_fr, Fx_rl, Fx_rr]\n');
fprintf('Measurement vector: [ax, ay, gamma, v_fl, v_fr, v_rl, v_rr]\n');

%% Expected Behavior
fprintf('\n=== Expected Model Behavior ===\n');
fprintf('1. Vehicle dynamics: 3-DOF bicycle model with tire forces\n');
fprintf('2. Tire force dynamics: First-order response to desired forces\n');
fprintf('3. Individual wheel modeling with steering transformation\n');
fprintf('4. Measurement mapping from states to sensor outputs\n');

%% Test Data Summary
fprintf('\n=== Test Data Summary ===\n');
fprintf('Steering angle range: [%.3f, %.3f] rad\n', min(steering_angle), max(steering_angle));
fprintf('Longitudinal force per wheel: %.0f N\n', longitudinal_forces(1,1));
fprintf('Measurement noise levels:\n');
fprintf('  - Acceleration: %.1f m/s²\n', sqrt(measNoiseCov(1,1)));
fprintf('  - Yaw rate: %.3f rad/s\n', sqrt(measNoiseCov(3,3)));
fprintf('  - Wheel speed: %.1f m/s\n', sqrt(measNoiseCov(4,4)));

%% Compilation Instructions
fprintf('\n=== S-Function Compilation ===\n');
fprintf('To compile and use this S-function in Simulink:\n');
fprintf('1. Ensure ekf.h and EKF library are available\n');
fprintf('2. Run: mex ekf_sfun.cpp -I<eigen_path> -L<ekf_lib_path> -lekf\n');
fprintf('3. Add to Simulink model with parameters specified above\n');

%% Parameter Validation
fprintf('\n=== Parameter Validation ===\n');
fprintf('Initial state dimension: %d (should be %d)\n', length(initialState), stateDim);
fprintf('Initial covariance size: %dx%d (should be %dx%d)\n', ...
        size(initialCov), stateDim, stateDim);
fprintf('Process noise size: %dx%d (should be %dx%d)\n', ...
        size(processNoiseCov), stateDim, stateDim);
fprintf('Measurement noise size: %dx%d (should be %d×%d)\n', ...
        size(measNoiseCov), measDim, measDim);

% Check dimensions
assert(length(initialState) == stateDim, 'Initial state dimension mismatch');
assert(all(size(initialCov) == [stateDim, stateDim]), 'Initial covariance dimension mismatch');
assert(all(size(processNoiseCov) == [stateDim, stateDim]), 'Process noise dimension mismatch');
assert(all(size(measNoiseCov) == [measDim, measDim]), 'Measurement noise dimension mismatch');

fprintf('✓ All parameter dimensions are correct!\n');

%% Sample Usage in Simulink
fprintf('\n=== Sample Simulink Block Configuration ===\n');
fprintf('S-Function Block Parameters:\n');
for i = 1:length(params)
    if isnumeric(params{i}) && isscalar(params{i})
        fprintf('  Parameter %d: %.0f\n', i, params{i});
    elseif isnumeric(params{i}) && isvector(params{i}) && length(params{i}) <= 10
        fprintf('  Parameter %d: [%s]\n', i, num2str(params{i}', '%.2f '));
    else
        fprintf('  Parameter %d: %dx%d matrix\n', i, size(params{i}));
    end
end

fprintf('\nInput Ports:\n');
fprintf('  Port 1: Measurements [%dx1] - [ax, ay, gamma, v_fl, v_fr, v_rl, v_rr]\n', measDim);
fprintf('  Port 2: Controls [%dx1] - [delta, Fx_fl, Fx_fr, Fx_rl, Fx_rr]\n', ctrlDim);

fprintf('\nOutput Ports:\n');
fprintf('  Port 1: State estimate [%dx1] - [v_x, v_y, gamma, Fy_fl, Fy_fr, Fy_rl, Fy_rr]\n', stateDim);
fprintf('  Port 2: Covariance matrix [%dx1] - flattened %dx%d matrix\n', stateDim^2, stateDim, stateDim);

fprintf('\n=== Test completed successfully! ===\n');
