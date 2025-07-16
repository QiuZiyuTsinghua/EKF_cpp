% filepath: /home/ubuntu/code/dev/examples/simulink/compile_sfun_direct.m
% Compile EKF S-Function directly from source code for vehicle velocity estimation
fprintf('Compiling EKF S-Function from source code...\n');

% Configure paths - automatically detect OS and set default paths
if ispc
    % Windows default paths
    DEFAULT_EKF_SRC_PATH = 'C:\Users\username\Documents\code\cpp\EKF_cpp';
    DEFAULT_EIGEN_INCLUDE_PATH = 'C:\Users\username\Documents\lib\eigen-3.4.0';
else
    % Linux/Mac default paths
    DEFAULT_EKF_SRC_PATH = '/home/ubuntu/code/dev';
    DEFAULT_EIGEN_INCLUDE_PATH = '/usr/include/eigen3';
end

% Allow for custom paths if set in the MATLAB workspace
if ~exist('EKF_SRC_PATH', 'var')
    EKF_SRC_PATH = DEFAULT_EKF_SRC_PATH;
end

if ~exist('EIGEN_INCLUDE_PATH', 'var')
    EIGEN_INCLUDE_PATH = DEFAULT_EIGEN_INCLUDE_PATH;
end

% Display configuration for user verification
fprintf('Using the following paths:\n');
fprintf('  - EKF source code path: %s\n', EKF_SRC_PATH);
fprintf('  - Eigen library path: %s\n', EIGEN_INCLUDE_PATH);

% Check if source files exist
ekf_h_file = fullfile(EKF_SRC_PATH, 'ekf.h');
ekf_cpp_file = fullfile(EKF_SRC_PATH, 'ekf.cpp');
ekf_sfun_file = fullfile(pwd, 'ekf_sfun.cpp');  % S-Function in current directory

% Verify all required files exist
if ~exist(ekf_h_file, 'file')
    error('ekf.h file not found. Please check the path: %s', ekf_h_file);
end

if ~exist(ekf_cpp_file, 'file')
    error('ekf.cpp file not found. Please check the path: %s', ekf_cpp_file);
end

if ~exist(ekf_sfun_file, 'file')
    error('ekf_sfun.cpp file not found. Please check the path: %s', ekf_sfun_file);
end

% Extract S_FUNCTION_NAME from ekf_sfun.cpp
fileContent = fileread(ekf_sfun_file);
sfunNameMatch = regexp(fileContent, '#define\s+S_FUNCTION_NAME\s+(\w+)', 'tokens', 'once');
if isempty(sfunNameMatch)
    error('S_FUNCTION_NAME not defined in ekf_sfun.cpp.');
end
sfunName = sfunNameMatch{1};  % Extracted S_FUNCTION_NAME (should be ekf_sfun)

fprintf('  - S-Function name: %s\n', sfunName);

% Compile S-Function and EKF source code directly
try
    mex('-v', ...
        '-output', sfunName, ...
        ['-I"', EKF_SRC_PATH, '"'], ...
        ['-I"', EIGEN_INCLUDE_PATH, '"'], ...
        ['"', ekf_sfun_file, '"'], ...
        ['"', ekf_cpp_file, '"']);
    
    fprintf('\nCompilation successful! %s.%s file has been generated.\n', ...
        sfunName, mexext);
catch ME
    fprintf('\nCompilation failed: %s\n', ME.message);
    
    % Provide detailed error information and suggestions
    if contains(lower(ME.message), 'eigen')
        fprintf('\nPossible issue with Eigen library path. Please verify that the path contains the Eigen directory.\n');
        fprintf('The Eigen directory should contain subdirectories like "Dense", "Core", etc.\n');
    elseif contains(lower(ME.message), 'ekf.h') || contains(lower(ME.message), 'ekf.cpp')
        fprintf('\nEKF source files not found. Please verify that ekf.h and ekf.cpp exist in the specified path.\n');
    else
        fprintf('\nPossible compiler issue. Please run "mex -setup C++" to confirm that a C++ compiler is installed.\n');
    end
    return;
end

% Display usage instructions for vehicle velocity estimation model
fprintf('\n--- Vehicle Velocity Estimation Model Usage Instructions ---\n');
fprintf('This S-Function implements a 3-DOF vehicle model with state vector [v_x, v_y, γ], where:\n');
fprintf('  - v_x: Longitudinal velocity (m/s)\n');
fprintf('  - v_y: Lateral velocity (m/s)\n');
fprintf('  - γ: Yaw rate (rad/s)\n\n');

fprintf('S-Function Block Parameters:\n');
fprintf('  - Parameter 1: STATE_DIM [3] (State dimension)\n');
fprintf('  - Parameter 2: MEAS_DIM [7] (Measurement dimension [ax, ay, γ, v_fl, v_fr, v_rl, v_rr])\n');
fprintf('  - Parameter 3: CTRL_DIM [1] (Control input dimension [δ] - steering angle)\n');
fprintf('  - Parameter 4: DT [0.01] (Time step in seconds)\n');
fprintf('  - Parameter 5: INITIAL_STATE [10; 0; 0] (Initial state vector [v_x; v_y; γ])\n');
fprintf('  - Parameter 6: INITIAL_COV (Initial covariance matrix)\n');
fprintf('  - Parameter 7: PROCESS_NOISE_COV (Process noise covariance matrix)\n');
fprintf('  - Parameter 8: MEAS_NOISE_COV (Measurement noise covariance matrix)\n\n');

fprintf('Example Parameter Configuration:\n');
fprintf('[3]                                   %% STATE_DIM: 3 states [v_x, v_y, γ]\n');
fprintf('[7]                                   %% MEAS_DIM: 7 measurements\n');
fprintf('[1]                                   %% CTRL_DIM: 1 control input (steering angle)\n');
fprintf('[0.01]                                %% DT: 10ms sampling time\n');
fprintf('[10; 0; 0]                            %% INITIAL_STATE: initial velocity 10 m/s forward\n');
fprintf('diag([1.0, 0.1, 0.01])                %% INITIAL_COV: initial uncertainties\n');
fprintf('diag([0.5, 0.1, 0.01])                %% PROCESS_NOISE_COV: model uncertainties\n');
fprintf('diag([0.1, 0.1, 0.01, 0.2, 0.2, 0.2, 0.2])  %% MEAS_NOISE_COV: sensor uncertainties\n');