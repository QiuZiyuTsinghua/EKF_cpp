% filepath: /home/ubuntu/code/dev/examples/simulink/compile_sfun_direct.m
% Compile EKF S-Function directly from source code
fprintf('Compiling EKF S-Function from source code...\n');

% Configure paths
EKF_SRC_PATH = 'C:\Users\ziyu.qiu\Documents\code\cpp\EKF_cpp';  % EKF source code directory
EIGEN_INCLUDE_PATH = 'C:\Users\ziyu.qiu\Documents\lib\eigen-3.4.0';  % Eigen library directory

% Check if source files exist
ekf_h_file = fullfile(EKF_SRC_PATH, 'ekf.h');
ekf_cpp_file = fullfile(EKF_SRC_PATH, 'ekf.cpp');
ekf_sfun_file = fullfile(pwd, 'ekf_sfun.cpp');  % Path to ekf_sfun.cpp

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
sfunName = sfunNameMatch{1};  % Extracted S_FUNCTION_NAME

fprintf('Using the following configuration:\n');
fprintf('  - EKF source code path: %s\n', EKF_SRC_PATH);
fprintf('  - Eigen library path: %s\n', EIGEN_INCLUDE_PATH);
fprintf('  - ekf.h location: %s\n', ekf_h_file);
fprintf('  - ekf.cpp location: %s\n', ekf_cpp_file);
fprintf('  - S-Function name: %s\n', sfunName);

% Compile S-Function and EKF source code directly - without using precompiled libraries
try
    mex('-v', ...
        '-output', sfunName, ...  % Use the extracted S_FUNCTION_NAME
        ['-I"', EKF_SRC_PATH, '"'], ...
        ['-I"', EIGEN_INCLUDE_PATH, '"'], ...
        ['"', ekf_sfun_file, '"'], ...
        ['"', ekf_cpp_file, '"']);
    
    fprintf('\nCompilation successful! %s.mexw64 file has been generated.\n', sfunName);
catch ME
    fprintf('\nCompilation failed: %s\n', ME.message);
    
    % Provide more detailed error information and suggestions
    if contains(lower(ME.message), 'eigen')
        fprintf('\nPossible issue with Eigen library path. Please verify that the path contains the Eigen directory with Dense and other header files.\n');
        fprintf('The Eigen directory should be: %s\n', fullfile(EIGEN_INCLUDE_PATH, 'Eigen'));
    elseif contains(lower(ME.message), 'ekf.h') || contains(lower(ME.message), 'ekf.cpp')
        fprintf('\nEKF source files not found. Please verify that ekf.h and ekf.cpp exist in the specified path.\n');
    else
        fprintf('\nPossible compiler issue. Please run "mex -setup C++" to confirm that a C++ compiler is installed.\n');
    end
end

% Usage instructions
fprintf('\nUsage in Simulink:\n');
fprintf('1. Add S-Function block to your model\n');
fprintf('2. Set the S-Function name to "ekf_sfun"\n');
fprintf('3. Configure the following parameters:\n');
fprintf('   - Parameter 1: STATE_DIM (state vector dimension)\n');
fprintf('   - Parameter 2: MEAS_DIM (measurement vector dimension)\n');
fprintf('   - Parameter 3: CTRL_DIM (control input dimension, 0 if none)\n');
fprintf('   - Parameter 4: DT (time step in seconds)\n');
fprintf('   - Parameter 5: INITIAL_STATE (initial state vector)\n');
fprintf('   - Parameter 6: INITIAL_COV (initial covariance matrix)\n');
fprintf('   - Parameter 7: PROCESS_NOISE_COV (process noise covariance matrix)\n');
fprintf('   - Parameter 8: MEAS_NOISE_COV (measurement noise covariance matrix)\n');