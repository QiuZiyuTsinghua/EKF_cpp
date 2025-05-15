% Script to compile the EKF S-Function for Simulink
%
% Before running this script:
% 1. Build the EKF library as a shared library using:
%    cmake -DBUILD_SHARED_LIBS=ON ..
%    make
% 
% 2. Set the paths below to match your environment

% Configuration - Modify these paths
EKF_INCLUDE_PATH = '../../';            % Path to ekf.h
EIGEN_INCLUDE_PATH = '/usr/include/eigen3';   % Path to Eigen library
EKF_LIB_PATH = '../../build/libekf';    % Path to built EKF library (without extension)

% Detect platform-specific library extension
if ispc
    lib_ext = '.dll';
elseif ismac
    lib_ext = '.dylib';
else  % Linux
    lib_ext = '.so';
end

% Full path to library
ekf_lib = [EKF_LIB_PATH lib_ext];

% Check if files exist
if ~exist(fullfile(EKF_INCLUDE_PATH, 'ekf.h'), 'file')
    error('Cannot find ekf.h in %s', EKF_INCLUDE_PATH);
end

if ~exist(ekf_lib, 'file')
    error('Cannot find EKF library at %s', ekf_lib);
end

% Print configuration
fprintf('Compiling EKF S-Function with:\n');
fprintf('  - EKF include path: %s\n', EKF_INCLUDE_PATH);
fprintf('  - Eigen include path: %s\n', EIGEN_INCLUDE_PATH);
fprintf('  - EKF library: %s\n', ekf_lib);

% Compile the S-Function
try
    mex('-v', ...
        ['-I', EKF_INCLUDE_PATH], ...
        ['-I', EIGEN_INCLUDE_PATH], ...
        'ekf_sfun.cpp', ...
        ekf_lib);
    fprintf('S-Function compiled successfully!\n');
catch ME
    fprintf('Compilation failed: %s\n', ME.message);
end

% Print usage information
fprintf('\nUsage in Simulink:\n');
fprintf('1. Add an S-Function block to your model\n');
fprintf('2. Set the S-Function name to "ekf_sfun"\n');
fprintf('3. Configure the following parameters:\n');
fprintf('   - Parameter 1: STATE_DIM (state vector dimension)\n');
fprintf('   - Parameter 2: MEAS_DIM (measurement vector dimension)\n');
fprintf('   - Parameter 3: CTRL_DIM (control input dimension, 0 if none)\n');
fprintf('   - Parameter 4: DT (time step in seconds)\n');
fprintf('   - Parameter 5: INITIAL_STATE (column vector)\n');
fprintf('   - Parameter 6: INITIAL_COV (square matrix)\n');
fprintf('   - Parameter 7: PROCESS_NOISE_COV (square matrix)\n');
fprintf('   - Parameter 8: MEAS_NOISE_COV (square matrix)\n');
