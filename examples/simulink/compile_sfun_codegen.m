% Compile code generation compatible EKF S-Function
fprintf('Compiling code generation compatible EKF S-Function...\n');

% Check if source file exists
ekf_sfun_file = fullfile(pwd, 'ekf_sfun.cpp');

if ~exist(ekf_sfun_file, 'file')
    error('ekf_sfun.cpp file not found. Please check the path: %s', ekf_sfun_file);
end

% Extract S_FUNCTION_NAME from ekf_sfun.cpp
fileContent = fileread(ekf_sfun_file);
sfunNameMatch = regexp(fileContent, '#define\s+S_FUNCTION_NAME\s+(\w+)', 'tokens', 'once');
if isempty(sfunNameMatch)
    error('S_FUNCTION_NAME not defined in ekf_sfun.cpp.');
end
sfunName = sfunNameMatch{1};

fprintf('Compiling %s for code generation compatibility...\n', sfunName);

% Compile S-Function (no external dependencies)
try
    mex('-v', ...
        '-output', sfunName, ...
        ['"', ekf_sfun_file, '"']);
    
    fprintf('\nCompilation successful! %s.mexw64 file has been generated.\n', sfunName);
    fprintf('This S-Function is compatible with Simulink code generation.\n');
catch ME
    fprintf('\nCompilation failed: %s\n', ME.message);
end

% Usage instructions
fprintf('\nUsage in Simulink for code generation:\n');
fprintf('1. Add S-Function block to your model\n');
fprintf('2. Set the S-Function name to "%s"\n', sfunName);
fprintf('3. Configure the following parameters:\n');
fprintf('   - Parameter 1: DT (time step in seconds)\n');
fprintf('   - Parameter 2: INITIAL_STATE (3x1 vector: [vx; vy; gamma])\n');
fprintf('   - Parameter 3: INITIAL_COV (3x3 matrix)\n');
fprintf('   - Parameter 4: PROCESS_NOISE_COV (3x3 matrix)\n');
fprintf('   - Parameter 5: MEAS_NOISE_COV (7x7 matrix)\n');
fprintf('4. This S-Function supports C code generation\n');
