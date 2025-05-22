# Using Extended Kalman Filter in Simulink

This directory contains files for integrating the C++ EKF implementation with MATLAB/Simulink.

## Files

- `ekf_sfun.cpp` - S-Function source code that interfaces with the EKF library
- `compile_sfun.m` - MATLAB script to compile the S-Function
- `ekf_example.slx` - Example Simulink model (created when you follow the steps below)

## Setup Instructions

1. First, build the EKF library as a shared library:
   ```bash
   cd /path/to/ekf
   mkdir -p build && cd build
   cmake -DBUILD_SHARED_LIBS=ON ..
   make
   ```

2. Open MATLAB and navigate to this directory.

3. Edit `compile_sfun.m` to update the paths to match your environment:
   - `EKF_INCLUDE_PATH`: path to the directory containing `ekf.h`
   - `EIGEN_INCLUDE_PATH`: path to the Eigen library header files
   - `EKF_LIB_PATH`: path to the built EKF shared library (without extension)

4. Run the compilation script:
   ```matlab
   compile_sfun
   ```

5. Create a new Simulink model or open the example:
   ```matlab
   open_system('ekf_example')
   ```
   If the example doesn't exist yet, create a new model:
   ```matlab
   new_system('ekf_example')
   open_system('ekf_example')
   ```

## Creating a Simulink Model with EKF

1. Add an S-Function block from the Simulink library browser.

2. Double-click the block and set:
   - S-Function Name: `ekf_sfun`
   - S-Function Parameters:
     ```
     [4]          % STATE_DIM: State dimension (e.g., 4 for 2D position/velocity)
     [2]          % MEAS_DIM: Measurement dimension (e.g., 2 for position only)
     [0]          % CTRL_DIM: Control dimension (0 if not used)
     [0.1]        % DT: Time step in seconds
     [0;0;1;1]    % INITIAL_STATE: Initial state vector [x;y;vx;vy]
     [eye(4)]     % INITIAL_COV: Initial covariance matrix
     [diag([0.01,0.01,0.1,0.1])] % PROCESS_NOISE_COV: Process noise 
     [eye(2)*0.1] % MEAS_NOISE_COV: Measurement noise
     ```

3. Add input and output ports to connect to your model:
   - Input: Measurement vector
   - Output 1: Estimated state vector
   - Output 2: Covariance matrix (flattened)

## Configuring Parameters for the EKF S-Function

The EKF S-Function requires 8 parameters that define its behavior. Here's how to configure them with clear examples:

### Parameter Definition Format

Parameters must be defined as MATLAB expressions that evaluate to the correct dimensions:

```matlab
% Example parameter format for S-Function block
[
  [3],                       % STATE_DIM: State dimension - [v_x, v_y, γ]
  [7],                       % MEAS_DIM: Measurement vector - [ax, ay, γ, v_fl, v_fr, v_rl, v_rr]
  [1],                       % CTRL_DIM: Control dimension - [δ]
  [0.01],                    % DT: Time step in seconds
  [10; 0; 0],                % INITIAL_STATE: Initial state vector [v_x; v_y; γ]
  diag([1.0, 0.1, 0.01]),    % INITIAL_COV: Initial covariance matrix
  diag([0.5, 0.1, 0.01]),    % PROCESS_NOISE_COV: Process noise
  diag([0.1, 0.1, 0.01, 0.2, 0.2, 0.2, 0.2]) % MEAS_NOISE_COV: Measurement noise
]
```

### Vehicle Velocity Estimation Model Example

For the 3-DOF vehicle model with state vector `[v_x, v_y, γ]`, configure the parameters as follows:

1. Double-click the S-Function block in your Simulink model
2. Set the S-Function name to `ekf_sfun` 
3. In the "S-function parameters" field, enter the following:

```matlab
[3]                                   % STATE_DIM: 3 states [v_x, v_y, γ]
[7]                                   % MEAS_DIM: 7 measurements
[1]                                   % CTRL_DIM: 1 control input (steering angle)
[0.01]                                % DT: 10ms sampling time
[10; 0; 0]                            % INITIAL_STATE: initial velocity 10 m/s forward
diag([1.0, 0.1, 0.01])                % INITIAL_COV: initial uncertainties
diag([0.5, 0.1, 0.01])                % PROCESS_NOISE_COV: model uncertainties
diag([0.1, 0.1, 0.01, 0.2, 0.2, 0.2, 0.2])  % MEAS_NOISE_COV: sensor uncertainties
```

### Vehicle Parameters

When using the vehicle model, additional parameters need to be defined in your Simulink model:

```matlab
% Define vehicle parameters in your MATLAB workspace before simulation
vehicle_params.m = 1500.0;       % Mass (kg)
vehicle_params.Iz = 2500.0;      % Yaw moment of inertia (kg*m^2)
vehicle_params.lf = 1.2;         % Distance from CG to front axle (m)
vehicle_params.lr = 1.4;         % Distance from CG to rear axle (m)
vehicle_params.track = 1.6;      % Track width (m)
vehicle_params.Cf = 50000.0;     % Front cornering stiffness (N/rad)
vehicle_params.Cr = 50000.0;     % Rear cornering stiffness (N/rad)
```

### Understanding Parameter Types

* **STATE_DIM**: Dimension of your state vector
* **MEAS_DIM**: Number of measurement inputs to the filter
* **CTRL_DIM**: Number of control inputs (0 if none)
* **DT**: Time step in seconds (should match your Simulink fixed-step solver)
* **INITIAL_STATE**: Column vector with initial values for each state
* **INITIAL_COV**: Covariance matrix for initial state uncertainty
* **PROCESS_NOISE_COV**: Covariance matrix of process noise (model uncertainty)
* **MEAS_NOISE_COV**: Covariance matrix of measurement noise (sensor uncertainty)

### Tips for Parameter Configuration

1. Use `diag([...])` for diagonal covariance matrices
2. For full covariance matrices, use `[row1; row2; ...]` format
3. Match dimensions exactly with your state and measurement vectors
4. Use realistic noise values based on your sensors and model
5. Matrices can be defined as MATLAB variables and referenced by name

## Example Simulink Model Structure

```
[Measurement Generator] --> [EKF S-Function] --> [Display/Scope]
                                            \--> [Reshape] --> [Covariance Display]
```

## Customizing the EKF Model

The default S-Function implementation uses a simple linear constant-velocity motion model.
To customize the state transition and measurement models:

1. Modify the `ekf_sfun.cpp` file:
   - Find the `setStateTransitionFunction` call in `mdlStart`
   - Replace the default lambda function with your specific model
   - Do the same for `setStateJacobianFunction`, `setMeasurementFunction`, and `setMeasurementJacobianFunction`

2. Recompile the S-Function using the `compile_sfun.m` script.

## Example MATLAB Code to Generate Test Inputs

```matlab
% Create a test signal generator
t = 0:0.1:10;
x = sin(t);
y = cos(t);
measurements = [x; y];

% Create a Simulink signal builder block with these values
```
