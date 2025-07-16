# Using Extended Kalman Filter in Simulink

This directory contains files for integrating the C++ EKF implementation with MATLAB/Simulink, specifically for vehicle velocity estimation.

## Files

- `ekf_sfun.cpp` - S-Function source code that implements a vehicle velocity estimation model
- `compile_sfun_direct.m` - MATLAB script to compile the S-Function directly from source
- `ekf_example.slx` - Example Simulink model (created when you follow the steps below)

## Vehicle Velocity Estimation Model

The implementation uses a 3-DOF vehicle model with the following state vector:
- `v_x` - Longitudinal velocity (m/s)
- `v_y` - Lateral velocity (m/s)
- `γ` - Yaw rate (rad/s)

The model uses the following measurements:
- `a_x` - Longitudinal acceleration (m/s²)
- `a_y` - Lateral acceleration (m/s²)
- `γ` - Measured yaw rate (rad/s)
- `v_fl`, `v_fr`, `v_rl`, `v_rr` - Wheel speeds (m/s) for front-left, front-right, rear-left, and rear-right wheels

## Setup Instructions

### Method 1: Direct Compilation (Recommended)

1. Ensure you have the following prerequisites:
   - MATLAB with Simulink
   - C++ compiler compatible with your MATLAB version
   - Eigen library (3.3 or newer)

2. Open MATLAB and navigate to this directory.

3. Run the compilation script with your specific paths:
   ```matlab
   % Define paths to match your environment (optional)
   EKF_SRC_PATH = '/path/to/ekf/source';  % Path to directory containing ekf.h and ekf.cpp
   EIGEN_INCLUDE_PATH = '/path/to/eigen'; % Path to Eigen library
   
   % Run the compilation script
   compile_sfun_direct
   ```

### Method 2: Using Pre-compiled Library

1. First, build the EKF library as a shared library:
   ```bash
   cd /path/to/ekf
   mkdir -p build && cd build
   cmake -DBUILD_SHARED_LIBS=ON ..
   make
   ```

2. Edit `compile_sfun.m` to update the paths to match your environment.

3. Run the compilation script:
   ```matlab
   compile_sfun
   ```

## Creating a Simulink Model with EKF

1. Add an S-Function block from the Simulink library browser.

2. Double-click the block and set:
   - S-Function Name: `ekf_sfun`
   - S-Function Parameters:
     ```
     [3]          % STATE_DIM: 3 states [v_x, v_y, γ]
     [7]          % MEAS_DIM: 7 measurements [ax, ay, γ, v_fl, v_fr, v_rl, v_rr]
     [1]          % CTRL_DIM: 1 control input (steering angle)
     [0.01]       % DT: Time step in seconds (10ms)
     [10;0;0]     % INITIAL_STATE: Initial state vector [v_x;v_y;γ] (10 m/s forward)
     diag([1.0, 0.1, 0.01])  % INITIAL_COV: Initial uncertainties
     diag([0.5, 0.1, 0.01])  % PROCESS_NOISE_COV: Process noise
     diag([0.1, 0.1, 0.01, 0.2, 0.2, 0.2, 0.2]) % MEAS_NOISE_COV: Sensor noise
     ```

3. Connect inputs and outputs:
   - Input 1: Measurement vector [ax, ay, γ, v_fl, v_fr, v_rl, v_rr]
   - Input 2 (optional): Control input [δ] (steering angle)
   - Output 1: Estimated state vector [v_x, v_y, γ]
   - Output 2: Covariance matrix (flattened)

## Configuring Parameters for the EKF S-Function

The EKF S-Function requires 8 parameters that define its behavior. Here's how to configure them for the vehicle velocity estimation model:

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

### Understanding Parameter Types

* **STATE_DIM [3]**: Dimension of the state vector [v_x, v_y, γ]
* **MEAS_DIM [7]**: Number of measurement inputs [ax, ay, γ, v_fl, v_fr, v_rl, v_rr]
* **CTRL_DIM [1]**: Number of control inputs [δ] (steering angle)
* **DT [0.01]**: Time step in seconds (should match your Simulink fixed-step solver)
* **INITIAL_STATE [10;0;0]**: Column vector with initial values for each state
* **INITIAL_COV**: Covariance matrix for initial state uncertainty
* **PROCESS_NOISE_COV**: Covariance matrix of process noise (model uncertainty)
* **MEAS_NOISE_COV**: Covariance matrix of measurement noise (sensor uncertainty)

### Vehicle Parameters

The S-Function uses internal vehicle parameters that can be adjusted in the code if needed:

```cpp
// Default vehicle parameters in ekf_sfun.cpp
double m = 1500.0;      // Vehicle mass (kg)
double Iz = 2500.0;     // Yaw moment of inertia (kg*m^2)
double lf = 1.2;        // Distance from CG to front axle (m)
double lr = 1.4;        // Distance from CG to rear axle (m)
double track = 1.6;     // Track width (m)
double Cf = 50000.0;    // Front cornering stiffness (N/rad)
double Cr = 50000.0;    // Rear cornering stiffness (N/rad)
```

If you need to change these parameters, you'll need to modify `ekf_sfun.cpp` and recompile.

## Example Simulink Model Structure

```
Sensor Inputs [7] --> [EKF S-Function] --> [Display/Scope] (State Estimates)
Steering Angle [1] -->                 \--> [Reshape] --> [Covariance Display]
```

## Example Code for Generating Test Inputs

```matlab
% Create a simple simulation with accelerometer, gyro and wheel speed signals
t = 0:0.01:10;  % 10 seconds at 100Hz
steeringAngle = 0.1 * sin(0.5*t);  % Steering angle input (radians)

% Simple vehicle states
vx = 10 * ones(size(t));           % Constant forward velocity (m/s)
vy = 0.5 * sin(0.5*t);             % Oscillating lateral velocity
yawRate = 0.2 * sin(0.5*t);        % Oscillating yaw rate

% Calculate accelerations
ax = zeros(size(t));               % No longitudinal acceleration
ay = 0.25 * cos(0.5*t);            % Lateral acceleration

% Calculate wheel speeds (simple kinematics)
trackWidth = 1.6;
v_fl = vx - (yawRate * trackWidth/2);  % Front-left wheel
v_fr = vx + (yawRate * trackWidth/2);  % Front-right wheel
v_rl = vx - (yawRate * trackWidth/2);  % Rear-left wheel
v_rr = vx + (yawRate * trackWidth/2);  % Rear-right wheel

% Add some noise to measurements
ax_noisy = ax + 0.1*randn(size(t));
ay_noisy = ay + 0.1*randn(size(t));
yawRate_noisy = yawRate + 0.01*randn(size(t));
v_fl_noisy = v_fl + 0.2*randn(size(t));
v_fr_noisy = v_fr + 0.2*randn(size(t));
v_rl_noisy = v_rl + 0.2*randn(size(t));
v_rr_noisy = v_rr + 0.2*randn(size(t));

% Combine into measurement matrix for Simulink (time x channels)
measurements = [ax_noisy; ay_noisy; yawRate_noisy; v_fl_noisy; v_fr_noisy; v_rl_noisy; v_rr_noisy]';

% Save to workspace for Simulink
simout.time = t;
simout.signals.values = measurements;
simout.signals.dimensions = 7;

steerOut.time = t;
steerOut.signals.values = steeringAngle';
steerOut.signals.dimensions = 1;
```
