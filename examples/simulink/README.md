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
