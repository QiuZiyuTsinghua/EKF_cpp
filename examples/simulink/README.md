# Using Extended Kalman Filter in Simulink

This directory contains files for integrating the C++ EKF implementation with MATLAB/Simulink.

## Files

- `ekf_sfun.cpp` - S-Function source code that interfaces with the EKF library
- `compile_sfun.m` - MATLAB script to compile the S-Function
- `ekf_example.slx` - Example Simulink model (created when you follow the steps below)
- `vehicleMotionGenerator.m` - MATLAB function to generate vehicle motion profiles
- `measurementSimulator.m` - MATLAB function to simulate sensor measurements

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

## Creating a Simulink Model with EKF

1. Add an S-Function block from the Simulink library browser.

2. Double-click the block and set:
   - S-Function Name: `ekf_sfun`
   - S-Function Parameters: (see below)

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

## Simulink Model Connection Guide

Here's how to connect the blocks to create a complete vehicle speed estimation simulation:

### Step 1: Add Blocks to Your Simulink Model

1. Create a new Simulink model: `File > New > Simulink Model`
2. Add the following blocks:
   - **Clock** block (Sources library)
   - **MATLAB Function** block (User-Defined Functions library) - for vehicle motion generation
   - **MATLAB Function** block (User-Defined Functions library) - for measurement simulation
   - **S-Function** block (User-Defined Functions library) - for the EKF
   - **Mux** block (Signal Routing library) - for combining measurement signals
   - **Demux** block (Signal Routing library) - for splitting state outputs
   - **Scope** blocks (Sinks library) - for visualization

### Step 2: Configure the MATLAB Function Blocks

1. **Vehicle Motion Generator Block**:
   - Double-click the first MATLAB Function block
   - Enter: `[vx, vy, gamma, acceleration, steering] = vehicleMotionGenerator(t)`
   - Make sure `vehicleMotionGenerator.m` is in your MATLAB path

2. **Measurement Simulator Block**:
   - Double-click the second MATLAB Function block
   - Enter: `[ax, ay, gamma_meas, v_fl, v_fr, v_rl, v_rr] = measurementSimulator(vx, vy, gamma)`
   - Make sure `measurementSimulator.m` is in your MATLAB path

3. **EKF S-Function Block**:
   - Configure as described in the "Configuring Parameters" section

### Step 3: Connect the Blocks

Connect the blocks in the following sequence:

1. **Clock** → input port of the **Vehicle Motion Generator**
2. Connect the outputs of **Vehicle Motion Generator**:
   - `vx`, `vy`, `gamma` → inputs of the **Measurement Simulator**
   - Connect these same signals to a **Scope** (for ground truth visualization)
3. Connect the outputs of **Measurement Simulator**:
   - All outputs → **Mux** block to create a 7-element vector
   - **Mux** → input port of the **EKF S-Function**
4. Connect the outputs of **EKF S-Function**:
   - State output (port 1) → **Demux** block to split into 3 signals
   - **Demux** → **Scope** (for estimated state visualization)
   - Covariance output (port 2) → optional display or analysis

### Step 4: Configure Simulation Parameters

1. Set simulation time to 10 seconds
2. Set fixed-step solver with step size matching the EKF DT parameter (e.g., 0.01s)
3. Configure scopes to display multiple signals for comparison

### Complete Simulink Block Diagram

The final block diagram should look like this:

```
[Clock] → [Vehicle Motion Generator] → [Measurement Simulator] → [Mux] → [EKF S-Function] → [Demux] → [Scope]
                      │                                                                 │
                      │                                                                 ↓
                      └───────────────────────────→ [Scope: Ground Truth] [Scope: Covariance]
```

### Example Visualization Setup

Configure your scopes to compare true vs. estimated states:

1. **Scope 1 (Ground Truth)**: Shows the true vehicle states
   - Configure with 3 input ports for vx, vy, and gamma
   - Set appropriate y-axis limits (e.g., 0-15 for vx)

2. **Scope 2 (Estimated States)**: Shows the EKF estimated states
   - Configure with 3 input ports for estimated vx, vy, and gamma
   - Use the same axis limits as Scope 1 for easy comparison

3. **Scope 3 (Error)**: Optional scope to show estimation errors
   - Connect difference blocks between true and estimated states
   - Monitor real-time estimation performance

With this setup, you can visualize how well the EKF tracks the true vehicle states under various driving conditions.

## Example MATLAB Code to Generate Test Inputs

```matlab
% Create a test signal generator
t = 0:0.01:10;
vx = 10 * ones(size(t));
vy = zeros(size(t));
gamma = zeros(size(t));

% Add lane change maneuver
idx = (t >= 2) & (t < 4);
gamma(idx) = 0.1;
vy(idx) = 0.05;

% Add braking event
idx = (t >= 5) & (t < 7);
vx(idx) = 10 - 2*(t(idx)-5);  % Decelerate at 2 m/s²

% Create a Simulink signal builder block with these values
```

## Troubleshooting

If you encounter issues with the simulation:

1. **S-Function not found**: Verify the MEX file was compiled successfully
2. **Parameter dimension errors**: Double-check all parameter dimensions
3. **Numerical instability**: Try increasing process/measurement noise
4. **Slow execution**: Consider optimizing the C++ code or reducing model complexity
