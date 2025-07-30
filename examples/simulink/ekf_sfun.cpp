/*
 * S-Function wrapper for Extended Kalman Filter library
 *
 * This file provides a Simulink S-Function interface to the EKF C++ library
 * for an extended vehicle dynamics and tire force estimation model. 
 * The state vector is [v_x, v_y, γ, Fy_fl, Fy_fr, Fy_rl, Fy_rr], where:
 *   - v_x: Longitudinal velocity
 *   - v_y: Lateral velocity  
 *   - γ: Yaw rate (angular velocity around z-axis)
 *   - Fy_fl, Fy_fr, Fy_rl, Fy_rr: Lateral tire forces
 *
 * The control vector is [δ, Fx_fl, Fx_fr, Fx_rl, Fx_rr], where:
 *   - δ: Steering angle
 *   - Fx_fl, Fx_fr, Fx_rl, Fx_rr: Longitudinal tire forces
 *
 * The measurement vector is [ax, ay, γ, v_fl, v_fr, v_rl, v_rr], where:
 *   - ax, ay: Measured accelerations
 *   - γ: Measured yaw rate
 *   - v_fl, v_fr, v_rl, v_rr: Measured wheel speeds
 *
 * This extended model provides simultaneous estimation of vehicle states
 * and tire forces for advanced vehicle control applications.
 */

#define S_FUNCTION_NAME ekf_sfun
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include "ekf.h"
#include <vector>
#include <memory>

// Define parameters indices
enum ParamIndex {
    STATE_DIM = 0,    // Dimension of state vector [v_x, v_y, γ, Fy_fl, Fy_fr, Fy_rl, Fy_rr]
    MEAS_DIM,         // Dimension of measurement vector [ax, ay, γ, v_fl, v_fr, v_rl, v_rr]
    CTRL_DIM,         // Dimension of control vector [δ, Fx_fl, Fx_fr, Fx_rl, Fx_rr]
    DT,               // Time step
    INITIAL_STATE,    // Initial state values
    INITIAL_COV,      // Initial state covariance
    PROCESS_NOISE_COV, // Process noise covariance
    MEAS_NOISE_COV,    // Measurement noise covariance
    NUM_PARAMS
};

// Define parameters checking function
#define MDL_CHECK_PARAMETERS
#if defined(MDL_CHECK_PARAMETERS) && defined(MATLAB_MEX_FILE)
static void mdlCheckParameters(SimStruct *S)
{
    // Check parameter count
    if (ssGetSFcnParamsCount(S) != NUM_PARAMS) {
        ssSetErrorStatus(S, "Wrong number of parameters. Expected 8.");
        return;
    }
    
    // Check state dimension
    if (!mxIsDouble(ssGetSFcnParam(S, STATE_DIM)) || mxGetNumberOfElements(ssGetSFcnParam(S, STATE_DIM)) != 1) {
        ssSetErrorStatus(S, "STATE_DIM must be a scalar.");
        return;
    }
    
    // Check measurement dimension
    if (!mxIsDouble(ssGetSFcnParam(S, MEAS_DIM)) || mxGetNumberOfElements(ssGetSFcnParam(S, MEAS_DIM)) != 1) {
        ssSetErrorStatus(S, "MEAS_DIM must be a scalar.");
        return;
    }
    
    // Check control dimension
    if (!mxIsDouble(ssGetSFcnParam(S, CTRL_DIM)) || mxGetNumberOfElements(ssGetSFcnParam(S, CTRL_DIM)) != 1) {
        ssSetErrorStatus(S, "CTRL_DIM must be a scalar.");
        return;
    }
    
    // Check dt
    if (!mxIsDouble(ssGetSFcnParam(S, DT)) || mxGetNumberOfElements(ssGetSFcnParam(S, DT)) != 1) {
        ssSetErrorStatus(S, "DT must be a scalar.");
        return;
    }
    
    // Check initial state
    int stateDim = static_cast<int>(*mxGetPr(ssGetSFcnParam(S, STATE_DIM)));
    if (!mxIsDouble(ssGetSFcnParam(S, INITIAL_STATE)) || mxGetNumberOfElements(ssGetSFcnParam(S, INITIAL_STATE)) != stateDim) {
        ssSetErrorStatus(S, "INITIAL_STATE dimension must match STATE_DIM.");
        return;
    }
    
    // Check initial covariance (should be stateDim x stateDim)
    if (!mxIsDouble(ssGetSFcnParam(S, INITIAL_COV)) || 
        mxGetM(ssGetSFcnParam(S, INITIAL_COV)) != stateDim || 
        mxGetN(ssGetSFcnParam(S, INITIAL_COV)) != stateDim) {
        ssSetErrorStatus(S, "INITIAL_COV must be a square matrix with dimensions matching STATE_DIM.");
        return;
    }
    
    // Check process noise covariance
    if (!mxIsDouble(ssGetSFcnParam(S, PROCESS_NOISE_COV)) || 
        mxGetM(ssGetSFcnParam(S, PROCESS_NOISE_COV)) != stateDim || 
        mxGetN(ssGetSFcnParam(S, PROCESS_NOISE_COV)) != stateDim) {
        ssSetErrorStatus(S, "PROCESS_NOISE_COV must be a square matrix with dimensions matching STATE_DIM.");
        return;
    }
    
    // Check measurement noise covariance
    int measDim = static_cast<int>(*mxGetPr(ssGetSFcnParam(S, MEAS_DIM)));
    if (!mxIsDouble(ssGetSFcnParam(S, MEAS_NOISE_COV)) || 
        mxGetM(ssGetSFcnParam(S, MEAS_NOISE_COV)) != measDim || 
        mxGetN(ssGetSFcnParam(S, MEAS_NOISE_COV)) != measDim) {
        ssSetErrorStatus(S, "MEAS_NOISE_COV must be a square matrix with dimensions matching MEAS_DIM.");
        return;
    }
}
#endif

// Function to initialize sizes
static void mdlInitializeSizes(SimStruct *S)
{
    // Set number of parameters
    ssSetNumSFcnParams(S, NUM_PARAMS);
    
    // Parameter mismatch will be reported by Simulink
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        return;
    }
    
    // Register parameter check function
    ssSetSFcnParamNotTunable(S, 0);  // STATE_DIM
    ssSetSFcnParamNotTunable(S, 1);  // MEAS_DIM
    ssSetSFcnParamNotTunable(S, 2);  // CTRL_DIM
    ssSetSFcnParamNotTunable(S, 3);  // DT
    ssSetSFcnParamNotTunable(S, 4);  // INITIAL_STATE
    ssSetSFcnParamNotTunable(S, 5);  // INITIAL_COV
    ssSetSFcnParamNotTunable(S, 6);  // PROCESS_NOISE_COV
    ssSetSFcnParamNotTunable(S, 7);  // MEAS_NOISE_COV
    
    // No continuous states
    ssSetNumContStates(S, 0);
    
    // No discrete states
    ssSetNumDiscStates(S, 0);
    
    // Get dimensions from parameters
    int stateDim = static_cast<int>(*mxGetPr(ssGetSFcnParam(S, STATE_DIM)));
    int measDim = static_cast<int>(*mxGetPr(ssGetSFcnParam(S, MEAS_DIM)));
    int ctrlDim = static_cast<int>(*mxGetPr(ssGetSFcnParam(S, CTRL_DIM)));
    
    // Configure inputs
    if (!ssSetNumInputPorts(S, ctrlDim > 0 ? 2 : 1)) return;
    
    // Measurement input - [ax, ay, γ, v_fl, v_fr, v_rl, v_rr]
    ssSetInputPortWidth(S, 0, measDim);
    ssSetInputPortDataType(S, 0, SS_DOUBLE);
    ssSetInputPortDirectFeedThrough(S, 0, 1);
    ssSetInputPortRequiredContiguous(S, 0, 1);
    
    // Control input (steering angle and longitudinal forces)
    if (ctrlDim > 0) {
        ssSetInputPortWidth(S, 1, ctrlDim);
        ssSetInputPortDataType(S, 1, SS_DOUBLE);
        ssSetInputPortDirectFeedThrough(S, 1, 1);
        ssSetInputPortRequiredContiguous(S, 1, 1);
    }
    
    // Configure outputs
    if (!ssSetNumOutputPorts(S, 2)) return;
    
    // State estimate output - [v_x, v_y, γ, Fy_fl, Fy_fr, Fy_rl, Fy_rr]
    ssSetOutputPortWidth(S, 0, stateDim);
    ssSetOutputPortDataType(S, 0, SS_DOUBLE);
    
    // Covariance output (flattened)
    ssSetOutputPortWidth(S, 1, stateDim * stateDim);
    ssSetOutputPortDataType(S, 1, SS_DOUBLE);
    
    // Sample time: inherited
    ssSetNumSampleTimes(S, 1);
    
    // Need to save EKF state between time steps
    ssSetNumPWork(S, 1);  // for the EKF pointer
    
    // No other states needed
    ssSetNumModes(S, 0);
    ssSetNumNonsampledZCs(S, 0);
    
    // Specify simulation options
    ssSetOptions(S, 0);
}

// Function to initialize sample times
static void mdlInitializeSampleTimes(SimStruct *S)
{
    // Set sample time from parameter
    real_T dt = *mxGetPr(ssGetSFcnParam(S, DT));
    ssSetSampleTime(S, 0, dt);
    ssSetOffsetTime(S, 0, 0.0);
}

// Function to start
#define MDL_START
#if defined(MDL_START)
static void mdlStart(SimStruct *S)
{
    // Get dimensions from parameters
    int stateDim = static_cast<int>(*mxGetPr(ssGetSFcnParam(S, STATE_DIM)));
    int measDim = static_cast<int>(*mxGetPr(ssGetSFcnParam(S, MEAS_DIM)));
    int ctrlDim = static_cast<int>(*mxGetPr(ssGetSFcnParam(S, CTRL_DIM)));
    
    // Create EKF instance
    EKF* ekf = new EKF(stateDim, measDim, ctrlDim);
    
    // Get initial state parameter - [v_x, v_y, γ, Fy_fl, Fy_fr, Fy_rl, Fy_rr]
    double* initialState = mxGetPr(ssGetSFcnParam(S, INITIAL_STATE));
    Eigen::VectorXd x0(stateDim);
    for (int i = 0; i < stateDim; i++) {
        x0(i) = initialState[i];
    }
    
    // Get initial covariance parameter
    double* initialCov = mxGetPr(ssGetSFcnParam(S, INITIAL_COV));
    Eigen::MatrixXd P0(stateDim, stateDim);
    for (int i = 0; i < stateDim; i++) {
        for (int j = 0; j < stateDim; j++) {
            P0(i, j) = initialCov[i + j*stateDim];
        }
    }
    
    // Get process noise covariance parameter
    double* procNoise = mxGetPr(ssGetSFcnParam(S, PROCESS_NOISE_COV));
    Eigen::MatrixXd Q(stateDim, stateDim);
    for (int i = 0; i < stateDim; i++) {
        for (int j = 0; j < stateDim; j++) {
            Q(i, j) = procNoise[i + j*stateDim];
        }
    }
    
    // Get measurement noise covariance parameter
    double* measNoise = mxGetPr(ssGetSFcnParam(S, MEAS_NOISE_COV));
    Eigen::MatrixXd R(measDim, measDim);
    for (int i = 0; i < measDim; i++) {
        for (int j = 0; j < measDim; j++) {
            R(i, j) = measNoise[i + j*measDim];
        }
    }
    
    // Set up the EKF
    ekf->setInitialState(x0, P0);
    ekf->setProcessNoise(Q);
    ekf->setMeasurementNoise(R);
    
    // Define vehicle parameters (typical values)
    double m = 1500.0;      // Vehicle mass (kg)
    double Iz = 2500.0;     // Yaw moment of inertia (kg*m^2)
    double lf = 1.2;        // Distance from CG to front axle (m)
    double lr = 1.4;        // Distance from CG to rear axle (m)
    double track_f = 1.6;   // Front track width (m)
    double track_r = 1.6;   // Rear track width (m)
    double Cf = 50000.0;    // Front cornering stiffness (N/rad)
    double Cr = 50000.0;    // Rear cornering stiffness (N/rad)
    double tau_fy = 0.05;   // Lateral force time constant (s)
    
    // Define state transition function for extended vehicle model with tire forces
    // State vector [v_x, v_y, γ, Fy_fl, Fy_fr, Fy_rl, Fy_rr] where:
    // - v_x: Longitudinal velocity
    // - v_y: Lateral velocity
    // - γ: Yaw rate
    // - Fy_fl, Fy_fr, Fy_rl, Fy_rr: Lateral tire forces
    // Control vector [δ, Fx_fl, Fx_fr, Fx_rl, Fx_rr] where:
    // - δ: Steering angle
    // - Fx_fl, Fx_fr, Fx_rl, Fx_rr: Longitudinal tire forces
    ekf->setStateTransitionFunction([m, Iz, Cf, Cr, lf, lr, track_f, track_r, tau_fy](const Eigen::VectorXd& x, const Eigen::VectorXd& u, double dt) {
        // Extract state variables
        double vx = x(0);       // Longitudinal velocity
        double vy = x(1);       // Lateral velocity
        double gamma = x(2);    // Yaw rate
        double Fy_fl = x(3);    // Front-left lateral force
        double Fy_fr = x(4);    // Front-right lateral force
        double Fy_rl = x(5);    // Rear-left lateral force
        double Fy_rr = x(6);    // Rear-right lateral force
        
        // Extract control inputs
        double delta = u.size() > 0 ? u(0) : 0.0;    // Steering angle
        double Fx_fl = u.size() > 1 ? u(1) : 0.0;    // Front-left longitudinal force
        double Fx_fr = u.size() > 2 ? u(2) : 0.0;    // Front-right longitudinal force
        double Fx_rl = u.size() > 3 ? u(3) : 0.0;    // Rear-left longitudinal force
        double Fx_rr = u.size() > 4 ? u(4) : 0.0;    // Rear-right longitudinal force
        
        Eigen::VectorXd next_x = x;
        
        // Calculate wheel positions and velocities in vehicle coordinates
        // Front-left wheel
        double v_wheel_x_fl = vx - gamma * track_f/2;
        double v_wheel_y_fl = vy + gamma * lf;
        
        // Front-right wheel
        double v_wheel_x_fr = vx + gamma * track_f/2;
        double v_wheel_y_fr = vy + gamma * lf;
        
        // Rear-left wheel
        double v_wheel_x_rl = vx - gamma * track_r/2;
        double v_wheel_y_rl = vy - gamma * lr;
        
        // Rear-right wheel
        double v_wheel_x_rr = vx + gamma * track_r/2;
        double v_wheel_y_rr = vy - gamma * lr;
        
        // Transform front wheel velocities to tire coordinates (considering steering)
        double cos_delta = cos(delta);
        double sin_delta = sin(delta);
        
        double v_tire_long_fl = v_wheel_x_fl * cos_delta + v_wheel_y_fl * sin_delta;
        double v_tire_lat_fl = -v_wheel_x_fl * sin_delta + v_wheel_y_fl * cos_delta;
        
        double v_tire_long_fr = v_wheel_x_fr * cos_delta + v_wheel_y_fr * sin_delta;
        double v_tire_lat_fr = -v_wheel_x_fr * sin_delta + v_wheel_y_fr * cos_delta;
        
        // Rear wheels (no steering)
        double v_tire_long_rl = v_wheel_x_rl;
        double v_tire_lat_rl = v_wheel_y_rl;
        
        double v_tire_long_rr = v_wheel_x_rr;
        double v_tire_lat_rr = v_wheel_y_rr;
        
        // Calculate tire slip angles
        double alpha_fl = atan2(v_tire_lat_fl, fabs(v_tire_long_fl) + 1e-6);
        double alpha_fr = atan2(v_tire_lat_fr, fabs(v_tire_long_fr) + 1e-6);
        double alpha_rl = atan2(v_tire_lat_rl, fabs(v_tire_long_rl) + 1e-6);
        double alpha_rr = atan2(v_tire_lat_rr, fabs(v_tire_long_rr) + 1e-6);
        
        // Calculate desired lateral forces based on tire model
        double Fy_fl_desired = -Cf * alpha_fl;
        double Fy_fr_desired = -Cf * alpha_fr;
        double Fy_rl_desired = -Cr * alpha_rl;
        double Fy_rr_desired = -Cr * alpha_rr;
        
        // Transform tire forces to vehicle coordinates
        double Fx_veh_fl = Fx_fl * cos_delta - Fy_fl * sin_delta;
        double Fy_veh_fl = Fx_fl * sin_delta + Fy_fl * cos_delta;
        
        double Fx_veh_fr = Fx_fr * cos_delta - Fy_fr * sin_delta;
        double Fy_veh_fr = Fx_fr * sin_delta + Fy_fr * cos_delta;
        
        double Fx_veh_rl = Fx_rl;
        double Fy_veh_rl = Fy_rl;
        
        double Fx_veh_rr = Fx_rr;
        double Fy_veh_rr = Fy_rr;
        
        // Calculate total forces and moments
        double Fx_total = Fx_veh_fl + Fx_veh_fr + Fx_veh_rl + Fx_veh_rr;
        double Fy_total = Fy_veh_fl + Fy_veh_fr + Fy_veh_rl + Fy_veh_rr;
        
        double Mz_total = (Fx_veh_fl * (-track_f/2) - Fy_veh_fl * lf) +
                          (Fx_veh_fr * (track_f/2) - Fy_veh_fr * lf) +
                          (Fx_veh_rl * (-track_r/2) - Fy_veh_rl * (-lr)) +
                          (Fx_veh_rr * (track_r/2) - Fy_veh_rr * (-lr));
        
        // Update vehicle states using dynamics equations
        // v̇_x = (Fx_total + m*v_y*γ) / m
        next_x(0) += dt * (Fx_total / m + vy * gamma);
        
        // v̇_y = (Fy_total - m*v_x*γ) / m
        next_x(1) += dt * (Fy_total / m - vx * gamma);
        
        // γ̇ = Mz_total / Iz
        next_x(2) += dt * (Mz_total / Iz);
        
        // Update tire lateral forces using first-order dynamics
        // Ḟy_i = (Fy_desired - Fy_i) / tau_fy
        next_x(3) += dt * (Fy_fl_desired - Fy_fl) / tau_fy;  // Fy_fl
        next_x(4) += dt * (Fy_fr_desired - Fy_fr) / tau_fy;  // Fy_fr
        next_x(5) += dt * (Fy_rl_desired - Fy_rl) / tau_fy;  // Fy_rl
        next_x(6) += dt * (Fy_rr_desired - Fy_rr) / tau_fy;  // Fy_rr
        
        return next_x;
    });
    
    // Define state Jacobian function for extended vehicle model
    ekf->setStateJacobianFunction([m, Iz, Cf, Cr, lf, lr, track_f, track_r, tau_fy](const Eigen::VectorXd& x, const Eigen::VectorXd& u, double dt) {
        Eigen::MatrixXd J = Eigen::MatrixXd::Identity(x.size(), x.size());
        
        // Extract state variables
        double vx = x(0);
        double vy = x(1);
        double gamma = x(2);
        double Fy_fl = x(3);
        double Fy_fr = x(4);
        double Fy_rl = x(5);
        double Fy_rr = x(6);
        
        // Extract control inputs
        double delta = u.size() > 0 ? u(0) : 0.0;
        double cos_delta = cos(delta);
        double sin_delta = sin(delta);
        
        // Jacobian elements for vehicle dynamics (first 3 equations)
        // ∂(v̇_x)/∂v_y = γ
        J(0, 1) += dt * gamma;
        
        // ∂(v̇_x)/∂γ = v_y
        J(0, 2) += dt * vy;
        
        // ∂(v̇_x)/∂Fy_i terms from force transformation
        J(0, 3) += dt * (-sin_delta) / m;  // Front-left
        J(0, 4) += dt * (-sin_delta) / m;  // Front-right
        // Rear wheels don't contribute to Fx in this simplified model
        
        // ∂(v̇_y)/∂v_x = -γ
        J(1, 0) += -dt * gamma;
        
        // ∂(v̇_y)/∂γ = -v_x
        J(1, 2) += -dt * vx;
        
        // ∂(v̇_y)/∂Fy_i terms
        J(1, 3) += dt * cos_delta / m;     // Front-left
        J(1, 4) += dt * cos_delta / m;     // Front-right
        J(1, 5) += dt * 1.0 / m;           // Rear-left
        J(1, 6) += dt * 1.0 / m;           // Rear-right
        
        // ∂(γ̇)/∂Fy_i terms for yaw moment
        J(2, 3) += dt * (-lf * cos_delta + track_f/2 * sin_delta) / Iz;  // Front-left
        J(2, 4) += dt * (-lf * cos_delta - track_f/2 * sin_delta) / Iz;  // Front-right
        J(2, 5) += dt * lr / Iz;                                          // Rear-left
        J(2, 6) += dt * lr / Iz;                                          // Rear-right
        
        // Jacobian elements for tire force dynamics (equations 4-7)
        // ∂(Ḟy_i)/∂Fy_i = -1/tau_fy (diagonal terms already set by identity matrix)
        J(3, 3) += -dt / tau_fy;
        J(4, 4) += -dt / tau_fy;
        J(5, 5) += -dt / tau_fy;
        J(6, 6) += -dt / tau_fy;
        
        // ∂(Ḟy_i)/∂(v_x, v_y, γ) terms from slip angle derivatives
        // These are complex derivatives that would require detailed slip angle Jacobians
        // For now, using simplified approximations
        
        return J;
    });
    
    // Define measurement function
    // Maps state [v_x, v_y, γ, Fy_fl, Fy_fr, Fy_rl, Fy_rr] to measurements [ax, ay, γ, v_fl, v_fr, v_rl, v_rr]
    ekf->setMeasurementFunction([m, Iz, Cf, Cr, lf, lr, track_f, track_r](const Eigen::VectorXd& x, const Eigen::VectorXd& u) {
        // Extract state variables
        double vx = x(0);
        double vy = x(1);
        double gamma = x(2);
        double Fy_fl = x(3);
        double Fy_fr = x(4);
        double Fy_rl = x(5);
        double Fy_rr = x(6);
        
        // Extract control inputs
        double delta = u.size() > 0 ? u(0) : 0.0;
        double Fx_fl = u.size() > 1 ? u(1) : 0.0;
        double Fx_fr = u.size() > 2 ? u(2) : 0.0;
        double Fx_rl = u.size() > 3 ? u(3) : 0.0;
        double Fx_rr = u.size() > 4 ? u(4) : 0.0;
        
        // Create measurement vector (size should match measDim)
        Eigen::VectorXd z(7);  // [ax, ay, gamma, v_fl, v_fr, v_rl, v_rr]
        
        double cos_delta = cos(delta);
        double sin_delta = sin(delta);
        
        // Transform tire forces to vehicle coordinates
        double Fx_veh_fl = Fx_fl * cos_delta - Fy_fl * sin_delta;
        double Fy_veh_fl = Fx_fl * sin_delta + Fy_fl * cos_delta;
        
        double Fx_veh_fr = Fx_fr * cos_delta - Fy_fr * sin_delta;
        double Fy_veh_fr = Fx_fr * sin_delta + Fy_fr * cos_delta;
        
        double Fx_veh_rl = Fx_rl;
        double Fy_veh_rl = Fy_rl;
        
        double Fx_veh_rr = Fx_rr;
        double Fy_veh_rr = Fy_rr;
        
        // Calculate total forces
        double Fx_total = Fx_veh_fl + Fx_veh_fr + Fx_veh_rl + Fx_veh_rr;
        double Fy_total = Fy_veh_fl + Fy_veh_fr + Fy_veh_rl + Fy_veh_rr;
        
        // Calculate measured accelerations
        double ax = Fx_total / m;
        double ay = Fy_total / m;
        
        // Calculate wheel velocities in vehicle coordinates
        double v_wheel_x_fl = vx - gamma * track_f/2;
        double v_wheel_y_fl = vy + gamma * lf;
        
        double v_wheel_x_fr = vx + gamma * track_f/2;
        double v_wheel_y_fr = vy + gamma * lf;
        
        double v_wheel_x_rl = vx - gamma * track_r/2;
        double v_wheel_y_rl = vy - gamma * lr;
        
        double v_wheel_x_rr = vx + gamma * track_r/2;
        double v_wheel_y_rr = vy - gamma * lr;
        
        // Transform front wheel velocities to tire coordinates
        double v_tire_long_fl = v_wheel_x_fl * cos_delta + v_wheel_y_fl * sin_delta;
        double v_tire_lat_fl = -v_wheel_x_fl * sin_delta + v_wheel_y_fl * cos_delta;
        
        double v_tire_long_fr = v_wheel_x_fr * cos_delta + v_wheel_y_fr * sin_delta;
        double v_tire_lat_fr = -v_wheel_x_fr * sin_delta + v_wheel_y_fr * cos_delta;
        
        // Rear wheels (no steering transformation needed)
        double v_tire_long_rl = v_wheel_x_rl;
        double v_tire_lat_rl = v_wheel_y_rl;
        
        double v_tire_long_rr = v_wheel_x_rr;
        double v_tire_lat_rr = v_wheel_y_rr;
        
        // Calculate wheel speeds (magnitude of tire velocity vectors)
        double v_fl = sqrt(v_tire_long_fl*v_tire_long_fl + v_tire_lat_fl*v_tire_lat_fl);
        double v_fr = sqrt(v_tire_long_fr*v_tire_long_fr + v_tire_lat_fr*v_tire_lat_fr);
        double v_rl = sqrt(v_tire_long_rl*v_tire_long_rl + v_tire_lat_rl*v_tire_lat_rl);
        double v_rr = sqrt(v_tire_long_rr*v_tire_long_rr + v_tire_lat_rr*v_tire_lat_rr);
        
        // Populate measurement vector
        z << ax, ay, gamma, v_fl, v_fr, v_rl, v_rr;
        
        return z;
    });
    
    // Define measurement Jacobian function
    ekf->setMeasurementJacobianFunction([m, Iz, Cf, Cr, lf, lr, track_f, track_r](const Eigen::VectorXd& x, const Eigen::VectorXd& u) {
        // Create Jacobian matrix H mapping from state to measurement
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(7, 7);  // 7 measurements, 7 states
        
        // Extract state variables
        double vx = x(0);
        double vy = x(1);
        double gamma = x(2);
        double Fy_fl = x(3);
        double Fy_fr = x(4);
        double Fy_rl = x(5);
        double Fy_rr = x(6);
        
        // Extract control inputs
        double delta = u.size() > 0 ? u(0) : 0.0;
        double cos_delta = cos(delta);
        double sin_delta = sin(delta);
        
        // Acceleration measurement derivatives
        // ∂(ax)/∂Fy_i terms from force transformation
        H(0, 3) = -sin_delta / m;  // ∂(ax)/∂Fy_fl
        H(0, 4) = -sin_delta / m;  // ∂(ax)/∂Fy_fr
        // Rear wheels don't contribute to ax in this model
        
        // ∂(ay)/∂Fy_i terms
        H(1, 3) = cos_delta / m;   // ∂(ay)/∂Fy_fl
        H(1, 4) = cos_delta / m;   // ∂(ay)/∂Fy_fr
        H(1, 5) = 1.0 / m;         // ∂(ay)/∂Fy_rl
        H(1, 6) = 1.0 / m;         // ∂(ay)/∂Fy_rr
        
        // Yaw rate measurement (direct observation)
        // ∂(γ_meas)/∂γ = 1
        H(2, 2) = 1.0;
        
        // Wheel speed measurements derivatives
        // Calculate current wheel velocities for normalization
        double v_wheel_x_fl = vx - gamma * track_f/2;
        double v_wheel_y_fl = vy + gamma * lf;
        double v_tire_long_fl = v_wheel_x_fl * cos_delta + v_wheel_y_fl * sin_delta;
        double v_tire_lat_fl = -v_wheel_x_fl * sin_delta + v_wheel_y_fl * cos_delta;
        double v_fl_norm = sqrt(v_tire_long_fl*v_tire_long_fl + v_tire_lat_fl*v_tire_lat_fl) + 1e-6;
        
        double v_wheel_x_fr = vx + gamma * track_f/2;
        double v_wheel_y_fr = vy + gamma * lf;
        double v_tire_long_fr = v_wheel_x_fr * cos_delta + v_wheel_y_fr * sin_delta;
        double v_tire_lat_fr = -v_wheel_x_fr * sin_delta + v_wheel_y_fr * cos_delta;
        double v_fr_norm = sqrt(v_tire_long_fr*v_tire_long_fr + v_tire_lat_fr*v_tire_lat_fr) + 1e-6;
        
        double v_wheel_x_rl = vx - gamma * track_r/2;
        double v_wheel_y_rl = vy - gamma * lr;
        double v_rl_norm = sqrt(v_wheel_x_rl*v_wheel_x_rl + v_wheel_y_rl*v_wheel_y_rl) + 1e-6;
        
        double v_wheel_x_rr = vx + gamma * track_r/2;
        double v_wheel_y_rr = vy - gamma * lr;
        double v_rr_norm = sqrt(v_wheel_x_rr*v_wheel_x_rr + v_wheel_y_rr*v_wheel_y_rr) + 1e-6;
        
        // Front-left wheel speed derivatives
        H(3, 0) = (cos_delta * v_tire_long_fl - sin_delta * v_tire_lat_fl) / v_fl_norm;  // ∂v_fl/∂vx
        H(3, 1) = (sin_delta * v_tire_long_fl + cos_delta * v_tire_lat_fl) / v_fl_norm;  // ∂v_fl/∂vy
        H(3, 2) = ((-track_f/2 * cos_delta + lf * sin_delta) * v_tire_long_fl + 
                   (track_f/2 * sin_delta + lf * cos_delta) * v_tire_lat_fl) / v_fl_norm;  // ∂v_fl/∂γ
        
        // Front-right wheel speed derivatives
        H(4, 0) = (cos_delta * v_tire_long_fr - sin_delta * v_tire_lat_fr) / v_fr_norm;  // ∂v_fr/∂vx
        H(4, 1) = (sin_delta * v_tire_long_fr + cos_delta * v_tire_lat_fr) / v_fr_norm;  // ∂v_fr/∂vy
        H(4, 2) = ((track_f/2 * cos_delta + lf * sin_delta) * v_tire_long_fr + 
                   (-track_f/2 * sin_delta + lf * cos_delta) * v_tire_lat_fr) / v_fr_norm;  // ∂v_fr/∂γ
        
        // Rear-left wheel speed derivatives
        H(5, 0) = v_wheel_x_rl / v_rl_norm;  // ∂v_rl/∂vx
        H(5, 1) = v_wheel_y_rl / v_rl_norm;  // ∂v_rl/∂vy
        H(5, 2) = (-track_r/2 * v_wheel_x_rl - lr * v_wheel_y_rl) / v_rl_norm;  // ∂v_rl/∂γ
        
        // Rear-right wheel speed derivatives
        H(6, 0) = v_wheel_x_rr / v_rr_norm;  // ∂v_rr/∂vx
        H(6, 1) = v_wheel_y_rr / v_rr_norm;  // ∂v_rr/∂vy
        H(6, 2) = (track_r/2 * v_wheel_x_rr - lr * v_wheel_y_rr) / v_rr_norm;  // ∂v_rr/∂γ
        
        return H;
    });
    
    // Store pointer to EKF instance
    ssSetPWorkValue(S, 0, ekf);
}
#endif

// Function to compute outputs
static void mdlOutputs(SimStruct *S, int_T tid)
{
    // Get EKF pointer
    EKF* ekf = static_cast<EKF*>(ssGetPWorkValue(S, 0));
    
    // Get dimensions
    int stateDim = static_cast<int>(*mxGetPr(ssGetSFcnParam(S, STATE_DIM)));
    int ctrlDim = static_cast<int>(*mxGetPr(ssGetSFcnParam(S, CTRL_DIM)));
    real_T dt = *mxGetPr(ssGetSFcnParam(S, DT));
    
    // Get input ports (measurements and control if available)
    const real_T *measurement = ssGetInputPortRealSignal(S, 0);
    const real_T *control = ctrlDim > 0 ? ssGetInputPortRealSignal(S, 1) : nullptr;
    
    // Get output ports for state and covariance
    real_T *stateOut = ssGetOutputPortRealSignal(S, 0);
    real_T *covOut = ssGetOutputPortRealSignal(S, 1);
    
    // Convert measurement to Eigen vector
    int measDim = ssGetInputPortWidth(S, 0);
    Eigen::VectorXd z(measDim);
    for (int i = 0; i < measDim; i++) {
        z(i) = measurement[i];
    }
    
    // Convert control to Eigen vector if available
    Eigen::VectorXd u(ctrlDim);
    if (control && ctrlDim > 0) {
        for (int i = 0; i < ctrlDim; i++) {
            u(i) = control[i];
        }
    }
    
    // Perform EKF prediction and update steps
    if (ctrlDim > 0) {
        ekf->predict(dt, u);
    } else {
        ekf->predict(dt);
    }
    ekf->update(z, u);
    
    // Get state and covariance estimates
    Eigen::VectorXd state = ekf->getState();
    Eigen::MatrixXd cov = ekf->getCovariance();
    
    // Output state [v_x, v_y, γ, Fy_fl, Fy_fr, Fy_rl, Fy_rr]
    for (int i = 0; i < stateDim; i++) {
        stateOut[i] = state(i);
    }
    
    // Output covariance (flattened matrix)
    for (int i = 0; i < stateDim; i++) {
        for (int j = 0; j < stateDim; j++) {
            covOut[i + j*stateDim] = cov(i, j);
        }
    }
}

// Function to terminate
static void mdlTerminate(SimStruct *S)
{
    // Get and delete EKF instance to prevent memory leaks
    EKF* ekf = static_cast<EKF*>(ssGetPWorkValue(S, 0));
    if (ekf) {
        delete ekf;
        ssSetPWorkValue(S, 0, nullptr);
    }
}

// Required S-function trailer
#ifdef MATLAB_MEX_FILE
#include "simulink.c"
#else
#include "cg_sfun.h"
#endif
