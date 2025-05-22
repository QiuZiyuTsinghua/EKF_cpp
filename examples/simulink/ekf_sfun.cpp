/*
 * S-Function wrapper for Extended Kalman Filter library
 *
 * This file provides a Simulink S-Function interface to the EKF C++ library
 * for a vehicle velocity estimation model. The state vector is [v_x, v_y, γ],
 * where:
 *   - v_x: Longitudinal velocity
 *   - v_y: Lateral velocity
 *   - γ: Yaw rate (angular velocity around z-axis)
 *
 * This model is designed for vehicle dynamics state estimation.
 */

#define S_FUNCTION_NAME ekf_sfun
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include "ekf.h"
#include <vector>
#include <memory>

// Define parameters indices
enum ParamIndex {
    STATE_DIM = 0,    // Dimension of state vector [v_x, v_y, γ]
    MEAS_DIM,         // Dimension of measurement vector [ax, ay, γ, v_fl, v_fr, v_rl, v_rr]
    CTRL_DIM,         // Dimension of control vector [δ] (steering angle)
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
    
    // Control input (steering angle δ if needed)
    if (ctrlDim > 0) {
        ssSetInputPortWidth(S, 1, ctrlDim);
        ssSetInputPortDataType(S, 1, SS_DOUBLE);
        ssSetInputPortDirectFeedThrough(S, 1, 1);
        ssSetInputPortRequiredContiguous(S, 1, 1);
    }
    
    // Configure outputs
    if (!ssSetNumOutputPorts(S, 2)) return;
    
    // State estimate output - [v_x, v_y, γ]
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
    
    // Get initial state parameter - [v_x, v_y, γ]
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
    double track = 1.6;     // Track width (m)
    double Cf = 50000.0;    // Front cornering stiffness (N/rad)
    double Cr = 50000.0;    // Rear cornering stiffness (N/rad)
    
    // Define state transition function for vehicle velocity model
    // State vector [v_x, v_y, γ] where:
    // - v_x: Longitudinal velocity
    // - v_y: Lateral velocity
    // - γ: Yaw rate
    ekf->setStateTransitionFunction([m, Iz, Cf, Cr, lf, lr](const Eigen::VectorXd& x, double dt) {
        // Extract state variables
        double vx = x(0);    // Longitudinal velocity
        double vy = x(1);    // Lateral velocity
        double gamma = x(2); // Yaw rate
        
        Eigen::VectorXd next_x = x;
        
        // Calculate tire slip angles
        double beta = atan2(vy, vx);  // Vehicle sideslip angle
        
        // Front and rear tire slip angles
        double alpha_f = beta - lf * gamma / vx;  // Front slip angle
        double alpha_r = beta + lr * gamma / vx;  // Rear slip angle
        
        // Calculate tire forces
        double Fyf = -Cf * alpha_f;  // Front lateral force
        double Fyr = -Cr * alpha_r;  // Rear lateral force
        
        // Calculate acceleration components
        double ax = 0.0;  // Assume zero longitudinal acceleration for simplicity
        double ay = (Fyf + Fyr) / m;  // Lateral acceleration
        double Mz = Fyf * lf - Fyr * lr;  // Yaw moment
        
        // Update state using the vehicle dynamics equations:
        // v̇_x = a_x + v_y·γ
        next_x(0) += dt * (ax + vy * gamma);
        
        // v̇_y = a_y - v_x·γ
        next_x(1) += dt * (ay - vx * gamma);
        
        // γ̇ = M_z/I_z
        next_x(2) += dt * (Mz / Iz);
        
        return next_x;
    });
    
    // Define state Jacobian function for vehicle model
    ekf->setStateJacobianFunction([m, Iz, Cf, Cr, lf, lr](const Eigen::VectorXd& x, double dt) {
        Eigen::MatrixXd J = Eigen::MatrixXd::Identity(x.size(), x.size());
        
        // Extract state variables
        double vx = x(0);
        double vy = x(1);
        double gamma = x(2);
        
        // Jacobian elements based on linearized model
        // ∂(v̇_x)/∂v_y = γ
        J(0, 1) = dt * gamma;
        
        // ∂(v̇_x)/∂γ = v_y
        J(0, 2) = dt * vy;
        
        // ∂(v̇_y)/∂v_x = -γ
        J(1, 0) = -dt * gamma;
        
        // ∂(v̇_y)/∂γ = -v_x
        J(1, 2) = -dt * vx;
        
        // Note: Additional Jacobian terms for the tire model derivatives are
        // simplified here. In a complete implementation, these would include
        // the effect of slip angle changes on forces.
        
        return J;
    });
    
    // Define measurement function
    // Maps state [v_x, v_y, γ] to measurements [ax, ay, γ, v_fl, v_fr, v_rl, v_rr]
    ekf->setMeasurementFunction([m, Iz, Cf, Cr, lf, lr, track](const Eigen::VectorXd& x) {
        // Extract state variables
        double vx = x(0);
        double vy = x(1);
        double gamma = x(2);
        
        // Create measurement vector (size should match measDim)
        Eigen::VectorXd z(7);  // [ax, ay, gamma, v_fl, v_fr, v_rl, v_rr]
        
        // Calculate tire slip angles
        double beta = atan2(vy, vx);
        double alpha_f = beta - lf * gamma / vx;
        double alpha_r = beta + lr * gamma / vx;
        
        // Calculate tire forces
        double Fyf = -Cf * alpha_f;
        double Fyr = -Cr * alpha_r;
        
        // Calculate measured accelerations
        double ax = vy * gamma;  // Centripetal component only (simplified)
        double ay = (Fyf + Fyr) / m - vx * gamma;
        
        // Calculate wheel speeds
        double v_fl = vx - (gamma * track/2);  // Front-left wheel
        double v_fr = vx + (gamma * track/2);  // Front-right wheel
        double v_rl = vx - (gamma * track/2);  // Rear-left wheel
        double v_rr = vx + (gamma * track/2);  // Rear-right wheel
        
        // Populate measurement vector
        z << ax, ay, gamma, v_fl, v_fr, v_rl, v_rr;
        
        return z;
    });
    
    // Define measurement Jacobian function
    ekf->setMeasurementJacobianFunction([m, Iz, Cf, Cr, lf, lr, track](const Eigen::VectorXd& x) {
        // Create Jacobian matrix H mapping from state to measurement
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(7, 3);  // 7 measurements, 3 states
        
        // Extract state variables
        double vx = x(0);
        double vy = x(1);
        double gamma = x(2);
        
        // Acceleration measurement derivatives
        // ∂(ax)/∂v_y = γ
        H(0, 1) = gamma;
        
        // ∂(ax)/∂γ = v_y
        H(0, 2) = vy;
        
        // ∂(ay)/∂v_x = -γ
        H(1, 0) = -gamma;
        
        // ∂(ay)/∂γ = -v_x
        H(1, 2) = -vx;
        
        // Yaw rate measurement (direct observation)
        // ∂(γ_meas)/∂γ = 1
        H(2, 2) = 1.0;
        
        // Wheel speed measurements
        // Front-left wheel: v_fl = vx - (gamma * track/2)
        H(3, 0) = 1.0;       // ∂(v_fl)/∂v_x = 1
        H(3, 2) = -track/2;  // ∂(v_fl)/∂γ = -track/2
        
        // Front-right wheel: v_fr = vx + (gamma * track/2)
        H(4, 0) = 1.0;       // ∂(v_fr)/∂v_x = 1
        H(4, 2) = track/2;   // ∂(v_fr)/∂γ = track/2
        
        // Rear-left wheel: v_rl = vx - (gamma * track/2)
        H(5, 0) = 1.0;       // ∂(v_rl)/∂v_x = 1
        H(5, 2) = -track/2;  // ∂(v_rl)/∂γ = -track/2
        
        // Rear-right wheel: v_rr = vx + (gamma * track/2)
        H(6, 0) = 1.0;       // ∂(v_rr)/∂v_x = 1
        H(6, 2) = track/2;   // ∂(v_rr)/∂γ = track/2
        
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
    
    // Perform EKF prediction and update steps
    ekf->predict(dt);
    ekf->update(z);
    
    // Get state and covariance estimates
    Eigen::VectorXd state = ekf->getState();
    Eigen::MatrixXd cov = ekf->getCovariance();
    
    // Output state [v_x, v_y, γ]
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
