/*
 * S-Function wrapper for Extended Kalman Filter library
 *
 * This file provides a Simulink S-Function interface to the EKF C++ library.
 */

#define S_FUNCTION_NAME ekf_sfun
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include "ekf.h"
#include <vector>
#include <memory>

// Define parameters indices
enum ParamIndex {
    STATE_DIM = 0,
    MEAS_DIM,
    CTRL_DIM,
    DT,
    INITIAL_STATE,
    INITIAL_COV,
    PROCESS_NOISE_COV,
    MEAS_NOISE_COV,
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
    
    // Measurement input
    ssSetInputPortWidth(S, 0, measDim);
    ssSetInputPortDataType(S, 0, SS_DOUBLE);
    ssSetInputPortDirectFeedThrough(S, 0, 1);
    ssSetInputPortRequiredContiguous(S, 0, 1);
    
    // Control input (if needed)
    if (ctrlDim > 0) {
        ssSetInputPortWidth(S, 1, ctrlDim);
        ssSetInputPortDataType(S, 1, SS_DOUBLE);
        ssSetInputPortDirectFeedThrough(S, 1, 1);
        ssSetInputPortRequiredContiguous(S, 1, 1);
    }
    
    // Configure outputs
    if (!ssSetNumOutputPorts(S, 2)) return;
    
    // State estimate output
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
    
    // Get initial state parameter
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
    
    // Define state transition function - Set to linear motion model by default
    // Users should modify this to match their specific system model
    ekf->setStateTransitionFunction([](const Eigen::VectorXd& x, double dt) {
        int stateSize = x.size();
        Eigen::VectorXd newState = x;
        
        // For position-velocity models, update positions based on velocities
        // This is a simple example - replace with your specific model
        if (stateSize >= 4) {  // Assuming at least 2D position-velocity
            newState(0) += x(2) * dt;  // x += vx * dt
            newState(1) += x(3) * dt;  // y += vy * dt
        }
        
        return newState;
    });
    
    // Define state Jacobian function - Set to linear motion Jacobian by default
    ekf->setStateJacobianFunction([](const Eigen::VectorXd& x, double dt) {
        int stateSize = x.size();
        Eigen::MatrixXd F = Eigen::MatrixXd::Identity(stateSize, stateSize);
        
        // For position-velocity models, set Jacobian accordingly
        if (stateSize >= 4) {  // Assuming at least 2D position-velocity
            F(0, 2) = dt;  // dx/dvx = dt
            F(1, 3) = dt;  // dy/dvy = dt
        }
        
        return F;
    });
    
    // Define measurement function - Set to direct observation of positions by default
    ekf->setMeasurementFunction([measDim](const Eigen::VectorXd& x) {
        Eigen::VectorXd z(measDim);
        
        // By default, assume we're measuring the first measDim states directly
        // Replace with your specific measurement model
        for (int i = 0; i < measDim; i++) {
            z(i) = x(i);
        }
        
        return z;
    });
    
    // Define measurement Jacobian function
    ekf->setMeasurementJacobianFunction([measDim, stateDim](const Eigen::VectorXd& x) {
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(measDim, stateDim);
        
        // By default, set direct observation Jacobian
        // Replace with your specific Jacobian
        for (int i = 0; i < measDim && i < stateDim; i++) {
            H(i, i) = 1.0;
        }
        
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
    
    // Get input ports
    const real_T *measurement = ssGetInputPortRealSignal(S, 0);
    const real_T *control = ctrlDim > 0 ? ssGetInputPortRealSignal(S, 1) : nullptr;
    
    // Get output ports
    real_T *stateOut = ssGetOutputPortRealSignal(S, 0);
    real_T *covOut = ssGetOutputPortRealSignal(S, 1);
    
    // Convert measurement to Eigen
    int measDim = ssGetInputPortWidth(S, 0);
    Eigen::VectorXd z(measDim);
    for (int i = 0; i < measDim; i++) {
        z(i) = measurement[i];
    }
    
    // Perform EKF prediction and update
    ekf->predict(dt);
    ekf->update(z);
    
    // Get state and covariance
    Eigen::VectorXd state = ekf->getState();
    Eigen::MatrixXd cov = ekf->getCovariance();
    
    // Output state
    for (int i = 0; i < stateDim; i++) {
        stateOut[i] = state(i);
    }
    
    // Output covariance (flattened)
    for (int i = 0; i < stateDim; i++) {
        for (int j = 0; j < stateDim; j++) {
            covOut[i + j*stateDim] = cov(i, j);
        }
    }
}

// Function to terminate
static void mdlTerminate(SimStruct *S)
{
    // Get and delete EKF instance
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
