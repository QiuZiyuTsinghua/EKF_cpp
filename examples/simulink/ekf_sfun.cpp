/*
 * S-Function wrapper for Extended Kalman Filter library
 * Code generation compatible version (inlined S-Function)
 */

#define S_FUNCTION_NAME ekf_sfun
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include <math.h>
#include <string.h>

// Fixed dimensions for code generation
#define STATE_DIM 3      // [v_x, v_y, gamma]
#define MEAS_DIM 7       // [ax, ay, gamma, v_fl, v_fr, v_rl, v_rr]
#define CTRL_DIM 1       // [delta] (steering angle)

// Vehicle parameters (constants for code generation)
#define VEHICLE_MASS 2200.0
#define VEHICLE_IZ 3000.0
#define VEHICLE_LF 1.015
#define VEHICLE_LR 1.795
#define VEHICLE_TRACK 1.9
#define TIRE_CF 50000.0
#define TIRE_CR 50000.0

// Define parameters indices
enum ParamIndex {
    DT_PARAM = 0,         // Time step
    INITIAL_STATE_PARAM,  // Initial state values
    INITIAL_COV_PARAM,    // Initial state covariance
    PROCESS_NOISE_PARAM,  // Process noise covariance
    MEAS_NOISE_PARAM,     // Measurement noise covariance
    NUM_PARAMS
};

// EKF state structure for static memory
typedef struct {
    real_T x[STATE_DIM];                    // State vector
    real_T P[STATE_DIM * STATE_DIM];        // Covariance matrix (flattened)
    real_T Q[STATE_DIM * STATE_DIM];        // Process noise (flattened)
    real_T R[MEAS_DIM * MEAS_DIM];          // Measurement noise (flattened)
    boolean_T initialized;                   // Initialization flag
} EKFState;

// Matrix operations for code generation compatibility
static void matrix_multiply_3x3(const real_T* A, const real_T* B, real_T* C) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            C[i*3 + j] = 0.0;
            for (int k = 0; k < 3; k++) {
                C[i*3 + j] += A[i*3 + k] * B[k*3 + j];
            }
        }
    }
}

static void matrix_add_3x3(const real_T* A, const real_T* B, real_T* C) {
    for (int i = 0; i < 9; i++) {
        C[i] = A[i] + B[i];
    }
}

static void matrix_transpose_3x3(const real_T* A, real_T* AT) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            AT[j*3 + i] = A[i*3 + j];
        }
    }
}

// Vehicle dynamics prediction function
static void predict_state(const real_T* x, real_T dt, real_T* x_next) {
    real_T vx = x[0];
    real_T vy = x[1];
    real_T gamma = x[2];
    
    // Calculate tire slip angles
    real_T beta = atan2(vy, vx);
    real_T alpha_f = beta - VEHICLE_LF * gamma / vx;
    real_T alpha_r = beta + VEHICLE_LR * gamma / vx;
    
    // Calculate tire forces
    real_T Fyf = -TIRE_CF * alpha_f;
    real_T Fyr = -TIRE_CR * alpha_r;
    
    // Calculate accelerations
    real_T ax = 0.0;
    real_T ay = (Fyf + Fyr) / VEHICLE_MASS;
    real_T Mz = Fyf * VEHICLE_LF - Fyr * VEHICLE_LR;
    
    // Update state
    x_next[0] = vx + dt * (ax + vy * gamma);
    x_next[1] = vy + dt * (ay - vx * gamma);
    x_next[2] = gamma + dt * (Mz / VEHICLE_IZ);
}

// State Jacobian calculation
static void calculate_state_jacobian(const real_T* x, real_T dt, real_T* F) {
    // Initialize as identity matrix
    memset(F, 0, 9 * sizeof(real_T));
    F[0] = F[4] = F[8] = 1.0;
    
    real_T vx = x[0];
    real_T vy = x[1];
    real_T gamma = x[2];
    
    // Fill Jacobian elements
    F[1] = dt * gamma;     // df1/dx2
    F[2] = dt * vy;        // df1/dx3
    F[3] = -dt * gamma;    // df2/dx1
    F[5] = -dt * vx;       // df2/dx3
}

// Measurement function
static void measurement_function(const real_T* x, real_T* z) {
    real_T vx = x[0];
    real_T vy = x[1];
    real_T gamma = x[2];
    
    // Calculate measurements
    z[0] = vy * gamma;                           // ax (simplified)
    z[1] = -vx * gamma;                          // ay (simplified)
    z[2] = gamma;                                // gamma (direct)
    z[3] = vx - (gamma * VEHICLE_TRACK/2);       // v_fl
    z[4] = vx + (gamma * VEHICLE_TRACK/2);       // v_fr
    z[5] = vx - (gamma * VEHICLE_TRACK/2);       // v_rl
    z[6] = vx + (gamma * VEHICLE_TRACK/2);       // v_rr
}

// Measurement Jacobian calculation
static void calculate_measurement_jacobian(const real_T* x, real_T* H) {
    memset(H, 0, MEAS_DIM * STATE_DIM * sizeof(real_T));
    
    real_T vx = x[0];
    real_T vy = x[1];
    real_T gamma = x[2];
    
    // Fill H matrix (7x3)
    H[0*3 + 1] = gamma;                    // dax/dvy
    H[0*3 + 2] = vy;                       // dax/dgamma
    H[1*3 + 0] = -gamma;                   // day/dvx
    H[1*3 + 2] = -vx;                      // day/dgamma
    H[2*3 + 2] = 1.0;                      // dgamma/dgamma
    H[3*3 + 0] = 1.0;                      // dv_fl/dvx
    H[3*3 + 2] = -VEHICLE_TRACK/2;         // dv_fl/dgamma
    H[4*3 + 0] = 1.0;                      // dv_fr/dvx
    H[4*3 + 2] = VEHICLE_TRACK/2;          // dv_fr/dgamma
    H[5*3 + 0] = 1.0;                      // dv_rl/dvx
    H[5*3 + 2] = -VEHICLE_TRACK/2;         // dv_rl/dgamma
    H[6*3 + 0] = 1.0;                      // dv_rr/dvx
    H[6*3 + 2] = VEHICLE_TRACK/2;          // dv_rr/dgamma
}

#define MDL_CHECK_PARAMETERS
#if defined(MDL_CHECK_PARAMETERS) && defined(MATLAB_MEX_FILE)
static void mdlCheckParameters(SimStruct *S)
{
    if (ssGetSFcnParamsCount(S) != NUM_PARAMS) {
        ssSetErrorStatus(S, "Wrong number of parameters. Expected 5.");
        return;
    }
}
#endif

static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, NUM_PARAMS);
    
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        return;
    }
    
    // Set parameters as non-tunable
    for (int i = 0; i < NUM_PARAMS; i++) {
        ssSetSFcnParamNotTunable(S, i);
    }
    
    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);
    
    // Configure inputs
    if (!ssSetNumInputPorts(S, 1)) return;
    ssSetInputPortWidth(S, 0, MEAS_DIM);
    ssSetInputPortDataType(S, 0, SS_DOUBLE);
    ssSetInputPortDirectFeedThrough(S, 0, 1);
    ssSetInputPortRequiredContiguous(S, 0, 1);
    
    // Configure outputs
    if (!ssSetNumOutputPorts(S, 2)) return;
    ssSetOutputPortWidth(S, 0, STATE_DIM);
    ssSetOutputPortDataType(S, 0, SS_DOUBLE);
    ssSetOutputPortWidth(S, 1, STATE_DIM * STATE_DIM);
    ssSetOutputPortDataType(S, 1, SS_DOUBLE);
    
    ssSetNumSampleTimes(S, 1);
    
    // Use DWork for EKF state storage
    if (!ssSetNumDWork(S, 1)) return;
    ssSetDWorkWidth(S, 0, sizeof(EKFState)/sizeof(real_T));
    ssSetDWorkDataType(S, 0, SS_DOUBLE);
}

static void mdlInitializeSampleTimes(SimStruct *S)
{
    real_T dt = *mxGetPr(ssGetSFcnParam(S, DT_PARAM));
    ssSetSampleTime(S, 0, dt);
    ssSetOffsetTime(S, 0, 0.0);
}

#define MDL_START
#if defined(MDL_START)
static void mdlStart(SimStruct *S)
{
    // Get DWork pointer for EKF state
    real_T* dwork = ssGetDWork(S, 0);
    EKFState* ekf = (EKFState*)dwork;
    
    // Initialize from parameters
    real_T* initialState = mxGetPr(ssGetSFcnParam(S, INITIAL_STATE_PARAM));
    real_T* initialCov = mxGetPr(ssGetSFcnParam(S, INITIAL_COV_PARAM));
    real_T* processNoise = mxGetPr(ssGetSFcnParam(S, PROCESS_NOISE_PARAM));
    real_T* measNoise = mxGetPr(ssGetSFcnParam(S, MEAS_NOISE_PARAM));
    
    // Copy initial state
    memcpy(ekf->x, initialState, STATE_DIM * sizeof(real_T));
    
    // Copy matrices (flattened)
    memcpy(ekf->P, initialCov, STATE_DIM * STATE_DIM * sizeof(real_T));
    memcpy(ekf->Q, processNoise, STATE_DIM * STATE_DIM * sizeof(real_T));
    memcpy(ekf->R, measNoise, MEAS_DIM * MEAS_DIM * sizeof(real_T));
    
    ekf->initialized = true;
}
#endif

static void mdlOutputs(SimStruct *S, int_T tid)
{
    // Get DWork pointer
    real_T* dwork = ssGetDWork(S, 0);
    EKFState* ekf = (EKFState*)dwork;
    
    if (!ekf->initialized) {
        return;
    }
    
    // Get inputs and outputs
    const real_T* measurement = ssGetInputPortRealSignal(S, 0);
    real_T* stateOut = ssGetOutputPortRealSignal(S, 0);
    real_T* covOut = ssGetOutputPortRealSignal(S, 1);
    real_T dt = *mxGetPr(ssGetSFcnParam(S, DT_PARAM));
    
    // Temporary arrays for calculations
    real_T x_pred[STATE_DIM];
    real_T F[STATE_DIM * STATE_DIM];
    real_T FT[STATE_DIM * STATE_DIM];
    real_T FP[STATE_DIM * STATE_DIM];
    real_T FPFT[STATE_DIM * STATE_DIM];
    real_T P_pred[STATE_DIM * STATE_DIM];
    
    // Prediction step
    predict_state(ekf->x, dt, x_pred);
    calculate_state_jacobian(ekf->x, dt, F);
    
    // Predict covariance: P = F*P*F' + Q
    matrix_multiply_3x3(F, ekf->P, FP);
    matrix_transpose_3x3(F, FT);
    matrix_multiply_3x3(FP, FT, FPFT);
    matrix_add_3x3(FPFT, ekf->Q, P_pred);
    
    // Update step (simplified for fixed dimensions)
    real_T z_pred[MEAS_DIM];
    real_T H[MEAS_DIM * STATE_DIM];
    real_T y[MEAS_DIM];
    
    measurement_function(x_pred, z_pred);
    calculate_measurement_jacobian(x_pred, H);
    
    // Calculate innovation
    for (int i = 0; i < MEAS_DIM; i++) {
        y[i] = measurement[i] - z_pred[i];
    }
    
    // Simplified Kalman gain calculation (would need proper matrix inversion for full implementation)
    // For now, just copy predicted state
    memcpy(ekf->x, x_pred, STATE_DIM * sizeof(real_T));
    memcpy(ekf->P, P_pred, STATE_DIM * STATE_DIM * sizeof(real_T));
    
    // Output results
    memcpy(stateOut, ekf->x, STATE_DIM * sizeof(real_T));
    memcpy(covOut, ekf->P, STATE_DIM * STATE_DIM * sizeof(real_T));
}

static void mdlTerminate(SimStruct *S)
{
    // No dynamic memory to free
}

#ifdef MATLAB_MEX_FILE
#include "simulink.c"
#else
#include "cg_sfun.h"
#endif
