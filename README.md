# Extended Kalman Filter Implementation

This project provides a flexible C++ implementation of the Extended Kalman Filter (EKF) algorithm for nonlinear state estimation. The implementation is designed to be easy to use while allowing customization for different models and problems.

## What is Extended Kalman Filter?

The Extended Kalman Filter is an extension of the Kalman Filter for nonlinear systems. It linearizes the nonlinear models around the current estimate using Taylor series expansion (first-order approximation). The EKF consists of two main steps:

1. **Prediction Step**: Predicts the state and its uncertainty based on the system model.
2. **Update Step**: Corrects the prediction using actual measurements.

## Requirements

- C++ compiler with C++11 support
- Eigen library (for linear algebra operations)

## Code Structure

- `ekf.h` - Header file defining the EKF class interface
- `ekf.cpp` - Implementation of the EKF class
- `main.cpp` - Example usage of the EKF class

## How to Use

### 1. Include the necessary header

```cpp
#include "ekf.h"
```

### 2. Create an EKF instance

```cpp
// stateSize: dimensions of your state vector
// measureSize: dimensions of your measurement vector
// controlSize: dimensions of your control vector (optional)
EKF ekf(stateSize, measureSize, controlSize);
```

### 3. Set initial state and covariance

```cpp
Eigen::VectorXd x0 = ...;  // Initial state
Eigen::MatrixXd P0 = ...;  // Initial covariance
ekf.setInitialState(x0, P0);
```

### 4. Set noise covariance matrices

```cpp
Eigen::MatrixXd Q = ...;  // Process noise covariance
Eigen::MatrixXd R = ...;  // Measurement noise covariance
ekf.setProcessNoise(Q);
ekf.setMeasurementNoise(R);
```

### 5. Define model functions

Set the state transition function (how the state evolves):
```cpp
ekf.setStateTransitionFunction([](const Eigen::VectorXd& x, double dt) {
    // Implement your state evolution here
    // Return the predicted next state
});
```

Set the Jacobian of state transition:
```cpp
ekf.setStateJacobianFunction([](const Eigen::VectorXd& x, double dt) {
    // Implement the Jacobian matrix computation here
    // Return the Jacobian matrix
});
```

Set the measurement function (how the state translates to measurements):
```cpp
ekf.setMeasurementFunction([](const Eigen::VectorXd& x) {
    // Implement your measurement model here
    // Return the predicted measurement
});
```

Set the Jacobian of measurement:
```cpp
ekf.setMeasurementJacobianFunction([](const Eigen::VectorXd& x) {
    // Implement the measurement Jacobian computation
    // Return the Jacobian matrix
});
```

### 6. Filtering Loop

In your main loop:

```cpp
double dt = 0.1;  // Time step
Eigen::VectorXd z = ...;  // Current measurement

// Prediction step
ekf.predict(dt);

// Update step
ekf.update(z);

// Get current state estimate
Eigen::VectorXd state = ekf.getState();
```

## Example

See `main.cpp` for a complete example of tracking an object moving with constant velocity. The example demonstrates:

1. Setting up the EKF
2. Defining model functions
3. Running the prediction-update loop
4. Displaying results

## Customizing for Different Models

To adapt this EKF implementation to your specific problem:

1. Define the appropriate state vector structure
2. Implement the state transition and measurement models
3. Derive the corresponding Jacobians
4. Set appropriate noise covariance matrices

## Performance Considerations

- The EKF works well when the nonlinearities are mild
- For highly nonlinear systems, consider using Unscented Kalman Filter or Particle Filters
- Numerical stability might be an issue; ensure matrix inversions are handled properly

## License

This project is available under the MIT license.
