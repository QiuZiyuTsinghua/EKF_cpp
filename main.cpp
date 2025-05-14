#include <iostream>
#include <vector>
#include <random>
#include "ekf.h"

int main() {
    std::cout << "Extended Kalman Filter Demonstration" << std::endl;
    
    // Initialize the system
    int stateSize = 4;  // x, y position and velocity
    int measureSize = 2;  // x, y position measurement
    int controlSize = 0;  // No control input in this example
    
    // Create the EKF with dimensions
    EKF ekf(stateSize, measureSize, controlSize);
    
    // Initial state [x, y, vx, vy]
    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(stateSize);
    x0 << 0.0, 0.0, 1.0, 1.0;  // Starting at origin with velocity (1,1)
    
    // Initial state covariance
    Eigen::MatrixXd P0 = Eigen::MatrixXd::Identity(stateSize, stateSize);
    P0 *= 1.0;  // Uncertainty in initial state
    
    // Process noise covariance
    Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(stateSize, stateSize);
    Q.topLeftCorner(2, 2) *= 0.01;  // Position noise
    Q.bottomRightCorner(2, 2) *= 0.1;  // Velocity noise
    
    // Measurement noise covariance
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(measureSize, measureSize);
    R *= 0.1;  // Measurement noise
    
    // Set up the EKF
    ekf.setInitialState(x0, P0);
    ekf.setProcessNoise(Q);
    ekf.setMeasurementNoise(R);
    
    // Set up state transition function (linear in this case)
    ekf.setStateTransitionFunction([](const Eigen::VectorXd& x, double dt) {
        Eigen::MatrixXd F = Eigen::MatrixXd::Identity(x.size(), x.size());
        F(0, 2) = dt;  // x += vx * dt
        F(1, 3) = dt;  // y += vy * dt
        return F * x;
    });
    
    // State Jacobian
    ekf.setStateJacobianFunction([](const Eigen::VectorXd& x, double dt) {
        Eigen::MatrixXd J = Eigen::MatrixXd::Identity(x.size(), x.size());
        J(0, 2) = dt;  // dx/dvx = dt
        J(1, 3) = dt;  // dy/dvy = dt
        return J;
    });
    
    // Measurement function (we measure x and y positions)
    ekf.setMeasurementFunction([](const Eigen::VectorXd& x) {
        Eigen::VectorXd z(2);
        z << x(0), x(1);  // Measure position only
        return z;
    });
    
    // Measurement Jacobian
    ekf.setMeasurementJacobianFunction([](const Eigen::VectorXd& x) {
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2, 4);
        H(0, 0) = 1.0;  // dz1/dx = 1
        H(1, 1) = 1.0;  // dz2/dy = 1
        return H;
    });
    
    // Simulate measurements
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<> noise(0, 0.1);
    double dt = 0.1;  // Time step
    
    // Simulation loop
    for (int i = 0; i < 100; i++) {
        // True state (for simulation)
        double true_x = i * dt;  // Constant velocity motion
        double true_y = i * dt;
        
        // Generate noisy measurement
        Eigen::VectorXd z(2);
        z << true_x + noise(gen), true_y + noise(gen);
        
        try {
            // Predict step
            ekf.predict(dt);
            
            // Update step with measurement
            ekf.update(z);
            
            // Print every 10 steps
            if (i % 10 == 0) {
                Eigen::VectorXd state = ekf.getState();
                std::cout << "Step " << i << ": True: (" << true_x << ", " << true_y << "), ";
                std::cout << "Estimated: (" << state(0) << ", " << state(1) << "), ";
                std::cout << "Velocity: (" << state(2) << ", " << state(3) << ")" << std::endl;
            }
        } catch (const std::exception& e) {
            std::cerr << "Error at step " << i << ": " << e.what() << std::endl;
            break;
        }
    }

    return 0;
}
