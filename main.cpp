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
    x0 << 0.0, 0.0, 0.5, 0.5;  // Starting at origin with velocity (0.5,0.5) - more moderate velocity
    
    // Initial state covariance - more reasonable uncertainty values
    Eigen::MatrixXd P0 = Eigen::MatrixXd::Identity(stateSize, stateSize);
    P0.topLeftCorner(2, 2) *= 0.5;     // Position uncertainty
    P0.bottomRightCorner(2, 2) *= 0.2; // Velocity uncertainty
    
    // Process noise covariance - more appropriate values
    Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(stateSize, stateSize);
    Q.topLeftCorner(2, 2) *= 0.005;     // Position process noise
    Q.bottomRightCorner(2, 2) *= 0.02;  // Velocity process noise
    
    // Measurement noise covariance - realistic values
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(measureSize, measureSize);
    R *= 0.05;  // More reasonable measurement noise
    
    // Set up the EKF
    ekf.setInitialState(x0, P0);
    ekf.setProcessNoise(Q);
    ekf.setMeasurementNoise(R);
    
    // Set up state transition function (linear in this case)
    ekf.setStateTransitionFunction([](const Eigen::VectorXd& x, double dt) {
        Eigen::VectorXd next_x = x;
        next_x(0) += x(2) * dt;  // x += vx * dt
        next_x(1) += x(3) * dt;  // y += vy * dt
        return next_x;
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
    
    // Simulate measurements with a more stable random number generator
    std::mt19937 gen(42);  // Fixed seed for reproducibility
    std::normal_distribution<> noise(0, 0.05);  // Reduced noise standard deviation
    double dt = 0.1;  // Time step
    
    // Storage for true and estimated trajectories
    std::vector<double> true_positions_x, true_positions_y;
    std::vector<double> estimated_positions_x, estimated_positions_y;
    
    // Simulation loop
    for (int i = 0; i < 100; i++) {
        // True state (for simulation) - actual constant velocity motion
        double true_x = i * dt * 0.5;  // Using velocity of 0.5 in x direction
        double true_y = i * dt * 0.5;  // Using velocity of 0.5 in y direction
        
        true_positions_x.push_back(true_x);
        true_positions_y.push_back(true_y);
        
        // Generate noisy measurement
        Eigen::VectorXd z(2);
        z << true_x + noise(gen), true_y + noise(gen);
        
        try {
            // Predict step
            ekf.predict(dt);
            
            // Update step with measurement
            ekf.update(z);
            
            // Get current state estimate
            Eigen::VectorXd state = ekf.getState();
            estimated_positions_x.push_back(state(0));
            estimated_positions_y.push_back(state(1));
            
            // Print every 10 steps
            if (i % 10 == 0) {
                Eigen::MatrixXd cov = ekf.getCovariance();
                std::cout << "Step " << i << ": True: (" << true_x << ", " << true_y << "), ";
                std::cout << "Estimated: (" << state(0) << ", " << state(1) << "), ";
                std::cout << "Velocity: (" << state(2) << ", " << state(3) << ")" << std::endl;
                std::cout << "Position uncertainty: " << sqrt(cov(0,0)) << ", " << sqrt(cov(1,1)) << std::endl;
            }
        } catch (const std::exception& e) {
            std::cerr << "Error at step " << i << ": " << e.what() << std::endl;
            break;
        }
    }

    std::cout << "\nSimulation completed successfully!" << std::endl;
    return 0;
}
