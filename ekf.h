#ifndef EKF_H
#define EKF_H

#include <functional>
#include <Eigen/Dense>
#include <iostream>  // Add for std::cerr

/**
 * Extended Kalman Filter implementation
 */
class EKF {
public:
    /**
     * Constructor
     * @param stateSize Dimension of the state vector
     * @param measureSize Dimension of the measurement vector
     * @param controlSize Dimension of the control input vector
     */
    EKF(int stateSize, int measureSize, int controlSize = 0);
    
    /**
     * Set the initial state and covariance
     * @param x0 Initial state vector
     * @param P0 Initial state covariance matrix
     */
    void setInitialState(const Eigen::VectorXd& x0, const Eigen::MatrixXd& P0);
    
    /**
     * Set the process noise covariance matrix
     * @param Q Process noise covariance matrix
     */
    void setProcessNoise(const Eigen::MatrixXd& Q);
    
    /**
     * Set the measurement noise covariance matrix
     * @param R Measurement noise covariance matrix
     */
    void setMeasurementNoise(const Eigen::MatrixXd& R);
    
    /**
     * Set the state transition function f(x, dt)
     * @param f Function that predicts the next state given current state and dt
     */
    void setStateTransitionFunction(std::function<Eigen::VectorXd(const Eigen::VectorXd&, double)> f);
    
    /**
     * Set the state transition Jacobian function
     * @param J Function that returns the Jacobian of state transition
     */
    void setStateJacobianFunction(std::function<Eigen::MatrixXd(const Eigen::VectorXd&, double)> J);
    
    /**
     * Set the measurement function h(x)
     * @param h Function that converts state to measurement
     */
    void setMeasurementFunction(std::function<Eigen::VectorXd(const Eigen::VectorXd&)> h);
    
    /**
     * Set the measurement Jacobian function
     * @param H Function that returns the Jacobian of measurement
     */
    void setMeasurementJacobianFunction(std::function<Eigen::MatrixXd(const Eigen::VectorXd&)> H);
    
    /**
     * Prediction step
     * @param dt Time step
     */
    void predict(double dt);
    
    /**
     * Update step
     * @param z Measurement vector
     */
    void update(const Eigen::VectorXd& z);
    
    /**
     * Get the current state estimate
     * @return Current state vector
     */
    Eigen::VectorXd getState() const;
    
    /**
     * Get the current state covariance matrix
     * @return Current state covariance matrix
     */
    Eigen::MatrixXd getCovariance() const;

private:
    int m_stateSize;
    int m_measureSize;
    int m_controlSize;
    
    Eigen::VectorXd m_x;  // State vector
    Eigen::MatrixXd m_P;  // State covariance matrix
    Eigen::MatrixXd m_Q;  // Process noise covariance
    Eigen::MatrixXd m_R;  // Measurement noise covariance
    
    // Function objects for the EKF model
    std::function<Eigen::VectorXd(const Eigen::VectorXd&, double)> m_f;  // State transition function
    std::function<Eigen::MatrixXd(const Eigen::VectorXd&, double)> m_J;  // State Jacobian
    std::function<Eigen::VectorXd(const Eigen::VectorXd&)> m_h;  // Measurement function
    std::function<Eigen::MatrixXd(const Eigen::VectorXd&)> m_H;  // Measurement Jacobian
};

#endif // EKF_H
