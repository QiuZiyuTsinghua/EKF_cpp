#include "ekf.h"
#include <stdexcept>

EKF::EKF(int stateSize, int measureSize, int controlSize)
    : m_stateSize(stateSize), m_measureSize(measureSize), m_controlSize(controlSize) {
    
    m_x = Eigen::VectorXd::Zero(stateSize);
    m_P = Eigen::MatrixXd::Identity(stateSize, stateSize);
    m_Q = Eigen::MatrixXd::Identity(stateSize, stateSize);
    m_R = Eigen::MatrixXd::Identity(measureSize, measureSize);
}

void EKF::setInitialState(const Eigen::VectorXd& x0, const Eigen::MatrixXd& P0) {
    if (x0.size() != m_stateSize) {
        throw std::invalid_argument("Initial state has incorrect dimension");
    }
    if (P0.rows() != m_stateSize || P0.cols() != m_stateSize) {
        throw std::invalid_argument("Initial covariance matrix has incorrect dimensions");
    }
    
    m_x = x0;
    m_P = P0;
}

void EKF::setProcessNoise(const Eigen::MatrixXd& Q) {
    if (Q.rows() != m_stateSize || Q.cols() != m_stateSize) {
        throw std::invalid_argument("Process noise covariance has incorrect dimensions");
    }
    m_Q = Q;
}

void EKF::setMeasurementNoise(const Eigen::MatrixXd& R) {
    if (R.rows() != m_measureSize || R.cols() != m_measureSize) {
        throw std::invalid_argument("Measurement noise covariance has incorrect dimensions");
    }
    m_R = R;
}

void EKF::setStateTransitionFunction(std::function<Eigen::VectorXd(const Eigen::VectorXd&, double)> f) {
    m_f = f;
}

void EKF::setStateJacobianFunction(std::function<Eigen::MatrixXd(const Eigen::VectorXd&, double)> J) {
    m_J = J;
}

void EKF::setMeasurementFunction(std::function<Eigen::VectorXd(const Eigen::VectorXd&)> h) {
    m_h = h;
}

void EKF::setMeasurementJacobianFunction(std::function<Eigen::MatrixXd(const Eigen::VectorXd&)> H) {
    m_H = H;
}

void EKF::predict(double dt) {
    if (!m_f || !m_J) {
        throw std::runtime_error("State transition function or Jacobian not set");
    }
    
    // Predict state
    m_x = m_f(m_x, dt);
    
    // Calculate Jacobian
    Eigen::MatrixXd F = m_J(m_x, dt);
    
    // Predict covariance
    m_P = F * m_P * F.transpose() + m_Q;
}

void EKF::update(const Eigen::VectorXd& z) {
    if (!m_h || !m_H) {
        throw std::runtime_error("Measurement function or Jacobian not set");
    }
    
    if (z.size() != m_measureSize) {
        throw std::invalid_argument("Measurement vector has incorrect dimension");
    }
    
    // Predicted measurement
    Eigen::VectorXd z_pred = m_h(m_x);
    
    // Calculate residual (innovation)
    Eigen::VectorXd y = z - z_pred;
    
    // Calculate measurement Jacobian
    Eigen::MatrixXd H = m_H(m_x);
    
    // Calculate innovation covariance
    Eigen::MatrixXd S = H * m_P * H.transpose() + m_R;
    
    // Calculate Kalman gain
    Eigen::MatrixXd K = m_P * H.transpose() * S.inverse();
    
    // Update state
    m_x = m_x + K * y;
    
    // Update covariance
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(m_stateSize, m_stateSize);
    m_P = (I - K * H) * m_P;
}

Eigen::VectorXd EKF::getState() const {
    return m_x;
}

Eigen::MatrixXd EKF::getCovariance() const {
    return m_P;
}
