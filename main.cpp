#include <iostream>
#include <vector>
#include <random>
#include <cmath>
#include "ekf.h"

int main() {
    std::cout << "Extended Kalman Filter Non-Linear Model Demonstration" << std::endl;
    
    // 初始化系统
    int stateSize = 4;  // x, y, θ, v (位置、航向角和速度)
    int measureSize = 2;  // r, φ (距离和方位角测量)
    int controlSize = 0;  // 本例中没有控制输入
    
    // 创建带有维度的EKF
    EKF ekf(stateSize, measureSize, controlSize);
    
    // 初始状态 [x, y, θ, v]
    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(stateSize);
    x0 << 5.0, 0.0, M_PI/2, 1.0;  // 起始位置(5,0)，初始航向90度，速度1.0
    
    // 初始状态协方差 - 合理的不确定性值
    Eigen::MatrixXd P0 = Eigen::MatrixXd::Identity(stateSize, stateSize);
    P0(0,0) = P0(1,1) = 0.5;   // 位置不确定性
    P0(2,2) = 0.1;             // 航向角不确定性
    P0(3,3) = 0.2;             // 速度不确定性
    
    // 过程噪声协方差 - 适当的值
    Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(stateSize, stateSize);
    Q(0,0) = Q(1,1) = 0.01;   // 位置过程噪声
    Q(2,2) = 0.01;            // 航向角过程噪声
    Q(3,3) = 0.05;            // 速度过程噪声
    
    // 测量噪声协方差 - 实际值
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(measureSize, measureSize);
    R(0,0) = 0.1;    // 距离测量噪声
    R(1,1) = 0.05;   // 角度测量噪声
    
    // 设置EKF
    ekf.setInitialState(x0, P0);
    ekf.setProcessNoise(Q);
    ekf.setMeasurementNoise(R);
    
    // 设置状态转移函数 (非线性模型 - 圆周运动)
    ekf.setStateTransitionFunction([](const Eigen::VectorXd& x, double dt) {
        Eigen::VectorXd next_x = x;
        double theta = x(2);
        double v = x(3);
        
        // 非线性运动方程
        next_x(0) += v * cos(theta) * dt;  // x += v*cos(θ)*dt
        next_x(1) += v * sin(theta) * dt;  // y += v*sin(θ)*dt
        // 角度和速度保持不变，此示例中我们假设匀速圆周运动
        
        return next_x;
    });
    
    // 状态雅可比矩阵
    ekf.setStateJacobianFunction([](const Eigen::VectorXd& x, double dt) {
        Eigen::MatrixXd J = Eigen::MatrixXd::Identity(x.size(), x.size());
        double theta = x(2);
        double v = x(3);
        
        // 偏导数
        J(0, 2) = -v * sin(theta) * dt;  // dx/dθ = -v*sin(θ)*dt
        J(0, 3) = cos(theta) * dt;       // dx/dv = cos(θ)*dt
        J(1, 2) = v * cos(theta) * dt;   // dy/dθ = v*cos(θ)*dt
        J(1, 3) = sin(theta) * dt;       // dy/dv = sin(θ)*dt
        
        return J;
    });
    
    // 测量函数 (我们测量距离r和方位角φ - 极坐标)
    ekf.setMeasurementFunction([](const Eigen::VectorXd& x) {
        Eigen::VectorXd z(2);
        double px = x(0);
        double py = x(1);
        
        // 非线性测量方程 - 从笛卡尔坐标转换为极坐标
        double r = sqrt(px*px + py*py);          // 距离 = sqrt(x² + y²)
        double phi = atan2(py, px);              // 方位角 = atan2(y, x)
        
        z << r, phi;
        return z;
    });
    
    // 测量雅可比矩阵
    ekf.setMeasurementJacobianFunction([](const Eigen::VectorXd& x) {
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2, 4);
        double px = x(0);
        double py = x(1);
        double d = px*px + py*py;
        double r = sqrt(d);
        
        // 极坐标测量的雅可比矩阵
        if (r < 1e-6) {
            // 避免除以零
            H(0, 0) = H(0, 1) = H(1, 0) = H(1, 1) = 0;
        } else {
            H(0, 0) = px / r;                    // dr/dx = x/r
            H(0, 1) = py / r;                    // dr/dy = y/r
            H(1, 0) = -py / d;                   // dφ/dx = -y/(x² + y²)
            H(1, 1) = px / d;                    // dφ/dy = x/(x² + y²)
        }
        
        return H;
    });
    
    // 使用更稳定的随机数生成器模拟测量
    std::mt19937 gen(42);  // 固定种子以确保可重现性
    std::normal_distribution<> dist_noise(0, 0.1);  // 距离噪声
    std::normal_distribution<> angle_noise(0, 0.05);  // 角度噪声
    double dt = 0.1;  // 时间步长
    
    // 存储真实和估计轨迹
    std::vector<double> true_positions_x, true_positions_y;
    std::vector<double> estimated_positions_x, estimated_positions_y;
    
    // 仿真参数 - 圆周运动
    double radius = 5.0;       // 圆半径
    double angular_vel = 0.2;  // 角速度 rad/s
    
    // 仿真循环
    for (int i = 0; i < 100; i++) {
        double t = i * dt;
        
        // 真实状态 (仿真) - 圆周运动
        double true_theta = angular_vel * t + M_PI/2;
        double true_x = radius * cos(true_theta);
        double true_y = radius * sin(true_theta);
        
        true_positions_x.push_back(true_x);
        true_positions_y.push_back(true_y);
        
        // 生成带噪声的测量 (极坐标形式)
        double true_r = sqrt(true_x*true_x + true_y*true_y);
        double true_phi = atan2(true_y, true_x);
        
        // 添加测量噪声
        double meas_r = true_r + dist_noise(gen);
        double meas_phi = true_phi + angle_noise(gen);
        
        Eigen::VectorXd z(2);
        z << meas_r, meas_phi;
        
        try {
            // 预测步骤
            ekf.predict(dt);
            
            // 更新步骤
            ekf.update(z);
            
            // 获取当前状态估计
            Eigen::VectorXd state = ekf.getState();
            estimated_positions_x.push_back(state(0));
            estimated_positions_y.push_back(state(1));
            
            // 每10步打印一次
            if (i % 10 == 0) {
                Eigen::MatrixXd cov = ekf.getCovariance();
                std::cout << "步骤 " << i << ": 真实位置: (" << true_x << ", " << true_y << "), ";
                std::cout << "估计位置: (" << state(0) << ", " << state(1) << "), ";
                std::cout << "角度: " << state(2) * 180/M_PI << "°, 速度: " << state(3) << std::endl;
                std::cout << "位置不确定性: " << sqrt(cov(0,0)) << ", " << sqrt(cov(1,1)) << std::endl;
            }
        } catch (const std::exception& e) {
            std::cerr << "步骤 " << i << " 出错: " << e.what() << std::endl;
            break;
        }
    }

    std::cout << "\n仿真成功完成！非线性模型测试演示结束。" << std::endl;
    return 0;
}
