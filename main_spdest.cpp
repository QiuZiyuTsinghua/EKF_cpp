#include <iostream>
#include <vector>
#include <random>
#include <cmath>
#include "ekf.h"

int main() {
    std::cout << "车辆纵向速度估计 - 基于修改的3自由度模型" << std::endl;
    
    // 初始化系统 - 修改状态向量为 [v_x, v_y, γ]
    int stateSize = 3;  // v_x, v_y, γ - 纵向速度、横向速度和横摆角速度
    int measureSize = 7;  // [ax, ay, ω, v_fl, v_fr, v_rl, v_rr] - 加速度、角速度和轮速
    int controlSize = 1;  // δ - 方向盘转角
    
    // 创建EKF
    EKF ekf(stateSize, measureSize, controlSize);
    
    // 车辆参数 - 典型小型轿车
    double m = 1500.0;     // 质量 (kg)
    double Iz = 2500.0;    // 转动惯量 (kg*m^2)
    double lf = 1.2;       // 前轴到质心的距离 (m)
    double lr = 1.4;       // 后轴到质心的距离 (m)
    double wheelbase = lf + lr;  // 轴距 (m)
    double track = 1.6;    // 轮距 (m)
    double r_wheel = 0.3;  // 车轮半径 (m)
    
    // 轮胎参数
    double Cf = 50000.0;   // 前轮侧偏刚度 (N/rad)
    double Cr = 50000.0;   // 后轮侧偏刚度 (N/rad)
    
    // 初始状态 [v_x, v_y, γ]
    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(stateSize);
    x0 << 10.0, 0.0, 0.0;  // 初始纵向速度为10 m/s
    
    // 初始状态协方差
    Eigen::MatrixXd P0 = Eigen::MatrixXd::Identity(stateSize, stateSize);
    P0(0,0) = 1.0;               // 纵向速度不确定性
    P0(1,1) = 0.1;               // 横向速度不确定性
    P0(2,2) = 0.01;              // 角速度不确定性
    
    // 过程噪声协方差
    Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(stateSize, stateSize);
    Q(0,0) = 0.5;                // 纵向速度过程噪声
    Q(1,1) = 0.1;                // 横向速度过程噪声
    Q(2,2) = 0.01;               // 角速度过程噪声
    
    // 测量噪声协方差
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(measureSize, measureSize);
    R(0,0) = 0.1;                // 纵向加速度噪声
    R(1,1) = 0.1;                // 横向加速度噪声
    R(2,2) = 0.01;               // 角速度噪声
    R(3,3) = R(4,4) = R(5,5) = R(6,6) = 0.2;  // 轮速噪声
    
    // 设置EKF
    ekf.setInitialState(x0, P0);
    ekf.setProcessNoise(Q);
    ekf.setMeasurementNoise(R);
    
    // 设置状态转移函数 - 根据提供的系统方程
    // ẋ(t) = [a_x + v_y γ + w_1, a_y - v_x γ + w_2, M_z/I_z + w_3]
    ekf.setStateTransitionFunction([m, Iz, Cf, Cr, lf, lr](const Eigen::VectorXd& x, double dt) {
        // 状态: [v_x, v_y, γ]
        Eigen::VectorXd next_x = x;
        double vx = x(0);
        double vy = x(1);
        double gamma = x(2);  // 横摆角速度(γ)
        
        // 根据系统方程计算加速度
        double ax = 0.0;  // 在这个简化模型中，假设纵向加速度为0（匀速直线行驶）
        double ay = 0.0;  // 横向加速度，来自轮胎力
        double Mz = 0.0;  // 横摆力矩
        
        // 计算轮胎侧偏角 - 假设前轮转角为0
        double beta = atan2(vy, vx);  // 车辆侧偏角
        
        // 前轮侧偏角
        double alpha_f = beta - lf * gamma / vx;
        // 后轮侧偏角
        double alpha_r = beta + lr * gamma / vx;
        
        // 轮胎侧向力
        double Fyf = -Cf * alpha_f;  // 前轮侧向力
        double Fyr = -Cr * alpha_r;  // 后轮侧向力
        
        ay = (Fyf + Fyr) / m;      // 横向加速度
        Mz = (Fyf * lf - Fyr * lr);  // 横摆力矩
        
        // 更新速度和角速度，根据系统方程
        // v̇_x = a_x + v_y γ
        next_x(0) += dt * (ax + vy * gamma);
        // v̇_y = a_y - v_x γ
        next_x(1) += dt * (ay - vx * gamma);
        // γ̇ = M_z/I_z
        next_x(2) += dt * (Mz / Iz);
        
        return next_x;
    });
    
    // 状态雅可比矩阵
    ekf.setStateJacobianFunction([m, Iz, Cf, Cr, lf, lr](const Eigen::VectorXd& x, double dt) {
        Eigen::MatrixXd J = Eigen::MatrixXd::Identity(x.size(), x.size());
        double vx = x(0);
        double vy = x(1);
        double gamma = x(2);
        
        // 计算雅可比矩阵元素
        // ∂(v̇_x)/∂v_y = γ
        J(0, 1) = dt * gamma;
        // ∂(v̇_x)/∂γ = v_y
        J(0, 2) = dt * vy;
        
        // ∂(v̇_y)/∂v_x = -γ
        J(1, 0) = -dt * gamma;
        // ∂(v̇_y)/∂γ = -v_x
        J(1, 2) = -dt * vx;
        
        // 简化后的雅可比矩阵，忽略轮胎模型导致的复杂非线性项
        
        return J;
    });
    
    // 设置测量函数 - 从状态变量映射到测量值
    ekf.setMeasurementFunction([r_wheel, lf, lr, track, m, Iz, Cf, Cr](const Eigen::VectorXd& x) {
        Eigen::VectorXd z(7);  // [ax, ay, gamma, v_fl, v_fr, v_rl, v_rr]
        
        double vx = x(0);
        double vy = x(1);
        double gamma = x(2);
        
        // 计算加速度 - 基于当前状态
        double beta = atan2(vy, vx);
        double alpha_f = beta - lf * gamma / vx;
        double alpha_r = beta + lr * gamma / vx;
        double Fyf = -Cf * alpha_f;
        double Fyr = -Cr * alpha_r;
        
        double ax = vy * gamma;  // 假设纵向加速度只来自角速度项
        double ay = (Fyf + Fyr) / m - vx * gamma;  // 横向加速度
        
        // 计算各车轮线速度
        double v_fl = vx - gamma * (track/2);  // 前左轮
        double v_fr = vx + gamma * (track/2);  // 前右轮
        double v_rl = vx - gamma * (track/2);  // 后左轮
        double v_rr = vx + gamma * (track/2);  // 后右轮
        
        z << ax, ay, gamma, v_fl, v_fr, v_rl, v_rr;
        return z;
    });
    
    // 测量雅可比矩阵
    ekf.setMeasurementJacobianFunction([r_wheel, track, m, Iz, Cf, Cr, lf, lr](const Eigen::VectorXd& x) {
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(7, 3);
        
        double vx = x(0);
        double vy = x(1);
        double gamma = x(2);
        
        // ax 对状态的偏导数
        H(0, 1) = gamma;  // ∂(ax)/∂v_y = γ
        H(0, 2) = vy;     // ∂(ax)/∂γ = v_y
        
        // ay 对状态的偏导数 - 简化版本，忽略复杂的轮胎模型导致的偏导
        H(1, 0) = -gamma;  // ∂(ay)/∂v_x = -γ
        H(1, 2) = -vx;     // ∂(ay)/∂γ = -v_x
        
        // gamma 的测量直接对应状态
        H(2, 2) = 1.0;     // ∂(γ_measured)/∂γ = 1
        
        // v_fl 对状态的偏导数
        H(3, 0) = 1.0;     // ∂(v_fl)/∂v_x = 1
        H(3, 2) = -track/2;  // ∂(v_fl)/∂γ = -track/2
        
        // v_fr 对状态的偏导数
        H(4, 0) = 1.0;     // ∂(v_fr)/∂v_x = 1
        H(4, 2) = track/2;   // ∂(v_fr)/∂γ = track/2
        
        // v_rl 对状态的偏导数
        H(5, 0) = 1.0;     // ∂(v_rl)/∂v_x = 1
        H(5, 2) = -track/2;  // ∂(v_rl)/∂γ = -track/2
        
        // v_rr 对状态的偏导数
        H(6, 0) = 1.0;     // ∂(v_rr)/∂v_x = 1
        H(6, 2) = track/2;   // ∂(v_rr)/∂γ = track/2
        
        return H;
    });
    
    // 随机数生成器
    std::mt19937 gen(42);  // 固定种子以确保可重现性
    std::normal_distribution<> acc_noise(0, 0.1);  // 加速度噪声
    std::normal_distribution<> omega_noise(0, 0.01);  // 角速度噪声
    std::normal_distribution<> wheel_noise(0, 0.2);  // 轮速噪声
    
    double dt = 0.01;  // 时间步长 (10ms)
    double sim_time = 10.0;  // 仿真时间 (s)
    int steps = static_cast<int>(sim_time / dt);
    
    // 存储真实和估计的速度
    std::vector<double> time_points;
    std::vector<double> true_vx_history, est_vx_history;
    std::vector<double> true_vy_history, est_vy_history;
    std::vector<double> true_gamma_history, est_gamma_history;
    
    // 真实车辆状态初始化
    double current_vx = 10.0, current_vy = 0.0, current_gamma = 0.0;
    
    // 加速度输入 - 模拟变化的驾驶场景
    auto generate_acceleration = [](double t) {
        // 在5秒时施加制动
        if (t > 5.0 && t < 7.0) {
            return -2.0;  // 减速度 2 m/s²
        } else {
            return 0.0;   // 匀速行驶
        }
    };
    
    // 转向输入 - 模拟转向场景
    auto generate_steering = [](double t) {
        // 在2秒时开始转向，4秒时回正
        if (t > 2.0 && t < 4.0) {
            return 0.1;  // 10度方向盘转角
        } else {
            return 0.0;  // 直线行驶
        }
    };

    // 仿真循环
    for (int i = 0; i < steps; i++) {
        double t = i * dt;
        time_points.push_back(t);
        
        // 生成控制输入
        double ax_input = generate_acceleration(t);
        double delta_input = generate_steering(t);
        
        // 更新真实车辆状态 - 根据提供的动力学模型
        double true_ax = ax_input + current_vy * current_gamma;
        double true_ay = -current_vx * current_gamma;  // 简化模型，假设ay主要来自圆周运动
        double true_moment = delta_input * current_vx * 0.8;  // 简化的转向导致的力矩
        
        // 使用系统方程更新状态
        current_vx += dt * true_ax;
        current_vy += dt * true_ay;
        current_gamma += dt * (true_moment / Iz);
        
        // 存储真实状态
        true_vx_history.push_back(current_vx);
        true_vy_history.push_back(current_vy);
        true_gamma_history.push_back(current_gamma);
        
        // 生成带噪声的测量
        double meas_ax = true_ax + acc_noise(gen);
        double meas_ay = true_ay + acc_noise(gen);
        double meas_gamma = current_gamma + omega_noise(gen);
        
        // 计算各车轮速度（理想值）
        double v_fl = current_vx - (current_gamma * track/2);  // 前左轮
        double v_fr = current_vx + (current_gamma * track/2);  // 前右轮
        double v_rl = current_vx - (current_gamma * track/2);  // 后左轮
        double v_rr = current_vx + (current_gamma * track/2);  // 后右轮
        
        // 添加噪声到轮速测量
        double meas_v_fl = v_fl + wheel_noise(gen);
        double meas_v_fr = v_fr + wheel_noise(gen);
        double meas_v_rl = v_rl + wheel_noise(gen);
        double meas_v_rr = v_rr + wheel_noise(gen);
        
        Eigen::VectorXd z(7);
        z << meas_ax, meas_ay, meas_gamma, meas_v_fl, meas_v_fr, meas_v_rl, meas_v_rr;
        
        try {
            // 预测步骤
            ekf.predict(dt);
            
            // 更新步骤
            ekf.update(z);
            
            // 获取当前状态估计
            Eigen::VectorXd state = ekf.getState();
            est_vx_history.push_back(state(0));
            est_vy_history.push_back(state(1));
            est_gamma_history.push_back(state(2));
            
            // 每100步打印一次
            if (i % 100 == 0) {
                std::cout << "时间: " << t << "s, 真实速度: " << current_vx << " m/s, 估计速度: " 
                          << state(0) << " m/s, 误差: " << (current_vx - state(0)) << " m/s" << std::endl;
                std::cout << "真实横向速度: " << current_vy << " m/s, 估计横向速度: " 
                          << state(1) << " m/s" << std::endl;
                std::cout << "真实横摆角速度: " << current_gamma << " rad/s, 估计横摆角速度: " 
                          << state(2) << " rad/s" << std::endl;
                std::cout << "------------------------" << std::endl;
            }
        } catch (const std::exception& e) {
            std::cerr << "时间 " << t << " 出错: " << e.what() << std::endl;
            break;
        }
    }

    // 计算均方根误差 (RMSE)
    double sum_sq_error_vx = 0.0;
    double sum_sq_error_vy = 0.0;
    double sum_sq_error_gamma = 0.0;
    
    for (size_t i = 0; i < true_vx_history.size(); ++i) {
        sum_sq_error_vx += (true_vx_history[i] - est_vx_history[i]) * (true_vx_history[i] - est_vx_history[i]);
        sum_sq_error_vy += (true_vy_history[i] - est_vy_history[i]) * (true_vy_history[i] - est_vy_history[i]);
        sum_sq_error_gamma += (true_gamma_history[i] - est_gamma_history[i]) * (true_gamma_history[i] - est_gamma_history[i]);
    }
    
    double rmse_vx = sqrt(sum_sq_error_vx / true_vx_history.size());
    double rmse_vy = sqrt(sum_sq_error_vy / true_vy_history.size());
    double rmse_gamma = sqrt(sum_sq_error_gamma / true_gamma_history.size());
    
    std::cout << "\n仿真完成！" << std::endl;
    std::cout << "纵向速度估计均方根误差: " << rmse_vx << " m/s" << std::endl;
    std::cout << "横向速度估计均方根误差: " << rmse_vy << " m/s" << std::endl;
    std::cout << "横摆角速度估计均方根误差: " << rmse_gamma << " rad/s" << std::endl;
    std::cout << "请注意，完整应用中应导出数据并绘制估计结果与真实值的对比图。" << std::endl;
    
    return 0;
}
