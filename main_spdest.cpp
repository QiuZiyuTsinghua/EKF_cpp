#include <iostream>
#include <vector>
#include <random>
#include <cmath>
#include "ekf.h"

int main() {
    std::cout << "车辆纵向速度估计 - 3自由度4轮模型" << std::endl;
    
    // 初始化系统
    int stateSize = 6;  // [x, y, θ, vx, vy, ω] - 位置、偏航角、速度和角速度
    int measureSize = 7;  // [ax, ay, ω, v_fl, v_fr, v_rl, v_rr] - 加速度、角速度和轮速
    int controlSize = 2;  // [δ, T] - 方向盘转角和驱动力矩
    
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
    
    // 初始状态 [x, y, θ, vx, vy, ω]
    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(stateSize);
    x0 << 0.0, 0.0, 0.0, 10.0, 0.0, 0.0;  // 初始纵向速度为10 m/s
    
    // 初始状态协方差
    Eigen::MatrixXd P0 = Eigen::MatrixXd::Identity(stateSize, stateSize);
    P0(0,0) = P0(1,1) = 0.1;     // 位置不确定性
    P0(2,2) = 0.01;              // 偏航角不确定性
    P0(3,3) = 1.0;               // 纵向速度不确定性
    P0(4,4) = 0.1;               // 横向速度不确定性
    P0(5,5) = 0.01;              // 角速度不确定性
    
    // 过程噪声协方差
    Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(stateSize, stateSize);
    Q(0,0) = Q(1,1) = 0.01;      // 位置过程噪声
    Q(2,2) = 0.005;              // 偏航角过程噪声
    Q(3,3) = 0.5;                // 纵向速度过程噪声
    Q(4,4) = 0.1;                // 横向速度过程噪声
    Q(5,5) = 0.01;               // 角速度过程噪声
    
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
    
    // 设置状态转移函数
    ekf.setStateTransitionFunction([m, Iz, Cf, Cr, lf, lr](const Eigen::VectorXd& x, double dt) {
        // 状态: [x, y, θ, vx, vy, ω]
        Eigen::VectorXd next_x = x;
        double theta = x(2);
        double vx = x(3);
        double vy = x(4);
        double omega = x(5);
        
        // 由于忽略风阻，车辆动力学模型简化
        // 计算车辆在惯性坐标系中的位置变化
        next_x(0) += dt * (vx * cos(theta) - vy * sin(theta));
        next_x(1) += dt * (vx * sin(theta) + vy * cos(theta));
        next_x(2) += dt * omega;
        
        // 基于3自由度模型的速度变化
        // 由于忽略风阻，轮胎力是唯一的外力
        double Fx = 0.0;  // 在这个简化模型中，假设纵向力保持不变
        double Fy = 0.0;  // 在这个简化模型中，假设横向力通过Cf和Cr计算
        
        // 车辆侧偏角
        double beta = atan2(vy, vx);
        
        // 前轮侧偏角 (假设方向盘转角为0，简化模型)
        double alpha_f = beta - lf * omega / vx;
        // 后轮侧偏角
        double alpha_r = beta + lr * omega / vx;
        
        // 轮胎侧向力
        double Fyf = -Cf * alpha_f;  // 前轮侧向力
        double Fyr = -Cr * alpha_r;  // 后轮侧向力
        
        Fy = Fyf + Fyr;  // 总侧向力
        
        // 力矩
        double Mz = Fyf * lf - Fyr * lr;  // 围绕z轴的总力矩
        
        // 更新速度和角速度
        // dvx/dt = Fx/m + omega*vy
        next_x(3) += dt * (Fx / m + omega * vy);
        // dvy/dt = Fy/m - omega*vx
        next_x(4) += dt * (Fy / m - omega * vx);
        // domega/dt = Mz/Iz
        next_x(5) += dt * (Mz / Iz);
        
        return next_x;
    });
    
    // 状态雅可比矩阵
    ekf.setStateJacobianFunction([m, Iz, Cf, Cr, lf, lr](const Eigen::VectorXd& x, double dt) {
        Eigen::MatrixXd J = Eigen::MatrixXd::Identity(x.size(), x.size());
        double theta = x(2);
        double vx = x(3);
        double vy = x(4);
        double omega = x(5);
        
        // 位置对状态的偏导数
        J(0, 2) = -dt * (vx * sin(theta) + vy * cos(theta));
        J(0, 3) = dt * cos(theta);
        J(0, 4) = -dt * sin(theta);
        
        J(1, 2) = dt * (vx * cos(theta) - vy * sin(theta));
        J(1, 3) = dt * sin(theta);
        J(1, 4) = dt * cos(theta);
        
        // 角度对角速度的偏导数
        J(2, 5) = dt;
        
        // vx 对相关状态的偏导数
        J(3, 4) = dt * omega;
        J(3, 5) = dt * vy;
        
        // vy 对相关状态的偏导数
        J(4, 3) = -dt * omega;
        J(4, 5) = -dt * vx;
        
        // ω 对状态的偏导数依赖于复杂的轮胎模型
        // 在这个简化实现中，我们假设它们是恒定的
        
        return J;
    });
    
    // 设置测量函数
    ekf.setMeasurementFunction([r_wheel, lf, lr, track](const Eigen::VectorXd& x) {
        Eigen::VectorXd z(7);  // [ax, ay, omega, v_fl, v_fr, v_rl, v_rr]
        
        double vx = x(3);
        double vy = x(4);
        double omega = x(5);
        
        // 简化的加速度模型（假设外力影响较小）
        double ax = 0.0;  // 纵向加速度
        double ay = 0.0;  // 横向加速度
        
        // 轮速 - 根据车辆几何参数计算各轮速度
        double w_fl = vx / r_wheel - (omega * (track/2)) / r_wheel;  // 前左轮
        double w_fr = vx / r_wheel + (omega * (track/2)) / r_wheel;  // 前右轮
        double w_rl = vx / r_wheel - (omega * (track/2)) / r_wheel;  // 后左轮
        double w_rr = vx / r_wheel + (omega * (track/2)) / r_wheel;  // 后右轮
        
        // 将角速度转换为线速度
        double v_fl = w_fl * r_wheel;
        double v_fr = w_fr * r_wheel;
        double v_rl = w_rl * r_wheel;
        double v_rr = w_rr * r_wheel;
        
        z << ax, ay, omega, v_fl, v_fr, v_rl, v_rr;
        return z;
    });
    
    // 测量雅可比矩阵
    ekf.setMeasurementJacobianFunction([r_wheel, track](const Eigen::VectorXd& x) {
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(7, 6);
        
        // ax 对状态的偏导数 - 简化模型下为0
        
        // ay 对状态的偏导数 - 简化模型下为0
        
        // omega 的测量直接对应状态
        H(2, 5) = 1.0;
        
        // v_fl 对状态的偏导数
        H(3, 3) = 1.0;  // 对vx的偏导为1
        H(3, 5) = -track/2;  // 对omega的偏导
        
        // v_fr 对状态的偏导数
        H(4, 3) = 1.0;  // 对vx的偏导为1
        H(4, 5) = track/2;  // 对omega的偏导
        
        // v_rl 对状态的偏导数
        H(5, 3) = 1.0;  // 对vx的偏导为1
        H(5, 5) = -track/2;  // 对omega的偏导
        
        // v_rr 对状态的偏导数
        H(6, 3) = 1.0;  // 对vx的偏导为1
        H(6, 5) = track/2;  // 对omega的偏导
        
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
    std::vector<double> true_omega_history, est_omega_history;
    
    // 真实车辆状态初始化 - 使用不同的变量名避免冲突
    double true_x = 0.0, true_y = 0.0, true_theta = 0.0;
    double current_vx = 10.0, current_vy = 0.0, current_omega = 0.0;
    
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
        
        // 更新真实车辆状态 - 简化版车辆动力学
        current_vx += ax_input * dt;
        true_theta += current_omega * dt;
        true_x += current_vx * cos(true_theta) * dt;
        true_y += current_vx * sin(true_theta) * dt;
        
        // 根据方向盘转角更新横向动态（简化模型）
        double steer_effect = delta_input * current_vx * 0.2;  // 简化的转向效应
        current_omega = steer_effect;
        current_vy = steer_effect * 0.5;  // 侧滑与角速度相关
        
        // 存储真实状态
        true_vx_history.push_back(current_vx);
        true_vy_history.push_back(current_vy);
        true_omega_history.push_back(current_omega);
        
        // 生成带噪声的测量
        double meas_ax = ax_input + acc_noise(gen);
        double meas_ay = current_vy * current_omega + acc_noise(gen);  // 简化的测量模型
        double meas_omega = current_omega + omega_noise(gen);
        
        // 计算各车轮速度（理想值）
        double wheel_base = lf + lr;
        double v_fl = current_vx - (current_omega * track/2);  // 前左轮
        double v_fr = current_vx + (current_omega * track/2);  // 前右轮
        double v_rl = current_vx - (current_omega * track/2);  // 后左轮
        double v_rr = current_vx + (current_omega * track/2);  // 后右轮
        
        // 添加噪声到轮速测量
        double meas_v_fl = v_fl + wheel_noise(gen);
        double meas_v_fr = v_fr + wheel_noise(gen);
        double meas_v_rl = v_rl + wheel_noise(gen);
        double meas_v_rr = v_rr + wheel_noise(gen);
        
        Eigen::VectorXd z(7);
        z << meas_ax, meas_ay, meas_omega, meas_v_fl, meas_v_fr, meas_v_rl, meas_v_rr;
        
        try {
            // 预测步骤
            ekf.predict(dt);
            
            // 更新步骤
            ekf.update(z);
            
            // 获取当前状态估计
            Eigen::VectorXd state = ekf.getState();
            est_vx_history.push_back(state(3));
            est_vy_history.push_back(state(4));
            est_omega_history.push_back(state(5));
            
            // 每100步打印一次
            if (i % 100 == 0) {
                std::cout << "时间: " << t << "s, 真实速度: " << current_vx << " m/s, 估计速度: " 
                          << state(3) << " m/s, 误差: " << (current_vx - state(3)) << " m/s" << std::endl;
            }
        } catch (const std::exception& e) {
            std::cerr << "时间 " << t << " 出错: " << e.what() << std::endl;
            break;
        }
    }

    // 计算均方根误差 (RMSE)
    double sum_sq_error = 0.0;
    for (size_t i = 0; i < true_vx_history.size(); ++i) {
        sum_sq_error += (true_vx_history[i] - est_vx_history[i]) * (true_vx_history[i] - est_vx_history[i]);
    }
    double rmse = sqrt(sum_sq_error / true_vx_history.size());
    
    std::cout << "\n仿真完成！纵向速度估计均方根误差: " << rmse << " m/s" << std::endl;
    std::cout << "请注意，完整应用中应导出数据并绘制估计结果与真实值的对比图。" << std::endl;
    
    return 0;
}
