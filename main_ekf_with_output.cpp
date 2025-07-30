#include <iostream>
#include <vector>
#include <random>
#include <cmath>
#include <fstream>
#include "ekf.h"

int main() {
    std::cout << "=== Extended Vehicle Dynamics EKF Test with Data Output ===" << std::endl;
    std::cout << "Testing: Vehicle states + Tire force estimation" << std::endl;
    
    // 初始化扩展EKF系统 - 7维状态模型
    int stateSize = 7;    // [v_x, v_y, γ, Fy_fl, Fy_fr, Fy_rl, Fy_rr]
    int measureSize = 7;  // [ax, ay, γ, v_fl, v_fr, v_rl, v_rr]
    int controlSize = 5;  // [δ, Fx_fl, Fx_fr, Fx_rl, Fx_rr]
    
    // 创建扩展EKF实例
    EKF ekf(stateSize, measureSize, controlSize);
    
    // 车辆参数 (典型小轿车参数)
    double m = 1500.0;      // 车辆质量 (kg)
    double Iz = 2500.0;     // 横摆转动惯量 (kg⋅m²)
    double lf = 1.2;        // 质心到前轴距离 (m)
    double lr = 1.4;        // 质心到后轴距离 (m)
    double track_f = 1.6;   // 前轮距 (m)
    double track_r = 1.6;   // 后轮距 (m)
    double Cf = 50000.0;    // 前轮侧偏刚度 (N/rad)
    double Cr = 50000.0;    // 后轮侧偏刚度 (N/rad)
    double tau_fy = 0.05;   // 横向力时间常数 (s)
    
    // 初始状态 [v_x, v_y, γ, Fy_fl, Fy_fr, Fy_rl, Fy_rr]
    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(stateSize);
    x0 << 15.0,    // 初始纵向速度 15 m/s
          0.0,     // 初始横向速度 0 m/s
          0.0,     // 初始横摆角速度 0 rad/s
          0.0,     // 初始左前轮横向力 0 N
          0.0,     // 初始右前轮横向力 0 N
          0.0,     // 初始左后轮横向力 0 N
          0.0;     // 初始右后轮横向力 0 N
    
    // 初始状态协方差矩阵
    Eigen::MatrixXd P0 = Eigen::MatrixXd::Identity(stateSize, stateSize);
    P0(0,0) = 1.0;      // 纵向速度不确定性
    P0(1,1) = 0.5;      // 横向速度不确定性
    P0(2,2) = 0.1;      // 横摆角速度不确定性
    P0(3,3) = 1000.0;   // 左前轮力不确定性
    P0(4,4) = 1000.0;   // 右前轮力不确定性
    P0(5,5) = 1000.0;   // 左后轮力不确定性
    P0(6,6) = 1000.0;   // 右后轮力不确定性
    
    // 过程噪声协方差矩阵
    Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(stateSize, stateSize);
    Q(0,0) = 0.1;       // 纵向速度过程噪声
    Q(1,1) = 0.05;      // 横向速度过程噪声
    Q(2,2) = 0.01;      // 横摆角速度过程噪声
    Q(3,3) = 1000.0;    // 轮胎力过程噪声
    Q(4,4) = 1000.0;
    Q(5,5) = 1000.0;
    Q(6,6) = 1000.0;
    
    // 测量噪声协方差矩阵
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(measureSize, measureSize);
    R(0,0) = 0.5;       // 纵向加速度测量噪声
    R(1,1) = 0.3;       // 横向加速度测量噪声
    R(2,2) = 0.01;      // 横摆角速度测量噪声
    R(3,3) = 0.2;       // 轮速测量噪声
    R(4,4) = 0.2;
    R(5,5) = 0.2;
    R(6,6) = 0.2;
    
    // 设置EKF
    ekf.setInitialState(x0, P0);
    ekf.setProcessNoise(Q);
    ekf.setMeasurementNoise(R);
    
    // 状态转移函数 - 扩展车辆动力学模型 (简化为无控制输入版本)
    ekf.setStateTransitionFunction([m, Iz, Cf, Cr, lf, lr, track_f, track_r, tau_fy]
        (const Eigen::VectorXd& x, double dt) {
        
        // 提取状态变量
        double vx = x(0);       // 纵向速度
        double vy = x(1);       // 横向速度
        double gamma = x(2);    // 横摆角速度
        double Fy_fl = x(3);    // 左前轮横向力
        double Fy_fr = x(4);    // 右前轮横向力
        double Fy_rl = x(5);    // 左后轮横向力
        double Fy_rr = x(6);    // 右后轮横向力
        
        // 假设控制输入为常数 (简化测试)
        double delta = 0.05 * sin(0.5 * dt);  // 时变转向角
        double Fx_fl = 1000.0;  // 常数纵向力
        double Fx_fr = 1000.0;
        double Fx_rl = 1000.0;
        double Fx_rr = 1000.0;
        
        Eigen::VectorXd next_x = x;
        
        // 计算各轮在车辆坐标系下的速度
        double v_wheel_x_fl = vx - gamma * track_f/2;
        double v_wheel_y_fl = vy + gamma * lf;
        double v_wheel_x_fr = vx + gamma * track_f/2;
        double v_wheel_y_fr = vy + gamma * lf;
        double v_wheel_x_rl = vx - gamma * track_r/2;
        double v_wheel_y_rl = vy - gamma * lr;
        double v_wheel_x_rr = vx + gamma * track_r/2;
        double v_wheel_y_rr = vy - gamma * lr;
        
        // 前轮速度转换到轮胎坐标系 (考虑转向角)
        double cos_delta = cos(delta);
        double sin_delta = sin(delta);
        
        double v_tire_long_fl = v_wheel_x_fl * cos_delta + v_wheel_y_fl * sin_delta;
        double v_tire_lat_fl = -v_wheel_x_fl * sin_delta + v_wheel_y_fl * cos_delta;
        double v_tire_long_fr = v_wheel_x_fr * cos_delta + v_wheel_y_fr * sin_delta;
        double v_tire_lat_fr = -v_wheel_x_fr * sin_delta + v_wheel_y_fr * cos_delta;
        
        // 后轮 (无转向)
        double v_tire_long_rl = v_wheel_x_rl;
        double v_tire_lat_rl = v_wheel_y_rl;
        double v_tire_long_rr = v_wheel_x_rr;
        double v_tire_lat_rr = v_wheel_y_rr;
        
        // 计算轮胎侧偏角
        double alpha_fl = atan2(v_tire_lat_fl, fabs(v_tire_long_fl) + 1e-6);
        double alpha_fr = atan2(v_tire_lat_fr, fabs(v_tire_long_fr) + 1e-6);
        double alpha_rl = atan2(v_tire_lat_rl, fabs(v_tire_long_rl) + 1e-6);
        double alpha_rr = atan2(v_tire_lat_rr, fabs(v_tire_long_rr) + 1e-6);
        
        // 计算期望横向力
        double Fy_fl_desired = -Cf * alpha_fl;
        double Fy_fr_desired = -Cf * alpha_fr;
        double Fy_rl_desired = -Cr * alpha_rl;
        double Fy_rr_desired = -Cr * alpha_rr;
        
        // 轮胎力转换到车辆坐标系
        double Fx_veh_fl = Fx_fl * cos_delta - Fy_fl * sin_delta;
        double Fy_veh_fl = Fx_fl * sin_delta + Fy_fl * cos_delta;
        double Fx_veh_fr = Fx_fr * cos_delta - Fy_fr * sin_delta;
        double Fy_veh_fr = Fx_fr * sin_delta + Fy_fr * cos_delta;
        double Fx_veh_rl = Fx_rl;
        double Fy_veh_rl = Fy_rl;
        double Fx_veh_rr = Fx_rr;
        double Fy_veh_rr = Fy_rr;
        
        // 计算总力和力矩
        double Fx_total = Fx_veh_fl + Fx_veh_fr + Fx_veh_rl + Fx_veh_rr;
        double Fy_total = Fy_veh_fl + Fy_veh_fr + Fy_veh_rl + Fy_veh_rr;
        double Mz_total = (Fx_veh_fl * (-track_f/2) - Fy_veh_fl * lf) +
                          (Fx_veh_fr * (track_f/2) - Fy_veh_fr * lf) +
                          (Fx_veh_rl * (-track_r/2) - Fy_veh_rl * (-lr)) +
                          (Fx_veh_rr * (track_r/2) - Fy_veh_rr * (-lr));
        
        // 车辆动力学方程
        next_x(0) += dt * (Fx_total / m + vy * gamma);      // v̇_x
        next_x(1) += dt * (Fy_total / m - vx * gamma);      // v̇_y
        next_x(2) += dt * (Mz_total / Iz);                  // γ̇
        
        // 轮胎横向力动态方程 (一阶响应)
        next_x(3) += dt * (Fy_fl_desired - Fy_fl) / tau_fy; // Ḟy_fl
        next_x(4) += dt * (Fy_fr_desired - Fy_fr) / tau_fy; // Ḟy_fr
        next_x(5) += dt * (Fy_rl_desired - Fy_rl) / tau_fy; // Ḟy_rl
        next_x(6) += dt * (Fy_rr_desired - Fy_rr) / tau_fy; // Ḟy_rr
        
        return next_x;
    });
    
    // 状态雅可比矩阵
    ekf.setStateJacobianFunction([m, Iz, Cf, Cr, lf, lr, track_f, track_r, tau_fy]
        (const Eigen::VectorXd& x, double dt) {
        
        Eigen::MatrixXd J = Eigen::MatrixXd::Identity(x.size(), x.size());
        
        double vx = x(0);
        double vy = x(1);
        double gamma = x(2);
        
        double delta = 0.05 * sin(0.5 * dt);
        double cos_delta = cos(delta);
        double sin_delta = sin(delta);
        
        // 车辆动力学雅可比项
        J(0, 1) += dt * gamma;
        J(0, 2) += dt * vy;
        J(1, 0) += -dt * gamma;
        J(1, 2) += -dt * vx;
        
        // 横向力对车辆动力学的贡献
        J(0, 3) += dt * (-sin_delta) / m;
        J(0, 4) += dt * (-sin_delta) / m;
        J(1, 3) += dt * cos_delta / m;
        J(1, 4) += dt * cos_delta / m;
        J(1, 5) += dt * 1.0 / m;
        J(1, 6) += dt * 1.0 / m;
        
        // 横向力对横摆动力学的贡献
        J(2, 3) += dt * (-lf * cos_delta + track_f/2 * sin_delta) / Iz;
        J(2, 4) += dt * (-lf * cos_delta - track_f/2 * sin_delta) / Iz;
        J(2, 5) += dt * lr / Iz;
        J(2, 6) += dt * lr / Iz;
        
        // 轮胎力动态雅可比项
        J(3, 3) += -dt / tau_fy;
        J(4, 4) += -dt / tau_fy;
        J(5, 5) += -dt / tau_fy;
        J(6, 6) += -dt / tau_fy;
        
        return J;
    });
    
    // 测量函数
    ekf.setMeasurementFunction([m, Iz, Cf, Cr, lf, lr, track_f, track_r]
        (const Eigen::VectorXd& x) {
        
        double vx = x(0);
        double vy = x(1);
        double gamma = x(2);
        double Fy_fl = x(3);
        double Fy_fr = x(4);
        double Fy_rl = x(5);
        double Fy_rr = x(6);
        
        double delta = 0.05 * sin(0.5);
        double Fx_fl = 100.0;
        double Fx_fr = 100.0;
        double Fx_rl = 200.0;
        double Fx_rr = 200.0;
        
        double cos_delta = cos(delta);
        double sin_delta = sin(delta);
        
        // 轮胎力转换到车辆坐标系
        double Fx_veh_fl = Fx_fl * cos_delta - Fy_fl * sin_delta;
        double Fy_veh_fl = Fx_fl * sin_delta + Fy_fl * cos_delta;
        double Fx_veh_fr = Fx_fr * cos_delta - Fy_fr * sin_delta;
        double Fy_veh_fr = Fx_fr * sin_delta + Fy_fr * cos_delta;
        
        // 计算加速度
        double Fx_total = Fx_veh_fl + Fx_veh_fr + Fx_rl + Fx_rr;
        double Fy_total = Fy_veh_fl + Fy_veh_fr + Fy_rl + Fy_rr;
        double ax = Fx_total / m;
        double ay = Fy_total / m;
        
        // 计算轮速
        double v_wheel_x_fl = vx - gamma * track_f/2;
        double v_wheel_y_fl = vy + gamma * lf;
        double v_tire_long_fl = v_wheel_x_fl * cos_delta + v_wheel_y_fl * sin_delta;
        double v_tire_lat_fl = -v_wheel_x_fl * sin_delta + v_wheel_y_fl * cos_delta;
        double v_fl = sqrt(v_tire_long_fl*v_tire_long_fl + v_tire_lat_fl*v_tire_lat_fl);
        
        double v_wheel_x_fr = vx + gamma * track_f/2;
        double v_wheel_y_fr = vy + gamma * lf;
        double v_tire_long_fr = v_wheel_x_fr * cos_delta + v_wheel_y_fr * sin_delta;
        double v_tire_lat_fr = -v_wheel_x_fr * sin_delta + v_wheel_y_fr * cos_delta;
        double v_fr = sqrt(v_tire_long_fr*v_tire_long_fr + v_tire_lat_fr*v_tire_lat_fr);
        
        double v_wheel_x_rl = vx - gamma * track_r/2;
        double v_wheel_y_rl = vy - gamma * lr;
        double v_rl = sqrt(v_wheel_x_rl*v_wheel_x_rl + v_wheel_y_rl*v_wheel_y_rl);
        
        double v_wheel_x_rr = vx + gamma * track_r/2;
        double v_wheel_y_rr = vy - gamma * lr;
        double v_rr = sqrt(v_wheel_x_rr*v_wheel_x_rr + v_wheel_y_rr*v_wheel_y_rr);
        
        Eigen::VectorXd h(7);
        h << ax, ay, gamma, v_fl, v_fr, v_rl, v_rr;
        return h;
    });
    
    // 测量雅可比矩阵
    ekf.setMeasurementJacobianFunction([m, Iz, Cf, Cr, lf, lr, track_f, track_r]
        (const Eigen::VectorXd& x) {
        
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(7, 7);
        
        double vx = x(0);
        double vy = x(1);
        double gamma = x(2);
        
        double delta = 0.05 * sin(0.5);
        double cos_delta = cos(delta);
        double sin_delta = sin(delta);
        
        // 加速度测量雅可比
        H(0, 3) = -sin_delta / m;
        H(0, 4) = -sin_delta / m;
        H(1, 3) = cos_delta / m;
        H(1, 4) = cos_delta / m;
        H(1, 5) = 1.0 / m;
        H(1, 6) = 1.0 / m;
        H(2, 2) = 1.0;
        
        // 轮速测量雅可比 (简化版本)
        H(3, 0) = 1.0;  H(3, 2) = -track_f/2;
        H(4, 0) = 1.0;  H(4, 2) = track_f/2;
        H(5, 0) = 1.0;  H(5, 2) = -track_r/2;
        H(6, 0) = 1.0;  H(6, 2) = track_r/2;
        
        return H;
    });
    
    // 创建数据输出文件
    std::ofstream dataFile("ekf_data.csv");
    dataFile << "time,vx,vy,gamma,Fy_fl,Fy_fr,Fy_rl,Fy_rr,sigma_vx,sigma_vy,sigma_gamma,sigma_Fy_fl,delta,Fx\n";
    
    // 仿真参数
    std::mt19937 gen(123);
    std::normal_distribution<> accel_noise(0, 0.3);
    std::normal_distribution<> gamma_noise(0, 0.005);
    std::normal_distribution<> wheel_noise(0, 0.1);
    
    double dt = 0.01;  // 10ms时间步长
    int steps = 500;   // 5秒仿真
    
    std::cout << "\nStarting simulation with CSV output..." << std::endl;
    std::cout << "- Time step: " << dt << "s" << std::endl;
    std::cout << "- Total steps: " << steps << std::endl;
    std::cout << "- Output file: ekf_data.csv" << std::endl;
    
    // 仿真循环
    for (int i = 0; i < steps; i++) {
        double t = i * dt;
        
        // 生成控制输入
        double steering_input = 0.1 * sin(0.5 * t);
        double brake_force = (t > 2.0 && t < 3.0) ? -500.0 : 1000.0;
        
        try {
            // EKF预测步骤
            ekf.predict(dt);
            
            // 生成带噪声的测量
            Eigen::VectorXd state = ekf.getState();
            double true_vx = state(0);
            double true_vy = state(1);
            double true_gamma = state(2);
            
            // 模拟真实测量值
            double true_ax = brake_force / m * 4;
            double true_ay = true_vy * true_gamma;
            double true_v_fl = sqrt(true_vx*true_vx + true_vy*true_vy) + gamma_noise(gen);
            double true_v_fr = sqrt(true_vx*true_vx + true_vy*true_vy) + gamma_noise(gen);
            double true_v_rl = sqrt(true_vx*true_vx + true_vy*true_vy) + gamma_noise(gen);
            double true_v_rr = sqrt(true_vx*true_vx + true_vy*true_vy) + gamma_noise(gen);
            
            Eigen::VectorXd z(measureSize);
            z << true_ax + accel_noise(gen),
                 true_ay + accel_noise(gen),
                 true_gamma + gamma_noise(gen),
                 true_v_fl + wheel_noise(gen),
                 true_v_fr + wheel_noise(gen),
                 true_v_rl + wheel_noise(gen),
                 true_v_rr + wheel_noise(gen);
            
            // EKF更新步骤
            ekf.update(z);
            
            // 获取估计结果
            Eigen::VectorXd estimated_state = ekf.getState();
            Eigen::MatrixXd cov = ekf.getCovariance();
            
            // 写入CSV文件
            dataFile << t << ","
                     << estimated_state(0) << "," << estimated_state(1) << "," << estimated_state(2) << ","
                     << estimated_state(3) << "," << estimated_state(4) << "," << estimated_state(5) << "," << estimated_state(6) << ","
                     << sqrt(cov(0,0)) << "," << sqrt(cov(1,1)) << "," << sqrt(cov(2,2)) << "," << sqrt(cov(3,3)) << ","
                     << steering_input << "," << brake_force << "\n";
            
            // 每100步打印一次进度
            if (i % 100 == 0) {
                std::cout << "Step " << i << "/" << steps << " (t=" << t << "s)" << std::endl;
            }
            
        } catch (const std::exception& e) {
            std::cerr << "Error at step " << i << ": " << e.what() << std::endl;
            break;
        }
    }
    
    dataFile.close();
    
    // 最终结果
    Eigen::VectorXd final_state = ekf.getState();
    std::cout << "\n=== Final Results ===" << std::endl;
    std::cout << "Final longitudinal velocity: " << final_state(0) << " m/s" << std::endl;
    std::cout << "Final lateral velocity: " << final_state(1) << " m/s" << std::endl;
    std::cout << "Final yaw rate: " << final_state(2)*180/M_PI << " °/s" << std::endl;
    
    std::cout << "\n✓ Data saved to ekf_data.csv" << std::endl;
    std::cout << "✓ Use plot_ekf_results.py to visualize the results" << std::endl;
    
    return 0;
}
