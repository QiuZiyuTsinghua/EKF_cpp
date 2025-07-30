#!/usr/bin/env python3
"""
Extended EKF Vehicle Dynamics Visualization
绘制扩展EKF车辆动力学测试结果
"""

import subprocess
import re
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from datetime import datetime

def run_ekf_test():
    """运行扩展EKF测试并解析输出"""
    print("Running Extended EKF test...")
    
    try:
        # 运行C++程序并捕获输出
        result = subprocess.run(['./build/ekf_extended_test'], 
                              capture_output=True, text=True, cwd='/home/ubuntu/code/dev')
        
        if result.returncode != 0:
            print(f"Error running EKF test: {result.stderr}")
            return None
            
        return result.stdout
    except Exception as e:
        print(f"Failed to run EKF test: {e}")
        return None

def parse_ekf_output(output):
    """解析EKF测试输出，提取数据"""
    data = {
        'time': [],
        'vx': [], 'vy': [], 'gamma': [],
        'Fy_fl': [], 'Fy_fr': [], 'Fy_rl': [], 'Fy_rr': [],
        'sigma_vx': [], 'sigma_vy': [], 'sigma_fy': [],
        'delta': [], 'Fx': []
    }
    
    # 正则表达式模式
    step_pattern = r"--- Step (\d+) \(t=([\d.]+)s\) ---"
    control_pattern = r"Control: δ=([-\d.]+)°, Fx=([-\d.]+)N"
    velocity_pattern = r"Velocity: vx=([-\d.]+), vy=([-\d.]+) m/s"
    yaw_pattern = r"Yaw rate: γ=([-\d.]+)°/s"
    tire_fl_fr_pattern = r"Tire forces: Fy_fl=([-\d.]+), Fy_fr=([-\d.]+) N"
    tire_rl_rr_pattern = r"            Fy_rl=([-\d.]+), Fy_rr=([-\d.]+) N"
    uncertainty_v_pattern = r"Uncertainty \(σ\): vx=([-\d.]+), vy=([-\d.]+) m/s"
    uncertainty_f_pattern = r"                Fy=([-\d.]+) N"
    
    lines = output.split('\n')
    i = 0
    
    while i < len(lines):
        line = lines[i].strip()
        
        # 匹配步骤信息
        step_match = re.search(step_pattern, line)
        if step_match:
            step_num = int(step_match.group(1))
            time_val = float(step_match.group(2))
            data['time'].append(time_val)
            
            # 解析接下来的几行数据
            try:
                # Control line
                i += 1
                control_match = re.search(control_pattern, lines[i])
                if control_match:
                    data['delta'].append(float(control_match.group(1)))
                    data['Fx'].append(float(control_match.group(2)))
                
                # Velocity line
                i += 1
                vel_match = re.search(velocity_pattern, lines[i])
                if vel_match:
                    data['vx'].append(float(vel_match.group(1)))
                    data['vy'].append(float(vel_match.group(2)))
                
                # Yaw rate line
                i += 1
                yaw_match = re.search(yaw_pattern, lines[i])
                if yaw_match:
                    data['gamma'].append(float(yaw_match.group(1)))
                
                # Tire forces line 1
                i += 1
                tire1_match = re.search(tire_fl_fr_pattern, lines[i])
                if tire1_match:
                    data['Fy_fl'].append(float(tire1_match.group(1)))
                    data['Fy_fr'].append(float(tire1_match.group(2)))
                
                # Tire forces line 2
                i += 1
                tire2_match = re.search(tire_rl_rr_pattern, lines[i])
                if tire2_match:
                    data['Fy_rl'].append(float(tire2_match.group(1)))
                    data['Fy_rr'].append(float(tire2_match.group(2)))
                
                # Uncertainty velocity line
                i += 1
                unc_v_match = re.search(uncertainty_v_pattern, lines[i])
                if unc_v_match:
                    data['sigma_vx'].append(float(unc_v_match.group(1)))
                    data['sigma_vy'].append(float(unc_v_match.group(2)))
                
                # Uncertainty force line
                i += 1
                unc_f_match = re.search(uncertainty_f_pattern, lines[i])
                if unc_f_match:
                    data['sigma_fy'].append(float(unc_f_match.group(1)))
                    
            except (IndexError, ValueError) as e:
                print(f"Warning: Could not parse data at step {step_num}: {e}")
        
        i += 1
    
    # 转换为numpy数组
    for key in data:
        data[key] = np.array(data[key])
    
    print(f"Parsed {len(data['time'])} data points")
    return data

def create_vehicle_visualization(data):
    """创建车辆动力学可视化图表"""
    
    if len(data['time']) == 0:
        print("No data to plot!")
        return
    
    # 设置中文字体
    plt.rcParams['font.sans-serif'] = ['SimHei', 'DejaVu Sans']
    plt.rcParams['axes.unicode_minus'] = False
    
    # 创建图形和子图
    fig = plt.figure(figsize=(16, 12))
    fig.suptitle('Extended EKF Vehicle Dynamics Analysis\n扩展EKF车辆动力学分析', fontsize=16, fontweight='bold')
    
    # 1. 车辆速度 (Vehicle Velocity)
    ax1 = plt.subplot(3, 3, 1)
    ax1.plot(data['time'], data['vx'], 'b-', linewidth=2, label='纵向速度 vx')
    ax1.fill_between(data['time'], 
                     data['vx'] - data['sigma_vx'], 
                     data['vx'] + data['sigma_vx'], 
                     alpha=0.3, color='blue')
    ax1.plot(data['time'], data['vy'], 'r-', linewidth=2, label='横向速度 vy')
    ax1.fill_between(data['time'], 
                     data['vy'] - data['sigma_vy'], 
                     data['vy'] + data['sigma_vy'], 
                     alpha=0.3, color='red')
    ax1.set_ylabel('速度 (m/s)')
    ax1.set_xlabel('时间 (s)')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    ax1.set_title('车辆速度估计\nVehicle Velocity Estimation')
    
    # 2. 横摆角速度 (Yaw Rate)
    ax2 = plt.subplot(3, 3, 2)
    ax2.plot(data['time'], data['gamma'], 'g-', linewidth=2, label='横摆角速度 γ')
    ax2.set_ylabel('横摆角速度 (°/s)')
    ax2.set_xlabel('时间 (s)')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    ax2.set_title('横摆动力学\nYaw Dynamics')
    
    # 3. 轮胎横向力 (Tire Lateral Forces)
    ax3 = plt.subplot(3, 3, 3)
    ax3.plot(data['time'], data['Fy_fl'], 'b-', linewidth=2, label='前左 FL')
    ax3.plot(data['time'], data['Fy_fr'], 'r-', linewidth=2, label='前右 FR')
    ax3.plot(data['time'], data['Fy_rl'], 'g-', linewidth=2, label='后左 RL')
    ax3.plot(data['time'], data['Fy_rr'], 'm-', linewidth=2, label='后右 RR')
    ax3.set_ylabel('横向力 (N)')
    ax3.set_xlabel('时间 (s)')
    ax3.legend()
    ax3.grid(True, alpha=0.3)
    ax3.set_title('轮胎横向力估计\nTire Lateral Force Estimation')
    
    # 4. 控制输入 (Control Inputs)
    ax4 = plt.subplot(3, 3, 4)
    ax4.plot(data['time'], data['delta'], 'purple', linewidth=2, label='转向角 δ')
    ax4.set_ylabel('转向角 (°)')
    ax4.set_xlabel('时间 (s)')
    ax4.legend()
    ax4.grid(True, alpha=0.3)
    ax4.set_title('转向输入\nSteering Input')
    
    # 5. 纵向力输入 (Longitudinal Force Input)
    ax5 = plt.subplot(3, 3, 5)
    ax5.plot(data['time'], data['Fx'], 'orange', linewidth=2, label='纵向力 Fx')
    ax5.set_ylabel('纵向力 (N)')
    ax5.set_xlabel('时间 (s)')
    ax5.legend()
    ax5.grid(True, alpha=0.3)
    ax5.set_title('纵向力输入\nLongitudinal Force Input')
    
    # 6. 估计不确定性 (Estimation Uncertainty)
    ax6 = plt.subplot(3, 3, 6)
    ax6.semilogy(data['time'], data['sigma_vx'], 'b-', linewidth=2, label='σ_vx')
    ax6.semilogy(data['time'], data['sigma_vy'], 'r-', linewidth=2, label='σ_vy')
    ax6.semilogy(data['time'], data['sigma_fy'], 'g-', linewidth=2, label='σ_Fy')
    ax6.set_ylabel('标准差 (对数尺度)')
    ax6.set_xlabel('时间 (s)')
    ax6.legend()
    ax6.grid(True, alpha=0.3)
    ax6.set_title('估计不确定性\nEstimation Uncertainty')
    
    # 7. 车辆轨迹和轮胎力可视化 (Vehicle Trajectory and Forces)
    ax7 = plt.subplot(3, 3, 7)
    
    # 计算车辆位置 (简化积分)
    if len(data['time']) > 1:
        dt = np.diff(data['time'])
        x_pos = np.cumsum(np.concatenate([[0], data['vx'][:-1] * dt]))
        y_pos = np.cumsum(np.concatenate([[0], data['vy'][:-1] * dt]))
        
        ax7.plot(x_pos, y_pos, 'k-', linewidth=2, label='车辆轨迹')
        ax7.scatter(x_pos[0], y_pos[0], color='green', s=100, marker='o', label='起点')
        ax7.scatter(x_pos[-1], y_pos[-1], color='red', s=100, marker='s', label='终点')
        
        # 绘制车辆示意图 (最后位置)
        vehicle_length = 4.0
        vehicle_width = 2.0
        vehicle = patches.Rectangle((x_pos[-1] - vehicle_length/2, y_pos[-1] - vehicle_width/2),
                                   vehicle_length, vehicle_width, 
                                   linewidth=2, edgecolor='blue', facecolor='lightblue', alpha=0.7)
        ax7.add_patch(vehicle)
    
    ax7.set_xlabel('X 位置 (m)')
    ax7.set_ylabel('Y 位置 (m)')
    ax7.legend()
    ax7.grid(True, alpha=0.3)
    ax7.axis('equal')
    ax7.set_title('车辆轨迹\nVehicle Trajectory')
    
    # 8. 速度矢量图 (Velocity Vector Plot)
    ax8 = plt.subplot(3, 3, 8)
    speed = np.sqrt(data['vx']**2 + data['vy']**2)
    slip_angle = np.arctan2(data['vy'], data['vx']) * 180 / np.pi
    
    ax8.plot(data['time'], speed, 'b-', linewidth=2, label='车速 |v|')
    ax8_twin = ax8.twinx()
    ax8_twin.plot(data['time'], slip_angle, 'r-', linewidth=2, label='侧滑角 β')
    
    ax8.set_xlabel('时间 (s)')
    ax8.set_ylabel('车速 (m/s)', color='blue')
    ax8_twin.set_ylabel('侧滑角 (°)', color='red')
    ax8.grid(True, alpha=0.3)
    ax8.set_title('车速与侧滑角\nSpeed and Slip Angle')
    
    # 9. 轮胎力分布 (Tire Force Distribution)
    ax9 = plt.subplot(3, 3, 9)
    time_indices = [0, len(data['time'])//4, len(data['time'])//2, 3*len(data['time'])//4, -1]
    
    for i, idx in enumerate(time_indices):
        if idx >= len(data['time']):
            continue
        forces = [data['Fy_fl'][idx], data['Fy_fr'][idx], data['Fy_rl'][idx], data['Fy_rr'][idx]]
        positions = ['FL', 'FR', 'RL', 'RR']
        colors = ['blue', 'red', 'green', 'magenta']
        
        bars = ax9.bar([p + i*0.15 for p in range(4)], forces, 
                      width=0.15, alpha=0.7, color=colors, 
                      label=f't={data["time"][idx]:.1f}s')
    
    ax9.set_xticks(range(4))
    ax9.set_xticklabels(['前左FL', '前右FR', '后左RL', '后右RR'])
    ax9.set_ylabel('横向力 (N)')
    ax9.legend()
    ax9.grid(True, alpha=0.3)
    ax9.set_title('轮胎力分布\nTire Force Distribution')
    
    plt.tight_layout()
    
    # 保存图片
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f'/home/ubuntu/code/dev/ekf_results_{timestamp}.png'
    plt.savefig(filename, dpi=300, bbox_inches='tight')
    print(f"Plot saved as: {filename}")
    
    plt.show()
    
    return fig

def create_summary_report(data):
    """创建测试结果摘要报告"""
    if len(data['time']) == 0:
        return
    
    print("\n" + "="*60)
    print("Extended EKF Vehicle Dynamics Test Summary")
    print("扩展EKF车辆动力学测试摘要")
    print("="*60)
    
    # 计算统计信息
    final_vx = data['vx'][-1]
    final_vy = data['vy'][-1]
    final_speed = np.sqrt(final_vx**2 + final_vy**2)
    avg_sigma_vx = np.mean(data['sigma_vx'])
    avg_sigma_vy = np.mean(data['sigma_vy'])
    max_tire_force = np.max([np.max(data['Fy_fl']), np.max(data['Fy_fr']), 
                            np.max(data['Fy_rl']), np.max(data['Fy_rr'])])
    
    print(f"仿真时长: {data['time'][-1]:.1f} 秒")
    print(f"数据点数: {len(data['time'])} 个")
    print(f"\n最终状态:")
    print(f"  纵向速度: {final_vx:.2f} ± {data['sigma_vx'][-1]:.3f} m/s")
    print(f"  横向速度: {final_vy:.3f} ± {data['sigma_vy'][-1]:.3f} m/s")
    print(f"  总车速: {final_speed:.2f} m/s ({final_speed*3.6:.1f} km/h)")
    print(f"  横摆角速度: {data['gamma'][-1]:.2f} °/s")
    
    print(f"\n估计性能:")
    print(f"  平均纵向速度不确定性: {avg_sigma_vx:.3f} m/s")
    print(f"  平均横向速度不确定性: {avg_sigma_vy:.3f} m/s")
    print(f"  最大轮胎横向力: {max_tire_force:.1f} N")
    
    print(f"\n轮胎力分布 (最终):")
    print(f"  前轮: FL={data['Fy_fl'][-1]:.1f}N, FR={data['Fy_fr'][-1]:.1f}N")
    print(f"  后轮: RL={data['Fy_rl'][-1]:.1f}N, RR={data['Fy_rr'][-1]:.1f}N")
    
    # 性能评估
    print(f"\n性能评估:")
    convergence_vx = data['sigma_vx'][-1] / data['sigma_vx'][0] if data['sigma_vx'][0] > 0 else 1.0
    convergence_vy = data['sigma_vy'][-1] / data['sigma_vy'][0] if data['sigma_vy'][0] > 0 else 1.0
    
    print(f"  纵向速度收敛比: {convergence_vx:.3f}")
    print(f"  横向速度收敛比: {convergence_vy:.3f}")
    
    if convergence_vx < 1.0 and convergence_vy < 5.0:
        print("  ✓ EKF估计收敛良好")
    else:
        print("  ⚠ EKF估计收敛需要改进")
    
    if final_speed > 10.0 and final_speed < 35.0:
        print("  ✓ 最终车速在合理范围内")
    else:
        print("  ⚠ 最终车速可能不合理")
    
    if max_tire_force < 5000.0:
        print("  ✓ 轮胎力在合理范围内")
    else:
        print("  ⚠ 轮胎力可能过大")
    
    print("="*60)

def main():
    """主函数"""
    print("Extended EKF Vehicle Dynamics Visualization Tool")
    print("扩展EKF车辆动力学可视化工具")
    print("-" * 50)
    
    # 运行EKF测试
    output = run_ekf_test()
    if output is None:
        print("Failed to run EKF test. Exiting.")
        return
    
    # 解析输出数据
    data = parse_ekf_output(output)
    if len(data['time']) == 0:
        print("No data extracted from EKF test output. Exiting.")
        return
    
    # 创建可视化图表
    try:
        fig = create_vehicle_visualization(data)
        create_summary_report(data)
        
        print("\nVisualization complete! Check the saved plot file.")
        
    except Exception as e:
        print(f"Error creating visualization: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()
