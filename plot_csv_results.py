#!/usr/bin/env python3
"""
Simple EKF Results Plotter - English Version
Reads CSV data and creates plots
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from datetime import datetime
import os

def plot_ekf_csv_data(csv_file='ekf_data.csv'):
    """Read data from CSV file and plot"""
    
    if not os.path.exists(csv_file):
        print(f"CSV file {csv_file} not found. Please run the EKF test first.")
        return
    
    # Read CSV data
    try:
        data = pd.read_csv(csv_file)
        print(f"Loaded {len(data)} data points from {csv_file}")
    except Exception as e:
        print(f"Error reading CSV file: {e}")
        return
    
    # 使用英文字体
    plt.rcParams['font.sans-serif'] = ['DejaVu Sans']
    plt.rcParams['axes.unicode_minus'] = False
    
    # Create figure
    fig, axes = plt.subplots(3, 3, figsize=(15, 12))
    fig.suptitle('Extended EKF Vehicle Dynamics Results', fontsize=16, fontweight='bold')
    
    # 1. Vehicle velocity
    ax = axes[0, 0]
    ax.plot(data['time'], data['vx'], 'b-', linewidth=2, label='Longitudinal Velocity vx')
    ax.fill_between(data['time'], 
                    data['vx'] - data['sigma_vx'], 
                    data['vx'] + data['sigma_vx'], 
                    alpha=0.3, color='blue')
    ax.plot(data['time'], data['vy'], 'r-', linewidth=2, label='Lateral Velocity vy')
    ax.fill_between(data['time'], 
                    data['vy'] - data['sigma_vy'], 
                    data['vy'] + data['sigma_vy'], 
                    alpha=0.3, color='red')
    ax.set_ylabel('Velocity (m/s)')
    ax.set_xlabel('Time (s)')
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_title('Vehicle Velocity Estimation')
    
    # 2. Yaw rate
    ax = axes[0, 1]
    ax.plot(data['time'], data['gamma'] * 180 / np.pi, 'g-', linewidth=2, label='Yaw Rate γ')
    ax.fill_between(data['time'], 
                    (data['gamma'] - data['sigma_gamma']) * 180 / np.pi, 
                    (data['gamma'] + data['sigma_gamma']) * 180 / np.pi, 
                    alpha=0.3, color='green')
    ax.set_ylabel('Yaw Rate (deg/s)')
    ax.set_xlabel('Time (s)')
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_title('Yaw Dynamics')
    
    # 3. Tire lateral force
    ax = axes[0, 2]
    ax.plot(data['time'], data['Fy_fl'], 'b-', linewidth=2, label='Front Left FL')
    ax.plot(data['time'], data['Fy_fr'], 'r-', linewidth=2, label='Front Right FR')
    ax.plot(data['time'], data['Fy_rl'], 'g-', linewidth=2, label='Rear Left RL')
    ax.plot(data['time'], data['Fy_rr'], 'm-', linewidth=2, label='Rear Right RR')
    ax.set_ylabel('Lateral Force (N)')
    ax.set_xlabel('Time (s)')
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_title('Tire Lateral Force Estimation')
    
    # 4. Control inputs
    ax = axes[1, 0]
    ax.plot(data['time'], data['delta'] * 180 / np.pi, 'purple', linewidth=2, label='Steering Angle δ')
    ax.set_ylabel('Steering Angle (deg)')
    ax.set_xlabel('Time (s)')
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_title('Control Inputs')
    
    # 5. Longitudinal force input
    ax = axes[1, 1]
    ax.plot(data['time'], data['Fx'], 'orange', linewidth=2, label='Longitudinal Force Fx')
    ax.set_ylabel('Longitudinal Force (N)')
    ax.set_xlabel('Time (s)')
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_title('Longitudinal Force Input')
    
    # 6. Estimation uncertainty
    ax = axes[1, 2]
    ax.semilogy(data['time'], data['sigma_vx'], 'b-', linewidth=2, label='σ_vx')
    ax.semilogy(data['time'], data['sigma_vy'], 'r-', linewidth=2, label='σ_vy')
    ax.semilogy(data['time'], data['sigma_Fy_fl'], 'g-', linewidth=2, label='σ_Fy')
    ax.set_ylabel('Standard Deviation (Log Scale)')
    ax.set_xlabel('Time (s)')
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_title('Estimation Uncertainty')
    
    # 7. Vehicle trajectory
    ax = axes[2, 0]
    # Calculate vehicle position (simplified integration)
    dt_vals = np.diff(data['time'])
    x_pos = np.cumsum(np.concatenate([[0], data['vx'].iloc[:-1] * dt_vals]))
    y_pos = np.cumsum(np.concatenate([[0], data['vy'].iloc[:-1] * dt_vals]))
    
    ax.plot(x_pos, y_pos, 'k-', linewidth=2, label='Vehicle Trajectory')
    ax.scatter(x_pos[0], y_pos[0], color='green', s=100, marker='o', label='Start')
    ax.scatter(x_pos[-1], y_pos[-1], color='red', s=100, marker='s', label='End')
    ax.set_xlabel('X Position (m)')
    ax.set_ylabel('Y Position (m)')
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.axis('equal')
    ax.set_title('Vehicle Trajectory')
    
    # 8. Vehicle speed and slip angle
    ax = axes[2, 1]
    speed = np.sqrt(data['vx']**2 + data['vy']**2)
    slip_angle = np.arctan2(data['vy'], data['vx']) * 180 / np.pi
    
    ax.plot(data['time'], speed, 'b-', linewidth=2, label='Speed |v|')
    ax_twin = ax.twinx()
    ax_twin.plot(data['time'], slip_angle, 'r-', linewidth=2, label='Slip Angle β')
    
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Speed (m/s)', color='blue')
    ax_twin.set_ylabel('Slip Angle (°)', color='red')
    ax.grid(True, alpha=0.3)
    ax.set_title('Vehicle Speed and Slip Angle')
    
    # 9. Tire force resultant
    ax = axes[2, 2]
    front_force = data['Fy_fl'] + data['Fy_fr']
    rear_force = data['Fy_rl'] + data['Fy_rr']
    total_force = front_force + rear_force
    
    ax.plot(data['time'], front_force, 'b-', linewidth=2, label='Front Force')
    ax.plot(data['time'], rear_force, 'r-', linewidth=2, label='Rear Force')
    ax.plot(data['time'], total_force, 'k--', linewidth=2, label='Total Force')
    ax.set_ylabel('Lateral Force (N)')
    ax.set_xlabel('Time (s)')
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_title('Tire Lateral Force Resultant')
    
    plt.tight_layout()
    
    # Save plot
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f'ekf_results_{timestamp}.png'
    plt.savefig(filename, dpi=300, bbox_inches='tight')
    print(f"Plot saved as: {filename}")
    
    plt.show()
    
    # 打印统计摘要
    print_summary(data)

def print_summary(data):
    """打印数据摘要"""
    print("\n" + "="*60)
    print("Extended EKF Test Summary")
    print("="*60)
    
    final_vx = data['vx'].iloc[-1]
    final_vy = data['vy'].iloc[-1]
    final_speed = np.sqrt(final_vx**2 + final_vy**2)
    final_gamma = data['gamma'].iloc[-1] * 180 / np.pi
    
    print(f"Simulation duration: {data['time'].iloc[-1]:.1f} seconds")
    print(f"Data points: {len(data)}")
    print(f"Time step: {data['time'].iloc[1] - data['time'].iloc[0]:.3f} seconds")
    
    print(f"\nFinal state:")
    print(f"  Longitudinal velocity: {final_vx:.2f} ± {data['sigma_vx'].iloc[-1]:.3f} m/s")
    print(f"  Lateral velocity: {final_vy:.3f} ± {data['sigma_vy'].iloc[-1]:.3f} m/s")
    print(f"  Total speed: {final_speed:.2f} m/s ({final_speed*3.6:.1f} km/h)")
    print(f"  Yaw rate: {final_gamma:.2f} °/s")
    
    print(f"\nTire force statistics (N):")
    print(f"  Front left: avg={data['Fy_fl'].mean():.1f}, max={data['Fy_fl'].max():.1f}")
    print(f"  Front right: avg={data['Fy_fr'].mean():.1f}, max={data['Fy_fr'].max():.1f}")
    print(f"  Rear left: avg={data['Fy_rl'].mean():.1f}, max={data['Fy_rl'].max():.1f}")
    print(f"  Rear right: avg={data['Fy_rr'].mean():.1f}, max={data['Fy_rr'].max():.1f}")
    
    print(f"\nEstimation performance:")
    avg_sigma_vx = data['sigma_vx'].mean()
    avg_sigma_vy = data['sigma_vy'].mean()
    convergence_vx = data['sigma_vx'].iloc[-1] / data['sigma_vx'].iloc[0]
    convergence_vy = data['sigma_vy'].iloc[-1] / data['sigma_vy'].iloc[0]
    
    print(f"  Average longitudinal velocity uncertainty: {avg_sigma_vx:.3f} m/s")
    print(f"  Average lateral velocity uncertainty: {avg_sigma_vy:.3f} m/s")
    print(f"  Longitudinal velocity convergence ratio: {convergence_vx:.3f}")
    print(f"  Lateral velocity convergence ratio: {convergence_vy:.3f}")
    
    # Performance assessment
    print(f"\nPerformance assessment:")
    if convergence_vx < 1.0 and convergence_vy < 5.0:
        print("  ✓ EKF estimation converges well")
    else:
        print("  ⚠ EKF estimation convergence needs improvement")
    
    if 10.0 < final_speed < 35.0:
        print("  ✓ Final speed within reasonable range")
    else:
        print("  ⚠ Final speed may be unreasonable")
    
    max_tire_force = max(data['Fy_fl'].max(), data['Fy_fr'].max(), 
                        data['Fy_rl'].max(), data['Fy_rr'].max())
    if max_tire_force < 5000.0:
        print("  ✓ Tire forces within reasonable range")
    else:
        print("  ⚠ Tire forces may be too large")
    
    print("="*60)

def main():
    """Main function"""
    print("Simple EKF CSV Results Plotter")
    print("English Version")
    print("-" * 40)
    
    # 检查是否存在CSV文件
    csv_file = 'ekf_data.csv'
    if not os.path.exists(csv_file):
        print(f"CSV file '{csv_file}' not found.")
        print("Please run the EKF test with output first:")
        print("  ./build/ekf_with_output")
        return
    
    # 绘制结果
    try:
        plot_ekf_csv_data(csv_file)
        print("\nVisualization complete!")
        
    except Exception as e:
        print(f"Error creating visualization: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()
