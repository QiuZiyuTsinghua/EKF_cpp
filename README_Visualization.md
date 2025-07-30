# Extended EKF Vehicle Dynamics Visualization Tools
# 扩展EKF车辆动力学可视化工具

This repository contains tools for visualizing the results of Extended Kalman Filter (EKF) applied to vehicle dynamics estimation.

本仓库包含用于可视化扩展卡尔曼滤波器(EKF)在车辆动力学估计中应用结果的工具。

## Quick Start / 快速开始

### 1. Compile and Run EKF Test with Data Output
### 1. 编译并运行带数据输出的EKF测试

```bash
# Build the project
cd /home/ubuntu/code/dev
cmake -B build .
make -C build ekf_with_output

# Run the EKF test to generate CSV data
./build/ekf_with_output
```

This will generate `ekf_data.csv` containing the simulation results.
这将生成包含仿真结果的 `ekf_data.csv` 文件。

### 2. Create Visualizations
### 2. 创建可视化图表

```bash
# Install required Python packages (if not already installed)
sudo apt update && sudo apt install -y python3-pandas python3-matplotlib python3-numpy

# Run the plotting script
python3 plot_csv_results.py
```

This will create a comprehensive visualization and save it as `ekf_results_YYYYMMDD_HHMMSS.png`.
这将创建一个综合可视化图表并保存为 `ekf_results_YYYYMMDD_HHMMSS.png`。

## Program Files / 程序文件

### C++ Programs / C++程序

1. **`main_extended_ekf_test.cpp`** - Extended EKF test with console output
   - 扩展EKF测试程序，带控制台输出
   - Executable: `ekf_extended_test`

2. **`main_ekf_with_output.cpp`** - Extended EKF test with CSV data output
   - 扩展EKF测试程序，带CSV数据输出
   - Executable: `ekf_with_output`
   - Generates: `ekf_data.csv`

### Python Scripts / Python脚本

1. **`plot_csv_results.py`** - Simple CSV data visualization
   - 简单CSV数据可视化
   - Reads `ekf_data.csv` and creates comprehensive plots
   - 读取 `ekf_data.csv` 并创建综合图表

2. **`plot_ekf_results.py`** - Advanced visualization (parses console output)
   - 高级可视化工具（解析控制台输出）
   - Runs EKF test automatically and parses output
   - 自动运行EKF测试并解析输出

## Extended EKF Model / 扩展EKF模型

### State Vector (7D) / 状态向量 (7维)
```
x = [vx, vy, γ, Fy_fl, Fy_fr, Fy_rl, Fy_rr]ᵀ
```
- `vx`: Longitudinal velocity (m/s) / 纵向速度
- `vy`: Lateral velocity (m/s) / 横向速度  
- `γ`: Yaw rate (rad/s) / 横摆角速度
- `Fy_fl`: Front-left tire lateral force (N) / 左前轮横向力
- `Fy_fr`: Front-right tire lateral force (N) / 右前轮横向力
- `Fy_rl`: Rear-left tire lateral force (N) / 左后轮横向力
- `Fy_rr`: Rear-right tire lateral force (N) / 右后轮横向力

### Measurement Vector (7D) / 测量向量 (7维)
```
z = [ax, ay, γ, v_fl, v_fr, v_rl, v_rr]ᵀ
```
- `ax, ay`: Longitudinal and lateral accelerations (m/s²) / 纵向和横向加速度
- `γ`: Yaw rate measurement (rad/s) / 横摆角速度测量
- `v_fl, v_fr, v_rl, v_rr`: Wheel speeds (m/s) / 车轮速度

### Vehicle Parameters / 车辆参数
- Mass: 1500 kg / 质量: 1500 kg
- Inertia: 2500 kg⋅m² / 转动惯量: 2500 kg⋅m²
- Wheelbase: 2.6 m (lf=1.2m, lr=1.4m) / 轴距: 2.6 m
- Track width: 1.6 m / 轮距: 1.6 m
- Cornering stiffness: 50000 N/rad / 侧偏刚度: 50000 N/rad

## Visualization Features / 可视化功能

The generated plots include / 生成的图表包括:

1. **Vehicle Velocity Estimation** / 车辆速度估计
   - Longitudinal and lateral velocities with uncertainty bands
   - 纵向和横向速度及其不确定性带

2. **Yaw Dynamics** / 横摆动力学
   - Yaw rate evolution over time
   - 横摆角速度随时间变化

3. **Tire Force Estimation** / 轮胎力估计
   - Individual tire lateral forces for all four wheels
   - 四个车轮的独立横向力

4. **Control Inputs** / 控制输入
   - Steering angle and longitudinal force inputs
   - 转向角和纵向力输入

5. **Estimation Uncertainty** / 估计不确定性
   - Standard deviations of state estimates (log scale)
   - 状态估计的标准差（对数尺度）

6. **Vehicle Trajectory** / 车辆轨迹
   - 2D path based on integrated velocities
   - 基于速度积分的2D路径

7. **Speed and Slip Angle** / 车速和侧滑角
   - Total speed and vehicle slip angle
   - 总车速和车辆侧滑角

8. **Tire Force Distribution** / 轮胎力分布
   - Combined forces and force distribution
   - 合力和力分布

## Performance Metrics / 性能指标

The visualization includes automatic analysis of:
可视化包括以下自动分析:

- **Convergence Analysis** / 收敛性分析
  - Uncertainty reduction over time
  - 不确定性随时间的减少

- **Physical Validity** / 物理有效性
  - Reasonable velocity and force ranges
  - 合理的速度和力范围

- **Estimation Quality** / 估计质量
  - Final state accuracy and uncertainty
  - 最终状态精度和不确定性

## Example Output / 输出示例

```
仿真时长: 5.0 秒
数据点数: 500 个
最终状态:
  纵向速度: 28.07 ± 0.191 m/s
  横向速度: -0.261 ± 5.024 m/s
  总车速: 28.07 m/s (101.1 km/h)
轮胎力统计:
  前左轮: 平均=264.5N, 最大=522.8N
  后左轮: 平均=535.2N, 最大=1002.2N
性能评估:
  ✓ 最终车速在合理范围内
  ✓ 轮胎力在合理范围内
```

## Troubleshooting / 故障排除

### Chinese Font Issues / 中文字体问题
If Chinese characters don't display properly in plots:
如果图表中中文字符显示不正常:

```bash
# Install Chinese fonts
sudo apt install fonts-noto-cjk

# Or modify the script to use English-only labels
# 或修改脚本使用纯英文标签
```

### Package Dependencies / 包依赖
Required packages / 必需的包:
- `python3-pandas` - Data manipulation / 数据处理
- `python3-matplotlib` - Plotting / 绘图
- `python3-numpy` - Numerical computation / 数值计算

## Integration with Simulink / 与Simulink集成

This visualization tool is designed to work with the Extended EKF Guide for Simulink S-Function development. The results help validate the vehicle dynamics model before implementing it in Simulink.

本可视化工具设计用于配合Simulink S-Function开发的扩展EKF指南。结果有助于在Simulink中实现之前验证车辆动力学模型。

---

For more information, see the Extended_EKF_Guide.md in the examples/simulink directory.
更多信息请参见 examples/simulink 目录下的 Extended_EKF_Guide.md。
