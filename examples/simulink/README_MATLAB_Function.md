# EKF Vehicle Velocity Estimator - MATLAB Function Implementation

本文档说明如何使用MATLAB函数版本的扩展卡尔曼滤波器(EKF)车速估计器，该实现与原始S-Function版本功能完全一致。

## 文件说明

### 1. `ekf_vehicle_velocity_estimator.m`
完整的MATLAB函数实现，包含所有参数配置功能，适用于：
- MATLAB脚本调用
- 需要灵活参数配置的应用
- 测试和验证
- **注意**: 已修复嵌套函数问题，但参数较多

### 2. `ekf_vehicle_estimator_simple.m` **[推荐用于Simulink]**
高度优化的简化版本，专门为Simulink设计：
- 只需单一输入（测量向量）
- 硬编码参数，避免复杂配置
- 支持代码生成(`#codegen`)
- 最适合Simulink实时仿真
- 紧凑的代码结构

### 3. `ekf_simulink_function.m`
中间版本，保留了一些灵活性但简化了接口

### 4. `test_ekf_matlab_function.m`
测试脚本和Simulink集成示例

## 输入输出接口

### 输入
- **measurements** [7×1]: 测量向量
  - `measurements(1)`: 纵向加速度 ax (m/s²)
  - `measurements(2)`: 横向加速度 ay (m/s²)
  - `measurements(3)`: 横摆角速度 gamma (rad/s)
  - `measurements(4)`: 左前轮速 v_fl (m/s)
  - `measurements(5)`: 右前轮速 v_fr (m/s)
  - `measurements(6)`: 左后轮速 v_rl (m/s)
  - `measurements(7)`: 右后轮速 v_rr (m/s)

- **control_input** [1×1]: 控制输入(方向盘转角，当前未使用)

### 输出
- **state_estimate** [3×1]: 状态估计向量
  - `state_estimate(1)`: 纵向速度 v_x (m/s)
  - `state_estimate(2)`: 横向速度 v_y (m/s)
  - `state_estimate(3)`: 横摆角速度 gamma (rad/s)

- **covariance_matrix** [9×1]: 展平的协方差矩阵(3×3 → 9×1)

## 在Simulink中使用

### 方法1: 使用简化版本 (`ekf_vehicle_estimator_simple.m`) **[强烈推荐]**

这是最简单且最可靠的方法：

1. 在Simulink模型中添加"MATLAB Function"块
2. 双击块打开编辑器
3. 将`ekf_vehicle_estimator_simple.m`的完整内容复制到编辑器中
4. 配置输入端口：
   ```
   measurements [7×1]  % 测量向量
   ```
5. 配置输出端口：
   ```
   state_out [3×1]      % 状态估计
   covariance_out [9×1] % 协方差矩阵（展平）
   ```
6. 设置采样时间为0.01秒（或修改代码中的dt值）

**优点**: 
- 无需复杂参数配置
- 代码紧凑，易于调试
- 支持代码生成
- 兼容性最好

### 方法2: 使用完整版本 (`ekf_vehicle_velocity_estimator.m`) **[高级用户]**

适用于需要自定义参数的情况：

1. 在Simulink模型中添加"MATLAB Function"块
2. 双击块打开编辑器
3. 将`ekf_vehicle_velocity_estimator.m`的内容复制到编辑器中
4. 配置输入端口：
   ```
   measurements     [7×1]
   control_input    [1×1]
   state_dim        scalar (值: 3)
   meas_dim         scalar (值: 7)
   ctrl_dim         scalar (值: 1)
   dt               scalar (时间步长)
   initial_state    [3×1]
   initial_cov      [3×3]
   process_noise_cov [3×3]
   meas_noise_cov   [7×7]
   ```
5. 配置输出端口：
   ```
   state_estimate   [3×1]
   covariance_matrix [9×1]
   ```

**注意**: 这种方法需要更多的配置工作。

## 车辆动力学模型

### 状态方程
```
状态向量: x = [v_x, v_y, γ]ᵀ

状态转移方程:
v̇_x = a_x + v_y·γ
v̇_y = a_y - v_x·γ  
γ̇ = M_z/I_z
```

其中：
- `a_x`: 纵向加速度
- `a_y`: 横向加速度 = (F_yf + F_yr)/m
- `M_z`: 横摆力矩 = F_yf·l_f - F_yr·l_r

### 轮胎模型
```
前轮侧偏角: α_f = β - l_f·γ/v_x
后轮侧偏角: α_r = β + l_r·γ/v_x
车辆侧偏角: β = arctan(v_y/v_x)

轮胎侧向力:
F_yf = -C_f·α_f
F_yr = -C_r·α_r
```

### 测量方程
```
测量向量: z = [a_x, a_y, γ, v_fl, v_fr, v_rl, v_rr]ᵀ

a_x = v_y·γ (向心加速度分量)
a_y = (F_yf + F_yr)/m - v_x·γ
γ_meas = γ (直接测量)
v_fl = v_x - γ·track/2
v_fr = v_x + γ·track/2
v_rl = v_x - γ·track/2  
v_rr = v_x + γ·track/2
```

## 参数配置

### 车辆参数
```matlab
m = 1500.0;      % 车辆质量 (kg)
Iz = 2500.0;     % 横摆转动惯量 (kg·m²)
lf = 1.2;        % 质心到前轴距离 (m)
lr = 1.4;        % 质心到后轴距离 (m)
track = 1.6;     % 轮距 (m)
Cf = 50000.0;    % 前轮侧偏刚度 (N/rad)
Cr = 50000.0;    % 后轮侧偏刚度 (N/rad)
```

### 滤波器参数
```matlab
% 过程噪声协方差矩阵 Q [3×3]
Q = diag([0.1, 0.1, 0.01]);

% 测量噪声协方差矩阵 R [7×7]
R = diag([0.5, 0.5, 0.01, 0.2, 0.2, 0.2, 0.2]);

% 初始状态协方差 P0 [3×3]
P0 = diag([1.0, 1.0, 0.1]);
```

## 测试运行

```matlab
% 运行测试脚本
run('test_ekf_matlab_function.m')
```

这将：
1. 测试EKF函数的基本功能
2. 创建Simulink集成所需的工作空间变量
3. 显示详细的使用说明

## 与S-Function版本的对比

| 特性 | S-Function版本 | MATLAB Function版本 |
|------|----------------|---------------------|
| 编译需求 | 需要MEX编译 | 无需编译 |
| 调试难度 | 较难 | 容易 |
| 代码生成 | 支持 | 支持(`#codegen`) |
| 参数配置 | 通过Simulink参数 | 函数参数或硬编码 |
| 性能 | 更高 | 略低 |
| 可移植性 | 需要编译环境 | 仅需MATLAB |

## 注意事项

1. **数值稳定性**: 实现中使用Joseph形式的协方差更新以提高数值稳定性
2. **奇点处理**: 在低速时避免除零错误
3. **持久变量**: 在Simulink中使用`persistent`变量保持状态
4. **代码生成**: 简化版本支持Simulink Coder的代码生成功能

## 故障排除

### 常见问题

1. **状态发散**: 检查过程噪声协方差矩阵Q是否合适
2. **测量不匹配**: 确认测量向量的单位和顺序
3. **低速不稳定**: 在计算侧偏角时添加速度阈值检查
4. **Simulink错误**: 确保输入输出维度匹配

### 调试建议

1. 首先在MATLAB脚本中测试函数
2. 使用`test_ekf_matlab_function.m`验证基本功能
3. 在Simulink中逐步添加输入信号
4. 使用Scope块监控状态估计结果
