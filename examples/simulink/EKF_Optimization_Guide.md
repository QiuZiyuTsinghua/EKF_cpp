# EKF 横向速度估计优化指南

## 问题诊断

横向速度估计偏大的常见原因：

1. **轮胎模型过于简化**
   - 线性轮胎模型在大侧偏角时不准确
   - 没有考虑轮胎饱和特性

2. **传感器标定问题**
   - 加速度计安装偏差
   - 轮速传感器精度差异

3. **模型参数不匹配**
   - 车辆参数与实际不符
   - 轮胎刚度参数过大

## 已实施的改进

### 1. 调整噪声协方差矩阵
```matlab
% 降低横向速度的过程噪声，提高约束性
Q = diag([0.1, 0.05, 0.01]);  % [vx, vy, gamma]

% 提高横向加速度测量的可信度
R = diag([0.5, 0.3, 0.01, 0.2, 0.2, 0.2, 0.2]);
```

### 2. 轮胎力限制
```matlab
% 限制侧偏角避免非线性区域
alpha_f = max(-0.2, min(0.2, alpha_f));  % ±0.2弧度 ≈ ±11.5度

% 限制轮胎力避免过大的横向加速度  
max_tire_force = m * 8.0;  % 最大8g横向加速度
```

### 3. 状态约束
```matlab
% 对更新后的状态进行合理性检查
x_hat(2) = max(-10.0, min(10.0, x_hat(2))); % 横向速度±10m/s
```

### 4. 改进的雅可比矩阵
考虑了轮胎力对状态的非线性影响：
```matlab
if abs(vx) > 0.5
    dFy_dvx = Cf * lf * gamma / (vx^2) + Cr * lr * gamma / (vx^2);
    dFy_dvy = -(Cf + Cr) / vx;
    F(2, 1) = F(2, 1) + dt * dFy_dvx / m;
    F(2, 2) = F(2, 2) + dt * dFy_dvy / m;
end
```

## 进一步优化建议

### 1. 参数调优
根据实际车辆调整以下参数：

```matlab
% 车辆参数优化
m = 实际车重;           % 准确的车辆质量
Iz = 实际转动惯量;      % 通过CAD或试验获得
Cf = 前轮实际刚度;      % 通过轮胎试验数据
Cr = 后轮实际刚度;      % 考虑载荷和气压影响
```

### 2. 自适应噪声调整
```matlab
% 根据车速自适应调整过程噪声
if vx < 5.0
    Q(2,2) = 0.01;  % 低速时降低横向速度噪声
else
    Q(2,2) = 0.05;  % 高速时适当增加
end
```

### 3. 多传感器融合
添加更多传感器信息：
- GPS速度（低频但准确）
- IMU姿态信息
- 转向角传感器

### 4. 非线性轮胎模型
使用更精确的轮胎模型：
```matlab
% Pacejka魔术公式（简化版）
function Fy = pacejka_tire_force(alpha, Fz, params)
    B = params.B;  % 刚度因子
    C = params.C;  % 形状因子  
    D = params.D * Fz;  % 峰值因子
    E = params.E;  % 曲率因子
    
    Fy = D * sin(C * atan(B * alpha - E * (B * alpha - atan(B * alpha))));
end
```

## 验证方法

### 1. 离线验证
```matlab
% 使用记录的车辆数据验证
load('vehicle_test_data.mat');
[estimated_states, errors] = validate_ekf(test_data);
```

### 2. 对比验证
- 与GPS速度对比
- 与高精度INS对比  
- 与车辆动力学仿真对比

### 3. 极限工况测试
- 紧急制动
- 急转弯
- 湿滑路面

## 调试技巧

### 1. 监控关键变量
```matlab
% 在EKF中添加调试输出
persistent debug_counter;
if isempty(debug_counter)
    debug_counter = 0;
end
debug_counter = debug_counter + 1;

if mod(debug_counter, 100) == 0
    fprintf('Step %d: vx=%.2f, vy=%.2f, alpha_f=%.3f\n', ...
        debug_counter, vx, vy, alpha_f);
end
```

### 2. 分析新息序列
```matlab
% 监控新息序列检测模型不匹配
innovation_norm = norm(y);
if innovation_norm > threshold
    warning('Large innovation detected: %.3f', innovation_norm);
end
```

### 3. 协方差监控
```matlab
% 检查协方差矩阵的数值稳定性
if any(diag(P_hat) < 0) || any(diag(P_hat) > 100)
    warning('Covariance matrix may be unstable');
end
```

## 常见问题解决

### 问题1: 横向速度发散
**解决方案:**
- 减小横向速度的过程噪声Q(2,2)
- 增加横向加速度的测量权重（减小R(2,2)）
- 添加状态约束

### 问题2: 估计延迟
**解决方案:**
- 增加卡尔曼增益（减小测量噪声）
- 使用预测-校正结构
- 考虑传感器延迟补偿

### 问题3: 低速不稳定
**解决方案:**
- 提高速度阈值判断
- 低速时切换到简化模型
- 增加轮速约束权重

## 性能指标

良好的横向速度估计应满足：
- RMSE < 0.5 m/s（正常驾驶）
- 最大瞬时误差 < 2.0 m/s
- 收敛时间 < 2秒
- 在±5m/s²横向加速度下稳定工作
