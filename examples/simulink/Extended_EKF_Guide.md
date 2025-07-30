# 扩展EKF车辆状态与轮胎力估计器

## 概述

本实现提供了一个先进的扩展卡尔曼滤波器，用于同时估计车辆状态和四轮横向力。相比传统的3状态EKF，该扩展版本实现了：

- **7维状态估计**：车辆状态 + 四轮横向力
- **5维控制输入**：前轮转角 + 四轮纵向力
- **单轮动力学建模**：更精确的轮胎特性建模
- **实时轮胎力监测**：支持高级车辆控制功能

## 状态向量定义

### 扩展状态向量 (7×1)
```
x = [v_x, v_y, γ, Fy_fl, Fy_fr, Fy_rl, Fy_rr]ᵀ
```

其中：
- `v_x`: 纵向速度 (m/s)
- `v_y`: 横向速度 (m/s)  
- `γ`: 横摆角速度 (rad/s)
- `Fy_fl`: 左前轮横向力 (N)
- `Fy_fr`: 右前轮横向力 (N)
- `Fy_rl`: 左后轮横向力 (N)
- `Fy_rr`: 右后轮横向力 (N)

## 控制输入向量

### 控制输入向量 (5×1)
```
u = [δ, Fx_fl, Fx_fr, Fx_rl, Fx_rr]ᵀ
```

其中：
- `δ`: 前轮转角 (rad)
- `Fx_fl`: 左前轮纵向力 (N)
- `Fx_fr`: 右前轮纵向力 (N)
- `Fx_rl`: 左后轮纵向力 (N)  
- `Fx_rr`: 右后轮纵向力 (N)

## 测量向量

### 测量向量 (7×1)
```
z = [ax, ay, γ_meas, v_fl, v_fr, v_rl, v_rr]ᵀ
```

其中：
- `ax`: 纵向加速度测量 (m/s²)
- `ay`: 横向加速度测量 (m/s²)
- `γ_meas`: 横摆角速度测量 (rad/s)
- `v_fl, v_fr, v_rl, v_rr`: 四轮速度测量 (m/s)

## 数学模型详细推导

### 1. 系统动力学模型

#### 1.1 车辆质心动力学方程

车辆质心处的力平衡方程：

$$\begin{aligned}
m \dot{v}_x &= F_{x,total} + m v_y \gamma \\
m \dot{v}_y &= F_{y,total} - m v_x \gamma \\
I_z \dot{\gamma} &= M_{z,total}
\end{aligned}$$

其中总力和力矩计算：

$$\begin{aligned}
F_{x,total} &= \sum_{i} F_{x,i}^{veh} \\
F_{y,total} &= \sum_{i} F_{y,i}^{veh} \\
M_{z,total} &= \sum_{i} (F_{x,i}^{veh} \cdot y_i - F_{y,i}^{veh} \cdot x_i)
\end{aligned}$$

#### 1.2 各轮在车辆坐标系下的力分量

**前轮力变换**（考虑转向角δ）：
$$\begin{aligned}
F_{x,fl}^{veh} &= F_{x,fl} \cos\delta - F_{y,fl} \sin\delta \\
F_{y,fl}^{veh} &= F_{x,fl} \sin\delta + F_{y,fl} \cos\delta \\
F_{x,fr}^{veh} &= F_{x,fr} \cos\delta - F_{y,fr} \sin\delta \\
F_{y,fr}^{veh} &= F_{x,fr} \sin\delta + F_{y,fr} \cos\delta
\end{aligned}$$

**后轮力**（无转向）：
$$\begin{aligned}
F_{x,rl}^{veh} &= F_{x,rl} \\
F_{y,rl}^{veh} &= F_{y,rl} \\
F_{x,rr}^{veh} &= F_{x,rr} \\
F_{y,rr}^{veh} &= F_{y,rr}
\end{aligned}$$

#### 1.3 轮胎横向力动态模型

横向力采用一阶动态响应：

$$\dot{F}_{y,i} = \frac{F_{y,i}^{desired} - F_{y,i}}{\tau_{fy}}$$

其中期望横向力通过线性轮胎模型计算：

$$F_{y,i}^{desired} = -C_i \alpha_i$$

#### 1.4 轮胎侧偏角计算

**各轮在车辆坐标系下的速度**：
$$\begin{aligned}
v_{wheel,x}^{fl} &= v_x - \gamma \cdot \frac{track_f}{2} \\
v_{wheel,y}^{fl} &= v_y + \gamma \cdot l_f \\
v_{wheel,x}^{fr} &= v_x + \gamma \cdot \frac{track_f}{2} \\
v_{wheel,y}^{fr} &= v_y + \gamma \cdot l_f \\
v_{wheel,x}^{rl} &= v_x - \gamma \cdot \frac{track_r}{2} \\
v_{wheel,y}^{rl} &= v_y - \gamma \cdot l_r \\
v_{wheel,x}^{rr} &= v_x + \gamma \cdot \frac{track_r}{2} \\
v_{wheel,y}^{rr} &= v_y - \gamma \cdot l_r
\end{aligned}$$

**前轮在轮胎坐标系下的速度**（考虑转向角）：
$$\begin{aligned}
v_{tire,long}^{fl} &= v_{wheel,x}^{fl} \cos\delta + v_{wheel,y}^{fl} \sin\delta \\
v_{tire,lat}^{fl} &= -v_{wheel,x}^{fl} \sin\delta + v_{wheel,y}^{fl} \cos\delta \\
v_{tire,long}^{fr} &= v_{wheel,x}^{fr} \cos\delta + v_{wheel,y}^{fr} \sin\delta \\
v_{tire,lat}^{fr} &= -v_{wheel,x}^{fr} \sin\delta + v_{wheel,y}^{fr} \cos\delta
\end{aligned}$$

**后轮在轮胎坐标系下的速度**：
$$\begin{aligned}
v_{tire,long}^{rl} &= v_{wheel,x}^{rl} \\
v_{tire,lat}^{rl} &= v_{wheel,y}^{rl} \\
v_{tire,long}^{rr} &= v_{wheel,x}^{rr} \\
v_{tire,lat}^{rr} &= v_{wheel,y}^{rr}
\end{aligned}$$

**侧偏角计算**：
$$\alpha_i = \arctan\left(\frac{v_{tire,lat}^i}{|v_{tire,long}^i|}\right)$$

#### 1.5 完整的状态方程

连续时间状态方程：
$$\dot{\mathbf{x}} = \mathbf{f}(\mathbf{x}, \mathbf{u}) + \mathbf{w}$$

其中：
$$\mathbf{f}(\mathbf{x}, \mathbf{u}) = \begin{bmatrix}
\frac{F_{x,total} + m v_y \gamma}{m} \\
\frac{F_{y,total} - m v_x \gamma}{m} \\
\frac{M_{z,total}}{I_z} \\
\frac{-C_f \alpha_{fl} - F_{y,fl}}{\tau_{fy}} \\
\frac{-C_f \alpha_{fr} - F_{y,fr}}{\tau_{fy}} \\
\frac{-C_r \alpha_{rl} - F_{y,rl}}{\tau_{fy}} \\
\frac{-C_r \alpha_{rr} - F_{y,rr}}{\tau_{fy}}
\end{bmatrix}$$

### 2. 观测模型

#### 2.1 加速度测量模型

**纵向加速度**：
$$a_x = \dot{v}_x - v_y \gamma = \frac{F_{x,total}}{m}$$

**横向加速度**：
$$a_y = \dot{v}_y + v_x \gamma = \frac{F_{y,total}}{m}$$

#### 2.2 轮速测量模型

各轮的轮速测量模型：
$$v_{wheel}^i = \sqrt{(v_{tire,long}^i)^2 + (v_{tire,lat}^i)^2}$$

#### 2.3 完整的观测方程

$$\mathbf{z} = \mathbf{h}(\mathbf{x}, \mathbf{u}) + \mathbf{v}$$

其中：
$$\mathbf{h}(\mathbf{x}, \mathbf{u}) = \begin{bmatrix}
\frac{F_{x,total}}{m} \\
\frac{F_{y,total}}{m} \\
\gamma \\
\sqrt{(v_{tire,long}^{fl})^2 + (v_{tire,lat}^{fl})^2} \\
\sqrt{(v_{tire,long}^{fr})^2 + (v_{tire,lat}^{fr})^2} \\
\sqrt{(v_{tire,long}^{rl})^2 + (v_{tire,lat}^{rl})^2} \\
\sqrt{(v_{tire,long}^{rr})^2 + (v_{tire,lat}^{rr})^2}
\end{bmatrix}$$

### 3. 线性化模型（EKF雅可比矩阵）

#### 3.1 状态转移雅可比矩阵 $\mathbf{F}$

$$\mathbf{F} = \frac{\partial \mathbf{f}}{\partial \mathbf{x}} \bigg|_{\mathbf{x}=\hat{\mathbf{x}}}$$

主要非零元素：

**速度方程对状态的偏导**：
$$\frac{\partial \dot{v}_x}{\partial v_x} = 0, \quad \frac{\partial \dot{v}_x}{\partial v_y} = \gamma, \quad \frac{\partial \dot{v}_x}{\partial \gamma} = v_y$$

$$\frac{\partial \dot{v}_y}{\partial v_x} = -\gamma, \quad \frac{\partial \dot{v}_y}{\partial v_y} = 0, \quad \frac{\partial \dot{v}_y}{\partial \gamma} = -v_x$$

**横摆角速度方程对状态的偏导**：
$$\frac{\partial \dot{\gamma}}{\partial v_x} = \frac{1}{I_z}\frac{\partial M_{z,total}}{\partial v_x}, \quad \frac{\partial \dot{\gamma}}{\partial v_y} = \frac{1}{I_z}\frac{\partial M_{z,total}}{\partial v_y}$$

**轮胎力方程对状态的偏导**：
$$\frac{\partial \dot{F}_{y,i}}{\partial F_{y,i}} = -\frac{1}{\tau_{fy}}, \quad \frac{\partial \dot{F}_{y,i}}{\partial v_x} = -\frac{C_i}{\tau_{fy}}\frac{\partial \alpha_i}{\partial v_x}$$

#### 3.2 状态转移雅可比矩阵的显式形式

状态转移雅可比矩阵 $\mathbf{F}$ 的完整形式为 7×7 矩阵：

$$\mathbf{F} = \begin{bmatrix}
0 & \gamma & v_y & \frac{\partial \dot{v}_x}{\partial F_{y,fl}} & \frac{\partial \dot{v}_x}{\partial F_{y,fr}} & \frac{\partial \dot{v}_x}{\partial F_{y,rl}} & \frac{\partial \dot{v}_x}{\partial F_{y,rr}} \\
-\gamma & 0 & -v_x & \frac{1}{m} & \frac{1}{m} & \frac{1}{m} & \frac{1}{m} \\
\frac{\partial \dot{\gamma}}{\partial v_x} & \frac{\partial \dot{\gamma}}{\partial v_y} & 0 & \frac{\partial \dot{\gamma}}{\partial F_{y,fl}} & \frac{\partial \dot{\gamma}}{\partial F_{y,fr}} & \frac{\partial \dot{\gamma}}{\partial F_{y,rl}} & \frac{\partial \dot{\gamma}}{\partial F_{y,rr}} \\
-\frac{C_f}{\tau_{fy}}\frac{\partial \alpha_{fl}}{\partial v_x} & -\frac{C_f}{\tau_{fy}}\frac{\partial \alpha_{fl}}{\partial v_y} & -\frac{C_f}{\tau_{fy}}\frac{\partial \alpha_{fl}}{\partial \gamma} & -\frac{1}{\tau_{fy}} & 0 & 0 & 0 \\
-\frac{C_f}{\tau_{fy}}\frac{\partial \alpha_{fr}}{\partial v_x} & -\frac{C_f}{\tau_{fy}}\frac{\partial \alpha_{fr}}{\partial v_y} & -\frac{C_f}{\tau_{fy}}\frac{\partial \alpha_{fr}}{\partial \gamma} & 0 & -\frac{1}{\tau_{fy}} & 0 & 0 \\
-\frac{C_r}{\tau_{fy}}\frac{\partial \alpha_{rl}}{\partial v_x} & -\frac{C_r}{\tau_{fy}}\frac{\partial \alpha_{rl}}{\partial v_y} & -\frac{C_r}{\tau_{fy}}\frac{\partial \alpha_{rl}}{\partial \gamma} & 0 & 0 & -\frac{1}{\tau_{fy}} & 0 \\
-\frac{C_r}{\tau_{fy}}\frac{\partial \alpha_{rr}}{\partial v_x} & -\frac{C_r}{\tau_{fy}}\frac{\partial \alpha_{rr}}{\partial v_y} & -\frac{C_r}{\tau_{fy}}\frac{\partial \alpha_{rr}}{\partial \gamma} & 0 & 0 & 0 & -\frac{1}{\tau_{fy}}
\end{bmatrix}$$

其中关键的偏导数项：

**横摆角速度对状态的偏导**：
$$\frac{\partial \dot{\gamma}}{\partial v_x} = \frac{1}{I_z}\left[\sin\delta(F_{y,fl} + F_{y,fr}) \frac{\partial \alpha_{fl}}{\partial v_x} C_f + \sin\delta(F_{y,fl} + F_{y,fr}) \frac{\partial \alpha_{fr}}{\partial v_x} C_f\right]$$

$$\frac{\partial \dot{\gamma}}{\partial v_y} = \frac{1}{I_z}\left[\cos\delta(F_{y,fl} + F_{y,fr}) \frac{\partial \alpha_{fl}}{\partial v_y} C_f + \cos\delta(F_{y,fl} + F_{y,fr}) \frac{\partial \alpha_{fr}}{\partial v_y} C_f\right]$$

$$\frac{\partial \dot{\gamma}}{\partial F_{y,i}} = \frac{1}{I_z} \begin{cases}
l_f \cos\delta \pm \frac{track_f}{2} \sin\delta & \text{前轮} \\
-l_r \pm \frac{track_r}{2} & \text{后轮}
\end{cases}$$

**侧偏角对状态的偏导**：

对于前轮（左前轮为例）：
$$\frac{\partial \alpha_{fl}}{\partial v_x} = \frac{-\sin\delta \cdot v_{tire,long}^{fl} + \cos\delta \cdot v_{tire,lat}^{fl}}{(v_{tire,long}^{fl})^2 + (v_{tire,lat}^{fl})^2}$$

$$\frac{\partial \alpha_{fl}}{\partial v_y} = \frac{\cos\delta \cdot v_{tire,long}^{fl} + \sin\delta \cdot v_{tire,lat}^{fl}}{(v_{tire,long}^{fl})^2 + (v_{tire,lat}^{fl})^2}$$

$$\frac{\partial \alpha_{fl}}{\partial \gamma} = \frac{(-l_f \sin\delta - \frac{track_f}{2} \cos\delta) \cdot v_{tire,long}^{fl} + (l_f \cos\delta - \frac{track_f}{2} \sin\delta) \cdot v_{tire,lat}^{fl}}{(v_{tire,long}^{fl})^2 + (v_{tire,lat}^{fl})^2}$$

对于后轮（左后轮为例）：
$$\frac{\partial \alpha_{rl}}{\partial v_x} = \frac{v_{tire,lat}^{rl}}{(v_{tire,long}^{rl})^2 + (v_{tire,lat}^{rl})^2}$$

$$\frac{\partial \alpha_{rl}}{\partial v_y} = \frac{-v_{tire,long}^{rl}}{(v_{tire,long}^{rl})^2 + (v_{tire,lat}^{rl})^2}$$

$$\frac{\partial \alpha_{rl}}{\partial \gamma} = \frac{l_r \cdot v_{tire,lat}^{rl} + \frac{track_r}{2} \cdot v_{tire,long}^{rl}}{(v_{tire,long}^{rl})^2 + (v_{tire,lat}^{rl})^2}$$

#### 3.3 观测雅可比矩阵的显式形式

观测雅可比矩阵 $\mathbf{H}$ 的完整形式为 7×7 矩阵：

$$\mathbf{H} = \begin{bmatrix}
\frac{\partial a_x}{\partial v_x} & \frac{\partial a_x}{\partial v_y} & \frac{\partial a_x}{\partial \gamma} & \frac{\partial a_x}{\partial F_{y,fl}} & \frac{\partial a_x}{\partial F_{y,fr}} & \frac{\partial a_x}{\partial F_{y,rl}} & \frac{\partial a_x}{\partial F_{y,rr}} \\
\frac{\partial a_y}{\partial v_x} & \frac{\partial a_y}{\partial v_y} & \frac{\partial a_y}{\partial \gamma} & \frac{1}{m} & \frac{1}{m} & \frac{1}{m} & \frac{1}{m} \\
0 & 0 & 1 & 0 & 0 & 0 & 0 \\
\frac{\partial v_{fl}}{\partial v_x} & \frac{\partial v_{fl}}{\partial v_y} & \frac{\partial v_{fl}}{\partial \gamma} & 0 & 0 & 0 & 0 \\
\frac{\partial v_{fr}}{\partial v_x} & \frac{\partial v_{fr}}{\partial v_y} & \frac{\partial v_{fr}}{\partial \gamma} & 0 & 0 & 0 & 0 \\
\frac{\partial v_{rl}}{\partial v_x} & \frac{\partial v_{rl}}{\partial v_y} & \frac{\partial v_{rl}}{\partial \gamma} & 0 & 0 & 0 & 0 \\
\frac{\partial v_{rr}}{\partial v_x} & \frac{\partial v_{rr}}{\partial v_y} & \frac{\partial v_{rr}}{\partial \gamma} & 0 & 0 & 0 & 0
\end{bmatrix}$$

其中关键的偏导数项：

**纵向加速度测量对状态的偏导**：
$$\frac{\partial a_x}{\partial v_x} = \frac{1}{m}\left[-\sin\delta(F_{y,fl} + F_{y,fr}) \frac{\partial \alpha_{fl}}{\partial v_x} C_f - \sin\delta(F_{y,fl} + F_{y,fr}) \frac{\partial \alpha_{fr}}{\partial v_x} C_f\right]$$

$$\frac{\partial a_x}{\partial v_y} = \frac{1}{m}\left[-\sin\delta(F_{y,fl} + F_{y,fr}) \frac{\partial \alpha_{fl}}{\partial v_y} C_f - \sin\delta(F_{y,fl} + F_{y,fr}) \frac{\partial \alpha_{fr}}{\partial v_y} C_f\right]$$

$$\frac{\partial a_x}{\partial F_{y,i}} = \frac{1}{m} \begin{cases}
-\sin\delta & \text{前轮} \\
0 & \text{后轮}
\end{cases}$$

**轮速测量对状态的偏导**：

对于前轮（左前轮为例）：
$$\frac{\partial v_{fl}}{\partial v_x} = \frac{(1 - \frac{\gamma \cdot track_f}{2}) \cos\delta - \gamma l_f \sin\delta}{\sqrt{(v_{tire,long}^{fl})^2 + (v_{tire,lat}^{fl})^2}}$$

$$\frac{\partial v_{fl}}{\partial v_y} = \frac{(1 - \frac{\gamma \cdot track_f}{2}) \sin\delta + \gamma l_f \cos\delta}{\sqrt{(v_{tire,long}^{fl})^2 + (v_{tire,lat}^{fl})^2}}$$

$$\frac{\partial v_{fl}}{\partial \gamma} = \frac{(-\frac{track_f}{2}(v_x \cos\delta + v_y \sin\delta) + l_f(-v_x \sin\delta + v_y \cos\delta))}{\sqrt{(v_{tire,long}^{fl})^2 + (v_{tire,lat}^{fl})^2}}$$

对于后轮（左后轮为例）：
$$\frac{\partial v_{rl}}{\partial v_x} = \frac{v_x - \gamma \cdot \frac{track_r}{2}}{\sqrt{(v_x - \gamma \cdot \frac{track_r}{2})^2 + (v_y - \gamma l_r)^2}}$$

$$\frac{\partial v_{rl}}{\partial v_y} = \frac{v_y - \gamma l_r}{\sqrt{(v_x - \gamma \cdot \frac{track_r}{2})^2 + (v_y - \gamma l_r)^2}}$$

$$\frac{\partial v_{rl}}{\partial \gamma} = \frac{-\frac{track_r}{2}(v_x - \gamma \cdot \frac{track_r}{2}) - l_r(v_y - \gamma l_r)}{\sqrt{(v_x - \gamma \cdot \frac{track_r}{2})^2 + (v_y - \gamma l_r)^2}}$$

#### 3.4 观测雅可比矩阵 $\mathbf{H}$

$$\mathbf{H} = \frac{\partial \mathbf{h}}{\partial \mathbf{x}} \bigg|_{\mathbf{x}=\hat{\mathbf{x}}}$$

**加速度测量对状态的偏导**：
$$\frac{\partial a_x}{\partial F_{y,i}} = \frac{1}{m}\frac{\partial F_{x,total}}{\partial F_{y,i}}, \quad \frac{\partial a_y}{\partial F_{y,i}} = \frac{1}{m}$$

**轮速测量对状态的偏导**：

对于前轮（左前轮为例）：
$$\frac{\partial v_{fl}}{\partial v_x} = \frac{(1 - \frac{\gamma \cdot track_f}{2}) \cos\delta - \gamma l_f \sin\delta}{\sqrt{(v_{tire,long}^{fl})^2 + (v_{tire,lat}^{fl})^2}}$$

$$\frac{\partial v_{fl}}{\partial v_y} = \frac{(1 - \frac{\gamma \cdot track_f}{2}) \sin\delta + \gamma l_f \cos\delta}{\sqrt{(v_{tire,long}^{fl})^2 + (v_{tire,lat}^{fl})^2}}$$

$$\frac{\partial v_{fl}}{\partial \gamma} = \frac{(-\frac{track_f}{2}(v_x \cos\delta + v_y \sin\delta) + l_f(-v_x \sin\delta + v_y \cos\delta))}{\sqrt{(v_{tire,long}^{fl})^2 + (v_{tire,lat}^{fl})^2}}$$

对于后轮（左后轮为例）：
$$\frac{\partial v_{rl}}{\partial v_x} = \frac{v_x - \gamma \cdot \frac{track_r}{2}}{\sqrt{(v_x - \gamma \cdot \frac{track_r}{2})^2 + (v_y - \gamma l_r)^2}}$$

$$\frac{\partial v_{rl}}{\partial v_y} = \frac{v_y - \gamma l_r}{\sqrt{(v_x - \gamma \cdot \frac{track_r}{2})^2 + (v_y - \gamma l_r)^2}}$$

$$\frac{\partial v_{rl}}{\partial \gamma} = \frac{-\frac{track_r}{2}(v_x - \gamma \cdot \frac{track_r}{2}) - l_r(v_y - \gamma l_r)}{\sqrt{(v_x - \gamma \cdot \frac{track_r}{2})^2 + (v_y - \gamma l_r)^2}}$$

## 车辆动力学模型

### 单轮运动学
对于每个车轮，计算其在车辆坐标系下的速度：
```
v_wheel_x = v_x - γ × y_wheel
v_wheel_y = v_y + γ × x_wheel
```

### 前轮转向变换
前轮考虑转向角δ的坐标变换：
```
v_wheel_longitudinal = v_wheel_x × cos(δ) + v_wheel_y × sin(δ)
v_wheel_lateral = -v_wheel_x × sin(δ) + v_wheel_y × cos(δ)
```

### 轮胎侧偏角计算
```
α = atan2(v_wheel_lateral, v_wheel_longitudinal)
```

### 轮胎力动态模型
横向力采用一阶滞后动态模型：
```
Fy_dot = (Fy_desired - Fy_current) / τ_fy
```

其中期望横向力由线性轮胎模型给出：
```
Fy_desired = -C × α  (C为轮胎侧偏刚度)
```

### 车辆整体动力学
```
m × v̇_x = Fx_total + m × v_y × γ
m × v̇_y = Fy_total - m × v_x × γ  
I_z × γ̇ = M_z_total
```

其中总力和力矩由各轮贡献叠加：
```
Fx_total = Σ Fx_veh_i
Fy_total = Σ Fy_veh_i
M_z_total = Σ (Fx_veh_i × y_i - Fy_veh_i × x_i)
```

## 使用方法

### 在Simulink中使用

1. **添加MATLAB Function块**
2. **复制函数代码**：将`ekf_simulink_function.m`内容复制到块中
3. **配置输入**：
   - `measurements` [7×1]
   - `control_inputs` [5×1]
4. **配置输出**：
   - `state_estimate` [7×1] 
   - `covariance_flat` [49×1]

### 函数调用示例

```matlab
% 测量向量 [ax, ay, gamma, v_fl, v_fr, v_rl, v_rr]
measurements = [0.5; 0.2; 0.1; 15.1; 14.9; 15.0; 15.0];

% 控制输入 [delta, Fx_fl, Fx_fr, Fx_rl, Fx_rr]
control_inputs = [0.05; 1000; 1000; 800; 800];

% 调用EKF
[states, covariance] = ekf_simulink_function(measurements, control_inputs);

% 提取结果
v_x = states(1);      % 纵向速度
v_y = states(2);      % 横向速度  
gamma = states(3);    % 横摆角速度
Fy_fl = states(4);    % 左前轮横向力
Fy_fr = states(5);    % 右前轮横向力
Fy_rl = states(6);    % 左后轮横向力
Fy_rr = states(7);    % 右后轮横向力
```

## 参数配置

### 车辆参数
```matlab
m = 1500.0;      % 车辆质量 (kg)
Iz = 2500.0;     % 横摆转动惯量 (kg⋅m²)
lf = 1.2;        % 质心到前轴距离 (m)
lr = 1.4;        % 质心到后轴距离 (m)
track_f = 1.6;   % 前轮距 (m)
track_r = 1.6;   % 后轮距 (m)
Cf = 50000.0;    % 前轮侧偏刚度 (N/rad)
Cr = 50000.0;    % 后轮侧偏刚度 (N/rad)
```

### 滤波器参数
```matlab
% 过程噪声协方差矩阵 Q [7×7]
Q = diag([0.1, 0.05, 0.01, 1000, 1000, 1000, 1000]);

% 测量噪声协方差矩阵 R [7×7]  
R = diag([0.5, 0.3, 0.01, 0.2, 0.2, 0.2, 0.2]);

% 轮胎力时间常数
tau_fy = 0.05;   % 横向力时间常数 (s)
```

## 应用场景

### 1. 高级驾驶辅助系统 (ADAS)
- ESP电子稳定程序
- ABS防抱死制动系统
- 牵引力控制系统

### 2. 自动驾驶
- 车辆状态监测
- 轮胎性能评估
- 路面附着系数估计

### 3. 车辆动力学研究
- 轮胎模型验证
- 车辆参数辨识
- 控制算法开发

## 优势特点

### 相比传统3状态EKF：
1. **更精确的横向速度估计**：通过四轮独立建模减少耦合误差
2. **轮胎力实时监测**：可检测轮胎性能变化和故障
3. **更好的低速性能**：避免传统方法在低速时的奇点问题
4. **支持高级控制**：为ESP、ABS等系统提供轮胎力反馈

### 技术创新：
1. **单轮动力学建模**：考虑转向对各轮运动的影响
2. **轮胎力动态特性**：引入轮胎力响应时间常数
3. **多输入控制架构**：同时考虑转向和驱动输入
4. **数值稳定性优化**：使用Joseph形式协方差更新

## 测试验证

运行测试脚本验证性能：
```matlab
run('test_extended_ekf.m')
```

测试场景包括：
- 变速行驶
- 转向机动
- 复合工况
- 噪声鲁棒性

## 注意事项

1. **计算负载**：7状态EKF比3状态版本计算量更大
2. **参数调优**：需要根据实际车辆调整轮胎刚度等参数
3. **传感器要求**：需要准确的轮速和IMU信息
4. **初始化**：轮胎力初值建议设为零

## 故障排除

### 常见问题：
1. **轮胎力发散**：检查轮胎刚度参数和过程噪声设置
2. **低速不稳定**：增加速度阈值或调整雅可比矩阵
3. **收敛缓慢**：适当增加测量权重或减小过程噪声

### 调试建议：
1. 监控状态约束是否被触发
2. 检查新息序列的大小
3. 验证协方差矩阵的正定性
