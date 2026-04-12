# armor_solver

## fyt::ArmorSolverNode

装甲板识别节点

### 发布话题 

* `armor_solver/target` (`rm_interfaces/msg/Target`) - 整车估计的状态
* `armor_solver/measurement` (`rm_interfaces/msg/Measurement`) - EKF的输入观测量
* `armor_solver/cmd_gimbal` (`rm_interfaces/msg/GimbalCmd`) - 云台控制指令

### 订阅话题

*  `armor_detector/armors` (`rm_interfaces/msg/Armors`) - 识别到的装甲板信息

  
### 参数 

* `debug` (`bool`, default: false) - 是否开启调试模式
* `target_frame` (`string`, default: "odom") - 目标坐标系
* `ekf.sigma2_q_xyz` (`double`, default: 0.05) - 状态转移噪声方差 (x,y,z)
* `ekf.sigma2_q_yaw` (`double`, default: 1.0) - 状态转移噪声方差 (yaw)
* `ekf.sigma2_q_r` (`double`, default: 80.0) - 状态转移噪声方差 (r)
* `r_xyz_factor` (`double`, default: 1.0) - 位置观测噪声方差系数 (x,y,z)
* `r_yaw_factor` (`double`, default: 1.0) - 位置观测噪声方差系数 (yaw)
* `tracker.max_match_distance` (`double`, default: 0.5) - 两帧间目标可匹配的最大距离
* `tracker.max_match_yaw_diff` (`double`, default: 0.5) - 两帧间目标同一块装甲板可匹配的最大yaw角差（大于这个值则认为装甲板发生跳变）
* `tracker.tracking_thres` (`int`, default: 2) - `DETECTING` 状态进入 `TRACKING` 状态需要连续识别到的帧数
* `tracker.lost_thres` (`double`, default: 1.0) - `TRACKING` 状态进入 `LOST` 状态需要连续丢失的时间（s）
* `solver.prediction_delay` (`double`, default: 0.0) - 预测延迟时间（s），会影响选版
* `solver.controller_delay` (`double`, default: 0.0) - 控制延迟时间（s），不会影响选版
* `solver.max_tracking_v_yaw` (`double`, default: 60.0) - 转速大于这个值时，瞄准中心
* `solver.side_angle` (`double`, default: 15.0) - 跳转到下一装甲板的角度阈值
* `solver.bullet_speed` (`double`, default: 25.0) - 子弹速度
* `solver.gravity` (`double`, default: 9.8) - 重力加速度
* `solver.compensator_type` (`string`, default: "ideal") - 补偿器类型
* `solver.resistance` (`double`, default: 0.001) - 空气阻力

### TinyMPC 优化器参数

* `solver.use_tinympc` (`bool`, default: false) - 是否启用tinyMPC轨迹优化器
* `solver.tinympc.dt` (`double`, default: 0.01) - MPC离散时间步长（s）
* `solver.tinympc.fire_thresh` (`double`, default: 0.04) - 射击准度阈值（rad）
* `solver.tinympc.decision_speed` (`double`, default: 8.0) - 高/低速决策阈值（rad/s），目标yaw角速度超过此值时选用高速延迟
* `solver.tinympc.high_speed_delay_time` (`double`, default: 0.12) - 高速旋转时的额外决策延迟（s）
* `solver.tinympc.low_speed_delay_time` (`double`, default: 0.05) - 低速旋转时的额外决策延迟（s）
* `solver.tinympc.max_yaw_acc` (`double`, default: 8.0) - 云台yaw轴最大加速度约束（rad/s²）
* `solver.tinympc.max_pitch_acc` (`double`, default: 8.0) - 云台pitch轴最大加速度约束（rad/s²）
* `solver.tinympc.Q_yaw` (matrix 2x2, default: diag(40.0, 1.0)) - yaw通道状态代价权重
* `solver.tinympc.Q_pitch` (matrix 2x2, default: diag(40.0, 1.0)) - pitch通道状态代价权重
* `solver.tinympc.R_yaw` (`double`, default: 40.0) - yaw通道控制输入代价权重
* `solver.tinympc.R_pitch` (`double`, default: 40.0) - pitch通道控制输入代价权重


## ArmorSolverNode
装甲板处理节点

订阅识别节点发布的装甲板三维位置及机器人的坐标转换信息，将装甲板三维位置变换到指定惯性系（一般是以云台中心为原点，IMU 上电时的 Yaw 朝向为 X 轴的惯性系）下，然后将装甲板目标送入跟踪器中，输出跟踪机器人在指定惯性系下的状态

订阅：
- 已识别到的装甲板 `/detector/armors`
- 机器人的坐标转换信息 `/tf` `/tf_static`

发布：
- 最终锁定的目标 `/tracker/target`

参数：
- 跟踪器参数 tracker
  - 两帧间目标可匹配的最大距离 max_match_distance
  - `DETECTING` 状态进入 `TRACKING` 状态的阈值 tracking_threshold
  - `TRACKING` 状态进入 `LOST` 状态的阈值 lost_threshold

## ExtendedKalmanFilter

$$ x_c = x_a + r * cos (\theta) $$
$$ y_c = y_a + r * sin (\theta) $$

$$ x = [x_c, y_c,z, yaw, v_{xc}, v_{yc},v_z, v_{yaw}, r]^T $$

参考 OpenCV 中的卡尔曼滤波器使用 Eigen 进行了实现

[卡尔曼滤波器](https://zh.wikipedia.org/wiki/%E5%8D%A1%E5%B0%94%E6%9B%BC%E6%BB%A4%E6%B3%A2)

![](docs/Kalman_filter_model.png)

考虑到自瞄任务中对于目标只有观测没有控制，所以输入－控制模型 $B$ 和控制器向量 $u$ 可忽略。

预测及更新的公式如下：

预测：

$$ x_{k|k-1} = F * x_{k-1|k-1} $$

$$ P_{k|k-1} = F * P_{k-1|k-1}* F^T + Q $$

更新:

$$ K = P_{k|k-1} * H^T * (H * P_{k|k-1} * H^T + R)^{-1} $$

$$ x_{k|k} = x_{k|k-1} + K * (z_k - H * x_{k|k-1}) $$

$$ P_{k|k} = (I - K * H) * P_{k|k-1} $$

## Tracker

参考 [SORT(Simple online and realtime tracking)](https://ieeexplore.ieee.org/abstract/document/7533003/) 中对于目标匹配的方法，使用卡尔曼滤波器对单目标在三维空间中进行跟踪

在此跟踪器中，卡尔曼滤波器观测量为目标在指定惯性系中的位置（xyz），状态量为目标位置及速度（xyz+vx vy vz）

在对目标的运动模型建模为在指定惯性系中的匀速线性运动，即状态转移为 $x_{pre} = x_{post} + v_{post} * dt$

目标关联的判断依据为三维位置的 L2 欧式距离

跟踪器共有四个状态：
- `DETECTING` ：短暂识别到目标，需要更多帧识别信息才能进入跟踪状态
- `TRACKING` ：跟踪器正常跟踪目标中
- `TEMP_LOST` ：跟踪器短暂丢失目标，通过卡尔曼滤波器预测目标
- `LOST` ：跟踪器完全丢失目标

工作流程：

- init：

  跟踪器默认选择离相机光心最近的目标作为跟踪对象，选择目标后初始化卡尔曼滤波器，初始状态设为当前目标位置，速度设为 0

- update:

  首先由卡尔曼滤波器得到目标在当前帧的预测位置，然后遍历当前帧中的目标位置与预测位置进行匹配，若当前帧不存在目标或所有目标位置与预测位置的偏差都过大则认为目标丢失，重置卡尔曼滤波器。
  
  最后选取位置相差最小的目标作为最佳匹配项，更新卡尔曼滤波器，将更新后的状态作为跟踪器的结果输出

## TinyMpcPlanner

基于tinyMPC库的云台轨迹优化模块，在遮挡和高速旋转场景下提供更精准的瞄准指令

### 工作原理

**双路径架构：**
- **Legacy路径**：基于锁定装甲板的几何关系直接计算云台指令（传统方案）
- **MPC路径**：通过预测优化生成最优轨迹，改善快速目标追踪性能（当 `use_tinympc=true` 时启用）

**MPC轨迹规划流程：**

1. **时间补偿计算**
   - 飞行时间补偿由 `armor_solver.cpp` 完成（在调用MPC规划前）
   - `armor_solver` 首先计算：$t_{dt} = t_{message\_age} + t_{fly} + t_{prediction\_delay}$
   - 使用此 $t_{dt}$ 预测目标的位置和方向，然后将预测结果作为目标消息传给MPC
   - **关键**：MPC接收的Target消息位置已经包含了飞行时间补偿，因此MPC规划器中**不再重复计算飞行时间**

2. **MPC决策延迟选择**
   - MPC规划器根据目标角速度 $v_{yaw}$ 选择决策延迟（不同于飞行时间）：
     - 若 $|v_{yaw}| > decision\_speed$ → 使用 `high_speed_delay_time` （高速旋转场景）
     - 否则 → 使用 `low_speed_delay_time` （低速追踪场景）
   - 基础时间点：$t_{base} = t_{dt\_to\_now} + t_{prediction\_delay} + t_{decision\_delay}$
   - 注意：$t_{dt\_to\_now}$ 是消息时戳到当前时刻的延迟，已经隐含包含了飞行时间补偿

3. **轨迹生成**
   - 从当前关节状态 $(y_{cur}, \dot{y}_{cur}, p_{cur}, \dot{p}_{cur})$ 出发
   - 构建时长 $t_{horizon} = 100 \times dt = 1.0$ 秒的预测轨迹
   - 参考轨迹为从基础时间点到地平线末端的目标角度曲线

4. **MPC求解**
   - 代价函数：$J = \sum_{k=0}^{N} (x_k^T Q x_k + u_k^T R u_k)$
     - $Q$ 矩阵：状态跟踪误差权重（位置权重 > 速度权重）
     - $R$ 矩阵：控制输入代价（制动能耗）
   - 约束条件：
     - 关节加速度：$|\ddot{y}|, |\ddot{p}| \leq a_{max}$
   - 离散化：向后欧拉法，步长 $dt = 0.01$ 秒

### 输出结果

实时返回 `PlanResult` 结构体：

```cpp
struct PlanResult {
  bool valid;              // 规划是否成功
  bool fire;               // 当前是否应该射击
  double plan_yaw;         // 目标yaw角指令
  double plan_pitch;       // 目标pitch角指令
  double aim_distance;     // 枪口到目标的水平距离（用于二级补偿）
  double aim_height;       // 枪口到目标的竖直高度（用于二级补偿）
};
```

### 调参建议

| 参数 | 推荐值范围 | 说明 |
|------|-----------|------|
| `decision_speed` | 5.0 ~ 15.0 rad/s | 根据实际目标旋转速度范围调整 |
| `high_speed_delay_time` | 0.08 ~ 0.15 s | 高速时需更长延迟来完成瞄准 |
| `low_speed_delay_time` | 0.03 ~ 0.08 s | 低速时延迟较短 |
| `Q_yaw / Q_pitch` 对角元 | (0.5 ~ 2.0, 0.05 ~ 0.2) | 前者控制位置精度，后者控制速度光滑度 |
| `R_yaw / R_pitch` | 0.05 ~ 0.5 | 越大越保守，控制能耗 |

### 与EKF和补偿器的数据流

```
EKF (Kalman Filter)
    ↓ 
    Target Message (包含 center_pos, v_yaw, radius_1 等)
    ↓
TinyMpcPlanner
    ├─ 调用 trajectory_compensator→getFlyingTime() 计算飞行时间
    ├─ 调用 trajectory_compensator→compensate() 生成轨迹点
    └─ 返回 aim_distance, aim_height
    ↓
ArmorSolver (选定MPC路径)
    ├─ 使用 plan_yaw, plan_pitch 作为主控制指令
    └─ 调用 manual_compensator→angleHardCorrect(aim_distance, aim_height) 
       进行二级角度微调
    ↓
GimbalCmd (发送至云台驱动)
```

### 调试建议

- 设置 `debug: true` 可在日志中观察 MPC 求解状态、飞行时间计算和延迟选择
- 若 MPC 频繁失败，检查 `max_yaw_acc` / `max_pitch_acc` 是否与实际云台能力匹配
- 若射击精度下降，逐步调整 Q 矩阵权重或 `decision_speed` 阈值进行微调


