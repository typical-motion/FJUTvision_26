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

* `debug` (`bool`, default: true) - 是否开启调试模式
* `target_frame` (`string`, default: "odom") - 目标坐标系
* `ekf.sigma2_q_x` (`double`, default: 20.0) - 状态转移噪声方差 (x)
* `ekf.sigma2_q_y` (`double`, default: 20.0) - 状态转移噪声方差 (y)
* `ekf.sigma2_q_z` (`double`, default: 20.0) - 状态转移噪声方差 (z)
* `ekf.sigma2_q_yaw` (`double`, default: 100.0) - 状态转移噪声方差 (yaw)
* `ekf.sigma2_q_r` (`double`, default: 800.0) - 状态转移噪声方差 (r)
* `ekf.sigma2_q_d_zc` (`double`, default: 800.0) - 状态转移噪声方差 (d_zc)
* `ekf.use_legacy_noise_model` (`bool`, default: false) - 是否使用旧版噪声模型
* `ekf.q_outpost_linear_factor` (`double`, default: 0.1) - outpost 线性通道 Q 缩放因子
* `ekf.q_outpost_yaw_factor` (`double`, default: 0.001) - outpost yaw 通道 Q 缩放因子
* `ekf.q_normal_linear_factor` (`double`, default: 1.0) - 非 outpost 线性通道 Q 缩放因子
* `ekf.q_normal_yaw_factor` (`double`, default: 1.0) - 非 outpost yaw 通道 Q 缩放因子
* `ekf.r_x` (`double`, default: 0.05) - x 方向观测噪声基值
* `ekf.r_y` (`double`, default: 0.05) - y 方向观测噪声基值
* `ekf.r_z` (`double`, default: 0.05) - z 方向观测噪声基值
* `ekf.r_yaw` (`double`, default: 0.02) - yaw 观测噪声基值
* `ekf.r_quality_clip` (`double`, default: 2.0) - 质量指标上限
* `ekf.r_position_gain` (`double`, default: 6.0) - 位置质量到 R 缩放的增益
* `ekf.r_yaw_gain` (`double`, default: 6.0) - yaw 质量到 R 缩放的增益
* `ekf.r_missing_quality_scale` (`double`, default: 4.0) - 质量缺失时额外放大系数
* `tracker.max_match_distance` (`double`, default: 0.2) - 两帧间目标可匹配的最大距离
* `tracker.max_match_yaw_diff` (`double`, default: 1.0) - 两帧间目标同一块装甲板可匹配的最大 yaw 角差（大于该值视为跳变）
* `tracker.tracking_thres` (`int`, default: 5) - `DETECTING` 状态进入 `TRACKING` 状态需要连续识别到的帧数
* `tracker.lost_time_thres` (`double`, default: 0.3) - `TRACKING` 状态进入 `LOST` 状态需要连续丢失的时间（s）
* `solver.shooting_range_width` (`double`, default: 0.135) - 射击判定窗口宽度
* `solver.shooting_range_height` (`double`, default: 0.135) - 射击判定窗口高度
* `solver.prediction_delay` (`double`, default: 0.0) - 预测延迟时间（s），会影响选版
* `solver.controller_delay` (`double`, default: 0.0) - 控制延迟时间（s），不会影响选版
* `solver.max_tracking_v_yaw` (`double`, default: 6.0) - 转速大于这个值时，瞄准中心
* `solver.min_switching_v_yaw` (`double`, default: 1.0) - 低速切换阈值
* `solver.side_angle` (`double`, default: 15.0) - 跳转到下一装甲板的角度阈值
* `solver.iteration_times` (`int`, default: 20) - 弹道补偿迭代次数
* `solver.bullet_speed` (`double`, default: 20.0) - 子弹速度
* `solver.gravity` (`double`, default: 9.8) - 重力加速度
* `solver.compensator_type` (`string`, default: "ideal") - 补偿器类型
* `solver.resistance` (`double`, default: 0.001) - 空气阻力
* `solver.angle_offset` (`string[]`, default: `[]`) - 手动补偿映射配置

### TinyMPC 优化器参数

* `solver.use_tinympc` (`bool`, default: false) - 是否启用tinyMPC轨迹优化器。true 时 MPC 结果覆盖原有瞄准指令；MPC 求解失败时自动回退到 legacy 逻辑
* `solver.tinympc.dt` (`double`, default: 0.01) - MPC离散时间步长（s）。总时域跨度 = 100 × dt = 1.0s。减小 dt 可提高时域分辨率但增加计算量
* `solver.tinympc.yaw_offset` (`double`, default: 0.0) - yaw 瞄准点额外偏置（deg）。正值=向右偏，通常用于补偿机构安装偏差
* `solver.tinympc.pitch_offset` (`double`, default: 0.0) - pitch 瞄准点额外偏置（deg）。正值=向上偏，通常用于补偿机构安装偏差
* `solver.tinympc.fire_thresh` (`double`, default: 0.04) - 射击判定阈值（rad），MPC 优化轨迹与参考轨迹在该点的偏差小于此值即允许开火。越小越严格
* `solver.tinympc.shoot_offset` (`int`, default: 2) - 射击判定点相对 horizon 中心的偏移步数。正值=前瞻，负值=滞后。默认 +2 表示往前看 2 步（20ms）
* `solver.tinympc.decision_speed` (`double`, default: 8.0) - 高/低速决策阈值（rad/s），目标 |v_yaw| 超过此值时选用高速延迟策略
* `solver.tinympc.high_speed_delay_time` (`double`, default: 0.12) - 高速（|v_yaw| > decision_speed）时的决策延迟（s），值越大瞄准越靠前
* `solver.tinympc.low_speed_delay_time` (`double`, default: 0.05) - 低速时的决策延迟（s），值越大瞄准越靠前
* `solver.tinympc.max_yaw_acc` (`double`, default: 8.0) - 云台 yaw 轴最大加速度约束（rad/s²）。必须 ≤ 实际云台物理能力，过大会导致轨迹不平滑
* `solver.tinympc.max_pitch_acc` (`double`, default: 8.0) - 云台 pitch 轴最大加速度约束（rad/s²）。同上
* `solver.tinympc.Q_yaw` (`double[2]`, default: `[40.0, 1.0]`) - yaw 通道状态代价对角权重：`[角度误差权重, 角速度误差权重]`
* `solver.tinympc.R_yaw` (`double[1]`, default: `[40.0]`) - yaw 通道控制代价权重。越大控制越"保守"（宁可慢也不超调）
* `solver.tinympc.Q_pitch` (`double[2]`, default: `[40.0, 1.0]`) - pitch 通道状态代价对角权重：`[角度误差权重, 角速度误差权重]`
* `solver.tinympc.R_pitch` (`double[1]`, default: `[40.0]`) - pitch 通道控制代价权重

### EKF 自适应 Q/R 说明

当 `ekf.use_legacy_noise_model=false` 时，系统启用自适应噪声模型。

#### 1) 自适应 Q（过程噪声）

Q 会根据目标类型切换倍率：

- outpost 目标：
  - 线性通道（x/y/z/r/d_zc）乘 `ekf.q_outpost_linear_factor`（默认 0.1）
  - yaw 通道乘 `ekf.q_outpost_yaw_factor`（默认 0.001）
- 非 outpost 目标：
  - 线性通道乘 `ekf.q_normal_linear_factor`（默认 1.0）
  - yaw 通道乘 `ekf.q_normal_yaw_factor`（默认 1.0）

这用于降低 outpost 这种特殊目标的过程噪声，减少估计抖动。

#### 2) 自适应 R（观测噪声）

R 会根据跟踪质量动态放大或缩小：

- 质量来源：
  - `position_quality = min(tracker.position_diff, ekf.r_quality_clip)`
  - `yaw_quality = min(tracker.yaw_diff, ekf.r_quality_clip)`
- 缩放公式：
  - `position_scale = 1 + ekf.r_position_gain * log1p(position_quality)`
  - `yaw_scale = 1 + ekf.r_yaw_gain * log1p(yaw_quality)`
- 当质量不可用（跟踪器未初始化）时，额外乘：
  - `ekf.r_missing_quality_scale`（默认 4.0）

最终观测噪声对角线近似为：

- `R_x = ekf.r_x * (|z_x| + 0.02) * position_scale`
- `R_y = ekf.r_y * (|z_y| + 0.02) * position_scale`
- `R_z = ekf.r_z * (|z_z| + 0.02) * position_scale`
- `R_yaw = ekf.r_yaw * yaw_scale`

#### 3) 数值稳定性保护（已实现）

为避免出现 e-41 或 e+38 这类极端值，当前实现对缩放因子做了限幅：

- `position_scale` 限制在 `[0.1, 100.0]`
- `yaw_scale` 限制在 `[0.1, 100.0]`

这可以防止 R 矩阵过小或过大导致滤波器数值不稳定。

#### 4) 推荐调参顺序

1. 固定 `r_quality_clip=2.0`，先调 `r_position_gain`、`r_yaw_gain`（建议 2.0~6.0）
2. 若初始化阶段抖动大，再调低 `r_missing_quality_scale`（建议 2.0~4.0）
3. 若滤波发散或响应过慢，再微调基础噪声 `r_x/r_y/r_z/r_yaw`


## ArmorSolverNode
装甲板处理节点

订阅识别节点发布的装甲板三维位置及机器人的坐标转换信息，将装甲板三维位置变换到指定惯性系（一般是以云台中心为原点，IMU 上电时的 Yaw 朝向为 X 轴的惯性系）下，然后将装甲板目标送入跟踪器中，输出跟踪机器人在指定惯性系下的状态

订阅：
- 已识别到的装甲板 `/armor_detector/armors`
- 机器人的坐标转换信息 `/tf` `/tf_static`

发布：
- 最终锁定的目标 `/armor_solver/target`

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

### MPC 调参指南

#### 调参顺序（按优先级排列）

```
第1步: max_yaw_acc / max_pitch_acc → 匹配物理极限
第2步: decision_speed / delay_time  → 匹配目标运动模式
第3步: fire_thresh / shoot_offset   → 控制开火时机
第4步: Q / R 权重                   → 微调轨迹质量
第5步: yaw_offset / pitch_offset    → 精校准瞄准点
```

---

#### 1. 加速度约束 `max_yaw_acc` / `max_pitch_acc`

这两个参数直接限定了 MPC 输出的最大加速度。**必须 ≤ 云台实际物理能力**。

| 情况 | 建议值 | 效果 |
|------|--------|------|
| 云台能力未知 | 从 4.0 开始 | 保守，轨迹平滑但可能跟不上快速目标 |
| 已知云台规格 | 设为规格的 80%~90% | 留安全余量 |
| 发现跟不上目标 | 逐步增大（每次 +1.0） | 增加响应速度，但可能引入超调 |
| 发现云台抖动 | 逐步减小（每次 -1.0） | 更平滑，减少机械冲击 |

> **诊断方法**：观察 MPC 求解是否频繁 hit 约束。若 `yaw_acc` 总是触及 `max_yaw_acc`，说明约束过紧。

---

#### 2. 决策速度与延迟 `decision_speed` / `*_delay_time`

时间线公式：$t_{base} = t_{消息延迟} + t_{预测延迟} + t_{决策延迟}$

决策延迟让瞄准点"往前看"，补偿系统响应延迟。

| 参数 | 作用 | 调大效果 | 调小效果 |
|------|------|----------|----------|
| `decision_speed` | 区分高/低速的阈值 | 更多目标用低速策略 | 更多目标用高速策略 |
| `high_speed_delay_time` | 高速目标的额外前瞻 | 超前瞄准，适合快转 | 滞后瞄准，可能跟不上 |
| `low_speed_delay_time` | 低速目标的额外前瞻 | 超前瞄准 | 滞后瞄准 |

> **典型场景**：
> - 哨兵/前哨站目标转速慢 → `decision_speed=5.0`, `low_speed_delay_time=0.03`
> - 步兵机器人常见 5~10 rad/s → `decision_speed=7.0`, `low=0.05`, `high=0.10`
> - 英雄/工程机器人转速低 → 可统一用低速策略

---

#### 3. 开火判定 `fire_thresh` / `shoot_offset`

判定位置：horizon 中心（第 50 步）+ `shoot_offset`

```
若 shoot_offset=2，则在第 52 步处比较 MPC 轨迹与参考轨迹偏差
若 hypot(yaw_err, pitch_err) < fire_thresh → fire=true
```

| 参数 | 默认 | 调大效果 | 调小效果 |
|------|------|----------|----------|
| `fire_thresh` | 0.04 rad (≈2.3°) | 更容易开火（宽松） | 更严格（可能不开火） |
| `shoot_offset` | 2 步 (20ms) | 更前瞻判定 | 更及时判定 |

> **典型问题**：
> - "明明瞄准了但不打" → 增大 `fire_thresh` 到 0.06~0.08
> - "没完全跟上就开火了" → 减小 `fire_thresh` 到 0.02~0.03
> - 弹道飞行时间长的场景（远距离）→ 增大 `shoot_offset` 到 3~4

---

#### 4. Q / R 权重矩阵

**物理含义**：
- `Q = diag(角度位置误差代价, 角速度误差代价)` — 希望 MPC 把"状态误差"降到多小
- `R = [控制输入代价]` — 希望 MPC 节省多少"控制能量"（即加速度）

**核心理念**：Q/R 比值决定"激进 vs 保守"

| 调法 | 效果 | 适用场景 |
|------|------|----------|
| Q 大 / R 小（如 Q=[100,1], R=[10]） | 激进跟踪，宁超调也要跟上 | 快速旋转目标 |
| Q 小 / R 大（如 Q=[10,1], R=[100]） | 保守平滑，宁可慢也不抖 | 低速/静止目标 |
| Q 位置分量 >> Q 速度分量 | 优先把角度对准 | 需要高射击精度 |
| Q 速度分量增大 | 优先让运动平滑 | 需要画面稳定 |

> **调参口诀**：
> - R_yaw 增大 → yaw 轨迹更平滑但可能滞后
> - R_yaw 减小 → yaw 跟踪更紧但可能振荡
> - Q_yaw[0]（角度权重）增大 → 稳态误差更小
> - Q_yaw[1]（速度权重）增大 → 速度跟踪更紧

**推荐起点**（默认值即为此）：
```
Q = [40.0, 1.0]   # 角度:速度 ≈ 40:1，角度优先
R = [40.0]         # Q/R ≈ 1:1，均衡
```

---

#### 5. 瞄准点偏置 `yaw_offset` / `pitch_offset`

| 参数 | 正值效果 | 典型用途 |
|------|----------|----------|
| `yaw_offset` | 瞄准点向右偏移 | 机械安装偏差、自瞄偏好 |
| `pitch_offset` | 瞄准点向上偏移 | 机械安装偏差、弹道余量 |

> **注意**：这两个参数是 MPC 内部偏置，**独立于** `angle_offset`（manual_compensator 的分段查表补偿）。MPC 路径中两者都会生效：
> 1. MPC 内部用 `yaw_offset/pitch_offset` 
> 2. 回传 Solver 后再叠加 `angle_offset` 查表结果
>
> 建议将固定偏差放入 `yaw_offset/pitch_offset`，距离/高度相关的偏差放入 `angle_offset`。

---

#### 6. 时间步长 `dt`

| dt 值 | 总时域 | 效果 |
|-------|--------|------|
| 0.01 (默认) | 1.0s | 标准 |
| 0.005 | 0.5s | 更快响应，时域短 |
| 0.02 | 2.0s | 更远预见，计算量不变 |

> 一般不需要修改。如果目标运动极快（>12 rad/s），可减小 dt 换取更密的控制点。

---

### EKF 自适应噪声调参指南

#### 一图看懂噪声模型

```
                 ┌─────────────────────────────┐
                 │    use_legacy_noise_model?   │
                 └────────────┬────────────────┘
                  true        │        false
        ┌─────────────────────┴─────────────────────┐
        │  固定 Q/R，z/yaw 通道复用 x 的噪声参数      │
        │  行为与旧版完全一致                        │
        └───────────────────────────────────────────┘
                              │
              ┌───────────────┴────────────────┐
              │  Q 区分 outpost / normal        │
              │  R 根据 tracker 质量自适应      │
              └───────────────┬────────────────┘
                              │
              ┌───────────────┴────────────────┐
              │  is_outpost?                    │
              ├─ Yes → Q_linear × 0.1, Q_yaw × 0.001
              ├─ No  → Q_linear × 1.0, Q_yaw × 1.0
              └────────────────────────────────┘
```

#### 自适应 R 核心公式

```
position_quality = min(tracker.position_diff, r_quality_clip)
yaw_quality      = min(tracker.yaw_diff,      r_quality_clip)

position_scale = clamp(1 + r_position_gain × log1p(position_quality), 0.1, 100)
yaw_scale      = clamp(1 + r_yaw_gain      × log1p(yaw_quality),      0.1, 100)

若 tracker 未初始化 → position_scale ×= r_missing_quality_scale
                      yaw_scale      ×= r_missing_quality_scale

R_diag = [r_x×(|z_x|+0.02)×position_scale, r_y×(...), r_z×(...), r_yaw×yaw_scale]
```

**直观理解**：
- `tracker.position_diff` 大 → 观测不可靠 → position_scale 大 → R 大 → EKF 更信任预测（平滑但收敛慢）
- `tracker.position_diff` 小 → 观测可靠 → position_scale 小 → R 小 → EKF 更信任观测（收敛快）
- 未初始化 → scale 额外放大 → EKF 谨慎初始化，避免错误观测干扰

#### 调参顺序与诊断

| 步骤 | 参数 | 诊断现象 | 调整方向 |
|------|------|----------|----------|
| 1 | `r_position_gain` (默认 6.0) | 跟踪滞后 → 减小 gain；跟踪抖动 → 增大 gain | 2.0 ~ 10.0 |
| 2 | `r_yaw_gain` (默认 6.0) | yaw 估计抖动 → 增大 gain；yaw 收敛慢 → 减小 gain | 2.0 ~ 10.0 |
| 3 | `r_quality_clip` (默认 2.0) | position_diff 经常很大但收敛不错 → 减小 clip | 0.5 ~ 5.0 |
| 4 | `r_missing_quality_scale` (默认 4.0) | 初始化阶段抖动太大 → 增大 scale；初始化太慢 → 减小 scale | 1.0 ~ 10.0 |
| 5 | `r_x/r_y/r_z/r_yaw` (基值) | 最后一招，改基值相当于全局缩放 | 微调 ±50% |

#### Outpost 特殊处理

前哨站目标的特点：**几乎静止，但装甲板很小**。因此：
- `q_outpost_linear_factor=0.1` → 过程噪声降为 normal 的 10%，估计更平滑
- `q_outpost_yaw_factor=0.001` → yaw 过程噪声几乎为 0，避免估计旋转

> 若发现前哨站目标估计有漂移，适当增大这两个 factor；若抖动过大，适当减小。

---

### 数据流（MPC 路径）

```
Tracker (EKF)
  │  发布 Target 消息（位置已含 flying_time 预补偿）
  ▼
Solver::solve()
  ├─ [Legacy 路径] 几何计算 + TRACKING_ARMOR/CENTER 状态机
  │
  └─ [MPC 路径, use_tinympc=true]
      │
      ▼
    TinyMpcPlanner::plan()
      ├─ refreshParams()         ← 运行时热更新 Q/R/阈值
      ├─ buildTrajectory()       ← 100 步参考轨迹
      │    └─ 每步: 匀速外推 + getClosestArmorPosition() + computeAim()
      │         └─ 弹道补偿 (TrajectoryCompensator::compensate)
      ├─ tiny_solve(yaw/pitch)   ← ADMM 求解带约束 MPC
      └─ PlanResult { plan_yaw, plan_pitch, fire, aim_distance, aim_height }
      │
      ▼
    Solver 后处理
      ├─ manual_compensator->angleHardCorrect(aim_distance, aim_height)
      └─ 叠加 yaw_offset/pitch_offset → 写入 GimbalCmd
```

### 调试建议

- 设置 `debug: true` 可在日志中观察 MPC 求解状态、延迟选择和开火判定
- **MPC 频繁失败**：检查 `max_yaw_acc` / `max_pitch_acc` 是否过紧，或目标距离是否导致弹道无解
- **云台抖动**：增大 R_yaw/R_pitch 或减小 max_yaw_acc/max_pitch_acc
- **跟踪滞后**：增大 Q_yaw[0]/Q_pitch[0] 或增大 high_speed_delay_time
- **不开火**：增大 fire_thresh 或检查 shoot_offset 是否过大
- **误开火**：减小 fire_thresh 或增大 shoot_offset 使判定更严格
- **MPC vs Legacy 对比测试**：运行时切换 `use_tinympc` 参数即可热切换，无需重启


