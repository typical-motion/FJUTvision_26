# rm_omniperception — 全向感知兜底模块

> 主自瞄链路（auto_aim）丢失目标时，由本模块接管云台控制，使用多路 USB 相机做装甲板检测，确保不跟丢。

---

## 架构总览

```
                    ┌─ video0 ──▶ Detector 0 ──┐
                    │                           │
 USB 相机 ──────────┼─ video2 ──▶ Detector 1 ──┼──▶ Decider ──▶ [队列] ──▶ omniperception/cmd_gimbal
 （1~3 路）         │                           │      │                         │
                    └─ video4 ──▶ Detector 2 ──┘   过滤→选优→角度换算            │
                                                                                ▼
                                                                     ╔═══════════════════════╗
 armor_solver/cmd_gimbal ──────────────────────────────────────────▶║ tracking ?            ║
 （主链路）                                                           ║                       ║
                                                                     ║  true  → 云台下位机    ║
                                                                     ║  false → 云台下位机    ║
                                                                     ╚═══════════════════════╝
                                                                          rm_serial_driver 仲裁
```

> **一句话**: 多路相机 → 并行检测 → 去噪选优 → 角度换算 → 发布 `GimbalCmd` → 串口层在 auto_aim 掉线时自动切换兜底。

---

## 代码结构

| 文件 | 职责 |
|------|------|
| [omniperception_node.cpp](src/omniperception_node.cpp) | 节点入口：参数加载、相机初始化、`set_mode` 服务、8ms 定时发布 |
| [omniperception_node.hpp](include/rm_omniperception/omniperception_node.hpp) | 节点声明 |
| [perceptron.cpp](src/omniperception/perceptron.cpp) | 多线程并行感知：读帧 → 检测 → 决策 → 入队，队列排序 |
| [perceptron.hpp](include/rm_omniperception/omniperception/perceptron.hpp) | Perceptron 声明 + `DetectorConfig` 结构 |
| [decider.cpp](src/omniperception/decider.cpp) | 决策逻辑：`armor_filter` 过滤 → `set_priority` 优先级 → `sort` 排序 → 角度换算 |
| [usbcamera.cpp](src/usbcamera/usbcamera.cpp) | USB 相机采集驱动（V4L2） |
| [detection.hpp](include/rm_omniperception/omniperception/detection.hpp) | `DetectionResult` 数据结构 |

---

## 运行流程

### ① 启动加载

1. 读取 `camera_count`（1~3 路）和 `camera_devices` 列表
2. 初始化每路 USB 相机（分辨率 / 帧率 / 曝光 / 增益）
3. 加载传统装甲板检测器参数（`DetectorConfig`）
4. 加载决策参数（`DeciderConfig`：偏置角、FOV、过滤规则）
5. 创建 Perceptron，每条相机独立线程
6. 注册 `rm_omniperception/set_mode` 服务（接收下位机红蓝切换）
7. 启动 8ms 定时器，周期性发布 `omniperception/cmd_gimbal`

### ② 每帧感知（Perceptron 线程）

```
USBCamera::read()  →  Detector::detect()  →  Decider::makeDecision()  →  入队
```

- 每条线程独占一个检测器实例，互不干扰
- 检测器复用 `armor_detector` 的传统 pipeline（二值化 → 灯条提取 → 装甲板匹配 → 数字分类）
- 只有 Decider 判定有效的结果才会入队

### ③ 决策流程（Decider）

#### 3.1 装甲板过滤 (`armor_filter`)

| 过滤规则 | 条件 | 配置参数 |
|----------|------|----------|
| 前哨站 | `number == "outpost"` | `ignore_outpost: true` |
| 基地 | `number == "base"` | `ignore_base: true` |
| 5号装甲板 | `number == "5"` | 25赛季固定过滤 |
| 干扰类型 | `number` 在 `ignore_numbers` 中 | `ignore_numbers: ["negative"]` |
| 无敌装甲板 | `number` 在 `invincible_armors_` 中 | 裁判系统动态更新 |

#### 3.2 优先级分类 (`set_priority`)

参考 sp_vision25 的优先级系统，按装甲板类型分配权重：

**模式 1 — 英雄优先** (`priority_mode: 1`)：
```
英雄(3/4) → 工程(1) → 哨兵(5/sentry) → 步兵(2) → 前哨站/基地
```

**模式 2 — 步兵优先** (`priority_mode: 2`)：
```
步兵(2) → 英雄(1/3/4/5) → 哨兵/前哨站/基地
```

#### 3.3 跨相机排序 (`sort`)

`popLatestDetection()` 取出队列中所有待处理结果：
1. 对每个 `DetectionResult` 内部先 `armor_filter` 再 `set_priority` 再按优先级排序
2. 移除全部被过滤的 `DetectionResult`
3. `DetectionResult` 之间按首个装甲板优先级排序
4. 返回优先级最高的结果

#### 3.4 场景区分：USB辅瞄 vs 背部相机

参考 sp_vision25 的 `delta_angle()`，本地 `makeDetection()` 根据相机位置自动选择不同 FOV：

| 场景 | camera_count | 背部相机位置 | 辅瞄相机用 FOV | 背部相机用 FOV |
|------|-------------|-------------|---------------|---------------|
| 单背部 | 1 | index=0 | — | `fov_h_deg / fov_v_deg` |
| 2路（1USB+1背） | 2 | index=1 | `new_fov_h_deg / new_fov_v_deg` | `fov_h_deg / fov_v_deg` |
| 3路全向（2USB+1背） | 3 | index=2 | `new_fov_h_deg / new_fov_v_deg` | `fov_h_deg / fov_v_deg` |

> 判断逻辑：`isBackCamera(index)` = 多相机时最后一个为背部，单相机时唯一相机即背部。

#### 3.5 角度换算

| 步骤 | 说明 |
|------|------|
| **选优** | 取排序后优先级第一的目标 |
| **换算** | 像素坐标 → 云台偏角（弧度） |

换算公式：

$$\Delta yaw = offset_{yaw}[cam_i] + (0.5 - \frac{x}{W}) \times FOV_H$$
$$\Delta pitch = offset_{pitch}[cam_i] + (\frac{y}{H} - 0.5) \times FOV_V$$

> 图像中心为原点：目标偏右 → 正 yaw（云台右转），目标偏上 → 正 pitch（云台上抬）

### ④ 定时发布

8ms 定时器每周期从队列取最新结果，组装 `GimbalCmd`：

| 字段 | 来源 |
|------|------|
| `yaw_diff` | `result.delta_yaw` |
| `pitch_diff` | `result.delta_pitch` |
| `fire_advice` | `true` 当队列中有有效目标时 |

---

## 敌方颜色动态切换

`rm_omniperception` 已接入下位机 SetMode 服务链路，比赛中红蓝对调时**无需重启**：

```
下位机 → 串口帧 → serial_driver 检测 mode 变化
  ├── armor_detector/set_mode
  ├── armor_solver/set_mode
  ├── rm_omniperception/set_mode  ← 本次新增
  └── rune_detector/set_mode（可选）
```

节点收到 `AUTO_AIM_RED(0)` 或 `AUTO_AIM_BLUE(1)` 后，通过 `Perceptron::setEnemyColor()` 同步更新所有检测器实例的 `detect_color`。Rune 模式不作响应（omniperception 仅负责自瞄兜底）。

配置文件中的 `enemy_color` 参数仅作为**初始值**，运行期以下位机下发的 mode 为准。

---

## 与 auto_aim 的仲裁

仲裁逻辑 **不在本模块内**，而在串口驱动层 (`rm_serial_driver`)：

```
订阅 armor_solver/target.tracking
  ├── tracking = true  →  使用 armor_solver/cmd_gimbal（主链路）
  └── tracking = false →  使用 omniperception/cmd_gimbal（兜底）
```

| 兵种 | 仲裁代码位置 |
|------|-------------|
| 默认协议 | [default_protocol.cpp](../rm_hardware_driver/rm_serial_driver/src/protocol/default_protocol.cpp) |
| 步兵协议 | [infantry_protocol.cpp](../rm_hardware_driver/rm_serial_driver/src/protocol/infantry_protocol.cpp) |
| 哨兵协议 | [sentry_protocol.cpp](../rm_hardware_driver/rm_serial_driver/src/protocol/sentry_protocol.cpp) |

---

## 启停控制

通过 `rm_bringup/config/launch_params.yaml` 中的开关控制：

```yaml
omniperception: false   # true = 启动, false = 不启动
```

启动入口在 [bringup.launch.py](../rm_bringup/launch/bringup.launch.py)，开关为 `true` 时才会拉起 `rm_omniperception_node`。

---

## 参数说明

参数文件：[omniperception_params.yaml](../rm_bringup/config/node_params/omniperception_params.yaml)

### 相机参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `camera_count` | `3` | USB 相机数量（1~3） |
| `camera_devices` | `["video0","video2","video4"]` | `/dev/` 下设备名 |
| `usb.width` | `1280` | 采集分辨率宽 |
| `usb.height` | `1024` | 采集分辨率高 |
| `usb.fps` | `120` | 目标帧率 |
| `usb.exposure` | `100` | 曝光 |
| `usb.gamma` | `80` | Gamma |
| `usb.gain` | `15` | 增益 |

### 检测器参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `enemy_color` | `"red"` | 敌方颜色初始值（运行期由下位机覆盖） |
| `binary_thres` | `160` | 二值化阈值 |
| `light.min_ratio` | `0.08` | 灯条最小长宽比 |
| `light.max_ratio` | `0.4` | 灯条最大长宽比 |
| `light.max_angle` | `40.0` | 灯条最大倾斜角 |
| `light.color_diff_thresh` | `25` | 颜色差异阈值 |
| `armor.min_light_ratio` | `0.6` | 装甲板最小灯条比例 |
| `armor.min_small_center_distance` | `0.8` | 小装甲板最小中心距 |
| `armor.max_small_center_distance` | `3.2` | 小装甲板最大中心距 |
| `armor.min_large_center_distance` | `3.2` | 大装甲板最小中心距 |
| `armor.max_large_center_distance` | `5.0` | 大装甲板最大中心距 |
| `armor.max_angle` | `35.0` | 装甲板最大倾斜角 |
| `use_classifier` | `true` | 是否启用数字分类器 |
| `classifier_threshold` | `0.7` | 分类置信度阈值 |
| `ignore_classes` | `["negative"]` | 忽略的分类标签 |

### 决策参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `camera_yaw_offsets_deg` | `[62.0, -62.0, 170.0]` | 每路相机的 yaw 安装偏角（度） |
| `camera_pitch_offsets_deg` | `[0.0, 0.0, 0.0]` | 每路相机的 pitch 安装偏角（度） |
| `fov_h_deg` | `54.2` | 背部相机水平视场角（度） |
| `fov_v_deg` | `44.5` | 背部相机垂直视场角（度） |
| `new_fov_h_deg` | `54.2` | USB辅瞄相机水平视场角（度），若换广角镜头需调整 |
| `new_fov_v_deg` | `44.5` | USB辅瞄相机垂直视场角（度） |
| `ignore_outpost` | `true` | 忽略前哨站 |
| `ignore_base` | `true` | 忽略基地 |
| `ignore_numbers` | `["negative"]` | 额外忽略的数字标签 |
| `priority_mode` | `1` | 优先级模式：`1`=英雄优先，`2`=步兵优先 |

---

## 调试指南

### ① 确认模块启动

将 `launch_params.yaml` 中 `omniperception` 设为 `true`，启动 bringup 后检查：

```bash
ros2 node list | grep rm_omniperception
```

应有 `rm_omniperception` 节点出现。

### ② 检查仲裁状态

```bash
# 主链路跟踪状态
ros2 topic echo armor_solver/target | grep tracking

# 主链路输出
ros2 topic echo armor_solver/cmd_gimbal

# 兜底输出
ros2 topic echo omniperception/cmd_gimbal
```

预期行为：
- `tracking=true` → 下位机走 `armor_solver/cmd_gimbal`
- `tracking=false` → 下位机走 `omniperception/cmd_gimbal`

### ③ 验证颜色切换

```bash
# 查看当前模式（收到下位机切换后应自动更新）
ros2 service call rm_omniperception/set_mode rm_interfaces/srv/SetMode "{mode: 1}"
ros2 topic echo omniperception/cmd_gimbal   # 观察是否还能正常检测目标
```

### ④ 相机排查

| 症状 | 检查项 |
|------|--------|
| 无图像 | `ls /dev/video*` 是否包含 `camera_devices` 中的设备 |
| 帧率低 | 降低 `usb.fps` 或 `usb.width × usb.height` |
| 检测率低 | 调整 `binary_thres`、确认 `enemy_color` 与下位机一致 |

---

## 已知局限

- **单帧单目标**：即使检测到多个装甲板，只输出优先级最高的一个
- **无跨帧融合**：未使用卡尔曼滤波或时序平滑，每帧独立决策
- **无跨相机融合**：多路相机各自独立检测，通过优先级排序合并，不做目标一致性校验
- **仅增量控制**：输出为 `delta_yaw / delta_pitch`，不直接提供世界坐标或跟踪 ID

后续若需要更接近 `sp_vision25` 的多相机时序融合行为，可在此基础上增加跨相机目标一致性校验与跟踪状态机。
