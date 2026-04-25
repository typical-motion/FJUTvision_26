# rm_omniperception

## 1. 功能定位

rm_omniperception 是自瞄链路的全向感知兜底模块。

设计目标：
- 使用 1 到 3 路 USB 摄像头做辅瞄输入。
- 复用 rm_auto_aim 的传统装甲板检测器（非 YOLO）。
- 当 auto_aim 无目标跟踪时，向云台输出兜底控制量。

## 2. 当前代码结构

- 节点入口：[src/omniperception_node.cpp](src/omniperception_node.cpp)
- 节点声明：[include/rm_omniperception/omniperception_node.hpp](include/rm_omniperception/omniperception_node.hpp)
- 多相机并行感知：[src/omniperception/perceptron.cpp](src/omniperception/perceptron.cpp)
- 决策与角度换算：[src/omniperception/decider.cpp](src/omniperception/decider.cpp)
- USB 相机驱动：[src/usbcamera/usbcamera.cpp](src/usbcamera/usbcamera.cpp)
- 检测结果结构：[include/rm_omniperception/omniperception/detection.hpp](include/rm_omniperception/omniperception/detection.hpp)

## 3. 全向感知主流程

### 3.1 启动阶段

OmniPerceptionNode 在启动时完成以下工作：
- 读取 camera_count（并限制在 1 到 3）。
- 读取 camera_devices（默认 video0/video2/video4）。
- 创建每路 USB 相机实例。
- 读取 DetectorConfig（传统检测器参数）和 DeciderConfig（偏置/FOV/过滤参数）。
- 创建 Perceptron。
- 启动 8ms 定时器，对外发布 omniperception/cmd_gimbal。

入口代码见 [src/omniperception_node.cpp](src/omniperception_node.cpp#L10)。

### 3.2 感知并行阶段

Perceptron 采用每路相机一个线程：
- 从对应 USBCamera 读取最新帧。
- 调用 fyt::auto_aim::Detector 进行装甲板检测。
- 将检测结果交给 Decider 进行过滤与角度计算。
- 将有效结果压入线程安全队列（内部互斥保护）。

实现见 [src/omniperception/perceptron.cpp](src/omniperception/perceptron.cpp#L12)。

### 3.3 目标决策阶段

Decider 主要做三件事：
- 过滤：忽略 outpost/base/negative 等类别。
- 选优：选择图像中心最近的目标。
- 角度计算：根据 camera_yaw_offsets_deg、camera_pitch_offsets_deg、fov_h_deg、fov_v_deg 输出 delta_yaw 和 delta_pitch（弧度）。

实现见 [src/omniperception/decider.cpp](src/omniperception/decider.cpp#L11)。

### 3.4 输出阶段

节点定时器从 Perceptron 取最新 DetectionResult，并组装 rm_interfaces/msg/GimbalCmd：
- yaw_diff = delta_yaw
- pitch_diff = delta_pitch
- fire_advice = armors 非空

发布话题：omniperception/cmd_gimbal

实现见 [src/omniperception_node.cpp](src/omniperception_node.cpp#L108)。

## 4. 与 auto_aim 的兜底仲裁逻辑

仲裁已经放在串口协议层（rm_serial_driver）：
- 订阅 armor_solver/target 的 tracking。
- tracking 为 true：转发 armor_solver/cmd_gimbal。
- tracking 为 false：转发 omniperception/cmd_gimbal。

对应实现：
- 默认协议：[../rm_hardware_driver/rm_serial_driver/src/protocol/default_protocol.cpp](../rm_hardware_driver/rm_serial_driver/src/protocol/default_protocol.cpp#L26)
- 步兵协议：[../rm_hardware_driver/rm_serial_driver/src/protocol/infantry_protocol.cpp](../rm_hardware_driver/rm_serial_driver/src/protocol/infantry_protocol.cpp#L49)
- 哨兵协议：[../rm_hardware_driver/rm_serial_driver/src/protocol/sentry_protocol.cpp](../rm_hardware_driver/rm_serial_driver/src/protocol/sentry_protocol.cpp#L102)

这保证了全向感知只在 auto_aim 丢失目标时接管输出。

## 5. 与 bringup 开关关系

开关文件：
- [../rm_bringup/config/launch_params.yaml](../rm_bringup/config/launch_params.yaml)

关键字段：
- omniperception: false

当该开关为 true 时，bringup 会启动 rm_omniperception_node；为 false 时不启动。

对应启动逻辑：
- [../rm_bringup/launch/bringup.launch.py](../rm_bringup/launch/bringup.launch.py#L124)
- [../rm_bringup/launch/bringup.launch.py](../rm_bringup/launch/bringup.launch.py#L217)

## 6. 参数文件

当前已接入参数文件：
- [../rm_bringup/config/node_params/omniperception_params.yaml](../rm_bringup/config/node_params/omniperception_params.yaml)

主要参数分组：
- 相机输入参数：camera_count、camera_devices、usb.*
- 检测器参数：binary_thres、light.*、armor.*、classifier*
- 决策参数：camera_yaw_offsets_deg、camera_pitch_offsets_deg、fov*、ignore*

## 7. 调试建议

### 7.1 启停验证

- 将 omniperception 打开后启动 bringup。
- 观察是否出现 rm_omniperception 节点日志。

### 7.2 话题链路验证

建议依次检查：
- armor_solver/target（tracking 状态）
- armor_solver/cmd_gimbal（主链路输出）
- omniperception/cmd_gimbal（兜底输出）

预期：
- tracking=true 时，下位机接收 auto_aim 输出。
- tracking=false 时，下位机接收 omniperception 输出。

### 7.3 相机验证

- 若无图像输出，优先检查 /dev/video* 是否匹配 camera_devices。
- 若帧率不稳，可降低 usb.fps 或分辨率。

## 8. 已知边界

- 当前 Decider 采用单目标中心优先策略，未做跨相机时序融合。
- 当前输出是云台增量控制，不直接输出多目标跟踪状态。
- 若后续需要更接近 sp_vision25 行为，可继续增加跨相机目标一致性与优先级状态机。
