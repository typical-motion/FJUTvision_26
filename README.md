# FJUTvision_26

> 福建理工大学苍侠战队 · FJUT · 26 赛季 RoboMaster 视觉项目

本项目基于 [typical-motion/FYT2024_vision](https://github.com/typical-motion/FYT2024_vision)（原 FYT2024 Vision，作者 [@baiyeweiguang](https://github.com/baiyeweiguang)）**深度二次开发**而来。原项目在 [rm_vision](https://gitlab.com/rm_vision) 的基础上扩展了自瞄选板、能量机关识别与预测、哨兵定位、自主导航等功能；本仓库在继承上述能力的同时，针对实际比赛需求进行了大量重构与增强，**与原项目已形成明显区分**。

## 与上游的主要区别

> 由 [@leezoneline](https://github.com/leezoneline) 主导，基于数十次提交持续迭代

### 相机驱动（重构 + 工业级鲁棒性）
- 新增**海康相机（Hikrobot MVS）**支持，与原有**大恒相机（Daheng）**并存
- 相机节点整体重构，新增**心跳检测（heartbeat）**，断连后自动重连
- 完整处理相机**断连、超时、锁死、循环重启**等异常场景
- 修复海康 SDK **缓冲池耗尽**导致的崩溃问题
- 新增内录视频 **recorder** 模块

### 前哨站（哨兵）装甲板支持
- 支持**前哨站（outpost）**装甲板识别与打击
- 新增 `armor_id` 字段下发云台，支持按板选择
- 新增自适应运动模型 `CONSTANT_ROTATION`，针对前哨站单独调节过程噪声

### 自瞄解算增强
- 引入 **TinyMPC** 模型预测控制（MPC）轨迹规划
- 新增 **ManualCompensator** 手动补偿器（距离/高度二维查找表插值）
- 抽象出 **ArmorPoseEstimator**（BA 优化）类
- 增加**粒子滤波器**，为状态估计提供新选择
- EKF 使用 Ceres 自动微分**自动求解 Jacobian**，并支持**自适应 Q/R**
- 重写 **PnP 选解逻辑**
- 自瞄解算改为**定时器回调，固定解算频率**

### 串口与模式
- 修复打符模式串口卡死问题
- 新增 `ignore_enemy_2`、虚拟串口（`virtual_serial`）等选项
- 支持敌方颜色**运行时动态切换**（SetMode 服务，无需重启）

### 工程结构
- 重组仓库目录为规范的 `src/` 布局，优化 CMake 结构

## 一、项目结构

```
.
├── src/
│   ├── rm_bringup            (启动及参数文件)
│   ├── rm_robot_description  (机器人 urdf，坐标系定义)
│   ├── rm_interfaces         (自定义 msg / srv)
│   ├── rm_hardware_driver
│   │   ├── rm_camera_driver  (相机驱动：海康 / 大恒 + 内录)
│   │   └── rm_serial_driver  (串口驱动：步兵 / 哨兵协议)
│   ├── rm_auto_aim
│   │   ├── armor_detector    (装甲板识别)
│   │   └── armor_solver      (解算 + EKF + MPC)
│   ├── rm_rune
│   │   ├── rune_detector     (能量机关识别)
│   │   └── rune_solver       (能量机关预测)
│   ├── rm_utils
│   │   ├── math              (PnP、弹道补偿、EKF、粒子滤波、MPC 等)
│   │   └── logger            (日志库)
│   └── rm_upstart            (自启动配置)
├── build/                    (编译产物)
├── install/
└── log/
```

> 原项目中的 `rm_localization`（定位）、`rm_perception`（感知）、`rm_navigation`（导航）、`rm_decision`（自主决策）以及 `livox_ros_driver2`（Livox 激光雷达驱动）等模块未在本仓库直接提供，其中部分已在 [CSU-RM-Sentry](https://github.com/baiyeweiguang/CSU-RM-Sentry) 开源。

## 二、环境

如果你不需要完整功能，可以直接把相关的功能包删除掉

### 1. 基础
- Ubuntu 22.04
- ROS2 Humble
- 大恒相机驱动（Daheng Galaxy SDK）
- 海康相机驱动（Hikrobot MVS SDK，可选，按实际相机型号安装）

### 2. 自瞄 
- fmt库
  ```bash
  sudo apt install libfmt-dev
  ```
- Sophus库 (G2O库依赖)
   ```bash
   git clone https://github.com/strasdat/Sophus
   cd Sophus
   mkdir build && cd build
   cmake ..
   make -j
   sudo make install
   ```
- G2O库 (优化装甲板Yaw角度)
    ```bash
    sudo apt install libeigen3-dev libspdlog-dev libsuitesparse-dev qtdeclarative5-dev qt5-qmake libqglviewer-dev-qt5
    git clone https://github.com/RainerKuemmerle/g2o
    cd g2o
    mkdir build && cd build
    cmake ..
    make -j
    sudo make install
    ```
### 3. 能量机关
- OpenVINO库 (能量机关识别)
  
   参考[OpenVINO官方文档](https://docs.openvino.ai/2022.3/openvino_docs_install_guides_installing_openvino_from_archive_linux.html)，建议同时安装GPU相关依赖

- Ceres库 (能量机关曲线拟合)
    ```bash
    sudo apt install libceres-dev
    ```

### 4. 导航
- Livox SDK2

  参考[Livox官方仓库](https://github.com/Livox-SDK/Livox-SDK2)

- Navigation2 (导航)
    ```bash
    sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
    ```

- PCL库 (导航时对点云的处理)
    ```
    sudo apt install libpcl-dev
    ```

### 5. 其他

本文档中可能有缺漏，如有，可以用`rosdep`安装剩下依赖

```bash
rosdep install --from-paths src --ignore-src -r -y

```

## 三、编译与运行

修改rm_bringup/config/launch_params.yaml，选择需要启动的功能

```bash
# 编译
colcon build --symlink-install --parallel-workers 4 #本仓库包含的功能包过多，建议限制同时编译的线程数
# 运行
source install/setup.bash
ros2 launch rm_bringup bringup.launch.py
```

默认日志和内录视频路径为`~/fyt2024-log/`

> 我们的日志库是用fmt搓的，不使用ros2的日志库


## 四、自启动

- 编译程序后，进入rm_upstart文件夹

```bash
cd rm_upstart
```

- 修改**rm_watch_dog.sh**中的`NAMESPACE`（ros命名空间）、`NODE_NAMES`（需要看门狗监控的节点）和`WORKING_DIR` （代码路径）

- 注册服务
  
```bash
sudo chmod +x ./register_service.sh
sudo ./register_service.sh

# 正常时有如下输出
# Creating systemd service file at /etc/systemd/system/rm.service...
# Reloading systemd daemon...
# Enabling service rm.service...
# Starting service rm.service...
# Service rm.service has been registered and started.
```

- 查看程序状态

```bash
systemctl status rm
```

- 查看终端输出
```
查看screen.output或~/fyt2024-log下的日志
```  

- 关闭程序

```bash
systemctl stop rm
```

- 取消自启动

```bash
systemctl disable rm
```

## 维护者及开源许可证

> 赛季结束开源

Maintainer : [leezoneline](https://github.com/leezoneline) · Typical Motion · FJUT · 26 赛季 RoboMaster 视觉组

```
Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
```

## 致谢

本项目是在原项目 [typical-motion/FYT2024_vision](https://github.com/typical-motion/FYT2024_vision) 基础上的二次开发成果，谨向以下项目与作者致以最诚挚的感谢：

**特别致谢**

- 🙏 **原项目 FYT2024_vision**（[typical-motion/FYT2024_vision](https://github.com/typical-motion/FYT2024_vision)，作者 [@baiyeweiguang](https://github.com/baiyeweiguang)）—— 本项目在其之上深度开发，继承了其自瞄选板、能量机关识别与预测、哨兵定位、自主导航等核心框架与思想，是本项目得以快速迭代的坚实基础。
- 🙏 **sp_vision26** —— 为本项目提供了重要参考与代码移植来源，在感知与工程实现思路上给予了诸多启发，特此致谢。

**其他参考项目**

感谢这个赛季视觉组的每一个成员的付出，以及以下开源项目：

- [rm_vision](https://gitlab.com/rm_vision) rv 是本项目的基础，提供了一套可参考的，规范、易用、高效的视觉算法框架
- [rmoss](https://github.com/robomaster-oss/rmoss_core) rmoss 项目为 RoboMaster 提供通用基础功能模块包，本项目的串口驱动模块基于 rmoss_base 进行开发
- [沈阳航空航天大学TUP战队2022赛季步兵视觉开源](https://github.com/tup-robomaster/TUP-InfantryVision-2022) 为本项目的能量机关识别与预测算法提供了参考
- [沈阳航空航天大学YOLOX关键点检测模型](https://github.com/tup-robomaster/TUP-NN-Train-2) 提供了本项目能量机关识别模型训练代码
- [四川大学OpenVINO异步推理代码](https://github.com/Ericsii/rm_vision-OpenVINO) 提供了本项目能量机关识别模型部署的代码
- [上海交通大学自适应扩展卡尔曼滤波](https://github.com/julyfun/rm.cv.fans/tree/main) 使用 Ceres 自动微分功能，自动计算 Jacobian 矩阵


## 更新日志

### 二次开发（leezoneline）

- 重构相机驱动：新增海康相机（MVS）支持、心跳检测、断连/超时/锁死处理，修复 SDK 缓冲池耗尽崩溃
- 新增内录视频 recorder 模块
- 新增前哨站（outpost）装甲板支持，下发 `armor_id` 至云台
- 引入 TinyMPC 模型预测控制轨迹规划
- 增加 ManualCompensator 手动补偿器
- 抽象出 ArmorPoseEstimator（BA 优化）类
- 增加粒子滤波器，为状态估计提供新选择
- EKF 参考上交开源实现 Ceres 自动求 Jacobian，并支持自适应 Q/R
- 重写 PnP 选解逻辑
- 将自瞄解算改为定时器回调，固定解算频率
- 修复打符模式串口卡死问题
- 修复打符崩溃问题（OpenVINO 推理时不能创建新的 InferRequest，通过互斥锁解决）
- 新增 `ignore_enemy_2`、虚拟串口等选项，支持敌方颜色运行时动态切换
- 优化仓库结构为规范的 `src/` 布局
