# R1LiteCollector

R1LiteCollector 是一个基于 ROS 2 的机器人数据收集和控制系统，主要用于收集机器人操作数据（包括图像、关节状态、控制命令等）并保存为 HDF5 格式，同时提供机械臂控制和推理功能。

## 项目概述

本项目包含以下主要功能模块：

- **数据收集器 (data_collector)**: 从多个 ROS 2 话题收集数据并保存为 HDF5 格式
- **推理模块 (inference)**: 提供相机和关节基础推理节点
- **机械臂控制**: 支持 ARX R5 和 YAM 两种机械臂的控制
- **相机驱动**: 集成 Intel RealSense 相机驱动

## 系统要求

- **ROS 2**: 建议使用 ROS 2 Foxy 或更高版本
- **Python**: Python 3.8+
- **操作系统**: Linux (Ubuntu 20.04+ 推荐)

## 依赖项

### ROS 2 包依赖
- `rclpy`
- `sensor_msgs`
- `std_srvs`
- `nav_msgs`
- `cv_bridge`
- `std_msgs`
- `geometry_msgs`
- `tf2_ros`

### Python 依赖
- `h5py`
- `numpy`
- `opencv-python`
- `yaml`

## 安装

1. 克隆仓库到你的 ROS 2 工作空间：

```bash
cd ~/your_ros2_ws/src
git clone <repository_url> R1LiteCollector
```

2. 安装依赖：

```bash
cd ~/your_ros2_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

3. 构建项目：

```bash
cd ~/your_ros2_ws
colcon build --symlink-install
```

4. 设置环境：

```bash
source install/setup.bash
```

## 项目结构

```
R1LiteCollector/
├── src/
│   ├── data_collector/          # 数据收集器包
│   │   ├── config/              # 配置文件
│   │   ├── launch/              # Launch 文件
│   │   └── data_collector/      # Python 模块
│   ├── inference/               # 推理模块
│   │   ├── inference/           # Python 模块
│   │   └── launch/              # Launch 文件
│   ├── arx/                     # ARX 机械臂相关
│   │   ├── arx_r5_controller/   # ARX R5 控制器
│   │   └── arx_cmd_pub/         # ARX 命令发布器
│   ├── yam/                     # YAM 机械臂相关
│   │   ├── yam_damiao_controller/ # YAM 控制器
│   │   ├── yam_cmd_pub/         # YAM 命令发布器
│   │   └── yam_description/     # YAM 机器人描述
│   ├── msg/                     # 自定义消息类型
│   └── rs_cameras/              # RealSense 相机驱动
├── build/                       # 构建目录
├── install/                     # 安装目录
└── log/                         # 日志目录
```

## 使用方法

### 数据收集

1. 配置数据收集参数：

编辑 `src/data_collector/config/collect_config.yaml` 文件，设置：
- `data_dir`: 数据保存目录
- `collection_frequency`: 收集频率（Hz）
- `max_queue_size`: 最大队列大小
- `max_age`: 数据最大年龄（秒）
- `datasets`: 要收集的数据集配置

2. 启动数据收集器：

```bash
ros2 launch data_collector data_collector.launch.py
```

3. 开始收集数据：

```bash
ros2 service call /start_collect std_srvs/srv/Trigger
```

4. 停止收集并保存数据：

```bash
ros2 service call /stop_collect std_srvs/srv/Trigger
```

数据将保存为 HDF5 格式，文件名格式为 `collection_<timestamp>.hdf5`。

### 推理模块

启动推理节点（以 YAM 机械臂为例）：

```bash
ros2 launch inference infer_yam.launch.py
```

或启动 ARX 机械臂推理：

```bash
ros2 launch inference infer_arx.launch.py
```

### 机械臂控制

#### ARX R5 机械臂

启动单臂控制：

```bash
ros2 launch arx_r5_controller open_single_arm.launch.py
```

启动双臂控制：

```bash
ros2 launch arx_r5_controller open_double_arm.launch.py
```

启动 VR 控制模式：

```bash
ros2 launch arx_r5_controller open_vr_double_arm.launch.py
```

#### YAM 机械臂

启动单臂控制：

```bash
ros2 launch yam_damiao_controller zj_single_arm.launch.py
```

启动 VR 控制模式：

```bash
ros2 launch yam_damiao_controller open_vr_double_arm.launch.py
```

### RealSense 相机

启动 RealSense 相机：

```bash
ros2 launch realsense2_camera rs_launch.py
```

## 配置说明

### 数据收集配置

`collect_config.yaml` 文件包含以下主要配置项：

- **data_dir**: 数据保存路径
- **collection_frequency**: 数据收集频率（默认 50 Hz）
- **max_queue_size**: 每个话题的最大缓冲队列大小
- **max_age**: 允许的数据最大时间差（秒）
- **datasets**: 定义要收集的数据集，包括：
  - 话题名称
  - 数据处理器类型
  - 输出形状和数据类型
- **topics**: 定义话题的消息类型

### 数据格式

收集的数据以 HDF5 格式保存，包含：
- 图像数据（如果配置了相机话题）
- 关节状态数据
- 控制命令数据
- 踏板状态数据
- 收集频率信息

## 主要节点

### data_collector_node
数据收集主节点，负责：
- 订阅配置的话题
- 按指定频率收集数据
- 将数据保存为 HDF5 格式

### pedal_pub
踏板状态发布节点，用于发布踏板输入状态。

### joint_base_node
关节基础推理节点，处理关节状态推理。

### camera_node
相机推理节点，处理图像数据推理。

## 自定义消息类型

项目定义了以下自定义消息：

- `arm_control/PosCmd`: 位置控制命令
- `arx5_arm_msg/RobotCmd`: ARX 机器人命令
- `arx5_arm_msg/RobotStatus`: ARX 机器人状态
- `yam_arm_msg/YamCmd`: YAM 机械臂命令
- `yam_arm_msg/YamStatus`: YAM 机械臂状态
- `quest2_button_msg/Quest2Button`: Quest2 按钮消息

## 故障排除

### 常见问题

1. **数据收集失败**
   - 检查话题是否正确发布
   - 确认配置文件路径正确
   - 检查数据保存目录权限

2. **构建错误**
   - 确保所有依赖已安装
   - 运行 `rosdep install` 安装缺失依赖
   - 检查 ROS 2 环境是否正确设置

3. **相机连接问题**
   - 检查 RealSense 相机是否正确连接
   - 确认 USB 权限设置正确
   - 查看相机驱动日志

## 开发

### 添加新的数据处理器

1. 在 `data_collector/data_collector/data_processor.py` 中创建新的处理器类
2. 继承 `DataProcessor` 基类
3. 实现 `process()` 和 `get_dataset_config()` 方法
4. 在配置文件中引用新的处理器

### 添加新的话题

1. 在 `collect_config.yaml` 中添加话题配置
2. 定义消息类型
3. 配置相应的数据处理器

## 许可证

TODO: 添加许可证信息

## 维护者

- **eai** - 主要维护者

## 贡献

欢迎提交 Issue 和 Pull Request。

## 更新日志

- **v0.0.0**: 初始版本
  - 实现数据收集功能
  - 支持 ARX 和 YAM 机械臂控制
  - 集成 RealSense 相机驱动
  - 添加推理模块

