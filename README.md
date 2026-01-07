# R1LiteCollector

R1LiteCollector 是一个基于 ROS 2 的机器人数据收集和控制系统，主要用于收集机器人操作数据（包括图像、关节状态、控制命令等）并保存为 HDF5 格式，同时提供机械臂控制和推理功能。

## 项目概述

本项目包含以下主要功能模块：

- **数据收集器 (data_collector)**: 从多个 ROS 2 话题收集数据并保存为 HDF5 格式
- **推理模块 (inference)**: 提供相机和关节基础推理节点
- **机械臂控制**: 支持 ARX R5 和 YAM 两种机械臂的控制
- **相机驱动**: 集成 Intel RealSense 相机驱动

## 系统要求

- **ROS 2**: 建议使用 ROS 2 Humble 或更高版本
- **Python**: Python 3.10.12
- **操作系统**: Linux (Ubuntu 22.04 推荐)

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
# 启动 data_collector_node + pedal
ros2 launch data_collector data_collector.launch.py
```
或者
```bash
# 单独启动d ata_collector_node
ros2 run data_collector data_collector_node     --ros-args -p config_path:=src/data_collector/config/collect_config.yaml
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

---

## 技术文档

### 数据收集器详细解析

#### 概述

`data_collector` 是一个用于机器人学习数据收集的 ROS 2 节点，主要功能是：
- **多传感器数据同步收集**：支持图像、点云、关节状态、电流等多种数据类型
- **高频率数据采集**：默认 50Hz 采集频率
- **时间戳对齐**：确保多传感器数据的时间同步
- **HDF5 格式存储**：高效存储大规模数据集
- **服务化控制**：通过 ROS 2 服务控制数据收集的开始和停止

#### 架构设计

```
┌─────────────────────────────────────────────────────────┐
│              Data Collector Node                        │
├─────────────────────────────────────────────────────────┤
│  ┌──────────────┐      ┌──────────────┐                │
│  │  Subscribers │      │  Processors  │                │
│  │  (多话题订阅)  │ ───> │  (数据处理)   │                │
│  └──────────────┘      └──────────────┘                │
│         │                      │                        │
│         │              ┌──────────────┐                 │
│         │              │   Buffers    │                 │
│         │              │ (时间戳队列)  │                 │
│         │              └──────────────┘                 │
│         │                      │                        │
│         │         ┌────────────────────┐               │
│         │         │  Collect Timer     │               │
│         │         │   (50Hz 定时器)     │               │
│         │         └────────────────────┘               │
│         │                      │                        │
│         │         ┌────────────────────┐               │
│         │         │  Data Synchronizer │               │
│         │         │  (时间戳对齐)       │               │
│         │         └────────────────────┘               │
│         │                      │                        │
│         │         ┌────────────────────┐               │
│         │         │  HDF5 Writer       │               │
│         │         │  (数据保存)         │               │
│         │         └────────────────────┘               │
│                                                          │
│  ┌──────────────────────────────────────┐              │
│  │  Service Servers                     │              │
│  │  - /start_collect (Trigger)          │              │
│  │  - /stop_collect (Trigger)           │              │
│  └──────────────────────────────────────┘              │
└─────────────────────────────────────────────────────────┘
```

#### 核心组件

**DataCollector 类主要属性**：
- `self.config`: 从 YAML 加载的配置
- `self.processors`: 数据处理器字典
- `self.buffers`: 数据缓冲区（时间戳+消息）
- `self.locks`: 线程锁字典
- `self.is_collecting`: 是否正在收集数据
- `self.collected_dataset_list`: 收集的数据列表

**关键方法**：
- `setup_processors()`: 根据配置文件动态加载数据处理器
- `setup_subscriptions()`: 创建 ROS 2 订阅者
- `collect_data()`: 定时收集数据（50Hz），实现时间戳对齐
- `save_data()`: 将收集的数据保存为 HDF5 文件

#### 数据流程

```
1. ROS 话题发布数据
   │
   ▼
2. Subscriber 回调函数
   │  - 提取时间戳
   │  - 存入缓冲区 (stamp, msg)
   │
   ▼
3. 定时器触发 (50Hz)
   │
   ▼
4. collect_data() 执行
   │  - 获取当前时间戳
   │  - 对每个数据集：
   │    ├─ 在缓冲区中二分查找最接近的数据
   │    ├─ 检查数据新鲜度
   │    └─ 使用 Processor 处理数据
   │
   ▼
5. 处理后的数据添加到 collected_dataset_list
   │
   ▼
6. 用户调用 /stop_collect 服务
   │
   ▼
7. save_data() 执行
   │  - 创建 HDF5 文件
   │  - 预创建所有数据集
   │  - 写入数据
   │  - 保存元数据
   │
   ▼
8. HDF5 文件保存完成
```

#### 时间戳对齐机制

使用二分查找 (`bisect_left`) 快速找到最接近的数据：

```python
def find_closest(self, stamps, idx, target):
    """找到最接近目标时间戳的数据索引"""
    if idx == 0:
        return 0 if stamps else None
    elif idx == len(stamps):
        return idx - 1
    else:
        before = stamps[idx - 1]
        after = stamps[idx]
        # 选择更接近的一个
        return idx - 1 if (target - before) <= (after - target) else idx
```

**关键点**：
- 使用 `bisect_left` 进行 O(log n) 的二分查找
- 检查数据新鲜度（`max_age` 参数，默认 0.5 秒）
- 如果数据过期，抛出错误并停止收集

#### 数据处理器详解

**处理器类层次结构**：
```
DataProcessor (基类)
├── ImageProcessor          # RGB 图像处理
├── UniversalImageProcessor # 通用图像处理（支持 Image 和 CompressedImage）
├── DepthProcessor          # 深度图像处理
├── PointcloudProcessor     # 点云处理
├── JointPositionProcessor  # 关节位置处理
├── JointCurrentProcessor   # 关节电流处理
├── TwistProcessor          # 速度命令处理
├── OdomProcessor           # 里程计处理
└── PedalProcessor          # 踏板状态处理
```

**基类接口**：
```python
class DataProcessor:
    def process(self, msg) -> Any:
        """处理 ROS 消息，返回 numpy 数组"""
        raise NotImplementedError
    
    def get_dataset_config(self) -> Dict[str, Any]:
        """返回数据集配置（shape, dtype, description）"""
        return {
            'shape': tuple(self.config['output_shape']),
            'dtype': self.config['dtype'],
            'description': self.config['description'],
        }
```

### data_collector_node.py 代码详解

#### 核心方法详解

**1. setup_processors() - 动态加载数据处理器**

```python
def setup_processors(self):
    datasets = self.config['datasets']
    for dataset_name, datset_value in datasets.items():
        # 解析处理器类路径
        module_name, class_name = datset_value['processor'].rsplit('.', 1)
        # 动态导入模块
        module = importlib.import_module(module_name)
        # 获取处理器类
        processor_class = getattr(module, class_name)
        # 实例化处理器
        self.processors[dataset_name] = processor_class(
            datset_value.get('processor_config', {})
        )
```

**关键点**：
- **动态加载**：支持在配置文件中指定任意处理器类
- **错误处理**：加载失败时立即抛出异常，避免部分初始化
- **线程锁管理**：多个数据集可能共享同一话题，只需一个锁

**2. setup_subscriptions() - 创建 ROS 订阅者**

使用动态消息类型解析机制，能够处理任意 ROS 2 消息类型：

```python
def setup_subscriptions(self):
    # 创建 QoS 配置（高频数据优化）
    qos_profile = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,  # 不保证可靠性
        history=HistoryPolicy.KEEP_LAST,            # 只保留最新数据
        depth=1                                      # 队列深度为 1
    )
    
    for topic_name, topic_cfg in topic_configs.items():
        # 动态解析消息类型
        msg_module, msg_type = topic_cfg['msg_type'].rsplit('.', 1)
        # 动态导入消息类
        module = importlib.import_module(msg_module)
        msg_class = getattr(module, msg_type)
        
        # 创建订阅者
        self.create_subscription(
            msg_class,
            topic_name,
            self.create_callback(topic_name),
            qos_profile
        )
```

**QoS 策略说明**：
- `BEST_EFFORT`: 适合高频实时数据，不保证每个消息都收到
- `KEEP_LAST` + `depth=1`: 只保留最新消息，避免数据堆积

**3. collect_data() - 核心数据收集方法**

这是整个节点的核心方法，实现多传感器数据的时间戳对齐和同步收集：

```python
def collect_data(self):
    # 获取当前时间戳（ROS 2 时钟）
    current_time = self.get_clock().now().nanoseconds / 1e9
    processed_data_dict = {}
    max_age = self.config['max_age']
    
    # 遍历所有数据集
    for dataset_name, dataset_cfg in self.config['datasets'].items():
        topic_name = dataset_cfg['topic']
        
        # 线程安全地访问缓冲区
        with self.locks[topic_name]:
            buffer = self.buffers[topic_name]
            
            # 提取所有时间戳
            stamps = [s for s, _ in buffer]
            
            # 二分查找最接近的时间戳
            idx = bisect_left(stamps, current_time)
            closest = self.find_closest(stamps, idx, current_time)
            
            # 检查数据新鲜度
            delta_time = abs(buffer[closest][0] - current_time)
            if delta_time > max_age:
                self.raise_error(f"Stale data in {topic_name}, delta time: {delta_time}")
                return
            
            # 提取消息并处理
            _, msg = buffer[closest]
            processed_data_dict[dataset_name] = self.processors[dataset_name].process(msg)
            
            # 保留当前数据及之后的数据（用于下次查找）
            self.buffers[topic_name] = buffer[closest:]
    
    # 添加到收集列表
    self.collected_dataset_list.append(processed_data_dict)
```

**4. save_data() - 保存数据到 HDF5**

```python
def save_data(self):
    # 创建数据目录
    data_dir = self.config['data_dir']
    os.makedirs(data_dir, exist_ok=True)
    
    # 生成文件名（时间戳）
    timestamp = int(time.time())
    file_path = os.path.join(data_dir, f'collection_{timestamp}.hdf5')
    
    with h5py.File(file_path, 'w') as hf:
        # 预创建所有数据集
        hdf5_dataset_dict = {}
        for dataset_name, dataset_cfg in self.dataset_cfg_dict.items():
            shape = (len(self.collected_dataset_list), *dataset_cfg['shape'])
            dataset = hf.create_dataset(
                dataset_name,
                shape,
                dtype=dataset_cfg['dtype'],
                chunks=(1, *dataset_cfg['shape'])  # 分块存储
            )
            dataset.attrs['description'] = dataset_cfg['description']
            hdf5_dataset_dict[dataset_name] = dataset
        
        # 写入数据
        for i, collected_dataset_dict in enumerate(self.collected_dataset_list):
            for collected_dataset_name, collected_dataset_data in collected_dataset_dict.items():
                hdf5_dataset = hdf5_dataset_dict[collected_dataset_name]
                hdf5_dataset[i] = collected_dataset_data
        
        # 保存元数据
        hf.create_dataset(
            'collection_frequency',
            data=self.config['collection_frequency'],
            dtype='f'
        )
    
    return file_path
```

**HDF5 优化策略**：
1. **预创建数据集**：提前知道数据大小，一次性分配空间
2. **分块存储**：每个样本一个块，提高读写性能
3. **属性存储**：存储元数据（描述信息）

### 消息类型处理机制

`setup_subscriptions()` 方法使用**动态消息类型解析**机制，能够处理任意 ROS 2 消息类型，无需硬编码。

#### 动态消息类型解析流程

**步骤 1: 解析消息类型字符串**
```python
msg_module, msg_type = topic_cfg['msg_type'].rsplit('.', 1)
# 例如: "sensor_msgs.msg.Image"
#      -> msg_module = "sensor_msgs.msg"
#      -> msg_type = "Image"
```

**步骤 2: 动态导入模块**
```python
module = importlib.import_module(msg_module)
# 等价于: from sensor_msgs import msg
```

**步骤 3: 获取消息类**
```python
msg_class = getattr(module, msg_type)
# 等价于: msg_class = module.Image
```

**步骤 4: 创建订阅者**
```python
self.create_subscription(
    msg_class,                    # 消息类（动态获取）
    topic_name,                   # 话题名称
    self.create_callback(topic_name),  # 回调函数
    qos_profile                   # QoS 配置
)
```

#### 支持的消息类型

**标准 ROS 2 消息**：
- `sensor_msgs.msg.Image` - RGB/深度图像
- `sensor_msgs.msg.PointCloud2` - 点云数据
- `sensor_msgs.msg.Joy` - 游戏手柄/踏板输入
- `sensor_msgs.msg.CompressedImage` - 压缩图像
- `nav_msgs.msg.Odometry` - 里程计数据
- `geometry_msgs.msg.TwistStamped` - 速度命令

**自定义消息**：
- `arx5_arm_msg.msg.RobotStatus` - ARX 机械臂状态
- `arx5_arm_msg.msg.RobotCmd` - ARX 机械臂命令
- `yam_arm_msg.msg.YamStatus` - YAM 机械臂状态
- `yam_arm_msg.msg.YamCmd` - YAM 机械臂命令

#### 设计优势

✅ **灵活性**：支持任意消息类型，无需修改代码  
✅ **可维护性**：配置驱动，集中管理  
✅ **类型安全**：使用实际的类对象，不是字符串  
✅ **性能**：延迟加载，只在需要时导入模块

### pedal.py 详细解析

#### 概述

`pedal.py` 是一个 ROS 2 节点，用于**将键盘输入转换为踏板状态信号**，并通过 ROS 话题发布。它的主要作用是**在数据收集过程中标记子任务阶段**，类似于物理踏板的数字替代品。

#### 核心功能

1. **键盘监听**：监听 `Scroll Lock` 键，使用 `pynput` 库进行全局键盘监听
2. **状态发布**：发布到 `/pedal_status` 话题，消息类型为 `sensor_msgs.msg.Joy`
3. **线程安全**：使用独立的监听线程，避免阻塞主线程

#### 工作原理

```
1. 节点启动
   ├─> 创建键盘监听线程（后台运行）
   │   └─> 监听 Scroll Lock 键按下事件
   ├─> 创建定时器（15Hz）
   │   └─> 定期执行 _publish_callback()
   └─> 进入 ROS 2 主循环

2. 用户按下 Scroll Lock 键
   └─> on_press() 回调触发
       └─> 设置 current_state = 1（线程锁保护）

3. 定时器触发（每 1/15 秒）
   └─> _publish_callback() 执行
       ├─> 读取 current_state（线程锁保护）
       ├─> 创建 Joy 消息
       │   └─> buttons[0] = current_state
       ├─> 发布到 /pedal_status 话题
       └─> 重置 current_state = 0（单次触发）
```

#### 代码详解

**初始化**：
```python
class PedalPub(Node):
    def __init__(self):
        super().__init__('PedalPub')
        
        # 配置发布频率（默认 15Hz）
        self.declare_parameter('publish_rate', 15)
        self.publish_rate = self.get_parameter('publish_rate').value
        
        # 线程安全的状态变量
        self.state_lock = Lock()
        self.current_state = 0
        
        # 创建 ROS 发布者
        self.publisher = self.create_publisher(Joy, 'pedal_status', 5)
        
        # 启动键盘监听线程
        self.monitor_thread = Thread(target=self._responsive_monitor, daemon=True)
        self.monitor_thread.start()
        
        # 创建定时器
        self.timer = self.create_timer(1.0/self.publish_rate, self._publish_callback)
```

**键盘监听**：
```python
def _responsive_monitor(self):
    def on_press(key):
        try:
            with self.state_lock:
                if key == keyboard.Key.scroll_lock:
                    self.current_state = 1
        except Exception as e:
            self.get_logger().error(f"Error in on_press: {str(e)}")
    
    with keyboard.Listener(on_press=on_press) as listener:
        listener.join()
```

**状态发布**：
```python
def _publish_callback(self):
    """定时发布回调"""
    with self.state_lock:
        msg = Joy()
        msg.buttons = [self.current_state]  # 0 或 1
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(msg)
        self.current_state = 0  # 重置为 0（单次触发）
```

#### 在数据收集中的作用

`pedal.py` 与 `data_collector` 节点配合使用，用于**标记数据收集的子任务阶段**：

1. **数据收集流程**：
   ```
   用户按下 Scroll Lock
   │
   └─> pedal_pub 发布 /pedal_status (buttons[0] = 1)
       │
       └─> data_collector 订阅该话题
           │
           └─> PedalProcessor 处理
               │
               └─> 转换为子任务 ID (10 = 单步，0 = 正常)
                   │
                   └─> 保存到 HDF5 文件的 /subtask 数据集
   ```

2. **PedalProcessor 的处理逻辑**：
   ```python
   def process(self, msg):
       data = msg.buttons
       if data[0] == 1:
           value = 10  # 子任务标记：单步
       else:
           value = 0   # 正常状态
       return value
   ```

#### 使用场景

- **任务分段**：在数据收集中标记不同的任务阶段
- **事件标注**：标记关键动作时刻（如抓取、放置）
- **数据筛选**：后续可以根据子任务 ID 筛选特定阶段的数据

#### 启动方式

```bash
# 通过 launch 文件启动（推荐）
ros2 launch data_collector data_collector.launch.py

# 单独启动
ros2 run data_collector pedal_pub

# 自定义发布频率
ros2 run data_collector pedal_pub --ros-args -p publish_rate:=30
```

### 性能优化

#### 1. QoS 策略

使用 `BEST_EFFORT` QoS 策略处理高频数据：
```python
qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,  # 不保证可靠性
    history=HistoryPolicy.KEEP_LAST,            # 只保留最新数据
    depth=1                                      # 队列深度为 1
)
```

**优点**：
- 减少延迟
- 避免数据堆积
- 适合实时控制场景

#### 2. 缓冲区管理

- **固定大小队列**：防止内存溢出
- **时间戳索引**：快速查找
- **线程锁保护**：多线程安全

#### 3. HDF5 优化

- **预创建数据集**：避免动态扩展开销
- **分块存储**：提高读写性能
- **数据类型优化**：使用 `float16` 节省空间

### 故障排查

#### 常见问题

1. **数据过期错误**
   ```
   Stale data in /topic_name, delta time: 0.6
   ```
   **原因**：数据延迟超过 `max_age` 阈值  
   **解决**：检查话题发布频率，或增加 `max_age` 值

2. **缓冲区为空**
   ```
   No data in /topic_name
   ```
   **原因**：话题没有数据或订阅失败  
   **解决**：检查话题是否正常发布，使用 `ros2 topic echo` 验证

3. **处理器加载失败**
   ```
   Failed to load processor for dataset_name
   ```
   **原因**：处理器类路径错误或模块不存在  
   **解决**：检查配置文件中的 `processor` 路径

4. **HDF5 保存失败**
   **原因**：磁盘空间不足或权限问题  
   **解决**：检查 `data_dir` 路径的写入权限和磁盘空间

### 扩展开发

#### 添加新的数据处理器

1. **在 `data_processor.py` 中创建新类**：
```python
class MyCustomProcessor(DataProcessor):
    def process(self, msg):
        # 处理逻辑
        return processed_data
    
    def get_message(self):
        # 可选：返回警告消息
        return ''
```

2. **在配置文件中使用**：
```yaml
datasets:
  "/my_dataset":
    topic: "/my_topic"
    processor: "data_collector.data_processor.MyCustomProcessor"
    processor_config:
      output_shape: [10]
      dtype: "float32"
      description: "My custom data"
```

#### 读取 HDF5 数据示例

```python
import h5py
import numpy as np

# 打开 HDF5 文件
with h5py.File('collection_1234567890.hdf5', 'r') as f:
    # 读取图像数据
    images = f['/observations/images/cam_right_wrist'][:]
    print(f"Image shape: {images.shape}")  # (N, 480, 640, 3)
    
    # 读取关节位置
    joint_pos = f['/state/joint_position/left'][:]
    print(f"Joint positions shape: {joint_pos.shape}")  # (N, 7)
    
    # 读取收集频率
    frequency = f['collection_frequency'][()]
    print(f"Collection frequency: {frequency} Hz")
    
    # 读取数据集属性
    for name in f.keys():
        if 'description' in f[name].attrs:
            print(f"{name}: {f[name].attrs['description']}")
```

