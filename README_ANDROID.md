# ROS Car - Android横屏遥控应用

基于Android Compose开发的ROSBridge遥控应用，支持2D栅格地图显示、TF机器人位姿可视化和远程控制。

## ✨ 功能特性

### 核心功能
- ✅ **WebSocket连接** - 通过ROSBridge连接ROS主机
- ✅ **强制横屏** - 专为横屏操作优化的UI布局
- ✅ **2D地图显示** - 订阅并渲染OccupancyGrid栅格地图
- ✅ **TF位姿显示** - 实时显示机器人在地图上的位置和朝向
- ✅ **遥控功能** - 左右按钮分别控制线速度和角速度
- ✅ **参数配置** - 可配置速度、话题、频率等参数
- ✅ **断线保护** - 自动发送0速度防止失控

### 页面流程
1. **连接页面** - 输入IP和端口，连接ROSBridge
2. **主页** - 两个卡片入口：主控界面、设置
3. **主控界面** - 地图背景 + 左下前进后退 + 右下左转右转
4. **设置页面** - 配置话题、速度参数、订阅频率等

## 🏗️ 架构设计

### 模块划分
```
com.example.ros_car/
├── rosbridge/              # ROSBridge通信层
│   ├── RosbridgeClient.kt  # WebSocket客户端
│   └── RosMessages.kt       # ROS消息数据类
├── data/                    # 数据管理层
│   ├── TfManager.kt         # TF变换管理
│   ├── MapManager.kt        # 地图数据处理
│   └── SettingsRepository.kt # 设置持久化
├── viewmodel/               # 视图模型
│   └── RosViewModel.kt      # 统一状态管理
└── ui/screens/              # UI页面
    ├── ConnectionScreen.kt  # 连接页面
    ├── HomeScreen.kt        # 主页
    ├── ControlScreen.kt     # 主控页面
    └── SettingsScreen.kt    # 设置页面
```

### 技术栈
- **UI框架**: Jetpack Compose
- **网络通信**: OkHttp WebSocket
- **数据持久化**: DataStore Preferences
- **状态管理**: StateFlow + ViewModel
- **JSON解析**: Gson
- **地图渲染**: Canvas + Bitmap

## 📋 前置要求

### ROS端准备
1. **启动rosbridge_server**
```bash
# ROS1
roslaunch rosbridge_server rosbridge_websocket.launch port:=9090

# ROS2
ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9090
```

2. **确保以下话题存在**
- `/map` (nav_msgs/OccupancyGrid) - 栅格地图
- `/tf` (tf2_msgs/TFMessage) - 动态TF
- `/tf_static` (tf2_msgs/TFMessage) - 静态TF
- `/cmd_vel` (geometry_msgs/Twist) - 速度控制（发布）

3. **网络要求**
- 手机和电脑在同一局域网
- 电脑防火墙允许9090端口
- 可以用 `ping <电脑IP>` 测试连通性

### Android端要求
- Android SDK 34+
- Kotlin 1.9+
- Gradle 8.0+

## 🚀 快速开始

### 1. 构建项目
```bash
cd /home/lx/AndroidStudioProjects/ros_car
./gradlew build
```

### 2. 安装到设备
```bash
./gradlew installDebug
```

### 3. 使用流程
1. **打开应用** - 自动横屏
2. **输入连接信息**
   - IP地址：电脑的局域网IP（如192.168.1.100）
   - 端口：9090（默认）
3. **点击连接** - 等待连接成功
4. **进入主页** - 选择"主控界面"或"设置"
5. **主控界面**
   - 查看地图和机器人位置
   - 左下：按住"前进"或"后退"控制线速度
   - 右下：按住"左转"或"右转"控制角速度
   - 松开按钮自动停止

## ⚙️ 配置说明

### 默认参数
```kotlin
// 速度参数
最大线速度: 0.3 m/s
最大角速度: 0.8 rad/s

// 话题名称
cmd_vel: /cmd_vel
map: /map
tf: /tf
tf_static: /tf_static

// 订阅频率（throttle_rate）
地图: 1000ms (1Hz)
TF: 50ms (20Hz)
```

### 修改参数
在"设置"页面可以修改：
- 速度参数（线速度、角速度）
- 话题名称
- 订阅频率

所有修改自动保存到本地。

## 🔧 关键实现细节

### 1. ROSBridge通信
```kotlin
// 订阅话题
{
  "op": "subscribe",
  "topic": "/map",
  "type": "nav_msgs/OccupancyGrid",
  "throttle_rate": 1000,
  "queue_length": 1
}

// 发布cmd_vel
{
  "op": "publish",
  "topic": "/cmd_vel",
  "type": "geometry_msgs/Twist",
  "msg": {
    "linear": {"x": 0.3, "y": 0, "z": 0},
    "angular": {"x": 0, "y": 0, "z": 0}
  }
}
```

### 2. 速度控制策略
- **按下按钮** → 启动10Hz定时发布
- **松开按钮** → 发送一次0速度并停止定时器
- **App暂停** → 自动发送0速度
- **退出页面** → 自动发送0速度

### 3. 地图渲染
- OccupancyGrid → Bitmap缓存
- 支持手势缩放和拖动
- 机器人位姿叠加显示
- 值映射：-1=灰色（未知）、0=白色（空闲）、100=黑色（障碍）

### 4. TF管理
- 维护简化的TF树
- 自动查找 map→base_link 或 odom→base_link
- 四元数转欧拉角计算yaw

## 🐛 常见问题

### 1. 连接失败
- 检查IP地址是否正确
- 确认rosbridge_server已启动
- 测试网络连通性：`ping <IP>`
- 检查防火墙设置

### 2. 地图不显示
- 确认`/map`话题存在：`rostopic echo /map`
- 检查地图throttle不要太小（建议>=500ms）
- 查看logcat是否有解析错误

### 3. 机器人位置不更新
- 确认TF链完整：`rosrun tf view_frames`
- 检查frame名称（base_link/base_footprint）
- 查看`/tf`话题频率

### 4. 控制无响应
- 检查`/cmd_vel`话题：`rostopic echo /cmd_vel`
- 确认机器人节点订阅了该话题
- 查看logcat确认发布成功

## 📱 UI设计风格

参考了example前端的设计：
- 深色主题（黑灰蓝配色）
- 渐变背景
- 圆角卡片
- 半透明悬浮控件
- 大字体标题
- 清晰的状态指示

## 🔄 后续扩展方向

### V2功能
- [ ] 激光雷达数据叠加 (`/scan`)
- [ ] 目标点导航 (`/move_base_simple/goal`)
- [ ] 路径显示 (`/move_base/global_plan`)
- [ ] 虚拟摇杆（连续控制）
- [ ] 断线自动重连

### V3功能
- [ ] 多机器人支持
- [ ] 视频流显示
- [ ] 语音控制
- [ ] 手势识别

## 📄 许可证

MIT License

## 👨‍💻 开发说明

### 调试
```bash
# 查看日志
adb logcat | grep -E "RosbridgeClient|TfManager|MapManager"

# 查看网络
adb shell dumpsys connectivity
```

### 测试
```bash
# 单元测试
./gradlew test

# UI测试
./gradlew connectedAndroidTest
```

## 📞 支持

如有问题，请查看：
1. logcat日志
2. ROS端话题状态
3. 网络连接状态

---

**Enjoy your ROS robot control! 🤖**
