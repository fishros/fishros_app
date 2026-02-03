# ROS Car - Android 遥控应用

一个基于 Kotlin + Jetpack Compose 的 Android 横屏 ROS 机器人遥控应用，支持通过 ROSBridge WebSocket 连接 ROS 系统，实现实时控制、地图显示和 TF 可视化。

## ✨ 功能特性

### 核心功能
- 🌐 **WebSocket 连接**: 通过 ROSBridge 连接到 ROS 主机
- 🎮 **机器人控制**: 左右分区的线速度和角速度控制
- 🗺️ **地图显示**: 订阅并渲染 OccupancyGrid 栅格地图
- 🤖 **TF 可视化**: 显示机器人实时位姿和朝向
- ⚙️ **灵活配置**: 完整的参数配置系统
- 📱 **强制横屏**: 优化的横屏操控体验

### UI 特色
- 深色主题设计，参考前端 example 风格
- 渐变背景和卡片式布局
- 流畅的触控交互
- 实时状态反馈

## 🏗️ 架构设计

```
app/
├── rosbridge/              # ROSBridge 通信层
│   ├── RosbridgeClient.kt  # WebSocket 客户端
│   └── RosMessages.kt      # ROS 消息数据类
├── data/                   # 数据管理层
│   ├── TfManager.kt        # TF 数据处理
│   ├── MapManager.kt       # 地图数据处理
│   └── SettingsRepository.kt # 配置持久化
├── viewmodel/              # ViewModel 层
│   └── RosViewModel.kt     # 统一状态管理
└── ui/screens/             # UI 界面层
    ├── ConnectionScreen.kt # 连接页面
    ├── HomeScreen.kt       # 主页（两卡片）
    ├── ControlScreen.kt    # 主控界面
    └── SettingsScreen.kt   # 设置页面
```

## 🚀 快速开始

### 前置要求

#### ROS 端（电脑主机）
```bash
# ROS1
sudo apt-get install ros-<distro>-rosbridge-suite
roslaunch rosbridge_server rosbridge_websocket.launch port:=9090

# ROS2
sudo apt-get install ros-<distro>-rosbridge-suite
ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9090
```

#### Android 端
- Android Studio Arctic Fox 或更高版本
- 最低 Android SDK 34
- Kotlin 1.9+

### 安装步骤

1. **克隆项目**
```bash
cd /path/to/AndroidStudioProjects
git clone <your-repo-url> ros_car
```

2. **打开项目**
   - 用 Android Studio 打开 `ros_car` 目录
   - 等待 Gradle 同步完成

3. **构建运行**
   - 连接 Android 设备或启动模拟器
   - 点击 Run 按钮（或 Shift+F10）

## 📖 使用说明

### 1. 连接 ROSBridge

启动应用后：
1. 输入 ROS 主机的 **IP 地址**（如 `192.168.1.100`）
2. 输入 **端口号**（默认 `9090`）
3. 点击 **连接** 按钮
4. 等待连接成功（绿色指示灯）

> **提示**: 确保手机和电脑在同一局域网内，可以互相 ping 通

### 2. 主控界面

连接成功后进入主页，有两个功能卡片：

#### 🎮 主控界面
- **背景**: 显示 `/map` 的 2D 栅格地图
- **机器人**: 绿色圆圈 + 方向箭头显示当前位姿
- **左下角**: 前进/后退按钮（线速度控制）
  - 按住持续发送速度指令
  - 松开立即停止
- **右下角**: 左转/右转按钮（角速度控制）
  - 按住持续转向
  - 松开立即停止
- **地图操作**: 双指缩放、拖动平移

#### ⚙️ 设置
- **连接状态**: 查看连接状态，断开连接
- **ROS 话题**: 配置 cmd_vel、map、tf 话题名称
- **速度参数**: 设置最大线速度和角速度
- **订阅频率**: 调整地图和 TF 的更新频率

### 3. 控制逻辑

**按键控制**:
- **前进**: `linear.x = +V`, `angular.z = 0`
- **后退**: `linear.x = -V`, `angular.z = 0`
- **左转**: `linear.x = 0`, `angular.z = +W`
- **右转**: `linear.x = 0`, `angular.z = -W`
- **停止**: `linear.x = 0`, `angular.z = 0`

**发布策略**:
- 按下按钮: 以 10Hz 持续发布速度指令
- 松开按钮: 立即发送一次零速度并停止发布
- 应用暂停: 自动发送零速度（安全机制）

## 🔧 配置说明

### 默认配置

| 参数 | 默认值 | 说明 |
|------|--------|------|
| IP地址 | 192.168.1.100 | ROS 主机 IP |
| 端口 | 9090 | rosbridge 端口 |
| cmd_vel话题 | /cmd_vel | 速度控制话题 |
| map话题 | /map | 地图话题 |
| tf话题 | /tf | TF 话题 |
| tf_static话题 | /tf_static | 静态 TF 话题 |
| 最大线速度 | 0.3 m/s | 前进后退速度 |
| 最大角速度 | 0.8 rad/s | 转向速度 |
| 地图更新频率 | 1000 ms | 地图订阅节流 |
| TF更新频率 | 50 ms | TF 订阅节流 |

### 持久化存储

所有配置通过 DataStore 保存，重启应用后自动恢复：
- 上次输入的 IP 和端口
- 话题名称配置
- 速度参数设置
- 订阅频率参数

## 🛠️ 技术栈

- **语言**: Kotlin
- **UI**: Jetpack Compose
- **网络**: OkHttp WebSocket
- **数据**: DataStore Preferences
- **架构**: MVVM
- **依赖管理**: Gradle (Kotlin DSL)

### 主要依赖

```kotlin
// WebSocket
implementation("com.squareup.okhttp3:okhttp:4.12.0")

// Navigation
implementation("androidx.navigation:navigation-compose:2.7.6")

// ViewModel
implementation("androidx.lifecycle:lifecycle-viewmodel-compose:2.7.0")

// DataStore
implementation("androidx.datastore:datastore-preferences:1.0.0")

// JSON
implementation("com.google.code.gson:gson:2.10.1")
```

## 📡 ROSBridge 协议

### 订阅消息

**订阅地图**:
```json
{
  "op": "subscribe",
  "topic": "/map",
  "type": "nav_msgs/OccupancyGrid",
  "throttle_rate": 1000,
  "queue_length": 1
}
```

**订阅 TF**:
```json
{
  "op": "subscribe",
  "topic": "/tf",
  "type": "tf2_msgs/TFMessage",
  "throttle_rate": 50,
  "queue_length": 10
}
```

### 发布控制指令

```json
{
  "op": "publish",
  "topic": "/cmd_vel",
  "type": "geometry_msgs/Twist",
  "msg": {
    "linear": {"x": 0.3, "y": 0.0, "z": 0.0},
    "angular": {"x": 0.0, "y": 0.0, "z": 0.0}
  }
}
```

## 🐛 故障排查

### 连接失败

**问题**: 点击连接后显示"连接失败"

**解决方案**:
1. 确认 rosbridge 已启动: `rostopic list`
2. 检查网络连通性: `ping <手机IP>`
3. 确认防火墙允许 9090 端口
4. 检查 IP 地址是否正确（不要用 localhost）

### 地图不显示

**问题**: 进入主控界面后看不到地图

**解决方案**:
1. 确认 `/map` 话题有数据: `rostopic echo /map`
2. 检查地图话题名称配置是否正确
3. 查看 Logcat 日志中的错误信息
4. 尝试增加地图更新频率（降低 throttle）

### 机器人不动

**问题**: 按下按钮但机器人没反应

**解决方案**:
1. 确认 `/cmd_vel` 话题有数据: `rostopic echo /cmd_vel`
2. 检查速度参数是否为 0
3. 确认机器人控制节点已启动
4. 检查 cmd_vel 话题名称是否匹配

### TF 不显示

**问题**: 地图显示但看不到机器人位置

**解决方案**:
1. 确认 TF 树: `rosrun tf view_frames`
2. 检查 `map -> base_link` 或 `odom -> base_link` 是否连通
3. 查看 `/tf` 话题: `rostopic echo /tf`
4. 检查 TF 话题配置

## 🔒 安全注意事项

1. **局域网使用**: rosbridge 默认无鉴权，仅在受信任的局域网使用
2. **速度限制**: 根据机器人实际情况设置合理的速度上限
3. **紧急停止**: 应用暂停/退出时会自动发送零速度
4. **网络延迟**: 注意 WiFi 信号质量，避免控制延迟

## 📝 开发计划

### V1（当前版本）✅
- [x] WebSocket 连接与断线处理
- [x] 基础 cmd_vel 控制
- [x] 地图订阅与渲染
- [x] TF 订阅与机器人显示
- [x] 配置持久化
- [x] 横屏适配

### V2（未来版本）
- [ ] 手柄式摇杆控制（连续变速）
- [ ] 激光雷达显示 (`/scan`)
- [ ] 目标点点击导航
- [ ] 路径规划可视化
- [ ] 更丰富的 TF frame 选择
- [ ] 自动重连机制
- [ ] 连接历史记录

### V3（高级功能）
- [ ] 多机器人支持
- [ ] 录制回放功能
- [ ] 性能监控面板
- [ ] 自定义按钮映射
- [ ] 蓝牙手柄支持

## 🤝 贡献

欢迎提交 Issue 和 Pull Request！

## 📄 许可证

[MIT License](LICENSE)

## 👨‍💻 作者

基于 example 前端风格开发的 Android 遥控应用

---

**祝你遥控愉快！🚀**
