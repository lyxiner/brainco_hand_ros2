# BrainCo Revo1 右手 ROS 2 驱动

这是一个面向 ROS 2 Humble 的精简驱动，仅用于通过 RS-485 Modbus RTU 控制一只 BrainCo Revo1 右手。驱动接入 `ros2_control`，提供六个主动关节的位置控制和位置反馈。

不包含 Revo2、右手、CAN/CANFD、EtherCAT、触觉、Gazebo、MoveIt 或机械臂集成功能。

## 环境与硬件

- Ubuntu 22.04
- ROS 2 Humble
- Revo1 右手
- USB 转 RS-485 适配器
- 默认串口参数：`115200` baud、从站 ID `1`

确认当前用户有串口权限：

```bash
sudo usermod -aG dialout "$USER"
```

执行后需要注销并重新登录。也可用 `ls -l /dev/ttyUSB*` 检查设备及权限。

## 编译

仓库应位于工作空间的 `src/brainco_hand_ros2`：

```bash
cd /home/wz/ROS2/Brainco/revo1_ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select brainco_hand_driver --symlink-install
source install/setup.bash
```

驱动所需的 Stark SDK 头文件和 Linux 共享库已经放在包内，不需要另外下载。

## 串口配置

默认配置文件为 `brainco_hand_driver/config/protocol_modbus_revo1_right.yaml`。默认开启 Revo1 自动检测：

```yaml
hardware:
  slave_id: 1
  port: /dev/ttyUSB0
  baudrate: 115200
  auto_detect: true
  auto_detect_quick: true
  auto_detect_port: /dev/ttyUSB0
```

默认只在 `/dev/ttyUSB0` 上自动检测，并在瞬时通信失败时重试三次。如果设备节点不同，请同时修改 `port` 和 `auto_detect_port`；将 `auto_detect_port` 留空会扫描所有串口。如果自动检测不可用，将 `auto_detect` 改为 `false`，驱动会直接使用 `port`、`baudrate` 和 `slave_id`。

## 启动

```bash
source /home/wz/ROS2/Brainco/revo1_ros2_ws/install/setup.bash
ros2 launch brainco_hand_driver revo1_right_system.launch.py
```

启动后会加载：

- `joint_state_broadcaster`
- `right_revo1_hand_position_controller`

Modbus 控制循环固定为 20 Hz，与 Revo1 官方 SDK 示例的 50 ms 通信周期一致。相同目标不会重复下发，以减少 RTU 总线负载和解码报错。

## 控制手指

控制话题为 `/right_revo1_hand_position_controller/commands`，消息类型为 `std_msgs/msg/Float64MultiArray`。

六个值按以下顺序排列，单位是弧度：

| 索引 | 关节 | 范围 |
| --- | --- | --- |
| 0 | 拇指弯曲 | 0 ～ 0.9599 rad（55°） |
| 1 | 拇指侧摆 | 0 ～ 1.5708 rad（90°） |
| 2 | 食指弯曲 | 0 ～ 1.2217 rad（70°） |
| 3 | 中指弯曲 | 0 ～ 1.2217 rad（70°） |
| 4 | 无名指弯曲 | 0 ～ 1.2217 rad（70°） |
| 5 | 小指弯曲 | 0 ～ 1.2217 rad（70°） |

完全张开：

```bash
ros2 topic pub --once \
  /right_revo1_hand_position_controller/commands \
  std_msgs/msg/Float64MultiArray \
  "{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```

约半握拳：

```bash
ros2 topic pub --once \
  /right_revo1_hand_position_controller/commands \
  std_msgs/msg/Float64MultiArray \
  "{data: [0.48, 0.78, 0.61, 0.61, 0.61, 0.61]}"
```

接近完全弯曲：

```bash
ros2 topic pub --once \
  /right_revo1_hand_position_controller/commands \
  std_msgs/msg/Float64MultiArray \
  "{data: [0.95, 1.55, 1.20, 1.20, 1.20, 1.20]}"
```

位置反馈可从 `/joint_states` 读取：

```bash
ros2 topic echo /joint_states
```

## 测试

```bash
cd /home/wz/ROS2/Brainco/revo1_ros2_ws
source /opt/ros/humble/setup.bash
colcon test --packages-select brainco_hand_driver
colcon test-result --verbose
```

## 常见问题

出现 `Giving up to decode frame after 20 retries` 时，通常说明 RTU 通信受到干扰或串口被争用。当前配置已使用验证过的 20 Hz 周期并跳过重复命令；如果仍持续出现，请依次检查：

1. 只有一个进程占用该串口。
2. USB 转 RS-485 接线、终端和供电稳定。
3. 波特率为 `115200`，从站 ID 与硬件一致。
4. 当前用户属于 `dialout` 组。
5. 将 `auto_detect_port` 固定为实际串口，避免扫描其他设备。

偶发一两条解码日志但控制与反馈正常，通常不影响使用；持续高频出现并伴随动作或反馈中断时才需要排查通信链路。

## 目录

```text
brainco_hand_ros2/
├── README.md
├── LICENSE
└── brainco_hand_driver/
    ├── config/       # Revo1 右手硬件、URDF 与控制器配置
    ├── include/      # ros2_control 驱动头文件
    ├── launch/       # 唯一启动文件
    ├── src/          # Revo1 Modbus 驱动实现
    ├── test/         # 配置与 xacro 测试
    └── vendor/       # 必需的 Stark SDK
```

本项目采用 Apache-2.0 许可证。
