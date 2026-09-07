# Piper ROS 2 GUI

Piper X 机械臂的 ROS 2 桌面控制界面。通过 `agx_arm_ctrl` 接收状态并发送控制指令，
使用 PyQt5 构建界面，使用 VTK 显示实时 3D 姿态。

[![AgileX 官方 GitHub](https://img.shields.io/badge/AgileX-Official%20GitHub-24292f?logo=github)](https://github.com/agilexrobotics)

![Piper ROS 2 GUI 界面](docs/images/piper-ros2-gui.png)

> [!WARNING]
> 本项目会控制真实机械臂。运行前请清空工作区域并确认实体急停可用。
> 软件停止不能替代实体急停。

## 功能

- 六关节状态、目标控制、微调和点动
- 整机使能、失能、回零和停止保持
- 夹爪开度与力度控制
- TCP 位姿、控制器状态和操作日志
- 单关节使能、失能和零点标定
- 基于官方 STL 模型的实时 3D 预览
- 角度限制、反馈超时和操作确认

## 环境

- Ubuntu 24.04
- ROS 2 Jazzy
- Piper X 与官方 USB-CAN 模块
- AGX Gripper（可选）
- Python 3、PyQt5、VTK、NumPy
- [`pyAgxArm`](https://github.com/agilexrobotics/pyAgxArm)
- [`agx_arm_ros`](https://github.com/agilexrobotics/agx_arm_ros/tree/ros2)

完整功能需要配套的 `agx_arm_ctrl` 服务扩展。使用原始驱动时，回零结果、
单关节使能和零点标定等功能可能不可用。

## 安装

### 1. 安装依赖

先安装 [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)，再执行：

```bash
sudo apt update
sudo apt install -y \
  git python3-pip python3-rosdep python3-colcon-common-extensions \
  python3-pyqt5 python3-vtk9 python3-numpy python3-pytest \
  can-utils ethtool iproute2
```

### 2. 安装 SDK 和驱动

```bash
mkdir -p ~/robot_src
cd ~/robot_src
git clone https://github.com/agilexrobotics/pyAgxArm.git
python3 -m pip install ./pyAgxArm --break-system-packages

mkdir -p ~/agx_arm_ws/src
cd ~/agx_arm_ws/src
git clone -b ros2 --recurse-submodules \
  https://github.com/agilexrobotics/agx_arm_ros.git

cd ~/agx_arm_ws
source /opt/ros/jazzy/setup.bash
sudo rosdep init  # 仅首次使用 rosdep 时执行
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## 配置 CAN

USB-CAN 需要注册为 `can0`，波特率必须为 `1 Mbps`：

```bash
cd ~/agx_arm_ws/src/agx_arm_ros/scripts
bash find_all_can_port.sh
bash can_activate.sh can0 1000000
ip -details link show can0
```

输出应包含 `UP`、`state ERROR-ACTIVE` 和 `bitrate 1000000`。
`ERROR-ACTIVE` 是正常活动状态。重启或重新插拔 CAN 后，可能需要再次激活。
多 CAN 模块配置请参考[官方 CAN 指南](https://github.com/agilexrobotics/agx_arm_ros/blob/ros2/docs/CAN_USER.md)。

可使用以下命令确认机械臂正在发送数据：

```bash
candump can0
```

收到数据后按 `Ctrl+C` 退出。不要同时运行多个控制 `can0` 的机械臂驱动。

## 启动

先启动机械臂驱动：

```bash
source /opt/ros/jazzy/setup.bash
source ~/agx_arm_ws/install/setup.bash

ros2 launch agx_arm_ctrl start_single_agx_arm.launch.py \
  can_port:=can0 \
  arm_type:=piper_x \
  effector_type:=agx_gripper \
  auto_enable:=false \
  speed_percent:=5 \
  control_enabled:=false
```

新建终端，在本项目目录启动 GUI：

```bash
export AGX_ARM_WS=~/agx_arm_ws
./run_gui.sh
```

界面顶部应显示 `CAN: UP · 1 Mbps` 和 `ROS: 反馈在线`。

## 测试

```bash
source /opt/ros/jazzy/setup.bash
source ~/agx_arm_ws/install/setup.bash
python3 -m pytest -q
```

无真机界面检查：

```bash
QT_QPA_PLATFORM=offscreen ./run_gui.sh --smoke-test
```
