# JetArm 项目依赖与环境配置

## 项目概述

JetArm 是一个基于 ROS 2 Humble 的机械臂控制项目，主要运行在 Ubuntu 22.04 或 Jetson 设备上。

## 系统要求

### 推荐环境
- **操作系统**: Ubuntu 22.04 LTS (Jetson Orin/Nano 也可)
- **ROS 2 版本**: Humble Hawksbill
- **Python 版本**: 3.10+
- **架构**: arm64/aarch64 (Jetson) 或 x86_64

### macOS 兼容性警告

**⚠️ macOS 不支持完整运行此项目**

原因：
1. ROS 2 Humble 官方不支持 macOS
2. 硬件驱动依赖 Linux 特定接口（/dev/ttyUSB0, /dev/video0 等）
3. Orbbec 深度相机 SDK 仅支持 Linux
4. 串口通信库 `pyserial` 在 macOS 上设备路径不同

**macOS 上可以做的**：
- Python 代码语法检查
- 离线阅读和理解代码
- 部分纯 Python 逻辑测试（需 mock 硬件接口）

---

## Ubuntu 22.04 完整依赖安装

### 1. 系统 ROS 2 Humble 安装

```bash
# 添加 ROS 2 apt 仓库
sudo apt update && sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 安装 ROS 2 Humble
sudo apt update
sudo apt install -y ros-humble-desktop
sudo apt install -y ros-humble-ros-base
```

### 2. Python 依赖

```bash
# 基础 Python 包
sudo apt install -y python3-pip python3-vcstool

# ROS 2 Python 构建
pip3 install colcon-common-extensions
```

### 3. 项目专用依赖

```bash
# 串口通信
sudo apt install -y python3-serial

# OpenCV 和视觉处理
sudo apt install -y python3-opencv python3-cvbridge

# NumPy 和科学计算
sudo apt install -y python3-numpy python3-scipy

# YAML 配置文件
sudo apt install -y python3-yaml

# 媒体管道（用于相机）
sudo apt install -y python3-libcamera

# 测试工具
pip3 install pytest pytest-cov flake8 pep257
```

### 4. 硬件相关依赖

```bash
# Orbbec 深度相机 SDK
# 需要从 Orbbec 官网下载适用于 Ubuntu 的 SDK

# 舵机控制相关（已包含在项目中，通过串口通信）

# 其他外设
sudo apt install -y joystick  # 手柄支持
```

### 5. 编译项目

```bash
# 创建工作空间（如果还没有）
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# 复制项目到 src 目录
# cp -r /path/to/jetarm_src .

# 安装依赖
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# 编译
colcon build --symlink-install

# Source 环境
source install/setup.bash
```

---

## macOS 上的开发环境（仅代码分析）

### 可安装的依赖

```bash
# 使用 Homebrew 安装 Python 3.10+
brew install python@3.10

# 安装基础 Python 包（不含 ROS 2）
pip3 install numpy opencv-python pyyaml pyserial

# 代码质量工具
pip3 install flake8 pep257 pytest
```

### 语法检查

```bash
# 检查 Python 代码风格
flake8 app/app/*.py
flake8 driver/**/*.py

# 运行测试（需要先 mock ROS 2 接口）
# pytest app/test/
```

---

## 依赖包清单

### Python 包 (pip)

| 包名 | 用途 | macOS 支持 |
|------|------|-----------|
| numpy | 数值计算 | ✅ |
| opencv-python | 图像处理 | ✅ |
| opencv-contrib-python | 扩展视觉功能 | ✅ |
| pyyaml | YAML 配置 | ✅ |
| pyserial | 串口通信 | ⚠️ 设备路径不同 |
| scipy | 科学计算 | ✅ |
| pytest | 测试框架 | ✅ |
| flake8 | 代码风格检查 | ✅ |

### ROS 2 包 (apt)

| 包名 | 用途 | Ubuntu 命令 |
|------|------|-----------|
| ros-humble-desktop | ROS 2 桌面环境 | `sudo apt install ros-humble-desktop` |
| ros-humble-cv-bridge | 图像转换 | `sudo apt install ros-humble-cv-bridge` |
| ros-humble-depthimage-to-laserscan | 深度图像处理 | `sudo apt install ros-humble-depthimage-to-laserscan` |
| ros-humble-rosbridge-server | Web 接口 | `sudo apt install ros-humble-rosbridge-server` |
| ros-humble-web-video-server | 视频流服务器 | `sudo apt install ros-humble-web-video-server` |
| ros-humble-joy | 手柄支持 | `sudo apt install ros-humble-joy` |
| ros-humble-xacro | 宏处理 | `sudo apt install ros-humble-xacro` |

### 硬件驱动

| 硬件 | 驱动/SDK | macOS 支持 |
|------|----------|-----------|
| Orbbec 深度相机 | OrbbecSDK | ❌ |
| 舵机控制板 | 串口通信 (pyserial) | ⚠️ 需适配 |
| 舵机控制板 | `/dev/rrc` 或 `/dev/ttyUSB0` | ❌ macOS 是 `/dev/tty.usb*` |
| LED/蜂鸣器 | 通过控制板 | ❌ |
| IMU | 通过控制板 | ❌ |
| 手柄 | ros-humble-joy | ⚠️ 部分支持 |

---

## 常见问题

### Q1: macOS 上如何测试代码？

A: 可以进行语法检查和基础逻辑测试，但需要 mock 硬件接口：

```python
# 示例 mock
class MockSerial:
    def write(self, data):
        pass
    def read(self):
        return b''

# 在测试中使用 mock 替代真实的硬件接口
```

### Q2: 如何在 macOS 上获得最佳开发体验？

A: 建议使用虚拟机或 Docker 运行 Ubuntu：

```bash
# 使用 Docker 运行 ROS 2
docker run -it osrf/ros:humble-desktop
```

或者使用云服务器（如 AWS、Jetson 设备等）进行远程开发。

### Q3: 项目必须在 Ubuntu 上编译吗？

A: 是的，完整的项目编译和运行需要 Ubuntu 22.04 环境。macOS 只能做代码阅读和语法检查。

---

## 环境变量

项目使用以下环境变量：

| 变量 | 值 | 说明 |
|------|-----|------|
| `CHASSIS_TYPE` | `Slide_Rails` 或默认 | 底盘类型，影响运动学参数 |
| `need_compile` | `True` 或 `False` | 是否使用编译后的包路径 |
| `ROS_DOMAIN_ID` | 数字 | ROS 2 DDS 域 ID，用于隔离网络 |
| `ROS_LOCALHOST_ONLY` | `1` | 限制本地通信 |

---

## 推荐开发工具

- **VS Code** + ROS 2 插件
- **PyCharm** (专业版)
- **CLion** (C++ 开发)

编辑器配置建议：
- Python 格式化: `black`
- Python 检查: `flake8`, `mypy`
- ROS 2 工作区支持: 安装 `ms-iot.vscode-ros` 插件
