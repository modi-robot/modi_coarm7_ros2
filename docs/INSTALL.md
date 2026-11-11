# 安装与编译

[返回首页](../README.md) · [配置说明](CONFIGURATION.md)

## 环境要求

| 项目 | 要求 |
| --- | --- |
| 操作系统 | Ubuntu 22.04 |
| ROS | ROS 2 Humble |
| 处理器 | x86_64 或 ARM64 |
| 编译工具 | CMake 3.19 及以上、支持 C++17 的编译器 |

## 1. 获取源码

获取源码并放到工作区 `src/coarm7`：

```bash
mkdir -p ~/coarm7_ws/src
cd ~/coarm7_ws/src
git clone https://github.com/modi-robot/modi_coarm7_ros2.git coarm7
```

目录结构：

```text
~/coarm7_ws/
└── src/
    └── coarm7/
        ├── coarm7_bringup/
        ├── coarm7_hardware/
        ├── dependencies/
        └── ...
```

## 2. 安装依赖

先按 [ROS 2 Humble 安装说明](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) 安装 ROS，再执行：

```bash
sudo apt-get update
sudo apt-get install -y \
  build-essential cmake git python3-colcon-common-extensions python3-yaml \
  python3-pytest libyaml-cpp-dev libspdlog-dev libgtest-dev \
  ros-humble-ament-cmake-gtest ros-humble-ament-cmake-pytest \
  ros-humble-moveit ros-humble-pilz-industrial-motion-planner \
  ros-humble-ros2-control ros-humble-ros2-controllers \
  ros-humble-xacro ros-humble-rviz2 iproute2 util-linux openssh-client
```

首次编译需要联网，CMake 会自动获取对应平台的官方 [CanEvo SDK v0.0.4](https://github.com/modi-robot/modi_canevo_sdk/releases/tag/v0.0.4) 并校验；已有同版本依赖时直接复用。使用 [远程 RViz](REMOTE_RVIZ.md) 时，控制器还需安装 `openssh-server`。

直接连接机械臂的控制器还需安装适配器驱动，确认系统出现对应的 CAN 接口。使用 KH-UCANFD 时参见 [SDK 使用说明书](https://github.com/modi-robot/modi_canevo_sdk/blob/main/docs/CanEvo%20SDK%20使用说明书.md) 的驱动安装部分，并完成 [实时权限与 CPU 配置](CONFIGURATION.md#realtime)。

## 3. 编译

```bash
cd ~/coarm7_ws
source /opt/ros/humble/setup.bash
colcon build --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=Release
```

完成后加载工作区：

```bash
source install/setup.bash
```

新终端执行以下两行即可使用：

```bash
source /opt/ros/humble/setup.bash
source ~/coarm7_ws/install/setup.bash
```

## 4. 启动验证

```bash
ros2 run coarm7_bringup start_coarm7.py --mode fake
```

RViz 打开后，按 [首页操作说明](../README.md#rviz) 测试双臂规划和夹爪开合。连接真实机械臂前，按 [配置说明](CONFIGURATION.md) 准备硬件参数。

<details>
<summary>可选：仅体验模型与规划（Mock）</summary>

```bash
cd ~/coarm7_ws
source /opt/ros/humble/setup.bash
colcon build --packages-up-to coarm7_bringup --parallel-workers 2 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
source install/setup.bash
ros2 launch coarm7_bringup mock.launch.py
```

此构建只包含模型、规划和演示所需的包。

</details>

<details>
<summary>可选：离线依赖与交叉编译</summary>

使用已解压的正式依赖包并禁止下载：

```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
  -DCOARM7_DOWNLOAD_SDK=OFF \
  -Dcanevo_sdk_x86_DIR=/path/to/sdk/lib/cmake/canevo_sdk_x86
```

自动下载缓存在 `build/coarm7_hardware/_deps`，可用 `FETCHCONTENT_BASE_DIR` 指定其他缓存目录。自动获取的必要运行库随工作区安装到 `install`；外部提供的依赖由其安装环境维护。

本机编译时，CMake 自动识别当前机器：x86_64 电脑使用 x86_64 包，ARM64 控制器使用 ARM64 包。交叉编译时，由工具链通过 `CMAKE_SYSTEM_PROCESSOR` 指定程序运行平台，并据此选择依赖。选择依赖不会自动创建交叉工具链或 sysroot。

使用 rosdep 安装其他依赖时，增加 `--skip-keys canevo_sdk_x86`。架构对应的下载地址和哈希保存在 `dependencies/sdk.lock.json`。

</details>
