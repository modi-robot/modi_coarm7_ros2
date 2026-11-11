# 远程 RViz（可选）

[返回首页](../README.md) · [安装编译](INSTALL.md) · [配置说明](CONFIGURATION.md)

控制器运行硬件控制和 MoveIt，另一台电脑运行 RViz，通过 SSH 转发显示与交互数据。适合将图形显示负载移出控制器；规划计算仍在控制器上。

## 准备

- 两端均按安装说明编译并加载 CoArm7 工作区，使用一致的模型和配置版本。
- 控制器安装并运行 OpenSSH 服务；显示端能够执行 `ssh user@controller` 登录。
- 显示端具备可用的图形加速。虚拟机还需检查其 3D 加速和客体图形驱动。

以下 `user@controller` 替换为实际 SSH 用户名与地址。

## 1. 控制器启动

在控制器上执行：

```bash
source /opt/ros/humble/setup.bash
source ~/coarm7_ws/install/setup.bash
ros2 run coarm7_bringup start_coarm7.py --mode hardware \
  --hardware-config ~/.config/coarm7/hardware_reviewed.yaml \
  --remote-rviz --no-rviz --domain-id 87
```

实机配置和实时环境按 [配置说明](CONFIGURATION.md#realtime) 准备。等待终端显示 `CoArm7 startup: Ready:` 后，再启动显示端。

## 2. 显示端启动

在显示端执行：

```bash
source /opt/ros/humble/setup.bash
source ~/coarm7_ws/install/setup.bash
ros2 launch coarm7_bringup remote_rviz.launch.py \
  controller:=user@controller domain_id:=87
```

两端使用相同的 domain ID。RViz 中按 [首页操作说明](../README.md#rviz) 规划与执行。

## 连接与退出

- 远程连接使用回环 TCP 端口 42161–42164，同一侧同时运行的实例应避免端口冲突。
- 如果提示 DDS listeners 缺失，先确认控制器启动成功，并启用了 `--remote-rviz`。
- 结束控制时，在控制器的启动终端按 **Ctrl+C**；关闭显示端 RViz 只结束显示，不会关闭控制器上的控制程序。
