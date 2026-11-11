# 配置说明

[返回首页](../README.md) · [安装编译](INSTALL.md)

## 配置文件

| 文件 | 用途 |
| --- | --- |
| `coarm7_description/urdf` | 模型、轴向和位置限位 |
| `coarm7_moveit_config/config` | 规划组、命名目标、求解器、规划速度/加速度 |
| `coarm7_bringup/config/ros2_controllers.yaml` | Mock 控制器，100 Hz |
| `coarm7_bringup/config/controllers_driver.yaml` | fake/实机控制器，200 Hz |
| `coarm7_hardware/config/hardware_pending.yaml` | 未核对的实机模板 |
| `coarm7_hardware/config/offline.yaml` | fake 测试配置，不能代替实机配置 |

实机配置保存在仓库外。命令行参数优先于 YAML 的 `startup` / `diagnostics`，然后才使用程序默认值。

## 准备实机配置

首次使用时复制模板到用户配置目录；已有核对过的配置可直接使用：

```bash
mkdir -p ~/.config/coarm7
cp "$(ros2 pkg prefix coarm7_hardware)/share/coarm7_hardware/config/hardware_pending.yaml" \
  ~/.config/coarm7/hardware_reviewed.yaml
```

按下表核对总线、节点、换算和权限后，设 `commissioning_confirmed: true`。模板默认允许夹爪开合（`gripper_read_only: false`）。

解析配置并检查启动参数：

```bash
ros2 run coarm7_bringup start_coarm7.py --mode hardware \
  --hardware-config ~/.config/coarm7/hardware_reviewed.yaml --check-only
```

`--check-only` 不启动硬件。现场只读取反馈可使用 `--no-activate`，使能保持但暂不启动运动控制器可使用 `--no-start-controllers`。正常启动见 [真机运行](../README.md#hardware)。

## 默认功能与按需关闭

常规启动默认包含双臂、双夹爪控制和本地 RViz。夹爪“启用”表示可以接受开合目标，启动时保持当前状态，不会自动张开。

| 使用方式 | 设置 |
| --- | --- |
| 双臂和双夹爪完整控制 | 默认：`gripper_read_only: false`、`startup.activate: true`、`startup.start_controllers: true` |
| 只控制双臂，夹爪只读反馈 | `gripper_read_only: true` |
| 先检查反馈，暂不使能 | 启动时增加 `--no-activate` |
| 使能保持，暂不启动运动控制器 | 启动时增加 `--no-start-controllers` |
| 不在本机显示 RViz | `--no-rviz` |
| 使用远程 RViz | `--remote-rviz --no-rviz` |

夹爪只读模式仍要求工具板和夹爪反馈正常，适合已安装夹爪但暂不操作开合的场景。已有实机 YAML 保留其显式设置；更新软件不会覆盖用户配置。

## 实机参数

| 参数 | 含义 |
| --- | --- |
| `commissioning_confirmed` | 核对配置后才设为 true |
| `buses`、轴 `bus/id` | CAN 接口与关节节点映射 |
| 轴 `sign/offset` | 方向与零点换算 |
| `gripper_read_only` | true 只读夹爪反馈；false 允许开合 |
| 夹爪 `tool_id/device_id`、`angle/raw/command_raw` | 工具板 ID 与开度换算 |
| 夹爪 `speed_raw` | 设备开合速度 |
| `timing.period_us` | 当前支持 5000 µs，与 200 Hz 控制周期对应 |
| `sdk_rx/sdk_tx.cpu`、`startup.control_cpu/process_cpus` | 可选绑核，默认不强制固定 CPU |
| `sdk_rx/sdk_tx.priority` | SDK 实时优先级，默认 90 |

双臂位置边界以 URDF 为准，MoveIt 与硬件核对同一组限位。SDK 读取的驱动器位置限位只作诊断，不取交集，也不改写设备设置。速度、反馈时效、故障和停止检查仍生效。

夹爪接收最终开度并根据反馈完成。取消/停用使用独立 stop 指令；发送路径确认不代表设备物理停止的独立测量。

<a id="realtime"></a>
## 实机运行环境

**实机控制应优先保证 5 ms 周期的稳定性。** 平均 CPU 占用低、程序能启动或已绑核，都不能单独证明通信周期稳定。建议使用适配目标硬件的 PREEMPT_RT 内核，并结合 CPU 亲和性、系统负载和 USB/CAN 中断配置进行验证。

CPU 编号、大小核布局和可用核心因机器而异，因此模板默认不绑核；投入实机使用前，应明确本机的线程分配，并在保持及规划执行负载下检查周期抖动和通信故障。

### 准备控制器的运行权限

这一步在**直接连接机械臂、运行 hardware 模式的机器上**完成，使用平时启动 CoArm7 的普通用户。

| 使用场景 | 是否需要这一步 |
| --- | --- |
| 电脑仅用于编译、显示远程 RViz，或运行 Mock/fake | 可跳过 |
| 控制器运行实机程序，电脑远程显示 RViz | 在控制器上检查 |
| 同一台电脑直接连接并控制机械臂 | 在这台电脑上检查 |

**1. 检查权限**

在启动实机程序的终端执行：

```bash
if chrt -f 50 true && chrt -f 90 true; then
  echo "实时调度权限检查通过"
else
  echo "实时调度权限不足，请按下方步骤配置"
fi
ulimit -l
```

显示“检查通过”表示可以使用默认配置需要的实时优先级。最后一行是可锁定的内存额度，单位 KiB；`unlimited` 表示无限额，数字表示有限额度。有限额度不一定不足；若实机启动报告内存锁定失败，再检查其是否满足程序需要。

这段检查不会连接机械臂。`sudo chrt ...` 成功只证明管理员能使用实时调度，不能代替普通用户的检查。

**2. 权限不足时配置一次**

仍以实际启动 CoArm7 的普通用户登录，执行下面命令。它自动使用当前用户名，写入该用户的运行权限，可能要求输入 sudo 密码：

```bash
coarm7_user=$(id -un)
printf '%s - rtprio 90\n%s - memlock unlimited\n' \
  "$coarm7_user" "$coarm7_user" | sudo tee /etc/security/limits.d/90-coarm7.conf
```

**3. 重新登录后再检查**

SSH 用户断开并重新连接；桌面用户注销并重新登录，仅关掉终端再打开可能不会生效。重新执行第 1 步，通过后用普通用户启动 CoArm7。

如果仍然失败，请管理员检查登录会话是否应用了该配置。通过 systemd 服务运行时，还需检查服务的 `LimitRTPRIO`、`LimitMEMLOCK`。

<details>
<summary>这些权限有什么作用？</summary>

控制程序每 5 ms 与机械臂通信。实时调度让控制线程优先于普通任务获得 CPU，减少等待；内存锁定用于避免相关内存被换出。50、90 是调度优先级，不是 CPU 占用率或核心编号。权限检查通过不代表已经保证所有周期准时。

CPU 绑定是另一项独立配置，用于限制线程在哪些核心上运行。默认不固定核心，需要时按本机 CPU 编号设置。

</details>

### CPU 分配与验证

在控制器上查看核心和当前会话可用范围：

```bash
lscpu -e=CPU,CORE,SOCKET,ONLINE
grep Cpus_allowed_list /proc/self/status
```

在仓库外的实机 YAML 中按本机情况设置：

| 参数 | 作用 |
| --- | --- |
| `sdk_rx.cpu`、`sdk_tx.cpu` | SDK 接收／发送线程的 CPU；`-1` 不指定固定核心 |
| `startup.control_cpu` | ROS 控制更新线程的 CPU；`-1` 不指定固定核心 |
| `startup.process_cpus` | 启动进程及未另行绑核线程的默认 CPU 范围；空字符串继承当前环境 |
| `sdk_rx.priority`、`sdk_tx.priority` | SDK 实时优先级，默认 90；不是 CPU 编号 |

优先为控制和通信线程安排负载较少的合适核心，避免与规划、渲染等任务争用。绑核仅限制线程在哪运行，不会自动隔离该核心或调整 IRQ。启动入口会检查显式配置的 CPU 是否在线且可绑定，但不会替用户选择最佳核心。

调优方法见 [SDK 实时系统配置指南](https://github.com/modi-robot/modi_canevo_sdk/blob/main/docs/CanEvo实时系统配置指南.md)。其中内核安装部分面向 x86 工控机，ARM64 应使用板卡适配的内核；核心编号与 IRQ 设置均需按实际硬件核对。

验证时分别观察主机调度和同一路相邻 SYNC 的实际发送间隔。USB OUT 耗时是其中一段，不能用单个耗时或平均 CPU 负载替代周期判断。驱动器允许的偏差应以对应固件协议为准，不将 5 ms 周期本身当作故障阈值。

### CAN 接口与设备匹配

CAN 是控制器与关节通信的接口。模板使用以下设置：

| 设置 | 含义 |
| --- | --- |
| CAN FD | 使用 CAN FD 帧格式 |
| 仲裁速率 1 Mbit/s | 报文仲裁阶段速率 |
| 数据速率 5 Mbit/s | 数据阶段速率 |
| `restart-ms: 1000` | 接口 bus-off 后自动恢复的等待时间，毫秒；不是重新使能机械臂 |
| `txqueuelen: 64` | 主机 CAN 发送队列长度；不是控制周期 |

这些设置需要与设备及适配器匹配。启动默认检查接口；接口处于 DOWN 时，可明确使用 `--initialize-can` 初始化。接口已 UP 但设置不符时，程序报错而不直接重置，以免打断正在运行的通信。同一 CAN 总线只能由一套控制程序接管。

## 可选显示与诊断

`startup.rviz/remote_rviz` 控制本地或远程显示。操作步骤见 [远程 RViz](REMOTE_RVIZ.md)。RViz 的图形加速由运行环境准备。

`diagnostics.can_capture/usb_capture/gripper_trace` 默认关闭，可通过对应的 `--can-capture`、`--usb-capture`、`--gripper-trace` 开启。CAN 采集需 can-utils；USB 采集需支持的 USB-CAN 适配器、usbmon 和读取权限，可能请求 sudo。`--diagnostics-required` 使所选采集准备失败时阻止启动。

采集目录由 `diagnostics.directory` 或 `--diagnostics-dir` 指定，默认在实机配置旁的 `diagnostics/`。长时间采集需留意磁盘容量。正常控制不依赖这些工具，诊断开关不关闭反馈与停止检查。
