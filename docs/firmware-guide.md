# CoFly Autopilot 固件指南

本文说明当前 Teensy 4.1 / A–G 级联固件的配置、接口和执行器行为。项目定位与快速开始见 [仓库首页](../README.md)。

## 当前固件配置

当前 `include/flight_config.h` 启用以下条件编译宏：

| 配置 | 当前值 | 说明 |
| --- | --- | --- |
| 飞机身份 | `FPLANE` | F 节点，属于左侧从机链路 |
| 组合规模 | `SEVENPLANE` | 七机组合构型 |
| 工作方式 | `TEAM` | 多机协同模式 |
| 奇偶构型 | `ODD` | 奇数机体构型 |
| IMU 路径 | `INTIMU` | 使用本机 IMU |
| 接收机 | `USE_SBUS_RX` | 使用 SBUS 接收机 |
| 俯仰控制 | `TESTINDI` | 增稳状态下启用俯仰 INDI |
| 机型配置 | `expensive` | 启用贵飞机专用限幅和启动逻辑 |

飞机身份和控制配置目前是源码宏。给其他节点烧录前，必须修改相应宏并重新编译，不能直接复用当前 F 机固件。

## 开发环境

- 开发板：Teensy 4.1
- MCU：NXP i.MX RT1062，600 MHz
- 框架：Arduino/Teensyduino
- 构建系统：PlatformIO
- 编译环境：`teensy41`
- 上传协议：`teensy-gui`
- 串口监视器：921600 baud

固件依赖由 `platformio.ini` 和 `lib/` 管理；电脑参数软件的 Python 依赖单独位于 `tools/parameter_console/requirements.txt`。

## VS Code Teensy 板上断点调试

连接 Teensy，打开“运行和调试”，选择“TeensyDebug：编译、上传并调试”，按 F5。程序在板上运行，单个 USB 串口专供 GDB 设置断点和查看变量；调试固件关闭普通 USB 输出和参数通信。使用参数软件时上传普通 `teensy41` 固件。详见 [板上调试说明](../tools/teensy_debug/README.md)。
克隆到新电脑后，先执行一次 PlatformIO 构建。本机的 `.vscode` 调试配置会自动生成，不纳入 Git。

## MT6701 夹角传感器

通过 `Wire1` 接入（SDA=17、SCL=16，7 位地址 `0x06`），与 OLED 和空速传感器共用 I²C。驱动位于 `lib/MT6701/`，按提供的示例先读 `0x03` 再读 `0x04`，将 14 位无符号值换算为 0～360°。`sensor_processing.cpp` 负责初始化、每周期采样、零点和安装方向，主函数只调用初始化与读取接口。

`relativeAngle_ready` 是减去本机零点后的夹角，跨 0°/360° 时转换到 ±180°；A/B/D/F 反向，C/E/G 正向。原来的按键清零继续写飞控 EEPROM 地址 100；更换传感器后应重新清零。读取失败时保留最后有效值，屏幕 `Relat` 显示 `ERR`，每 20 ms 重试且禁止保存错误零点。原有日志和机间状态帧继续使用该夹角。

默认每控制周期读取一次，无额外延时或串口输出。`userotatesensor` 仍控制控制器是否使用测量夹角，当前未启用。编译与驱动模拟测试已通过，接线、安装方向和实际循环耗时需要板上确认。

## USB 参数配置软件

电脑端软件位于 `tools/parameter_console/`，提供 57 项参数读取/就地修改（最多支持 512 项）、SD 自动保存、列排序及支持十六进制显示的串口助手。参数类型显示为 `float` / `int`；双击当前值编辑，Enter 保存，Esc 取消。双击 `tools/parameter_console/dist/CoFlyParameterConsole.exe` 可启动；源码启动、协议、SD 恢复见 [参数软件使用说明](../tools/parameter_console/README.md)。飞控开机在 SD 初始化后自动加载参数，修改成功后下次控制周期使用新值。

## PWM 中位与安装微调

`include/flight_config.h` 中统一定义 `PWM_CENTER_US = 1500`。遥控滚转、俯仰、偏航归一化以及通道 7 构型系数也以 1500 为零点；同一个遥控 PWM 数值产生的期望量会随中位修改而变化。

安装 trim 仅保留本机 `pwm_channel1_trim`～`pwm_channel5_trim`，编译默认值全部为 0 μs。各飞机通过 USB 参数表校准，在自己的 SD 卡中保存并于开机恢复；不再按机体宏选择 trim。机械中位为 `1500 + trim`，trim 不随 rev 反向。当前油门 PWM 转换以 `PWM_CENTER_US` 为基准，最终输出还受本机 rev、trim、锁定和限幅影响；遥控油门归一化见 `control_modes.cpp::getDesState()`。

物理输出统一在 `applyAndTransmitActuatorCommands()` 中限幅到 900～2100 μs；俯仰 INDI 还具有独立的可调物理 PWM 限幅。INDI 的绝对 PWM 到舵角标定继续使用原拟合系数，其中立初始化使用 `PWM_CENTER_US + pwm_channel3_trim`。

所有飞机的升降舵到副翼补偿统一使用本地俯仰控制偏移除以 15；INDI 使用实测物理 PWM，因此需扣除本机机械中位并还原控制方向。机械 trim 不参与控制补偿，零控制量时补偿为零。日志中本机舵面 PWM 扣除本机 trim 后以 1500 为基准，B～G 控制指令列以零偏移为基准；子机回传的升降舵 PWM 保留物理值。

迁移对比使用旧版 `scaleCommands()`、当前 `convertControlCommandsToPWM()` 和执行器实现，检查 A 主机与 F 从机分支各 42 组相同归一化指令，包括中立、刹车、控制模式和饱和情况：中立指令一致，测试中的 PWM 指令最大差异为 1 μs，来自中位变化后的整数截断。

## 执行器指令准备

主循环按以下顺序执行：

```cpp
controlMixer();
convertControlCommandsToPWM();
prepareActuatorCommands();
applyAndTransmitActuatorCommands();
```

`control_modes.cpp` 输出归一化的 `*_scaled` 和未反向的升降舵前馈偏移。`convertControlCommandsToPWM()` 位于 `actuator_output.cpp`，更新私有的 `*_control_us`：它们以 1500 为逻辑零点，尚未加机械 trim 或应用反向，不能当作最终 PWM。

`prepareActuatorCommands()` 按 `1500 + 本机安装trim + 本机rev × (控制偏移 + 本地补偿)` 生成本机最终的 `A*_PWM`。A 主机准备本机输出，发往 B～G 的旧名 `*_PWM` 缓存实际是有符号控制偏移，不含中位、rev、trim；从机叠加本地副翼补偿和阻尼后才转换成物理 PWM。手动升降舵和升降舵前馈也由接收机应用本机 rev/trim。

机间命令帧仍为 50 字节，但字段含义已改变，七架飞机须同步更新固件；旧固件不能混用。各机的 `pwm_channel1_trim`、`pwm_channel2_trim`、`pwm_channel3_trim` 保存本机安装偏置，需在对应机体上调整并保存参数。

A 机在 `#if defined APLANE` 中准备本机 `Local_*_control_us` 和子机缓存，B～G 从上级命令获取本机偏移；`#endif` 后所有飞机共同计算副翼补偿与最终物理 PWM。A 机没有升降舵上级前馈，其 `Local_ele_ff_control_us` 每周期设为零。

油门机间字段传递有符号控制偏移；各机在本地应用 `PWM_CENTER_US`、油门 rev/trim 和最终输出保护。参数 `usb_throttle_debug` 保留为油门调试开关，诊断函数输出接收的偏移和上级帧状态。

本地升降舵计算、从机手动覆盖、前馈、副翼补偿和滚转阻尼均在 prepare 阶段完成。INDI 使用实测的物理 PWM 标定输出，不重复应用 rev/trim。`ele_PWM` 表示最终本地升降舵输出，通信回传直接使用该值，不再重复叠加前馈。

## 执行器锁定与限幅

各机上电独立锁定。按键 36 长按 1 秒解锁并停止姿态屏幕刷新，再长按上锁、立即回到锁定位置并恢复刷新；短按继续将夹角清零。按键 31 的 IMU 标定会阻塞主循环，因此标定前也先上锁。锁定状态不经机间链路传播，每架飞机都需要单独操作。

锁定时五路输出统一为 `1500 + 本机该通道 trim`，不应用 rev 或控制指令。物理 PWM 在输出前统一限制到 900～2100 μs；非有限计算结果回退到该通道的锁定位置。舵机绑定后立即输出锁定位置，随后每周期继续执行锁定判断。日志记录限幅和锁定后实际写入的五路值。

**油门锁定值也为 `1500 + pwm_channel4_trim`。若 `pwm_channel4_trim=0`，电调将收到 1500 μs；要让锁定油门为 1000 μs，必须先在该机 SD 参数文件中将 `pwm_channel4_trim` 设为 -500 并确认上电恢复。**当前默认仍为 0，不能直接带电调试运行此固件。

## 串口映射

串口宏统一定义在 `include/serial_ports.h`，业务模块按设备用途调用；修改接线时只需调整此处映射。

| 名称 | 硬件串口 | 用途 |
| --- | --- | --- |
| `USBSerial` | `Serial` | USB 调试、参数读取与修改 |
| `ParentSerial` | `Serial6` | 上级机体链路 |
| `LeftChildSerial` | `Serial3` | 左侧子机链路 |
| `RightChildSerial` | `Serial5` | 右侧子机链路 |
| `ExternalImuSerial` | `Serial1` | 外置 IMU |
| `SbusSerial` | `Serial2` | SBUS 接收机 |
| `DsmSerial` | `Serial3` | DSM 接收机 |
| `StrainSensorSerial` | `Serial7` | 应变传感器 |
| `TelemetrySerial` | `Serial8` | 数传发送及 airdata 接收 |

## 构建

在仓库根目录执行：

```sh
pio run
```

清理构建产物：

```sh
pio run --target clean
```

连接 Teensy 并上传：

```sh
pio run --target upload
```

打开串口监视器：

```sh
pio device monitor
```

固件产物位于 `.pio/build/teensy41/`，其中 `firmware.hex` 用于烧录。`.pio/` 是生成目录，不应提交到 Git。

## 软件执行流程

`setup()` 完成串口、OLED、SD 卡、USB 参数服务、EEPROM、舵机、接收机、IMU 和空速传感器的初始化。当前 `initializeControlAllocation()` 调用已注释。随后 `loop()` 以目标 500 Hz 运行：

```text
传感器采样
  -> Madgwick 姿态估计
  -> 遥控/上级命令转换
  -> 手动或增稳控制
  -> INDI 与构型控制
  -> 当前构型的混控（动态分配调用已注释）
  -> PWM 输出与级联转发
  -> SD 日志、failsafe 和循环限频
```

当前主要控制模式为：

- `MANUAL_MODE`：执行手动命令，构型 PID 清零。
- `STABLIZE_MODE_NO_I`：串级姿态增稳，不启用积分。
- `STABILIZE_MODE`：串级姿态增稳并启用积分；当前配置还会启用俯仰 INDI。

## 主要硬件接口

| 接口 | 当前用途 |
| --- | --- |
| Servo 1–5 / 引脚 2–6 | 左副翼、右副翼、升降、油门、方向 |
| SPI CS 10、37 | BMI088 加速度计、陀螺仪 |
| `Serial1` / 115200 | 外置 IMU 数据 |
| `Serial2` | SBUS 接收机 |
| `Serial3` / 921600 | 左侧级联链路 |
| `Serial5` / 921600 | 右侧级联链路 |
| `Serial6` / 921600 | 上级控制命令 |
| `Serial7` / 115200 | Modbus 应变传感器 |
| `Serial8` / 115200 | 数传/气动数据 |
| `Wire1` | SSD1306 OLED、MS4525 空速传感器、MT6701 夹角传感器（SDA=17、SCL=16） |
| 内置 SD | 飞行日志 `datalogNNN.txt` |

## 通信协议概览

- `55 60`：50 字节级联控制帧，包含模式标志、舵机指令、俯仰目标、升降舵命令、前馈和灯光状态。
- `55 71`：33 字节状态帧，包含相对角、滚转角速度、姿态角和升降舵 PWM。
- Modbus RTU：通过 `Serial7` 请求五通道应变数据，使用 CRC16/Modbus。
- `AA 55`：通过 `Serial8` 接收二进制气动数据包。

级联协议没有版本字段。修改帧长度、字段顺序或缩放系数时，必须同步更新所有节点固件。

## 开发注意事项

- `platformio.ini` 当前使用 `-w` 屏蔽全部编译警告，容易隐藏类型、返回值和变量遮蔽问题。
- `control_allocation.cpp` 的 `getQx()` 存在局部变量遮蔽全局 `Qx` 的风险，修改控制分配前应优先验证。
- Servo 6 和 Servo 7 都配置为引脚 9；旧电机引脚配置还与 BMI088 CS 37 存在复用。
- OLED、SD 和串口任务位于同步主循环，必须测量实际循环频率；动态分配的矩阵求逆与 SVD 当前未调用，重新启用时需评估执行时间。
- MT6701 使用 `Wire1` 的 7 位地址 `0x06`，不再与应变传感器共用 `Serial7`。
- 源文件中存在历史编码乱码；编辑时统一使用 UTF-8，并避免无意义地整体格式化大文件。

## 需求与验证

功能、接口、性能和安全需求见 [需求说明](requirements.md)。提交控制代码前至少应执行一次完整 PlatformIO 构建；涉及真实硬件的变更还应完成拆桨地面测试和相应飞行试验记录。
