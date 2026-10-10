# CoFly Autopilot 架构说明

## 项目定位与实现边界

CoFly Autopilot（Collaborative Distributed Autopilot，分布式协同飞控系统）面向多体飞行器、大展弦比柔性飞行器及编队。当前固件实现 A–G 多体级联节点的控制与通信，运行平台为 Teensy 4.1；通用编队控制仍属于扩展方向。

七机物理顺序为 `F-D-B-A-C-E-G`。A 主节点接收遥控命令，计算组合控制并向左右链路分发；从节点接收上级命令，应用本机控制、执行器方向与安装偏置，并逐级回传状态。默认身份和控制宏见 [flight_config.h](../include/flight_config.h)，串口映射见 [serial_ports.h](../include/serial_ports.h)。

七机分支采用构型相关的固定系数混控。`control_allocation.cpp` 保留五体动力学矩阵和普通 SVD 伪逆；`main.cpp` 中初始化与周期计算调用目前均已注释。该模块不等于已接入七机输出的动态分配器，也没有实现参考论文的构型 INDI / IWPI。当前 `TESTINDI` 用于本机俯仰升降舵控制。

## 模块职责

| 模块 | 职责 |
| --- | --- |
| `src/main.cpp` | 初始化和周期任务编排 |
| `include/flight_config.h` | 节点身份、构型、IMU、接收机与控制宏 |
| `src/flight_clock.cpp` | 时间更新和循环限频 |
| `src/sensor_processing.cpp`、`src/filters.cpp` | 传感器采样、校准、滤波与姿态估计 |
| `src/radio_comm.cpp` | 遥控接收接口 |
| `src/control_modes.cpp` | 期望状态、飞行模式、姿态/构型控制和混控 |
| `src/control_allocation.cpp` | 保留的五体动力学与伪逆分配模型 |
| `src/actuator_output.cpp`、`src/flight_lock.cpp` | PWM 转换、本机输出准备、锁定和最终限幅 |
| `src/interaircraft_comm.cpp` | 主从级联命令、状态汇总和转发 |
| `src/parameter_service.cpp` | USB 参数读写与 SD 保存/恢复 |
| `src/logger.cpp`、`src/telemetry.cpp` | SD 日志与遥测 |
| `src/human_interface.cpp`、`src/debug_print.cpp` | 按键、OLED 与诊断输出 |
| `src/firmware_debug.cpp` | 板上断点调试集成 |
| `include/control_state.h` | 跨模块控制状态声明 |
| `include/parameter_registry.h` | 各模块参数注册接口 |

各模块通过 `include/` 下的对应头文件提供接口；模块内部状态保留在各自实现中。

## 主循环与输出

`setup()` 建立通信链路，初始化显示、SD 与参数服务、传感器和执行器，随后初始化遥控接收、姿态、控制模式及滤波器。

```text
时间更新与传感器采样
  -> 姿态估计、空速与夹角
  -> 期望状态、按键与相邻节点状态
  -> 所选控制模式与混控
  -> PWM 转换、执行器命令准备
  -> 锁定/限幅、本机输出与级联发送
  -> SD 日志
  -> 遥控更新、failsafe、USB 参数服务
  -> 500 Hz 节拍等待
```

遥控命令在周期末更新，供下一周期使用。500 Hz 是目标频率；OLED、SD、参数保存等同步操作的实际耗时仍需硬件测量，不能仅由 `loopRate(500)` 推断性能。

输出路径依次为 `controlMixer()`、`convertControlCommandsToPWM()`、`prepareActuatorCommands()`、`applyAndTransmitActuatorCommands()`。上级向从机发送有符号控制偏移，由本机应用中位、rev、trim 和补偿；锁定时直接输出 `1500 + trim`，解锁后才应用各通道独立的软件限幅（默认各路 1000～2000 μs）；非有限控制结果回退到锁定位置。Servo 驱动范围独立保持 1000～2000 μs。具体锁定油门行为见 [固件指南](firmware-guide.md#执行器锁定与限幅)。

## 兼容性与维护入口

项目名称、参数软件、USB 响应前缀 `@COFLY`、SD 文件头 `COFLY_PARAMS_V1/V2/V3` 及调试宏 `COFLY_TEENSY_DEBUG` 已统一使用 CoFly。固件与参数软件必须同步更新；改名前的 USB 响应与 SD 文件头不再识别。更新固件前应记录本机参数，更新后通过新版参数软件重新设置并保存，避免旧文件被拒绝后使用默认值。

修改身份与机数使用 `flight_config.h`；接线修改使用 `serial_ports.h` 和具体传感器/执行器模块；控制律与混控修改使用 `control_modes.cpp`；级联协议修改使用 `interaircraft_comm.cpp`。50 字节级联命令中的 PWM 旧字段名实际承载控制偏移，所有节点须使用一致的字段语义。

需求与验收目标见 [软件需求说明](requirements.md)，参考论文及旧版代码对照见 [文档导航](README.md)。
