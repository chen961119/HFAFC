# Highly Flexible Aircraft Flight Controller

基于 Teensy 4.1 和 PlatformIO 的复合柔性多体飞行器控制固件。本工程由 dRehmFlight 演化而来，面向 A–G 多机串联构型，包含姿态估计、增稳控制、俯仰 INDI、动态控制分配、主从串口通信、舵机输出和飞行数据记录。

> 本项目会直接驱动舵机和电调。首次烧录、切换飞机编号或修改控制律后，应拆除螺旋桨并在限位条件下完成地面测试。

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

项目依赖由 `platformio.ini` 和 `lib/` 管理，不需要 Python `requirements.txt`。

## 构建

在仓库根目录执行：

```powershell
& 'C:\Users\Chen\.platformio\penv\Scripts\platformio.exe' run
```

清理构建产物：

```powershell
& 'C:\Users\Chen\.platformio\penv\Scripts\platformio.exe' run --target clean
```

连接 Teensy 并上传：

```powershell
& 'C:\Users\Chen\.platformio\penv\Scripts\platformio.exe' run --target upload
```

打开串口监视器：

```powershell
& 'C:\Users\Chen\.platformio\penv\Scripts\platformio.exe' device monitor
```

固件产物位于 `.pio/build/teensy41/`，其中 `firmware.hex` 用于烧录。`.pio/` 是生成目录，不应提交到 Git。

## 仓库结构

```text
.
├── src/
│   ├── main.cpp                 # 硬件初始化和主循环调度
│   ├── flight_clock.cpp         # 循环时间状态与更新
│   ├── actuator_output.cpp      # 舵机初始化、PWM 修正与指令输出
│   ├── control_modes.cpp        # 控制状态、控制律、混控和 failsafe
│   ├── human_interface.cpp      # 显示与按键状态、SD 日志和数传
│   ├── debug_print.cpp          # 调试输出及其计时状态
│   ├── interaircraft_comm.cpp   # 机间串口收发、命令组装与级联转发
│   ├── sensor_processing.cpp    # 传感器初始化、采样与校准
│   ├── filters.cpp              # 低通滤波、角加速度滤波与 Madgwick 姿态融合
│   ├── math_utils.cpp           # 循环限速、数值渐变及四元数换算
│   ├── control_allocation.cpp   # 多体动力学矩阵及控制分配
│   └── radioComm.cpp            # SBUS/PWM/PPM/DSM 遥控接收
├── lib/                         # 随仓库保存的硬件驱动和 Arduino 库
├── include/flight_config.h      # 飞机身份与控制条件编译配置
├── include/flight_clock.h       # 循环时间接口
├── include/control_state.h      # 必须跨模块使用的控制量
├── include/control_allocation.h # 控制分配接口
├── include/radioComm.h          # 遥控接收接口
├── include/control_modes.h      # 控制模块接口
├── include/human_interface.h    # 人机交互模块接口
├── include/debug_print.h        # 调试输出接口
├── include/interaircraft_comm.h # 机间通信模块接口
├── include/sensor_processing.h  # 传感器模块接口与共享数据类型
├── include/math_utils.h         # 通用数学工具接口
├── test/                        # 预留的 PlatformIO 测试目录
├── docs/                        # 飞行数据、分析脚本、论文及试验文档
├── platformio.ini               # PlatformIO 构建与依赖配置
├── REQUIREMENTS.md              # 可追踪、可验证的项目需求
└── .gitignore                   # 生成文件与本机文件忽略规则
```

各模块在自己的 `.cpp` 中定义状态。显示、调试和控制器内部状态不在公共头文件中暴露；确需跨模块使用的控制量由 `control_state.h` 声明。

## 软件执行流程

`setup()` 完成串口、OLED、SD 卡、EEPROM、舵机、接收机、IMU、空速传感器和控制分配矩阵的初始化。随后 `loop()` 以目标 500 Hz 运行：

```text
传感器采样
  -> Madgwick 姿态估计
  -> 遥控/上级命令转换
  -> 手动或增稳控制
  -> INDI 与构型控制
  -> 动态控制分配和混控
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
| `Serial7` / 115200 | 转角或 Modbus 应变传感器 |
| `Serial8` / 115200 | 数传/气动数据 |
| `Wire1` | SSD1306 OLED 和 MS4525 空速传感器 |
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
- OLED、SD、串口打印、矩阵求逆和 SVD 都位于同步实时路径，启用调试输出后必须测量实际循环频率。
- `Serial7` 和 `Serial8` 各自承担多种候选职责，组合启用前应确认不存在总线冲突。
- 源文件中存在历史编码乱码；编辑时统一使用 UTF-8，并避免无意义地整体格式化大文件。

## 需求与验证

功能、接口、性能和安全需求见 [`REQUIREMENTS.md`](REQUIREMENTS.md)。提交控制代码前至少应执行一次完整 PlatformIO 构建；涉及真实硬件的变更还应完成拆桨地面测试和相应飞行试验记录。

