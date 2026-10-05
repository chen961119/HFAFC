# Highly Flexible Aircraft Flight Controller：仓库知识库

> 基于 2026-07-22 的代码扫描。本文描述当前源码实际行为；修改 `src/main.cpp` 顶部的条件编译宏后，硬件角色和控制路径会随之改变。

## 1. 项目定位

本仓库是面向 Teensy 4.1 的 Arduino/PlatformIO 飞控固件，源自 dRehmFlight，并扩展为多机串联复合翼控制系统。代码支持 A–G 七个飞行单元、主从级联通信、姿态与构型控制、INDI 俯仰控制、控制分配、SD 日志、OLED、空速/气动数据和应变采集。

当前源码选择的是 **F 飞机（左侧从机）**：

| 配置维度 | 当前宏 | 含义 |
| --- | --- | --- |
| 飞机身份 | `FPLANE` | 七机序列 `F-D-B-A-C-E-G` 中最外侧左机 |
| 规模 | `SEVENPLANE` | 七机复合构型 |
| 运行形态 | `TEAM` | 多机协同，而非单机 |
| IMU | `INTIMU` | 使用板载/本机 IMU 路径 |
| 接收机 | `USE_SBUS_RX` | SBUS，实例绑定 `Serial2` |
| 控制器 | `TESTINDI` | 增稳模式下启用俯仰 INDI |
| 机型 | `expensive` | 贵飞机专用舵面限幅/启动逻辑 |
| 奇偶构型 | `ODD` | 奇数机体构型 |

这些宏目前硬编码在 `src/main.cpp`，不是 PlatformIO 环境参数。烧录给其他飞机前必须先切换身份宏，并重新核对舵机 trim、串口方向和引脚。

## 2. 构建与运行

目标环境定义在 `platformio.ini`：

- 平台/开发板：Teensy 5.0.0 / Teensy 4.1（IMXRT1062，600 MHz）
- 框架：Arduino/Teensyduino
- 环境名：`teensy41`
- 上传：`teensy-gui`
- 串口监视器：921600 baud
- 编译优化：`-O3 -ffast-math -w`，USB 模式为 `USB_SERIAL`

标准编译命令：

```powershell
& 'C:\Users\Chen\.platformio\penv\Scripts\platformio.exe' run
```

上传与监视器常用命令：

```powershell
& 'C:\Users\Chen\.platformio\penv\Scripts\platformio.exe' run --target upload
& 'C:\Users\Chen\.platformio\penv\Scripts\platformio.exe' device monitor
```

2026-07-22 验证结果：编译成功，生成 `.pio/build/teensy41/firmware.elf` 和 `firmware.hex`。FLASH 为代码 239,388 B、数据 115,628 B、头 8,500 B；RAM1 使用变量 120,672 B、代码 236,312 B、填充 25,832 B，局部变量剩余 141,472 B；RAM2 使用 12,416 B。

## 3. 目录与职责

| 路径 | 职责 |
| --- | --- |
| `platformio.ini` | Teensy 4.1 构建、上传和依赖配置 |
| `src/main.cpp` | 入口和绝大多数业务逻辑：初始化、500 Hz 主循环、传感器、姿态、控制、执行器、通信、日志 |
| `src/control_allocation.cpp` | 复合体质量矩阵、舵效矩阵、广义力与 Eigen 伪逆控制分配 |
| `src/radioComm.cpp` | SBUS/PWM/PPM/DSM 接收机初始化和脉宽采集 ISR |
| `lib/` | 随仓库保存的传感器、接收机和显示驱动库 |
| `include/` | PlatformIO 预留公共头文件目录，当前无项目头文件 |
| `test/` | PlatformIO 预留测试目录，当前无自动化测试 |

`src/main.cpp` 约 6,607 行且依赖大量全局变量。跨文件接口通过 `extern` 和前置声明连接，没有项目级头文件。

## 4. 启动与主循环

### `setup()`

1. 初始化 USB 串口及 `Serial1/3/5/6/7/8`。
2. 初始化状态 LED、按键、SSD1306 OLED、内置 SD 卡和新的 `datalogNNN.txt`。
3. 从 EEPROM 读取转角零点和 IMU 校准结构。
4. 根据飞机身份显示 A–G 编号。
5. 将 Servo 1–7 绑定到输出引脚并给出启动安全位置。
6. 初始化 SBUS、BMI088/IMU、控制分配惯量和 35×20 舵效矩阵。
7. 校准 MS4525 空速传感器，初始化 Madgwick 姿态与 Butterworth 滤波器。
8. 默认进入 `MANUAL_MODE`。

### `loop()`

主循环目标频率为 500 Hz（`loopRate(500)`），主要数据流为：

```text
时间更新
  -> 解锁状态
  -> BMI088 采样 + Madgwick 姿态
  -> 角加速度 / 空速 / 期望状态
  -> 按键、OLED、应变、气动数据
  -> 上下级串口状态交换
  -> 手动 / 无积分增稳 / 有积分增稳
  -> 动态控制分配 + 混控 + PWM 缩放
  -> 本机舵机输出 + 下级命令转发
  -> SD 日志
  -> SBUS 命令更新 + failsafe
  -> 500 Hz 节拍等待
```

注意：命令接收和 failsafe 在循环末尾执行，因此控制器使用的是上一循环采集到的遥控数据。

## 5. 控制体系

### 飞行模式

- `MANUAL_MODE`：清零构型 PID，主机下发/本机执行手动舵面命令。
- `STABLIZE_MODE_NO_I`：串级姿态控制但禁用积分。
- `STABILIZE_MODE`：串级姿态控制并启用积分；定义 `TESTINDI` 时升降舵改用 `PITCH_INDI_control()`。
- `force_manual` 和 `int_is_valid` 会沿 50 字节命令帧传递给从机。

### 姿态与控制

- 当前本机传感器采样函数是 `getBMI088data()`，SPI 配置为 10 MHz、Mode 3。
- `Madgwick(dt)` 输出 `roll_IMU/pitch_IMU/yaw_IMU`。
- `controlANGLE2()` 是串级角度/角速度控制主路径；`controlRATE()` 和 `controlANGLE()` 仍保留。
- `PITCH_INDI_control()` 使用滤波角速度、角加速度和舵效模型生成俯仰 PWM。
- `increase_Clp()` 根据空速计算等效滚转气动补偿。

### 动态控制分配

`control_allocation.cpp` 通过 Eigen 动态矩阵执行以下步骤：

1. 由五段质量/惯量、四个相对转角和几何位置组装 18×18 质量矩阵 `M`。
2. 提取 10×10 的 `Mpi`。
3. 计算构型相关矩阵 `Be` 和固定 35×20 舵效矩阵 `H`。
4. 得到 `Bplus = Mpi^-1 * Be * H`。
5. 取整体/构型控制相关行和十个副翼列，再用 Jacobi SVD 求伪逆。
6. 在 `controlMixer()` 中将姿态与构型目标分配到各机副翼、升降、油门和方向舵。

`getpinvBplusmini()` 每次主循环都会重新组装矩阵并执行逆/伪逆，属于实时路径中计算量最大的部分之一。

## 6. 硬件接口

### PWM、SPI 与数字引脚

| 功能 | 引脚 |
| --- | --- |
| 左副翼、右副翼、升降、油门、方向 | 2、3、4、5、6 |
| Servo 6、Servo 7 | 都配置为 9 |
| BMI088 加速度 CS | 10 |
| BMI088 陀螺仪 CS | 37 |
| 状态 LED | 32、33 |
| 按键 | 36（先设 OUTPUT，随后设 INPUT_PULLUP） |
| PWM 接收候选 CH1–6 | 15、16、17、20、21、22 |
| PPM 候选 | 23 |

电机候选引脚中 `m1Pin/m2Pin` 都是 37，会与 BMI088 陀螺仪 CS 冲突；当前实际输出路径使用 Servo 1–5，但启用旧电机路径前必须处理冲突。

### 串口与总线

| 接口 | 波特率/用途 |
| --- | --- |
| USB `Serial` | 500000，调试输出 |
| `Serial1` | 115200，外置 IMU 数据 |
| `Serial2` | SBUS 接收机 |
| `Serial3` | 921600，左侧下级链路 |
| `Serial5` | 921600，右侧下级链路 |
| `Serial6` | 921600，上级命令接收 |
| `Serial7` | 115200，转角或 Modbus 应变传感器 |
| `Serial8` | 115200，数传/二进制气动数据 |
| `Wire1` | SSD1306 与 MS4525 |
| `Wire2` | 可选 BMP280 |
| SPI | BMI088；SD 使用 Teensy 内置卡接口 |

当前 F 从机从 `Serial6` 接收上级 50 字节控制帧，通过 `Serial3` 与左链内侧节点交换/转发数据，并将自身及外侧链数据汇总上送。

## 7. 线协议摘要

| 协议 | 帧头 | 长度 | 内容 |
| --- | --- | --- | --- |
| 级联控制命令 | `55 60` | 50 B | 积分/手动标志、12 路舵机、3 个俯仰目标、3 个升降 PWM、3 个前馈 PWM、3 个灯、累加校验 |
| 陀螺/转角状态 | `55 71` | 33 B | 3 个相对角、3 个滚转角速度、3 个滚转角、3 个俯仰角、3 个升降 PWM、累加校验；浮点量按 ×10 的 `int16` 传输 |
| 应变请求 | Modbus RTU | 8 B 请求 | `Serial7`，功能码 0x03，CRC16/Modbus |
| 气动数据 | `AA 55` | `sizeof(FC_Binary_Packet)` | `Serial8`，原生结构体载荷与 8 位累加校验 |

帧协议没有版本号，且部分结构依赖本机字节序/结构体布局。修改字段时必须同步所有 A–G 固件。

## 8. 依赖

`platformio.ini` 声明 ArduinoEigen、TinyGPSPlus、ICM42688 和 Bolder Flight Systems BMI088。仓库本地 `lib/` 还提供 MS4525、Adafruit GFX/SSD1306/BusIO、SBUS、DSMRX、MPU6050、MPU9250、I2Cdev、TFMPlus 等。PlatformIO 会同时扫描本地库和 `lib_deps`。

当前编译实际依赖图包含 ArduinoEigen、MS4525、Adafruit GFX/SSD1306、DSMRX、EEPROM、MPU6050/MPU9250、PWMServo、SBUS、SD、Servo、SPI 和 Wire；TinyGPSPlus、ICM42688 等虽已声明但当前主路径未使用。

## 9. 已识别风险与维护约束

以下是扫描发现、但未在本次知识库任务中改动的代码风险：

1. **广义力向量被遮蔽**：`control_allocation.cpp::getQx()` 声明为 `void`，内部又声明局部 `Qx` 并 `return Qx`。局部变量遮蔽全局 `Qx`，随后 `dw_rev=-Mpi_pinv*Qx` 可能使用未更新的全局向量。
2. **警告全部关闭**：`platformio.ini` 的 `-w` 隐藏了上一问题等编译器诊断；`-ffast-math` 也会改变 NaN/Inf 和浮点严格语义。
3. **引脚复用**：Servo 6/7 共用 9，电机 1/2 与 BMI088 陀螺仪 CS 共用 37；扩展输出前必须核对原理图。
4. **循环预算紧张**：循环目标 500 Hz，但 OLED、SD、串口打印、SVD/矩阵逆和传感器轮询都在同步主路径。代码注释已指出显示会把频率拖到约 80 Hz，F 从机还每循环打印一次本地油门。
5. **身份配置易误烧**：飞机身份、机数、IMU 和控制器全部是源码宏，单一 PlatformIO 环境不能防止把 F 固件烧到其他机体。
6. **串口职责冲突**：`Serial7` 同时用于转角与应变逻辑，`Serial8` 同时描述为数传和气动数据；组合启用前需明确互斥关系。
7. **测试空缺**：`test/` 目前没有单元测试或协议测试，控制分配与帧编解码缺少主机侧回归验证。
8. **编码历史**：文件前部部分中文注释已呈现乱码，后续保存时应统一 UTF-8，并避免用错误编码整体重写源文件。

## 10. 修改入口速查

| 需求 | 首选位置 |
| --- | --- |
| 切换 A–G 飞机身份/机数/IMU | `src/main.cpp` 顶部配置宏 |
| 改舵机方向与机械 trim | `src/main.cpp` 的 `pwm_channel*_rev/trim` 区域 |
| 改飞行模式和遥控映射 | `getDesState()`、`getCommands()` |
| 调姿态 PID | `controlANGLE2()` / `controlRATE()` 参数区 |
| 调俯仰 INDI | `PITCH_INDI_control()` 及 INDI 滤波器初始化 |
| 改混控/舵面限幅 | `controlMixer()`、`scaleCommands()` 和 `loop()` 的各机 PWM 修正段 |
| 改复合体动力学/控制分配 | `src/control_allocation.cpp` 和 `main.cpp` 的质量、惯量、几何参数 |
| 改主从协议 | `sendAllData*()`、`receiveCommandData()`、`sendGYROxANGLE()`、`getGYROxANGLE*()` |
| 改日志字段 | `setup()` CSV 表头与 `loggerTEAM()` / `loggerSINGLE()` |

建议后续优先将身份宏拆成多个 PlatformIO 环境，把线协议结构与常量提取到公共头文件，再为协议和控制分配建立宿主机测试。
