# HFAFC 参数配置与串口助手

电脑端通过 USB `Serial` 连接 Teensy 4.1。窗口上方为参数表，下方为串口助手。

## 启动与使用

1. 将当前工程编译得到的 `.pio/build/teensy41/firmware.hex` 烧录到对应飞机。
2. 插入 SD 卡，重新给飞控上电。关闭占用该 COM 端口的 PlatformIO/Arduino 串口监视器。
3. 断开旧软件的串口连接，双击 `dist/HFAFCParameterConsole.exe`，无需安装 Python。源码运行可双击 `start.cmd`；首次会创建本目录 `.venv` 并安装 pySerial，需要已安装包含 Tcl/Tk 的 Python 3。
4. 选择 COM 端口，点击“连接”，再点击“读取参数”。波特率默认 921600；Teensy USB Serial 实际以 USB 速度传输。
5. 选择参数行，在“新值”中输入数字，点击“写入内存并保存 SD”。只有飞控确认 SD 保存、读回校验及内存更新均成功，窗口才更新当前值。
6. 飞控重启后自动加载 SD 参数；再次读取可确认状态为“从 SD 加载”。

界面显示串口日志、参数回复和命令结果。勾选“16 进制显示”可查看原始字节，切换时也会转换保留的历史记录；不影响协议解析或原始记录。下方输入框以 UTF-8 发送文本命令，固定追加 LF（`\n`），点击发送或按回车即可。

窗口保留最近最多约 5000 行/50 万字符，格式切换可重建最近 128 KB 原始字节的历史；“记录全部接收”将开启后的全部原始接收字节保存到文件，不受窗口历史上限影响。接收显示每 100 ms 批量刷新。窗口最小 760×520，表格和日志区域随大小变化，写入和发送控件保持可见。

点击任意列标题可升序排列，再点击可降序排列；当前值与范围按数值排序，文本按名称自然排序（例如 `p2` 在 `p10` 前）。浮点值显示为能精确还原原始 float32 的简短数字，例如 `0.200000003` 显示为 `0.2`，实际精度保持。

SD 文件不存在、损坏、身份/配置不匹配时使用源码默认值，不会加载半份参数。首次成功修改会保存完整参数集。无 SD 时可以读取参数，但写入会被拒绝，内存保持原值。写入超时或连接中断表示结果未确认，应重新读取，不自动重试写入。

## 首版参数

共 28 项：27 项 float32 控制参数和 1 项 int32 调试开关，在 `src/control_modes.cpp::controlParameterTable()` 中注册；控制变量保持模块私有。类型由固件的变量类型决定，软件显示“浮点型”或“整型”，整数拒绝小数、指数形式及 int32 溢出。

| 分组 | 参数 |
| --- | --- |
| 姿态角 | `Kp_roll_angle`、`Ki_roll_angle`、`Kd_roll_angle`、`Kp_pitch_angle`、`Ki_pitch_angle`、`Kd_pitch_angle`、`B_loop_roll`、`B_loop_pitch` |
| 姿态角速度 | `Kp_roll_rate`、`Ki_roll_rate`、`Kd_roll_rate`、`Kff_roll_rate`、`Kp_pitch_rate`、`Ki_pitch_rate`、`Kd_pitch_rate`、`Kff_pitch_rate`、`Kp_yaw`、`Ki_yaw`、`Kd_yaw`、`Kff_yaw_rate` |
| 构型 | `Kp_Flap`、`Kp_FLAP_RATE`、`Ki_FLAP_RATE`、`Kff_FLAP_RATE`、`B_loop_FLAP` |
| 俯仰 INDI | `indi_pitch_q_gain`、`indi_pitch_effectiveness` |
| 调试（整型） | `usb_throttle_debug`：0 关闭油门输出，1 开启，每秒最多 10 行 |

旧固件持续打印 `0` 来自 `actuator_output.cpp` 每个控制周期调用 `printLocalThrottle()`，打印尚未收到 Serial6 上级有效指令时默认为 0 的 `Local_thro_PWM`。新版本默认关闭，开启后带 `parent_received` 和 `raw_pwm` 标签。开关实际绑定 int32，可保存到 SD、上电恢复并参与 `if` 判断。需要增加其他整数参数时，将已有 int32 变量的地址注册到表中：

```cpp
{"my_flag", &my_flag, 0, 1, "Configuration", "0 off, 1 on"},
```

当前 F 从机实际运行哪些回路由现有编译宏和模式决定；读取到的参数不表示它在所有模式下都会参与控制。角度 D 项仅在 `controlANGLE()` 使用，当前 `controlANGLE2()` 不使用。输入上下限是格式/数值约束，不能替代闭环整定验证。

参数通过 USB 在地面调试时修改，不再检查飞行模式、油门输出或上级命令状态。修改成功后写入内存并保存到 SD，开机自动恢复；数值范围、类型及 SD 保存校验仍然保留。

## 串口协议

请求为 ASCII 行，响应为带 `@HFAFC` 前缀的 TSV 行。请求 ID 取 uint32，用于区分回复；手动读取/帮助可省略 ID，使用 0。

```text
PARAM READ 1
PARAM SET 2 Kp_roll_angle 0.3
PARAM HELP
```

读取回复示例（`\t` 表示真实制表符）：

```text
@HFAFC\t1\tBEGIN\tF-TEAM-7-INDI-EXP\t28\tSD_READY\tLOADED
@HFAFC\t1\tVALUE\tKp_roll_angle\tfloat\t0.25\t0\t10\tAttitude\tRoll angle P
@HFAFC\t1\tVALUE\tusb_throttle_debug\tint\t0\t0\t1\tDebug\tThrottle USB log: 0 off, 1 on (10 Hz)
...其余参数...
@HFAFC\t1\tEND\t28
```

写入成功：`@HFAFC\t2\tOK\tKp_roll_angle\tfloat\t0.3\tSAVED`。
错误：`@HFAFC\t2\tERROR\tSD_WRITE\tSD save/verification failed; RAM unchanged`。

软件收到完整 BEGIN/VALUE/END 后才更新参数表；普通日志同时显示。串口助手手动 SET 后可点击“读取参数”刷新参数表。只支持已注册变量，不接受任意内存地址。float32 以最多 9 位有效数字的简短十进制精确往返，int32 以十进制整数保存，避免大整数经 float32 丢失精度。新版软件也支持读取旧版不带类型的浮点协议。

## SD 存储与恢复

SD 根目录为 `params0.cfg`、`params1.cfg` 两个文件：

```text
HFAFC_PARAMS_V2 F-TEAM-7-INDI-EXP
GEN=1
Kp_roll_angle:float=0.25
usb_throttle_debug:int=0
...完整参数集...
CRC32=xxxxxxxx
```

写入未选中的文件，关闭后读回检查身份、版本、参数名/数量、类型、范围、CRC32 和数值。验证成功再修改运行时内存；另一份有效文件保留。开机选择最新有效序号，损坏的一份不会覆盖另一份。校验包含飞机 A～G、TEAM/SINGLE、组合规模、TESTINDI 和 expensive 配置，以免跨机误加载。兼容原有完整 V1 浮点参数文件：新加的整数参数用源码默认值，下次成功写入自动保存 V2 文件。其他不兼容的注册表修改需要重新设置参数。

不要直接编辑文件而忽略 CRC，也不要删除最新文件来“整理”SD 卡；两份文件共同用于恢复。软件写入自动维护文件。

## 代码来源与适配

参考 [MicroConfig](https://github.com/janscience/MicroConfig) 的指针参数绑定、配置菜单、SD save/load 设计，和 [SerialUI](https://github.com/uutzinger/SerialUI) 的串口工作线程、接收显示和命令发送设计。本工程新增实现为独立编写，未复制或引入两项目的源码/依赖。

MicroConfig 的交互菜单会等待用户输入，不适合直接放入飞控主循环。本工程采用定长命令缓冲区，每轮最多接收 64 字节或发送一行，不使用串口等待。电脑窗口采用 Python 自带 tkinter，以减少安装依赖，串口采用 pySerial。SD 使用现有 Teensy SD 库。

源码入口为 `main.py`；协议解析为 `protocol.py`；后台串口为 `serial_worker.py`。固件服务为 `src/parameter_service.cpp`，在 `main.cpp` 初始化及周期末尾调用。

## 构建软件与测试

在本目录执行 `./build.ps1` 可生成 Windows exe。运行依赖在 `requirements.txt`；测试/打包依赖在 `requirements-dev.txt`。

在仓库根目录执行：

```powershell
& ./tools/parameter_console/.venv/Scripts/python.exe -m unittest discover -s tools/parameter_console -p 'test_*.py' -v
```

固件参数模块主机测试使用可选 Zig C++ 编译器（`pip install ziglang==0.13.0`），通过模拟串口和 SD 直接运行实际服务代码，并使用与工程相同的 `-O3 -ffast-math`：

```powershell
$env:ZIG_GLOBAL_CACHE_DIR = Join-Path (Get-Location) '.pio/zig-cache'
& ./tools/parameter_console/.venv/Scripts/python.exe -m ziglang c++ -std=c++17 -O3 -ffast-math -Itests/firmware_mocks -Iinclude src/parameter_service.cpp tests/test_parameter_service.cpp -o .pio/parameter_service_tests.exe
& ./.pio/parameter_service_tests.exe
```

主机测试覆盖协议分片、混合日志、USB 输出背压、输入范围和 NaN/Inf、SD 失败时内存保持、损坏文件回退、完整加载、配置身份、序号回绕、float32 往返、int32 极值和 V1 文件迁移。电脑端 10 项测试还覆盖排序、数字显示、十六进制切换及小窗口控件可见性。GUI 测试使用本机 socket 模拟飞控；真实 USB/SD 和上电恢复仍需要板上验证。
