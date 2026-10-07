# HFAFC 参数配置与串口助手

电脑端通过 USB `Serial` 连接 Teensy 4.1。窗口上方为参数表，下方为串口助手。

## 启动与使用

1. 将当前工程编译得到的 `.pio/build/teensy41/firmware.hex` 烧录到对应飞机。
2. 插入 SD 卡，重新给飞控上电。关闭占用该 COM 端口的 PlatformIO/Arduino 串口监视器。
3. 断开旧软件的串口连接，双击 `dist/HFAFCParameterConsole.exe`，无需安装 Python。源码运行可双击 `start.cmd`；首次会创建本目录 `.venv` 并安装 pySerial，需要已安装包含 Tcl/Tk 的 Python 3。
4. 选择 COM 端口，点击“连接”，再点击“读取参数”。波特率默认 921600；Teensy USB Serial 实际以 USB 速度传输。
5. 双击参数的“当前值”单元格，原数值会选中变蓝，直接输入新数值，按 Enter 写入内存并保存 SD；Esc 或点击其他位置取消编辑。只有飞控确认 SD 保存、读回校验及内存更新均成功，窗口才更新当前值。
6. 飞控重启后自动加载 SD 参数；再次读取可确认状态为“从 SD 加载”。

界面显示串口日志、参数回复和命令结果。勾选“16 进制显示”可查看原始字节，切换时也会转换保留的历史记录；不影响协议解析或原始记录。下方输入框以 UTF-8 发送文本命令，固定追加 LF（`\n`），点击发送或按回车即可。

窗口保留最近最多约 5000 行/50 万字符，格式切换可重建最近 128 KB 原始字节的历史；“记录全部接收”将开启后的全部原始接收字节保存到文件，不受窗口历史上限影响。接收显示每 100 ms 批量刷新。窗口最小 760×520，表格和日志区域随大小变化，参数值直接在表格中编辑，发送控件保持可见。

点击任意列标题可升序排列，再点击可降序排列；当前值与范围按数值排序，文本按名称自然排序（例如 `p2` 在 `p10` 前）。浮点值使用能精确还原原始 float32 的普通十进制，例如 `0.200000003` 显示为 `0.2`、`0.0002` 保持为 `0.0002`、`1000` 保持为 `1000`。界面、范围提示、发送的参数值、固件回复和 SD 文件均不使用科学计数法。

SD 文件不存在、损坏、身份/配置不匹配时使用源码默认值，不会加载半份参数。首次成功修改会保存完整参数集。无 SD 时可以读取参数，但写入会被拒绝，内存保持原值。写入超时或连接中断表示结果未确认，应重新读取，不自动重试写入。

## 可调参数

共 75 项：69 项 float32 参数和 6 项 int32 参数，服务容量上限为 512 项，在 `src/control_modes.cpp::controlParameterTable()` 中注册；控制变量保持模块私有。类型由固件的变量类型决定，软件显示 `float` 或 `int`，整数拒绝小数、指数形式及 int32 溢出。

| 分组 | 参数 |
| --- | --- |
| 姿态角 | `Kp_roll_angle`、`Ki_roll_angle`、`Kd_roll_angle`、`Kp_pitch_angle`、`Ki_pitch_angle`、`Kd_pitch_angle`、`B_loop_roll`、`B_loop_pitch` |
| 姿态角速度 | `Kp_roll_rate`、`Ki_roll_rate`、`Kd_roll_rate`、`Kff_roll_rate`、`Kp_pitch_rate`、`Ki_pitch_rate`、`Kd_pitch_rate`、`Kff_pitch_rate`、`Kp_yaw`、`Ki_yaw`、`Kd_yaw`、`Kff_yaw_rate` |
| 构型 | `Kp_Flap`、`Kp_FLAP_RATE`、`Ki_FLAP_RATE`、`Kff_FLAP_RATE`、`B_loop_FLAP` |
| 俯仰 INDI | `indi_pitch_q_gain`、`indi_pitch_effectiveness` |
| 调试（整型） | `usb_throttle_debug`：0 关闭油门输出，1 开启，每秒最多 10 行 |
| 控制限幅/滤波 | `k_Clp`、`i_limit`、`maxRoll`、`maxPitch`、`maxYaw`、`Trim_pitch_angle`、`roll_pid_lpf_fc`、`roll_pid_dot_lpf_fc`、`pitch_des_local_rate_lpf_fc` |
| INDI 舵机模型/输出 | `indi_pitch_pwm_to_deg_k`、`indi_pitch_pwm_to_deg_b`、`indi_pitch_servo_delay_s`、`indi_pitch_servo_tau_s`、`indi_pitch_deflection_min_deg`、`indi_pitch_deflection_max_deg`、`indi_pitch_rate_limit_deg_s`、`indi_pitch_cmd_lpf_fc_hz`、`indi_pitch_pwm_min`、`indi_pitch_pwm_max` |
| 执行器方向（int） | `pwm_channel1_rev` ～ `pwm_channel5_rev`，仅接受 -1 或 +1，拒绝 0 |
| 执行器安装偏置（float） | A 机 `pwm_channel1_trim`、`pwm_channel2_trim`、`pwm_channel3A_trim`；B～G 各机 `pwm_channel1B_trim` ～ `pwm_channel3G_trim`；`pwm_channel4_trim`、`pwm_channel5_trim` |

trim 仍表示相对 1500 μs 基准的安装偏置，不随 rev 反向；油门沿用原 1100 μs 起点。当前飞机的 `pwm_channel3_trim` 引用对应 A～G 的升降舵 trim，因此修改对应参数会直接影响本机俯仰计算。从机的襟副翼、油门、方向舵指令由主机计算，相关 trim 应在主机上调整；参数文件仍按飞机身份隔离，软件不会把一台飞控的修改同步给其他飞控。INDI 舵机纯延迟由现有 64 点队列实现，实际延迟最多 63 个采样间隔。


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
@HFAFC\t1\tBEGIN\tF-TEAM-7-INDI-EXP\t75\tSD_READY\tLOADED
@HFAFC\t1\tVALUE\tKp_roll_angle\tfloat\t0.25\t0\t10\tAttitude\tRoll angle P
@HFAFC\t1\tVALUE\tusb_throttle_debug\tint\t0\t0\t1\tDebug\tThrottle USB log: 0 off, 1 on (10 Hz)
...其余参数...
@HFAFC\t1\tEND\t75
```

写入成功：`@HFAFC\t2\tOK\tKp_roll_angle\tfloat\t0.3\tSAVED`。
错误：`@HFAFC\t2\tERROR\tSD_WRITE\tSD save/verification failed; RAM unchanged`。

软件收到完整 BEGIN/VALUE/END 后才更新参数表；普通日志同时显示。串口助手手动 SET 后可点击“读取参数”刷新参数表。只支持已注册变量，不接受任意内存地址。float32 以最多 9 位有效数字的简短十进制精确往返，int32 以十进制整数保存，避免大整数经 float32 丢失精度。新版软件也支持读取旧版不带类型的浮点协议。

## SD 存储与恢复

SD 根目录使用 `params.cfg`（当前主文件）和 `params_backup.cfg`（上次成功保存的备份）：

```text
HFAFC_PARAMS_V3 F-TEAM-7-INDI-EXP
GEN=1
COUNT=75
Kp_roll_angle:float=0.25
usb_throttle_debug:int=0
...完整参数集...
CRC32=xxxxxxxx
```

保存前先读取并验证上一份有效参数，写入并验证 `params_backup.cfg`，随后覆盖 `params.cfg`，关闭后读回检查身份、版本、参数名/数量、类型、范围、CRC32 和数值。确认成功才修改运行时内存；任何阶段失败均不修改内存。首次保存时没有旧参数可备份，只创建主文件；从备份恢复后再次保存时保留原备份。

开机优先加载有效的 `params.cfg`，主文件缺失或损坏时恢复 `params_backup.cfg`。两者都无效时尝试旧 `params0.cfg` / `params1.cfg`，按保存序号选择最新有效文件；下次成功修改自动迁移到新名称，旧文件不再更新。没有有效文件则使用源码默认值，开机读取本身不会写文件。

身份校验包含飞机 A～G、TEAM/SINGLE、组合规模、TESTINDI 和 expensive 配置。兼容原 V1 的 27 项浮点参数和 V2 的 28 项参数；新增参数使用源码默认值，已有值保留，下次保存为 V3。V3 使用 COUNT 声明文件中的条目数量，并按名称与类型恢复；后续增加参数时，新增项使用默认值。缺行、重复名称、未知名称、类型或配置不匹配均拒绝加载。

不要直接编辑参数文件而忽略 CRC；通过软件修改会自动维护主文件和备份。

## 容量、缓冲区与内存

固件最多注册 512 项参数，文件缓冲区为 98,304 B（96 KiB），参数名最长 96 字节。普通十进制的极小 float32 最多需要约 56 字节，因此单值缓冲区为 64 B，命令/回复缓冲区为 512 B。五个 512 项 double 数组共 20,480 B；文件条目去重标记为 512 B。大缓冲全部为模块静态内存，总计 119,296 B（116.5 KiB），不在函数栈上分配。文件读写复用同一个缓冲区，写完并关闭文件后再读回校验；保存候选、旧值和校验值使用独立数组。服务由主循环串行调用，不可在中断中重入。

当前 Teensy 4.1 配置编译结果：RAM1 变量 152,064 B，代码 144,136 B，填充 19,704 B，剩余 208,384 B（203.5 KiB）供局部变量和栈使用；RAM2 变量 12,416 B，堆剩余 511,872 B。该结果已包括为 512 项预留的静态容量，注册表当前为 75 项。

使用实际 ARM 编译命令附加 `-fstack-usage` 检查：参数文件读取栈帧 336 B、保存 160 B、服务轮询 280 B、初始化 56 B；最大单函数帧为 336 B。按模块调用链叠加约 848 B，不包括 C 库格式化、SD 库内部及中断的栈，尚未做板上栈高水位测量。

正常控制计算直接使用变量，不遍历参数表。USB 读取每轮最多接收 64 字节或发送一行；512 项完整返回在 500 Hz 下理论约 2.1 秒，实际取决于 USB 背压和其他输出。SET 的全表序列化、CRC、SD 主文件/备份写入与读回校验仍同步执行，会延长收到该命令的控制周期。本次 75 项板上读取约 0.34～0.63 秒，写入确认约 1.09～1.44 秒，独立控制周期耗时尚未测量，见 [板上测试记录](BOARD_TEST_REPORT.md)。

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
& ./tools/parameter_console/.venv/Scripts/python.exe -m ziglang c++ -std=c++17 -O3 -ffast-math -UNDEBUG -Itests/firmware_mocks -Iinclude src/parameter_service.cpp tests/test_parameter_service.cpp -o .pio/parameter_service_tests.exe
& ./.pio/parameter_service_tests.exe
```

主机测试覆盖协议分片、混合日志、USB 输出背压、输入范围和 NaN/Inf、SD 失败时内存保持、损坏文件回退、完整加载、配置身份、序号回绕、普通十进制 float32 往返与极值、int32 极值、V1/V2 文件迁移、512 项大文件读写、注册表扩容及 rev 零值拒绝。电脑端 12 项测试还覆盖完整读取 75/512 项参数、容量越界拒绝、排序、数字显示、十六进制切换、单元格选中/回车保存/Esc 取消及小窗口控件可见性。GUI 自动测试使用本机 socket 模拟飞控；真实 COM4 的参数读取、写入、SD 保存、完整断电上电恢复及原值恢复已验证，范围与已发现问题见 [板上测试记录](BOARD_TEST_REPORT.md)。
