# TeensyDebug 板上调试

程序在 Teensy 4.1 上运行，需要连接板子。安装 VS Code 的 PlatformIO 和 Microsoft C/C++ 扩展。
克隆仓库后，任意一次 PlatformIO 构建都会在本机生成 `.vscode/launch.json`、`tasks.json` 和 `extensions.json`。这些文件不进入 Git。也可以运行 `python tools/teensy_debug/configure_vscode.py` 手动生成；脚本会保留本机其他配置。
调试任务使用 PlatformIO 自带的 Python（含 pyserial），无需参数软件的虚拟环境。

1. 连接 USB；上传前关闭占用板子串口的软件。调试时地面站和 GDB 分别使用两个虚拟串口。
2. 在“运行和调试”选择“TeensyDebug：编译、上传并调试”，按 F5。
3. 自动编译、上传调试固件并识别两个 USB 串口；初始停在 `initializeFirmwareDebug()`。
4. 在源码行号左侧设置断点，F5 继续。在变量/监视窗口查看数值。优先使用断点配合继续运行；单步存在下述限制。

已经上传同一份调试固件时可以选择“TeensyDebug：连接现有调试固件”。修改代码后使用编译上传入口，保证 ELF 与板上程序一致。
自动检测结果在任务终端显示 `TeensyDebug: COM…` 和 `Ground station / USB console: COM…`。同一根 USB 线在调试固件下枚举两个 CDC 串口：`Serial`（接口 0）用于地面站、参数和普通输出，`SerialUSB1`（接口 2）专供 GDB。地面站选择终端提示的 Ground station 端口，不能打开 GDB 端口。端口根据 USB 接口号识别，不以 COM 编号大小区分；仅连接一块 Teensy。

双串口调试固件的 VID/PID 为 `16C0:048B`；普通 `teensy41` 固件仍使用单串口 `16C0:0483`。只在 `teensy41_debug` 环境启用双串口。必须重新上传本版调试固件，旧单串口固件不符合新的自动识别规则。
路径由工作区、脚本位置和用户环境推导，不固定盘符和 COM 编号。

调试固件先等待 GDB 打开USB 串口，然后初始化 TeensyDebug 并调用 `halt_cpu()`；未连接调试器时不会继续启动。
调试器连接并停住后，再按 F5 继续才开始初始化传感器、SD 和舵机，此后可同时连接地面站。停在断点时主循环暂停，地面站的数据请求/参数保存/重启命令也暂停处理，可能显示超时；继续运行后恢复，必要时重试请求。USB 双串口并不让主循环在断点期间继续运行。使用完后通过 PlatformIO 的 `teensy41` 环境上传普通固件；普通固件不包含 TeensyDebug。

如果任务提示没有找到板子，短按 PROGRAM 按钮触发上传。任务会检查 USB 串口是否出现，上传工具单独显示 SUCCESS 并不能确认固件已写入板子。

只在地面、执行器动力断开的情况下断点调试。暂停主程序时中断仍可运行，原有 PWM 可能保持。
TeensyDebug 使用软件断点，单步和部分指令支持存在限制。此前双串口测试中 F10 越过 `prepareLocalElevatorCommand()` 后未再次停住；本版未重新验证 F10。遇到无法停住的位置，可在后续源码行设置断点，再按 F5。
`main.cpp` 和 `actuator_output.cpp` 使用 `-Og`，其余模块保留 `-O2` 并生成调试符号，部分变量可能显示为 optimized out。
需要降低其他模块的优化时，在 `build_flags.py` 的 `selected` 集合中添加对应源码路径，再编译上传；不要直接将全部依赖和硬件核心设为无优化。
调试时不能用循环耗时代表正常固件性能。VS Code 的停止按钮只断开调试连接，重新从头运行请重新上传或让板子重新上电。

调试依赖使用仓库内 `tools/teensy_debug/vendor/TeensyDebug` 的源码，来自 [TeensyDebug 官方仓库](https://github.com/ftrias/TeensyDebug) 的 `e496fdc44fbaf3eb96f6248ee8a97df18f64c8e1` 提交。它只用于 `teensy41_debug` 环境，新电脑构建时无需再从 GitHub 安装此库。
如果旧电脑或旧检出中留下了缺少库清单的 `.pio/libdeps/teensy41_debug` 依赖副本，F5 的准备脚本会清理这些不完整副本，再重新安装。

## 历史单串口板上验证

- 单 USB 串口枚举为 COM4，之后 COM 编号以任务检测结果为准。
- Microsoft C/C++ 调试适配器命中 `prepareActuatorCommands()` 的源码断点，监视读取 `pwm_channel3_trim = 29`。
- 调试固件丢弃普通 USB 输出并停用 USB 参数通信；SD 参数加载和 SD 日志逻辑仍按原流程运行。

以上为旧单串口版本的记录，不代表本版双串口实现已完成板上验证。本版保留普通 USB 输出与地面站协议。端口选择测试可运行 `python -m unittest discover -s tools/teensy_debug -p 'test_*.py' -v`。
