# TeensyDebug 板上调试

程序在 Teensy 4.1 上运行，需要连接板子。安装 VS Code 的 PlatformIO 和 Microsoft C/C++ 扩展。
任务使用参数软件的 Python 虚拟环境（含 pyserial）；未创建时先运行 `tools/parameter_console/start.cmd`。

1. 关闭占用板子串口的软件，连接 USB。
2. 在“运行和调试”选择“TeensyDebug：编译、上传并调试”，按 F5。
3. 自动编译、上传调试固件并识别USB 串口；初始停在 `initializeFirmwareDebug()`。
4. 在源码行号左侧设置断点，F5 继续。在变量/监视窗口查看数值。优先使用断点配合继续运行；单步存在下述限制。

已经上传同一份调试固件时可以选择“TeensyDebug：连接现有调试固件”。修改代码后使用编译上传入口，保证 ELF 与板上程序一致。
自动检测结果在任务终端显示。调试固件的唯一 USB 串口专用于 GDB，不要用串口助手或参数软件打开。需要读写参数和普通输出时，上传 `teensy41` 普通固件。
路径由工作区、脚本位置和用户环境推导，不固定盘符和 COM 编号。

调试固件先等待 GDB 打开USB 串口，然后初始化 TeensyDebug 并调用 `halt_cpu()`；未连接调试器时不会继续启动。
调试器连接并停住后，再按 F5 继续才开始初始化传感器、SD 和舵机。使用完后通过 PlatformIO 的 `teensy41` 环境上传普通固件；普通固件不包含 TeensyDebug。

如果任务提示没有找到板子，短按 PROGRAM 按钮触发上传。任务会检查 USB 串口是否出现，上传工具单独显示 SUCCESS 并不能确认固件已写入板子。

只在地面、执行器动力断开的情况下断点调试。暂停主程序时中断仍可运行，原有 PWM 可能保持。
TeensyDebug 使用软件断点，单步和部分指令支持存在限制。此前双串口测试中 F10 越过 `prepareLocalElevatorCommand()` 后未再次停住；当前单串口已验证断点和继续运行，未重新验证 F10。遇到无法停住的位置，可在后续源码行设置断点，再按 F5。
`main.cpp` 和 `actuator_output.cpp` 使用 `-Og`，其余模块保留 `-O2` 并生成调试符号，部分变量可能显示为 optimized out。
需要降低其他模块的优化时，在 `build_flags.py` 的 `selected` 集合中添加对应源码路径，再编译上传；不要直接将全部依赖和硬件核心设为无优化。
调试时不能用循环耗时代表正常固件性能。VS Code 的停止按钮只断开调试连接，重新从头运行请重新上传或让板子重新上电。

依赖固定到 [TeensyDebug 官方仓库](https://github.com/ftrias/TeensyDebug) 的指定提交。

## 本次板上验证

- 单 USB 串口枚举为 COM4，之后 COM 编号以任务检测结果为准。
- Microsoft C/C++ 调试适配器命中 `prepareActuatorCommands()` 的源码断点，监视读取 `pwm_channel3_trim = 29`。
- 调试固件丢弃普通 USB 输出并停用 USB 参数通信；SD 参数加载和 SD 日志逻辑仍按原流程运行。
