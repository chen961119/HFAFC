# CoFly Autopilot

**面向多体飞行器、大展弦比柔性飞行器及编队的分布式协同飞控系统。**

| 名称 | 统一写法 |
| --- | --- |
| 仓库名 | `CoFly-Autopilot` |
| 项目名 | CoFly Autopilot |
| 英文全称 | Collaborative Distributed Autopilot |
| 中文名称 | 分布式协同飞控系统 |

CoFly 表达协同飞行，Autopilot 明确飞控定位；项目名称不限定机体构型，分布式架构由正式全称补充。

## 项目概况

当前实现是基于 Teensy 4.1、Arduino/Teensyduino 和 PlatformIO 的飞控固件，由 dRehmFlight 演化而来。它支持 A–G 节点的主从级联通信、姿态与构型控制、本机俯仰 INDI、执行器输出、SD 日志及 USB 参数配置。

当前开发基线为七机串联构型 `F-D-B-A-C-E-G`，默认编译 F 从机。项目面向多种协同飞行场景；通用编队控制属于扩展方向，现有七机混控也尚未接入源码中保留的五体动态分配器。实现边界见 [架构说明](docs/architecture.md)。

## 快速开始

1. 安装 PlatformIO Core，或在 VS Code 中安装 PlatformIO IDE，并确保终端可以运行 `pio`。
2. 在 [flight_config.h](include/flight_config.h) 中确认目标机体、组合规模、IMU 和控制模式。给其他节点烧录前必须修改身份宏并重新构建。
3. 在仓库根目录构建：

```sh
pio run -e teensy41
```

连接 Teensy 后上传，或打开串口监视器：

```sh
pio run -e teensy41 -t upload
pio device monitor -e teensy41
```

固件产物位于 `.pio/build/teensy41/firmware.hex`。依赖由 [platformio.ini](platformio.ini) 和 `lib/` 管理。完整配置、接线与构建说明见 [固件指南](docs/firmware-guide.md)。

> 首次上电、固件变更或节点身份切换后应拆桨测试。当前锁定输出为 `1500 + 本机通道 trim`；默认油门 trim 为 0，锁定时仍会输出 1500 μs。连接电调前必须核对其安全输入及实际 PWM，详见 [执行器锁定说明](docs/firmware-guide.md#执行器锁定与限幅)。

## 配套工具

- [USB 参数配置与串口助手](tools/parameter_console/README.md)：读取、修改与保存本机参数，支持 SD 恢复和串口日志。
- [Teensy 板上断点调试](tools/teensy_debug/README.md)：通过 `teensy41_debug` 环境与 VS Code 调试板上固件。

## 仓库结构

```text
CoFly-Autopilot/
├── src/                      # 飞控实现与主循环
├── include/                  # 配置、模块接口与共享类型
├── lib/                      # 随仓库保存的驱动和依赖库
├── tools/
│   ├── parameter_console/    # USB 参数配置软件与测试
│   └── teensy_debug/         # 板上调试脚本与配置模板
├── docs/
│   ├── README.md             # 文档导航
│   ├── architecture.md       # 当前架构与模块职责
│   ├── firmware-guide.md     # 固件配置、硬件和运行说明
│   ├── requirements.md       # 编号需求与验收矩阵
│   ├── references/           # 参考论文
│   └── archive/              # 历史源码分析
├── platformio.ini            # 构建、上传、调试与依赖配置
├── README.md                 # 项目入口
└── .gitignore                # 构建产物和本机文件忽略规则
```

## 文档与开发

详细资料见 [文档导航](docs/README.md)，目标需求和验收准则见 [软件需求说明](docs/requirements.md)。修改控制逻辑后应执行完整构建，涉及硬件行为的改动应记录地面验证及对应飞行试验结果。历史分析保留原版本的结论和行号，查找现行代码请使用架构文档中的模块入口。

项目源码与文档统一使用 UTF-8；`.pio/`、`.vscode/`、Python 环境和打包产物属于本机生成文件，不提交到仓库。
