# CoFly Autopilot 文档

CoFly Autopilot 的英文全称为 **Collaborative Distributed Autopilot**，中文名称为 **分布式协同飞控系统**。项目介绍与快速开始见 [仓库首页](../README.md)。

## 使用与开发

| 文档 | 内容 |
| --- | --- |
| [当前架构](architecture.md) | 系统边界、模块职责、主循环与兼容性 |
| [固件指南](firmware-guide.md) | 编译配置、PWM、传感器、硬件接线及级联协议 |
| [软件需求](requirements.md) | 功能与安全目标、唯一需求编号和验收矩阵 |
| [参数软件](../tools/parameter_console/README.md) | USB 参数配置、协议、SD 恢复与测试 |
| [板上调试](../tools/teensy_debug/README.md) | TeensyDebug 环境与 VS Code 使用说明 |
| [参数板上测试记录](../tools/parameter_console/BOARD_TEST_REPORT.md) | 2026-10-07 测试范围与结果 |

## 研究资料与历史分析

- [Zhu 等（2025）参考论文](references/zhu-2025-morphing-control.pdf)：Aerodynamics-Driven Morphing Control and Flight Test for Compound Flexible Multibody Aircraft。论文描述的控制方法与当前固件实现存在差异。
- [历史控制框架与控制律分析](archive/CONTROL_ARCHITECTURE_ANALYSIS.md)：保留论文推导、旧版源码公式与差异分析。
- [历史仓库知识](archive/REPOSITORY_KNOWLEDGE.md)：2026-07-22 源码扫描及当时的构建记录。

历史资料中的源码行号、资源占用和缺陷结论对应分析时版本；它们不能作为当前功能或测试通过的证明。
