# CoFly USB 地面站

参考 QGroundControl 的顶栏、侧栏与飞机设置布局，仅通过默认 USBSerial 通信。原 parameter_console 已整体迁入本目录，参数页和串口终端保持完整功能。

## 启动

1. 将本版本 `.pio/build/teensy41/firmware.hex` 烧录到飞控，插入 SD 卡后重新上电。
2. 关闭占用串口的 PlatformIO/Arduino 串口监视器。
3. 双击 `dist/CoFlyGroundStation.exe`，无需 Python。源码运行双击 `start.cmd`，首次需要含 Tcl/Tk 的 Python 3；脚本创建 `.venv` 并安装 pySerial。
4. 在“通讯连接”页选择 COM 端口和波特率（默认 921600），点击连接，自动识别固件。
5. “参数”页点击读取参数，双击当前值编辑，Enter 保存到内存和 SD，Esc 取消。无 SD 可读取，保存被拒绝。

旧固件仍支持参数页；数据和传感器页需要本版本固件。不同时运行旧控制台占用同一端口。

## 页面与实现范围

| 页面 | 本版本行为 |
| --- | --- |
| 通讯连接 | 枚举/手动输入串口、波特率、连接/断开、能力识别与重试、重启飞控 |
| 应用设置 | 室内深色、室外高对比浅色、9–18 号字体，自动保存 |
| 参数 | 参数表、筛选、排序、类型/范围验证、就地编辑、存储确认 |
| 校准 | 显示实际采样配置，选择启用的传感器请求校准/设置预留接口 |
| 遥控器 | 六路处理后的 PWM 输入条与数值，校准/低中高位模式映射接口预留 |
| 串口助手 | 位于飞机设置最下方；接收日志、命令发送、十六进制显示、自动滚动、导出与原始记录 |
| 飞行 | 姿态仪、欧拉角、角速度、加速度、空速、IMU 相对角、当前模式与锁定状态；底部后视翼尖连接构型图 |

地速和高度没有有效来源，显示 `—`。飞行/遥控器页最高 5 Hz 请求数据，两秒未收到有效数据则清空读数，其他页暂停轮询。全应用共用一个串口线程，参数事务与地面站请求互斥。设置保存在 `%LOCALAPPDATA%/CoFly/ground_station.json`，包含配色、字体、最近端口和波特率。

## 传感器识别说明

返回固件编译配置与实际采样路径，未进行硬件健康检测。当前 `INTIMU + USE_MPU6050_I2C` 的初始化/主循环实际使用 BMI088 SPI，写入沿用的 `*_6050` 变量，因此显示 `BMI088_SPI`。

主循环还采样 MS4525 空速和 MT6701 转角。外置 IMU 受 `EXTIMU` 控制。BMP280 即便启用，主循环目前未调用 `getbarodata()`，显示 `INITIALIZED_NOT_SAMPLED`。磁力计/GPS/地速/高度未报告为有效。选择 MPU9250 的现有配置与 BMI088 主循环不一致时返回 `BMI088_SAMPLING_CONFIG_MISMATCH`，惯性指标显示 `—`；后续需统一固件采样路径。

MT6701 仅用于测量与 SD 记录，不参与构型显示或控制计算。翼尖相对角恢复为 tag0.1 默认路径的相邻 IMU 滚转差：AB=B−A、AC=C−A、BD=D−B、CE=E−C、DF=F−D、EG=G−E。OLED、USB、构型控制、控制分配、调试输出和数传均使用 IMU 角度；原 MT 日志字段保留。

空速和 IMU 尚无统一健康/测量时间戳接口，此处显示已有共享测量值；USB 帧新鲜度不代表传感器健康。

## 实时构型图

位于飞行页最下方，从机尾向前看，各机简化为等翼展线段，翼尖始终连接。机数由固件配置提供，支持 1/3/4/5/7 机；三机 B—A—C，四机 D—B—A—C，五机 D—B—A—C—E，七机 F—D—B—A—C—E—G。

A 机姿态为基准，向外逐段累加相对角，正滚转表示画面右翼下降，A 机使用独立颜色。视图随窗口缩放并适配竖直/折叠构型。完整构型需要 USB 连接 A 主机；从机没有全机姿态，显示说明。左右相邻状态帧一秒未更新则不画完整构型，USB 构型帧两秒过期或断连则清空。现有级联状态协议没有远端单机独立的新鲜度标记，因此该状态检查只能保证直接相邻链路新鲜。

`GCS CONFIG id` 返回 count、master、roll_a、ab/ac/bd/ce/df/eg、left_valid/right_valid。角度均为度，count 为固件机数；master 标记是否拥有全机姿态。DATA 的 angle/angle_valid 现表示本机下游 IMU 相对角及直接链路新鲜度，不再表示 MT6701。

## 重启与解锁保护

通讯连接页提供“重启飞控”按钮，仅在固件报告 `reboot=SUPPORTED` 后启用。必须先锁定飞机，未锁定时固件回复 `ERROR / LOCK_REQUIRED`。重启请求为 `GCS REBOOT id`，成功回复 `REBOOT / status=REBOOTING`；飞控发送回复后等待 250 ms，刷新日志，再进行软件复位。地面站收到确认后断开连接，重启完成后手动重新连接 USB。

任何标记重启生效的参数成功保存不同值后，OLED 全屏显示 `REBOOT REQUIRED`，保持锁定与安全输出直至重启；改回原值也不解除。相同值及失败的保存不触发。

## USB 接口 v1

共用参数服务的 511 字节命令行缓冲和有背压的单行发送。每周期接收最多 64 字节；参数列表完成前不处理后续请求，无第二处 USB 读取者。

请求为 ASCII 行，LF 结尾，ID 是 uint32，响应为 `@COFLY\t{id}\t{kind}\t...\n`，与参数协议共用分片解析器。

```text
GCS CAPS 1
GCS DATA 2
GCS CONFIG 7
GCS REBOOT 8
GCS CALIBRATE 3 IMU
GCS SENSOR_SET 4 IMU DEFAULT
GCS RADIO_CALIBRATE 5
GCS FLIGHT_MODES 6 MANUAL STABILIZE_NO_I STABILIZE
```

CAPS/DATA 返回 TSV 的 `key=value` 字段，定义见 `station_protocol.py`。数据包含 ms（板上毫秒，可回绕）、roll/pitch/yaw（度）、gx/gy/gz（度/秒）、ax/ay/az（g）、airspeed（m/s）、angle（度）、angle_valid、rc1..rc6（μs）、mode、locked。桌面过期判断用本机单调时钟。

校准目标：IMU/GYRO/ACCEL、启用时的 EXTIMU、AIRSPEED、ROTATE。SENSOR_SET 接收单个 ASCII 设置标识，界面暂提供 DEFAULT。模式标识 MANUAL/STABILIZE_NO_I/STABILIZE 对应现有三种模式。

校准、设置、遥控器校准、模式映射均返回 ERROR / NOT_IMPLEMENTED。接口位于 `include/ground_station_service.h` 与 `src/ground_station_service.cpp`，不修改校准、EEPROM、模式或执行器状态。现有按键/开机校准存在阻塞流程，未直接接到 USB；下一版本通过异步任务、进度、取消与持久化实现接口。

参数协议和 SD 恢复见 [参数协议说明](PARAMETER_PROTOCOL.md)。[历史参数板上测试](BOARD_TEST_REPORT.md) 不代表本版地面站已完成板上验证。

## 构建与验证

本目录执行 `./build.ps1` 输出 `dist/CoFlyGroundStation.exe`，产物不提交 Git。仓库根目录运行：

```powershell
& ./tools/ground_station/.venv/Scripts/python.exe -m unittest discover -s tools/ground_station -p 'test_*.py' -v
& ./tools/ground_station/.venv/Scripts/python.exe ./tools/ground_station/main.py --self-test
pio run -e teensy41
```

2026-10-10：24 项自动测试通过，新增各机数翼尖连接、相对角累加、后视滚转方向、MT 值独立性和构型缺失状态验证。Teensy 4.1 release 固件编译通过。尚未完成真实飞控 USB 通信、传感器读数与控制周期耗时验证。

飞机编号/数量现在是 EEPROM 参数，首次设置 `aircraft_count` 和 `aircraft_id` 后重启。参数表显示存储介质；保存重启参数后状态栏提示重启，只读参数禁止编辑。详见 [统一参数与持久存储](../../docs/parameter-storage.md)。无 SD 也可保存 EEPROM 参数。
