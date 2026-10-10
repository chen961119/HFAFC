# USB 参数协议

参数页位于 `parameter_view.py`，完整地面站入口为 `main.py`。USB Serial 与地面站 `GCS` 指令共用一条连接。完整存储、迁移和开发规则见 [统一参数与持久存储](../../docs/parameter-storage.md)。

当前注册 75 项参数，协议和固件服务最多支持 512 项。每项显示类型、范围、存储位置、生效时机和只读状态；双击当前值编辑，Enter 保存、Esc 取消。浮点值采用能恢复 float32 的普通十进制，不使用科学计数法；整数拒绝小数、指数形式及 int32 溢出。列标题支持排序。

无 SD 时可以读写 EEPROM 参数；SD 参数保存会被拒绝。身份、校准和 EEPROM 参数要求锁定。飞机身份/数量保存后重启生效，校准标记由固件管理。超时或连接中断表示结果未确认，需重新读取，不自动重试。

## 请求与回复

请求为 ASCII 行，末尾 LF；请求 ID 为 uint32。日志中仅 `@COFLY` 前缀行按协议解析。

```text
PARAM READ 1
PARAM SET 2 aircraft_count 7
PARAM SET 3 aircraft_id 6
PARAM HELP
```

回复中 `\t` 代表实际制表符：

```text
@COFLY\t1\tBEGIN\tBOARD-<UID>-TEAM-INDI-EXP\t75\tSD_READY\tLOADED\t57\t18\t0
@COFLY\t1\tVALUE\tKp_roll_angle\tfloat\t0.25\t0\t10\tAttitude\tRoll angle P\tSD\tIMMEDIATE\tEDITABLE
@COFLY\t1\tVALUE\taircraft_id\tint\t0\t0\t7\tAircraft\tIdentity\tEEPROM\tREBOOT\tEDITABLE
...其余参数...
@COFLY\t1\tEND\t75
@COFLY\t2\tOK\taircraft_count\tint\t7\tSAVED
@COFLY\t3\tERROR\tLOCK_REQUIRED\tLock aircraft before changing identity or calibration
```

`BEGIN` 三个统计字段依次为所选 SD 文件的匹配项、未匹配项和跳过旧项；未匹配项可能来自 EEPROM 或默认值，不能把它理解为 EEPROM 是否加载成功。当前介质状态由固件启动日志进一步报告。

`VALUE` 为名称、类型、当前/待启动值、最小值、最大值、分组、说明、`SD|EEPROM`、`IMMEDIATE|REBOOT`、`EDITABLE|READ_ONLY`。电脑端仍支持以前未携带后三项元数据的格式。

`OK` 在声明介质保存并读回校验成功后返回。错误包含 `UNKNOWN_PARAMETER`、`RANGE`、`READ_ONLY`、`LOCK_REQUIRED`、`NO_SD`、`SAVE_FAILED` 等；失败后重新读取确认当前状态。

`GCS CONFIG` 返回当前生效的 `aircraft_id` 和 `config_valid`，以及构型数量/姿态。参数页的编号/数量是待启动值，飞行页显示当前生效身份。

## 文件与运行成本

SD 的 V5 主文件 `params.cfg` 与回退文件 `params_backup.cfg` 仅保存声明在 SD 的条目：

```text
COFLY_PARAMS_V5 BOARD-<UID>-TEAM-INDI-EXP
GEN=1
COUNT=57
Kp_roll_angle:float=0.25
usb_throttle_debug:int=0
...SD 条目...
CRC32=xxxxxxxx
```

板卡 UID 与硬件/算法配置限制加载来源。按名称和类型恢复；表重排保留值，改名、类型或范围失配单项回默认值。文件 CRC、缺行或重复名称错误拒绝整份文件。仅支持当前 V5 文件，不导入旧格式。不要手改文件而忽略 CRC。

EEPROM 为两个 2048 B 槽，每槽 24 B 头部加 2024 B 变长记录，参数记录与 SD 完全相同，共用编码和解析代码；版本、长度、CRC32、代次及最后提交标记保护快照。具体地址和校准保存规则见存储说明。

参数服务静态预留 512 项缓存，SD 文件缓冲由记录兼容长度自动推导；参数名默认上限 32，可在 `include/parameter_storage_config.h` 配置为 1～128，地面站兼容到 128 字符。主循环直接读取 RAM 变量，不访问持久存储或遍历参数表。USB 每轮最多接收 64 字节或发送一行。保存、CRC、读回校验同步执行，会延长命令所在控制周期；本版尚未真机验证最坏周期。

## 构建与验证

在本目录执行 `./build.ps1`，生成 `dist/CoFlyGroundStation.exe`。电脑测试使用本机 socket 模拟飞控：

```powershell
& ./tools/ground_station/.venv/Scripts/python.exe -m unittest discover -s tools/ground_station -p 'test_*.py' -v
./tests/parameter_storage/run.ps1
```

第二条命令需要 MinGW g++，直接编译真实存储模块并按实际写入次数逐位置模拟断电，并分别以 32 / 48 字符上限编译验证。详见 [存储测试](../../tests/parameter_storage/README.md)。[旧版板上记录](BOARD_TEST_REPORT.md) 不代表本版已完成真机验证。
