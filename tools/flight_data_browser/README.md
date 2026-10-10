# CoFly Autopilot 飞行数据浏览器

参照 `flight_data_browser2.m` 的数据流程实现，读取飞控 SD 卡导出的数值日志，提供多文件曲线对比、子图、缩放/偏移、显示平滑、飞行区段估算、模式标注、所选时间范围 FFT、播放和 CSV 导出。读取与分析逻辑在 `data.py`，窗口与命令行入口在 `main.py`。

## 启动

Windows 安装带 Tcl/Tk 和 pip 的 Python 3.10 或更新版本，双击本目录 `start.cmd`。首次启动会创建本目录 `.venv` 并安装依赖，需要网络；以后直接复用环境。

也可在仓库根目录手动配置：

```powershell
py -3 -m venv tools/flight_data_browser/.venv
& ./tools/flight_data_browser/.venv/Scripts/python.exe -m pip install -r tools/flight_data_browser/requirements.txt
& ./tools/flight_data_browser/.venv/Scripts/python.exe tools/flight_data_browser/main.py
```

macOS/Linux 使用 `python3 -m venv tools/flight_data_browser/.venv`，后续解释器路径为 `tools/flight_data_browser/.venv/bin/python`，并安装系统对应的 Tk 支持。

## 界面操作

1. 浏览数据目录并加载所选文件，或“打开文件…”一次选择多个文件。扫描当前目录，子目录中的日志可用文件选择器加载。
2. 在文件树中 Ctrl/Shift 多选变量，曲线叠加到当前子图；搜索框按字段名筛选。添加子图后可以为新子图选择另一组变量。
3. 选中变量，输入缩放与偏移并“应用”，得到 `显示值 = 原值 × 缩放 + 偏移`。“恢复”只重置所选变量。
4. 使用 Matplotlib 工具栏缩放、平移和保存图像；“全时段”恢复横轴，“飞行区段”缩放到当前子图参考文件最长的估算飞行段。
5. 勾选“区段 FFT”，对当前子图变量在可见时间范围内分析；缩放后自动更新。平滑只改变曲线显示，FFT 使用经过缩放/偏移的原始采样，不使用绘图抽稀数据。
6. “播放/停止”在约 5 秒内回放当前曲线范围。“导出 CSV”导出同一文件所选变量在当前横轴区段的原始值，附带秒时间列；不应用显示变换和平滑。

每个文件以第一个有效样本为零点，**多机曲线没有自动校准绝对起飞时间或时钟差**。模式色带使用子图内路径排序的第一个文件作为参考，标题明确指出来源：红色 Manual、蓝色 Stab1、绿色 Stab2、灰色 Unknown。空速/油门推断的淡黄色飞行区段仅辅助浏览，不是飞行状态真值。

相同文件、大小、修改时间和读取选项不变时直接复用内存。文件读取在后台执行，其他窗口操作仍可使用；清空后未完成的旧任务不会重新填入界面。

## 命令行与复用

以下命令在本目录的环境中执行；把 `python` 换成 `.venv/Scripts/python.exe`（Windows）或 `.venv/bin/python`（macOS/Linux）。

```sh
python main.py datalog109.txt --summary
python main.py A/datalog109.txt B/datalog064.txt
python main.py datalog109.txt --fft "ROLL_IMU(deg)" --from 30 --to 60
python main.py datalog109.txt --export selected.csv --columns "ROLL_IMU(deg)" "PITCH_IMU(deg)" --from 30 --to 60
python main.py datalog109.txt --summary --time-column "TimeStamp(us)" --time-unit us
python main.py damaged.txt --summary --skip-bad-rows
```

`--summary` 输出 JSON：行列数、持续时间、实际采样率、内存、时间来源、读取提示、估算飞行段和模式段。可以用 `--encoding gb18030` 指定旧编码，用 `--delimiter ";"` 指定分隔符。找不到时间列时使用 `--fallback-hz`，默认 50 Hz，并明确报告该假设。

在其他 Python 程序中把工具目录加入模块路径后，可直接调用：

```python
from data import load_log, compute_spectrum

log = load_log("datalog109.txt")
roll = log.column("ROLL_IMU(deg)")
fft = compute_spectrum(log.time, roll, bounds=(30, 60))
print(log.sample_rate, fft.peak_frequency)
```

## 读取与分析优化

- 首行为表头，每条数值记录占一行；支持逗号、制表符、分号以及 UTF-8 BOM。括号单位保留，重复/空白列名自动生成不冲突的名字。
- 常规文件通过 pandas C 解析器分块读取（默认 32,768 行），同时校验每行列数，最终每个日志保存独立 float64 数组。不同长度的多机数据不拼成填满 NaN 的大矩阵。
- 表头与数据宽度不一致直接报错，避免旧版按列数截断后错配字段。无效数值/坏行默认报错；显式选择“跳过坏行”后才用宽容读取并报告物理行号。空白字段保留 NaN，不补零。宽容回退比正常路径慢，适用于损坏文件。
- 时间单位优先服从表头或显式设置；仅未标单位时按增量推断并提示。使用真实中位采样间隔估计采样率，识别 Teensy uint32 微秒计数器回绕。重复时间戳保留第一条，缺失时间戳删除对应行并提示；普通倒退/复位报错，不能静默排序成一次连续飞行。
- 显示按当前视窗保留每个区间的极小值、极大值和缺口，通常约 6,000 点。缩放后从原始数据重新抽取，避免均匀抽点漏掉窄脉冲；不修改原始数据，也不为了画平滑线插值到四倍点数。
- FFT 去均值，使用 Hann 窗和单边幅值归一化。根据所选区段的真实间隔处理抖动与短缺失，必要时重采样并标注；大于五个采样周期的缺口拒绝分析，需缩小区段。最低八个有效样本。
- 飞行区段优先参考空速，阈值使用 99% 分位数降低孤立尖峰影响，其次使用绝对油门 PWM。当前 `thro_PWM` 名称受支持，`*_control_us` 是有符号偏移，不能按 1150～1400 μs 油门阈值判断飞行。

全量原始数据仍保存在内存中，内存量约为 `行数 × 列数 × 8` 字节，加上时间轴和加载期间临时缓冲；分块解析不代表完全不占内存。数据必须是带表头的数值表，不支持二进制遥测或跨行文本单元格。当前固件 `loggerSINGLE()` 与 TEAM 共用表头但数据布局不同，列数不匹配时应先修正日志格式，工具不会猜测列含义。

## 验证

在仓库根目录执行：

```powershell
& ./tools/flight_data_browser/.venv/Scripts/python.exe -m unittest discover -s tools/flight_data_browser -p 'test_*.py' -v
```

数值测试覆盖时间单位、重复/缺失时间、uint32 回绕与复位、坏行与分块边界列数、重名表头、模式缺失值、正弦幅值、FFT 抖动/缺口、窄脉冲显示和原始值导出。界面集成测试在隐藏窗口验证后台多文件加载、变换、搜索、FFT、子图和空视窗。

2026-10-10 本机验证：13 项测试通过；七机实际日志（约 348 MB，均为 117 列）全部读取成功，一次顺序读取与预处理合计约 5.9 秒。其中约 57 MB 的 A 机日志包含 79,291 行，读取与预处理约 0.9 秒。该测量受硬盘和系统缓存影响，不是与 MATLAB 的速度对照。七机时间戳的中位采样率均约 45.45 Hz，程序保留实际时间及抖动提示。

MATLAB 参考文件作为实现参考读取，原脚本与原始飞行日志不修改，也不复制到仓库。
