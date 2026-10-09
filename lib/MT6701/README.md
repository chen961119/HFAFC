# MT6701

按用户提供的 `aaaaa/src/main.cpp` 示例移植为 Arduino/Teensy 的只读驱动。

- 使用 `Wire1`：Teensy 4.1 SDA=17、SCL=16，与 OLED、MS4525 共用总线。
- Wire 使用 7 位地址 `0x06`；厂家示例的 `0x0C` 是带写方向位的 8 位地址。
- 按顺序读取 `0x03`、`0x04`，14 位无符号数据换算为 `raw * 360 / 16384` 度。
- 不修改芯片配置寄存器或烧写芯片 EEPROM；零点保存在飞控 EEPROM 地址 100。
- `readAngle()` 失败返回 false 并保持调用者原值；夹角跨过 0/360 度时归一化到 ±180 度。

`src/sensor_processing.cpp` 负责初始化、每周期采样和安装方向；读取失败后每隔 20 ms 重试，OLED 显示 `ERR`。传感器应使用与 Teensy 兼容的 3.3 V I²C 电平。
