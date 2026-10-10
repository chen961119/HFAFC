#include "firmware_debug.h"

#if defined(COFLY_TEENSY_DEBUG)
#include "serial_ports.h"
#include <TeensyDebug.h>
#endif

void initializeFirmwareDebug() {
#if defined(COFLY_TEENSY_DEBUG)
  DebugSerial.begin(921600);
  // GDB 打开 USB 串口后再初始化，确保 USB 已完成枚举。
  while (!DebugSerial) {
    delay(10);
  }
  debug.begin(DebugSerial);
  // 继续运行后返回 main，执行正常初始化。
  halt_cpu();
#endif
}
