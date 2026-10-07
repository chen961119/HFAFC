#include "serial_ports.h"
#include "human_interface.h"
#include "control_state.h"
#include "control_allocation.h"
#include "flight_clock.h"
#include <Adafruit_SSD1306.h>
#include "logger.h"
#include <Wire.h>
#include "interaircraft_comm.h"
#include "sensor_processing.h"

namespace {
// 按键相关
const int DEBOUNCE_DELAY = 10;              // 消抖时间(ms)
const unsigned long SHORT_PRESS_TIME = 50;  // 短按时间(ms)
const unsigned long LONG_PRESS_TIME = 1000; // 长按时间(ms)
bool buttonActive = false;
bool longPressActive = false;
unsigned long buttonPressTime = 0;

bool buttonActive1 = false; // 第二个按键
bool longPressActive1 = false;
unsigned long buttonPressTime1 = 0;


// OLED 显示器状态与初始化参数。
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire1, OLED_RESET);


// 特殊图案
const unsigned char logo[] PROGMEM = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xf0, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f,
    0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x1f, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0xff, 0x80, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f,
    0xff, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0xff, 0xff, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xff, 0xff, 0x80, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xff,
    0xff, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x7f, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x01, 0xff, 0x81, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0xff, 0x81, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f,
    0xff, 0x81, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xc0, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x1f, 0xff, 0x81, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0xff, 0x81, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f,
    0xff, 0x81, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x80, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x1f, 0xff, 0x81, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0xff, 0x81, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f,
    0xff, 0x81, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x1f, 0xff, 0x81, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0xff, 0x81, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f,
    0xff, 0x81, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf0, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x1f, 0xff, 0x81, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf0,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0xff, 0x81, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f,
    0xff, 0x81, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x03, 0xff, 0x81, 0xff, 0xff, 0xff, 0xff, 0xf8, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};


unsigned long lastdispTime = 0;
unsigned int displayfreq = 10; // 显示屏帧率。
bool isdisplay = 1;

unsigned long blink_counter, blink_delay;
bool blinkAlternate;

} // namespace

// 根据当前飞行模式设置 32、33 号模式指示灯电平。
void displayFlightModeIndicators(FlightMode mode) {
  if (mode == STABILIZE_MODE) {
    digitalWrite(33, HIGH);
    digitalWrite(32, HIGH);
  } else if (mode == STABLIZE_MODE_NO_I) {
    digitalWrite(33, LOW);
    digitalWrite(32, HIGH);
  } else if (mode == MANUAL_MODE) {
    digitalWrite(33, HIGH);
    digitalWrite(32, LOW);
  }
}

// 启动 USB 调试串口。
void beginHumanInterfaceLinks() {
  USBSerial.begin(500000);
}

// 初始化按键和 OLED。
void initializeHumanInterface() {
  pinMode(32, OUTPUT);
  pinMode(33, OUTPUT);
  pinMode(36, OUTPUT);
  pinMode(36, INPUT_PULLUP);
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    USBSerial.println(F("SSD1306 allocation failed"));
  }
  display.clearDisplay();
  displaythumbsup();

}

// 按编译配置在 OLED 上显示本机 A～G 编号。
void displayAircraftIdentity() {
#if defined APLANE
  displayID("A");
#elif defined BPLANE
  displayID("B");
#elif defined CPLANE
  displayID("C");
#elif defined DPLANE
  displayID("D");
#elif defined EPLANE
  displayID("E");
#elif defined FPLANE
  displayID("F");
#elif defined GPLANE
  displayID("G");
#endif
  if (!imuCalibrationValid()) displayAttitude();
}

// 按当前闪烁间隔更新板载 LED，供观察主循环是否仍在运行。
void loopBlink() {
  // 以长短交替的节奏闪烁板载 LED，表示主循环仍在运行。
  if (current_time - blink_counter > blink_delay) {
    blink_counter = micros();
    digitalWrite(13, blinkAlternate); // 13 号引脚连接板载 LED。

    if (blinkAlternate == 1) {
      blinkAlternate = 0;
      blink_delay = 100000;
    } else if (blinkAlternate == 0) {
      blinkAlternate = 1;
      blink_delay = 2000000;
    }
  }
}

// 按指定次数闪烁板载 LED；upTime、downTime 单位为 ms。
void setupBlink(int numBlinks, int upTime, int downTime) {
  // 初始化阶段按指定次数和亮灭时间闪烁；时间单位为 ms。
  for (int j = 1; j <= numBlinks; j++) {
    digitalWrite(13, LOW);
    delay(downTime);
    digitalWrite(13, HIGH);
    delay(upTime);
  }
}


//=========================================================================================//

// 显示和打包辅助函数。

// 把传入的机体编号文字绘制到 OLED 指定位置并立即刷新。
void displayID(char *ss) {
  // display.clearDisplay();
  display.setTextSize(2, 2);           // 使用双倍字体。
  display.setTextColor(SSD1306_WHITE); // 白色文字。
  display.setCursor(100, 12);          // 指定文本起点。
  display.println(ss);                 // 直接用指针的形式。
  display.display();
}

// 把 SD 卡状态文字绘制到 OLED 指定位置并立即刷新。
void displaySD(char *ss) {
  // display.clearDisplay();
  display.setTextSize(1);              // 使用默认字体大小。
  display.setTextColor(SSD1306_WHITE); // 白色文字。
  display.setCursor(100, 0);           // 指定文本起点。
  display.println(ss);                 // 直接用指针的形式。
  // display.println(F("sdsds")); //用字符串
  display.display();
}

// 将当前日志编号写入 OLED 显示缓冲区。
void displayfilenum() {
  if (!loggerSdReady()) {
    displaySD("MISS");
  }
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE); // 白色文字。
  display.setCursor(100, 0);
  display.print(loggerFileNumber());
  display.print((char)247); // 度符号°
}

// 按显示频率刷新相对转角及姿态角；关闭显示时直接返回。
void displayAttitude() {

  if (!isdisplay && imuCalibrationValid()) {
    return;
  }
  float invFreq = 1.0 / displayfreq * 1000000.0;
  unsigned long checker = micros();

  if (checker - lastdispTime < invFreq)
    return;
  lastdispTime = checker;

  // 提醒占用左侧传感器区域，右侧日志编号与机体编号保持可见。
  display.fillRect(0, 0, 96, 32, SSD1306_BLACK);
  if (!imuCalibrationValid()) {
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print("IMU CAL");
    display.setCursor(0, 8);
    display.print("REQUIRED");
    display.setCursor(0, 16);
    display.print("Keep flat/still");
    display.display();
    return;
  }

  // 第一行：相对滚转角
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE); // 白色文字。
  display.fillRect(30, 0, 50, 8, SSD1306_BLACK);
  display.setCursor(0, 0);
  display.print("Relat: ");
  display.print(relativeAngle_ready, 1);
  display.print((char)247); // 度符号°

  // 第二行：滚转角
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE); // 白色文字。
  display.fillRect(30, 8, 50, 8, SSD1306_BLACK);
  display.setCursor(0, 8);
  display.print("Roll: ");
  display.print(roll_IMU, 1);
  display.print((char)247); // 度符号°

  // 第三行：俯仰角
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE); // 白色文字。
  display.fillRect(30, 16, 50, 8, SSD1306_BLACK);
  display.setCursor(0, 16);
  display.print("Pitch:");
  display.print(pitch_IMU, 1);
  display.print((char)247);

  // 第四行：偏航角速度
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE); // 白色文字。
  display.fillRect(30, 24, 50, 8, SSD1306_BLACK);
  display.setCursor(0, 24);
  display.print("Yaw:  ");
  display.print(yaw_IMU, 1);
  display.print((char)247);

  // 图形化指示（简易人工地平仪）
  // drawArtificialHorizon(roll, pitch);

  display.display();
}

// 显示启动位图，等待 1 秒后清空 OLED 缓冲区。
void displaythumbsup() {
  display.drawBitmap(0,       // 居中X位置
                     0,       // 居中Y位置
                     logo,    // 位图数据
                     128, 32, // 宽度和高度
                     WHITE    // 颜色
  );

  // 显示绘制的内容
  display.display();

  delay(1000);
  display.clearDisplay();
}

// 处理 36、31 号按键：短按转角清零，长按切换显示或触发 IMU 标定。
// 使用 millis() 判断按压时长，触发校准时会阻塞主循环。
void ProcessButtonState() {

  int buttonState = digitalRead(36);
  int buttonState1 = digitalRead(31);

  // 2. 检测按键按下（下降沿）
  if (buttonState == LOW && !buttonActive) {
    buttonActive = true;
    buttonPressTime = millis();
    delay(DEBOUNCE_DELAY); // 消抖延迟
    USBSerial.println("按键按下");
  }

  // 3. 检测按键释放（上升沿）
  if (buttonState == HIGH && buttonActive) {
    buttonActive = false;

    // 判断是短按还是长按释放
    if (millis() - buttonPressTime < LONG_PRESS_TIME) {
      if (millis() - buttonPressTime >= SHORT_PRESS_TIME) {
        ResetRotateSensor();
      }
      // 如果短于SHORT_PRESS_TIME，不视为有效短按
    }
    USBSerial.println("按键释放");
  }

  // 4. 检测长按（持续按下）
  if (buttonActive && !longPressActive &&
      (millis() - buttonPressTime >= LONG_PRESS_TIME)) {
    longPressActive = true;
    USBSerial.println("长按一次");
    isdisplay = !isdisplay;
  }

  // 5. 重置长按状态
  if (buttonState == HIGH && longPressActive) {
    longPressActive = false;
  }

  // 第二个按键

  // 2. 检测按键按下（下降沿）
  if (buttonState1 == LOW && !buttonActive1) {
    buttonActive1 = true;
    buttonPressTime1 = millis();
    delay(DEBOUNCE_DELAY); // 消抖延迟
    USBSerial.println("按键2按下");
  }

  // 3. 检测按键释放（上升沿）
  if (buttonState1 == HIGH && buttonActive1) {
    buttonActive1 = false;
    USBSerial.println("按键2释放");
  }

  // 4. 检测长按（持续按下）
  if (buttonActive1 && !longPressActive1 &&
      (millis() - buttonPressTime1 >= LONG_PRESS_TIME)) {
    longPressActive1 = true;
    // USBSerial.println("长按一次");
    //  第一行：相对滚转角
    USBSerial.println("3s后开始校准IMU");
    // delay(500);
    USBSerial.println("2s开始校准IMU");
    // delay(500);
    USBSerial.println("1s开始校准IMU");
    // delay(500);
    calculate_IMU_error();
    USBSerial.println("校准IMU完成");
    // delay(500);
  }

  // 5. 重置长按状态
  if (buttonState1 == HIGH && longPressActive1) {
    longPressActive1 = false;
  }
}
