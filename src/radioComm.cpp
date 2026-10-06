// 遥控接收模块：支持编译配置选定的 PPM、PWM、SBUS 或 DSM 接收机。
// 原始实现来自 Nicholas Rehm 的 dRehmFlight 项目，后续按本项目硬件接线调整。
#include <Arduino.h>
#include "radioComm.h"
#include <SBUS.h>  // SBUS 接收机驱动。
// 以下引脚用于 PWM 接收机；SBUS 实际使用 Serial2，DSM 使用 Serial3。
const int ch1Pin = 15; // 通道 1：滚转
const int ch2Pin = 16; // 通道 2：俯仰
const int ch3Pin = 17; // 通道 3：油门
const int ch4Pin = 20; // 通道 4：偏航
const int ch5Pin = 21; // 通道 5：油门切断
const int ch6Pin = 22; // 通道 6：辅助功能
const int PPM_Pin = 23;

static unsigned long rising_edge_start_1, rising_edge_start_2, rising_edge_start_3, rising_edge_start_4, rising_edge_start_5, rising_edge_start_6;
static unsigned long channel_1_raw, channel_2_raw, channel_3_raw, channel_4_raw, channel_5_raw, channel_6_raw;
static int ppm_counter = 0;
static unsigned long time_ms = 0;


#if defined USE_SBUS_RX
SBUS sbus(Serial2);
uint16_t sbusChannels[16];
bool sbusFailSafe;
bool sbusLostFrame;
#endif
#if defined USE_DSM_RX
DSM1024 DSM;
#endif

// 按编译配置初始化 PPM、PWM、SBUS 或 DSM 接收机及相应中断。
void radioSetup() {
  // PPM：单引脚中断，按相邻上升沿的间隔分离各通道。
  #if defined USE_PPM_RX
    // 配置中断输入引脚。
    pinMode(PPM_Pin, INPUT_PULLUP);
    delay(20);
    // 每次边沿变化由中断服务函数解析。
    attachInterrupt(digitalPinToInterrupt(PPM_Pin), getPPM, CHANGE);

  // PWM：每个通道分别测量高电平脉宽。
  #elif defined USE_PWM_RX
    // 配置各通道输入引脚。
    pinMode(ch1Pin, INPUT_PULLUP);
    pinMode(ch2Pin, INPUT_PULLUP);
    pinMode(ch3Pin, INPUT_PULLUP);
    pinMode(ch4Pin, INPUT_PULLUP);
    pinMode(ch5Pin, INPUT_PULLUP);
    pinMode(ch6Pin, INPUT_PULLUP);
    delay(20);
    // 为每个通道绑定边沿中断。
    attachInterrupt(digitalPinToInterrupt(ch1Pin), getCh1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ch2Pin), getCh2, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ch3Pin), getCh3, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ch4Pin), getCh4, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ch5Pin), getCh5, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ch6Pin), getCh6, CHANGE);
    delay(20);

  // SBUS 接收机。
  #elif defined USE_SBUS_RX
    sbus.begin();

  // DSM 接收机。
  #elif defined USE_DSM_RX
    Serial3.begin(115000);
  #else
    // 未配置接收机类型时不执行初始化。
  #endif
}

// 读取 PWM/PPM 中断缓存的指定通道脉宽；通道号为 1～6，返回值单位为 μs。
unsigned long getRadioPWM(int ch_num) {
  // 返回中断采集的原始通道脉宽，单位 μs；无效通道返回 0。
  unsigned long returnPWM = 0;
  
  if (ch_num == 1) {
    returnPWM = channel_1_raw;
  }
  else if (ch_num == 2) {
    returnPWM = channel_2_raw;
  }
  else if (ch_num == 3) {
    returnPWM = channel_3_raw;
  }
  else if (ch_num == 4) {
    returnPWM = channel_4_raw;
  }
  else if (ch_num == 5) {
    returnPWM = channel_5_raw;
  }
  else if (ch_num == 6) {
    returnPWM = channel_6_raw;
  }
  
  return returnPWM;
}

// DSM 串口回调：逐字节交给协议解析器。
// 接收 DSM 串口字节并交给 DSM 协议解析器。
void serialEvent3(void)
{
  #if defined USE_DSM_RX
    while (Serial3.available()) {
        DSM.handleSerialEvent(Serial3.read(), micros());
    }
  #endif
}



// PPM/PWM 中断服务函数：只记录边沿时间和脉宽，不执行控制计算。

// PPM 上升沿中断：用帧间长间隔同步，再把后续脉宽写入各通道缓存。
void getPPM() {
  unsigned long dt_ppm;
  int trig = digitalRead(PPM_Pin);
  if (trig==1) { // 仅在上升沿测量相邻脉冲间隔。
    dt_ppm = micros() - time_ms;
    time_ms = micros();

    
    if (dt_ppm > 5000) { // 长间隔表示新的一帧开始。
      ppm_counter = 0;
    }
  
    if (ppm_counter == 1) { // 第 1 通道。
      channel_1_raw = dt_ppm;
    }
  
    if (ppm_counter == 2) { // 第 2 通道。
      channel_2_raw = dt_ppm;
    }
  
    if (ppm_counter == 3) { // 第 3 通道。
      channel_3_raw = dt_ppm;
    }
  
    if (ppm_counter == 4) { // 第 4 通道。
      channel_4_raw = dt_ppm;
    }
  
    if (ppm_counter == 5) { // 第 5 通道。
      channel_5_raw = dt_ppm;
    }
  
    if (ppm_counter == 6) { // 第 6 通道。
      channel_6_raw = dt_ppm;
    }
    
    ppm_counter = ppm_counter + 1;
  }
}

// 通道 1 PWM 边沿中断：记录上升沿，并在下降沿保存脉宽（μs）。
void getCh1() {
  int trigger = digitalRead(ch1Pin);
  if(trigger == 1) {
    rising_edge_start_1 = micros();
  }
  else if(trigger == 0) {
    channel_1_raw = micros() - rising_edge_start_1;
  }
}

// 通道 2 PWM 边沿中断：记录上升沿，并在下降沿保存脉宽（μs）。
void getCh2() {
  int trigger = digitalRead(ch2Pin);
  if(trigger == 1) {
    rising_edge_start_2 = micros();
  }
  else if(trigger == 0) {
    channel_2_raw = micros() - rising_edge_start_2;
  }
}

// 通道 3 PWM 边沿中断：记录上升沿，并在下降沿保存脉宽（μs）。
void getCh3() {
  int trigger = digitalRead(ch3Pin);
  if(trigger == 1) {
    rising_edge_start_3 = micros();
  }
  else if(trigger == 0) {
    channel_3_raw = micros() - rising_edge_start_3;
  }
}

// 通道 4 PWM 边沿中断：记录上升沿，并在下降沿保存脉宽（μs）。
void getCh4() {
  int trigger = digitalRead(ch4Pin);
  if(trigger == 1) {
    rising_edge_start_4 = micros();
  }
  else if(trigger == 0) {
    channel_4_raw = micros() - rising_edge_start_4;
  }
}

// 通道 5 PWM 边沿中断：记录上升沿，并在下降沿保存脉宽（μs）。
void getCh5() {
  int trigger = digitalRead(ch5Pin);
  if(trigger == 1) {
    rising_edge_start_5 = micros();
  }
  else if(trigger == 0) {
    channel_5_raw = micros() - rising_edge_start_5;
  }
}

// 通道 6 PWM 边沿中断：记录上升沿，并在下降沿保存脉宽（μs）。
void getCh6() {
  int trigger = digitalRead(ch6Pin);
  if(trigger == 1) {
    rising_edge_start_6 = micros();
  }
  else if(trigger == 0) {
    channel_6_raw = micros() - rising_edge_start_6;
  }
}
