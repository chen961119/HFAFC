#include "serial_ports.h"
#include "debug_print.h"
#include "control_state.h"
#include "flight_clock.h"
#include "interaircraft_comm.h"
#include "sensor_processing.h"

// 所有调试打印共用一个节流时间戳；同一周期调用多个打印函数时可能只输出第一个。
static unsigned long print_counter = 0;

// 输出本机当前油门控制偏移，单位为 μs。
static int32_t throttle_debug = 0;
int32_t &localThrottleDebugSetting() { return throttle_debug; }
void printLocalThrottle() {
  static unsigned long last = 0;
  if (throttle_debug == 0 || micros() - last < 100000UL) return;
  last = micros();
  const int room = USBSerial.availableForWrite();
  if (room < 80) return;
  USBSerial.printf("[THROTTLE] parent_received=%d control_us=%d\n",
                hasReceivedParentCommand() ? 1 : 0, Local_thro_control_us);
}

// 按共享时间戳节流输出遥控各通道 PWM。
void printRadioData() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    USBSerial.print(F(" CH1:"));
    USBSerial.print(channel_1_pwm);
    USBSerial.print(F(" CH2:"));
    USBSerial.print(channel_2_pwm);
    USBSerial.print(F(" CH3:"));
    USBSerial.print(channel_3_pwm);
    USBSerial.print(F(" CH4:"));
    USBSerial.print(channel_4_pwm);
    USBSerial.print(F(" CH5:"));
    USBSerial.print(channel_5_pwm);
    USBSerial.print(F(" CH6:"));
    USBSerial.println(channel_6_pwm);
  }
}

// 按共享时间戳节流输出油门与三轴期望状态。
void printDesiredState() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    USBSerial.print(F("thro_des:"));
    USBSerial.print(thro_des);
    USBSerial.print(F(" roll_des:"));
    USBSerial.print(roll_des);
    USBSerial.print(F(" pitch_des:"));
    USBSerial.print(pitch_des);
    USBSerial.print(F(" yaw_des:"));
    USBSerial.println(yaw_des);
  }
}

// 输出相邻机体夹角或原始姿态差，以及各子机滚转角速度。
void printConfigurationData() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
#if defined userotatesensor
    USBSerial.print(F("phiab:"));
    USBSerial.print(relativeAngle_ready);
    USBSerial.print(F("phiac:"));
    USBSerial.print(phiac);
    USBSerial.print(F("phibd:"));
    USBSerial.print(phibd);
    USBSerial.print(F("phice:"));
    USBSerial.print(phice);
    USBSerial.print(F("phidf:"));
    USBSerial.print(phidf);
    USBSerial.print(F("phieg:"));
    USBSerial.print(phieg);
#else
    USBSerial.print(F("phiab:"));
    USBSerial.print(Phiab_Mea);
    USBSerial.print(F("phiac:"));
    USBSerial.print(Phiac_Mea);
    USBSerial.print(F("phibd:"));
    USBSerial.print(Phibd_Mea);
    USBSerial.print(F("phice:"));
    USBSerial.print(Phice_Mea);
    USBSerial.print(F("phidf:"));
    USBSerial.print(Phidf_Mea);
    USBSerial.print(F("phieg:"));
    USBSerial.print(Phieg_Mea);
    USBSerial.print(F("phieq:"));
    USBSerial.print(roll_eq);
#endif
    USBSerial.print(F("Pb:"));
    USBSerial.print(GYRO_X_B);
    USBSerial.print(F("Pc:"));
    USBSerial.print(GYRO_X_C);
    USBSerial.print(F("Pd:"));
    USBSerial.print(GYRO_X_D);
    USBSerial.print(F("Pe:"));
    USBSerial.print(GYRO_X_E);
    USBSerial.print(F("Pf:"));
    USBSerial.print(GYRO_X_F);
    USBSerial.print(F("Pg:"));
    USBSerial.println(GYRO_X_G);
  }
}

// 输出各轴与构型控制器的 PID 结果，供串口调试。
void printPIDoutput() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    USBSerial.print(F("roll_PID:"));
    USBSerial.print(roll_PID);
    USBSerial.print(F(" pitch_PID:"));
    USBSerial.print(pitch_PID);
    USBSerial.print(F(" yaw_PID:"));
    USBSerial.print(yaw_PID);
    USBSerial.print(F(" Phiab_PID:"));
    USBSerial.print(Phiab_PID);
    USBSerial.print(F(" Phiac_PID:"));
    USBSerial.println(Phiac_PID);
    USBSerial.print(F(" Phibd_PID:"));
    USBSerial.print(Phibd_PID);
    USBSerial.print(F(" Phice_PID:"));
    USBSerial.println(Phice_PID);
  }
}

// 输出电机或油门相关控制指令，供串口调试。
void printMotorCommands() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    USBSerial.print(F("m1_command:"));
    USBSerial.print(m1_command_PWM);
    USBSerial.print(F(" m2_command:"));
    USBSerial.print(m2_command_PWM);
    USBSerial.print(F(" m3_command:"));
    USBSerial.print(m3_command_PWM);
    USBSerial.print(F(" m4_command:"));
    USBSerial.print(m4_command_PWM);
    USBSerial.print(F(" m5_command:"));
    USBSerial.print(m5_command_PWM);
    USBSerial.print(F(" m6_command:"));
    USBSerial.println(m6_command_PWM);
  }
}

// 输出各执行器 PWM 指令，供串口调试。
void printServoCommands() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    USBSerial.print(F("Aail1:"));
    USBSerial.print(ail1_PWM);
    USBSerial.print(F(" Aail2:"));
    USBSerial.print(ail2_PWM);
    USBSerial.print(F(" Aele:"));
    USBSerial.print(ele_PWM);
    USBSerial.print(F(" Athro:"));
    USBSerial.print(thro_PWM);
    USBSerial.print(F(" Arudd:"));
    USBSerial.print(rudd_PWM);
    USBSerial.print(F(" s6_command:"));
    USBSerial.print(s6_command_PWM);
    USBSerial.print(F(" s7_command:"));
    USBSerial.println(s7_command_PWM);
  }
}

// 输出主循环时间间隔 dt，单位为 μs；本函数不直接计算频率。
void printLoopRate() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    USBSerial.print(F("dt:"));
    USBSerial.println(dt * 1000000.0); // 一个微秒microsecons=1/1000000s
  }
}


// 输出内置 IMU 三轴角速度，单位为 °/s。
void printGyroData() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    USBSerial.print(F("GyroX_6050:"));
    USBSerial.print(GyroX_6050);
    USBSerial.print(F(" GyroY_6050:"));
    USBSerial.print(GyroY_6050);
    USBSerial.print(F(" GyroZ_6050:"));
    USBSerial.println(GyroZ_6050);

    USBSerial.print(F("GyroX_9250:"));
    USBSerial.print(GyroX_9250);
    USBSerial.print(F(" GyroY_9250:"));
    USBSerial.print(GyroY_9250);
    USBSerial.print(F(" GyroZ_9250:"));
    USBSerial.println(GyroZ_9250);
  }
}


// 输出内置 IMU 三轴加速度，单位为 g。
void printAccelData() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    USBSerial.print(F("AccX_6050:"));
    USBSerial.print(AccX_6050);
    USBSerial.print(F(" AccY_6050:"));
    USBSerial.print(AccY_6050);
    USBSerial.print(F(" AccZ_6050:"));
    USBSerial.println(AccZ_6050);

    USBSerial.print(F("AccX_9250:"));
    USBSerial.print(AccX_9250);
    USBSerial.print(F(" AccY_9250:"));
    USBSerial.print(AccY_9250);
    USBSerial.print(F(" AccZ_9250:"));
    USBSerial.println(AccZ_9250);
  }
}


// 输出磁力计三轴数据，单位沿用传感器处理模块的 μT。
void printMagData() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    USBSerial.print(F("MagX:"));
    USBSerial.print(MagX_9250);
    USBSerial.print(F(" MagY:"));
    USBSerial.print(MagY_9250);
    USBSerial.print(F(" MagZ:"));
    USBSerial.println(MagZ_9250);
  }
}


// 输出当前姿态解算的滚转、俯仰和偏航角，单位为度。
void printRollPitchYaw() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    USBSerial.print(F("roll:"));
    USBSerial.print(roll_IMU);
    USBSerial.print(F(" pitch:"));
    USBSerial.print(pitch_IMU);
    USBSerial.print(F(" yaw:"));
    USBSerial.println(yaw_IMU);
  }
}


// 输出姿态估计使用的四元数分量。
void printQuaternion() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    USBSerial.print(F("q0: "));
    USBSerial.print(q0);
    USBSerial.print(F(" q1: "));
    USBSerial.print(q1);
    USBSerial.print(F(" q2: "));
    USBSerial.print(q2);
    USBSerial.print(F(" q3: "));
    USBSerial.println(q3);
  }
}
