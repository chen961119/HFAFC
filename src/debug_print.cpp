#include "debug_print.h"
#include "control_state.h"
#include "flight_clock.h"
#include "interaircraft_comm.h"
#include "sensor_processing.h"

static unsigned long print_counter = 0;

void printLocalThrottle() { Serial.println(Local_thro_PWM); }

void printRadioData() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F(" CH1:"));
    Serial.print(channel_1_pwm);
    Serial.print(F(" CH2:"));
    Serial.print(channel_2_pwm);
    Serial.print(F(" CH3:"));
    Serial.print(channel_3_pwm);
    Serial.print(F(" CH4:"));
    Serial.print(channel_4_pwm);
    Serial.print(F(" CH5:"));
    Serial.print(channel_5_pwm);
    Serial.print(F(" CH6:"));
    Serial.println(channel_6_pwm);
  }
}

void printDesiredState() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("thro_des:"));
    Serial.print(thro_des);
    Serial.print(F(" roll_des:"));
    Serial.print(roll_des);
    Serial.print(F(" pitch_des:"));
    Serial.print(pitch_des);
    Serial.print(F(" yaw_des:"));
    Serial.println(yaw_des);
  }
}

void printConfigurationData() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
#if defined userotatesensor
    Serial.print(F("phiab:"));
    Serial.print(relativeAngle_ready);
    Serial.print(F("phiac:"));
    Serial.print(phiac);
    Serial.print(F("phibd:"));
    Serial.print(phibd);
    Serial.print(F("phice:"));
    Serial.print(phice);
    Serial.print(F("phidf:"));
    Serial.print(phidf);
    Serial.print(F("phieg:"));
    Serial.print(phieg);
#else
    Serial.print(F("phiab:"));
    Serial.print(Phiab_Mea);
    Serial.print(F("phiac:"));
    Serial.print(Phiac_Mea);
    Serial.print(F("phibd:"));
    Serial.print(Phibd_Mea);
    Serial.print(F("phice:"));
    Serial.print(Phice_Mea);
    Serial.print(F("phidf:"));
    Serial.print(Phidf_Mea);
    Serial.print(F("phieg:"));
    Serial.print(Phieg_Mea);
    Serial.print(F("phieq:"));
    Serial.print(roll_eq);
#endif
    Serial.print(F("Pb:"));
    Serial.print(GYRO_X_B);
    Serial.print(F("Pc:"));
    Serial.print(GYRO_X_C);
    Serial.print(F("Pd:"));
    Serial.print(GYRO_X_D);
    Serial.print(F("Pe:"));
    Serial.print(GYRO_X_E);
    Serial.print(F("Pf:"));
    Serial.print(GYRO_X_F);
    Serial.print(F("Pg:"));
    Serial.println(GYRO_X_G);
  }
}

void printPIDoutput() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("roll_PID:"));
    Serial.print(roll_PID);
    Serial.print(F(" pitch_PID:"));
    Serial.print(pitch_PID);
    Serial.print(F(" yaw_PID:"));
    Serial.print(yaw_PID);
    Serial.print(F(" Phiab_PID:"));
    Serial.print(Phiab_PID);
    Serial.print(F(" Phiac_PID:"));
    Serial.println(Phiac_PID);
    Serial.print(F(" Phibd_PID:"));
    Serial.print(Phibd_PID);
    Serial.print(F(" Phice_PID:"));
    Serial.println(Phice_PID);
  }
}

void printMotorCommands() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("m1_command:"));
    Serial.print(m1_command_PWM);
    Serial.print(F(" m2_command:"));
    Serial.print(m2_command_PWM);
    Serial.print(F(" m3_command:"));
    Serial.print(m3_command_PWM);
    Serial.print(F(" m4_command:"));
    Serial.print(m4_command_PWM);
    Serial.print(F(" m5_command:"));
    Serial.print(m5_command_PWM);
    Serial.print(F(" m6_command:"));
    Serial.println(m6_command_PWM);
  }
}

void printServoCommands() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("Aail1:"));
    Serial.print(Aail1_PWM);
    Serial.print(F(" Aail2:"));
    Serial.print(Aail2_PWM);
    Serial.print(F(" Aele:"));
    Serial.print(Aele_PWM);
    Serial.print(F(" Athro:"));
    Serial.print(Athro_PWM);
    Serial.print(F(" Arudd:"));
    Serial.print(Arudd_PWM);
    Serial.print(F(" s6_command:"));
    Serial.print(s6_command_PWM);
    Serial.print(F(" s7_command:"));
    Serial.println(s7_command_PWM);
  }
}

void printLoopRate() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("dt:"));
    Serial.println(dt * 1000000.0); // 一个微秒microsecons=1/1000000s
  }
}


void printGyroData() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("GyroX_6050:"));
    Serial.print(GyroX_6050);
    Serial.print(F(" GyroY_6050:"));
    Serial.print(GyroY_6050);
    Serial.print(F(" GyroZ_6050:"));
    Serial.println(GyroZ_6050);

    Serial.print(F("GyroX_9250:"));
    Serial.print(GyroX_9250);
    Serial.print(F(" GyroY_9250:"));
    Serial.print(GyroY_9250);
    Serial.print(F(" GyroZ_9250:"));
    Serial.println(GyroZ_9250);
  }
}


void printAccelData() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("AccX_6050:"));
    Serial.print(AccX_6050);
    Serial.print(F(" AccY_6050:"));
    Serial.print(AccY_6050);
    Serial.print(F(" AccZ_6050:"));
    Serial.println(AccZ_6050);

    Serial.print(F("AccX_9250:"));
    Serial.print(AccX_9250);
    Serial.print(F(" AccY_9250:"));
    Serial.print(AccY_9250);
    Serial.print(F(" AccZ_9250:"));
    Serial.println(AccZ_9250);
  }
}


void printMagData() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("MagX:"));
    Serial.print(MagX_9250);
    Serial.print(F(" MagY:"));
    Serial.print(MagY_9250);
    Serial.print(F(" MagZ:"));
    Serial.println(MagZ_9250);
  }
}


void printRollPitchYaw() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("roll:"));
    Serial.print(roll_IMU);
    Serial.print(F(" pitch:"));
    Serial.print(pitch_IMU);
    Serial.print(F(" yaw:"));
    Serial.println(yaw_IMU);
  }
}


void printQuaternion() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("q0: "));
    Serial.print(q0);
    Serial.print(F(" q1: "));
    Serial.print(q1);
    Serial.print(F(" q2: "));
    Serial.print(q2);
    Serial.print(F(" q3: "));
    Serial.println(q3);
  }
}
