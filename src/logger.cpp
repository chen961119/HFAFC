#include "logger.h"
#include "serial_ports.h"
#include "control_state.h"
#include "control_allocation.h"
#include "flight_clock.h"
#include "interaircraft_comm.h"
#include "sensor_processing.h"
#include <SD.h>

namespace {

// SD 卡日志状态。
char filename_sd[20];
File dataFile;
bool sdReady = false;
unsigned int fileNumber = 0;
unsigned int logfreq = 50; // 记录频率Hz 注意要是2000的因数。
unsigned long lastLogTime = 0;
unsigned long fileCycle = 0;
String dataString = "";

// 扫描 SD 卡中 datalogNNN.txt 文件，选择下一个 1～999 的日志编号。
void findMaxFileNumber() {
  File root = SD.open("/");
  while (true) {
    File entry = root.openNextFile();
    if (!entry)
      break;

    String name = entry.name();
    if (name.startsWith("datalog") && name.endsWith(".txt")) {
      // 提取编号部分
      String numStr = name.substring(7, 10); // 从第8个字符开始取3位

      // 验证是否为纯数字
      bool valid = true;
      for (uint8_t i = 0; i < 3; i++) {
        if (!isDigit(numStr.charAt(i))) {
          valid = false;
          break;
        }
      }

      if (valid) {
        uint16_t num = numStr.toInt();
        if (num > fileNumber) {
          fileNumber = num;
        }
      }
    }
    entry.close();
  }
  root.close();

  fileNumber++;                               // 新文件编号递增
  fileNumber = constrain(fileNumber, 1, 999); // 限制在1-999之间
}

} // namespace

// 初始化 SD 卡，并创建带表头的新日志文件。
void initializeLogger() {
  // Some cards need time to return to their startup state after a warm reset.
  // Retry only during setup; never stall the flight loop trying to remount SD.
  for (unsigned attempt = 0; attempt < 3; ++attempt) {
    if (attempt) {
      SD.sdfs.end();
      delay(100);
    }
    sdReady = SD.begin(BUILTIN_SDCARD);
    if (sdReady) break;
  }
  if (!sdReady) {
    USBSerial.println("Card failed, or not present");
    return;
  } else {
    USBSerial.println("card initialized.");
  }
  findMaxFileNumber();
  sprintf(filename_sd, "datalog%03d.txt", fileNumber);
  dataFile = SD.open(filename_sd, FILE_WRITE);
  if (!dataFile) {
    USBSerial.println("Error opening datalog.txt");
  }
  dataFile.println(String(
      "TimeStamp(us),ROLL_IMU(deg),ROLL_Eq(deg),PITCH_IMU(deg),YAW_IMU(deg),"
      "ROLL_des(deg),PITCH_des_local(deg),YAW_des(deg),CH1_PWM,CH2_PWM,CH3_PWM,"
      "CH4_PWM,CH5_PWM,CH6_PWM,CH7_PWM,CH8_PWM,ail1_PWM,ail2_PWM,ele_PWM,"
      "thro_PWM,rudd_PWM,Bail1_control_us,Bail2_control_us,Bthro_control_us,Brudd_control_us,Cail1_control_us,"
      "Cail2_control_us,Cthro_control_us,Crudd_control_us,Dail1_control_us,Dail2_control_us,Dthro_control_us,Drudd_control_us,"
      "Eail1_control_us,Eail2_control_us,Ethro_control_us,Erudd_control_us,Fail1_control_us,Fail2_control_us,Fthro_control_us,"
      "Frudd_control_us,Gail1_control_us,Gail2_control_us,Gthro_control_us,Grudd_control_us,Pab(deg),Pac(deg),Pbd("
      "deg),Pce(deg),Pdf(deg),Peg(deg),Phiab_Mea(deg),Phiab_des(deg),"
      "phiac(deg),phibd(deg),phice(deg),phidf(deg),phieg(deg),Apitchsp,"
      "Bpitchsp,Cpitchsp,Dpitchsp,Epitchsp,Fpitchsp,Gpitchsp,Bpitch_raw,Cpitch_"
      "raw,Dpitch_raw,Epitch_raw,Fpitch_raw,Gpitch_raw,Bele_pwm,Cele_pwm,Dele_"
      "pwm,Eele_pwm,Fele_pwm,Gele_pwm,rollAB_rad_Qua,roll_IMU_EXT,pitch_IMU_"
      "EXT,yaw_IMU_EXT,invAccX_6050,AccY_6050,AccZ_6050,Gyro_X_6050,Gyro_Y_"
      "6050,Gyro_Z_6050,Gyro_X_EXT,Gyro_Y_EXT,Gyro_Z_EXT,phiab_PID,phiac_PID,"
      "phibd_PID,phice_PID,phidf_PID,phieg_PID,roll_PID,pitch_PID,airspeed_A,"
      "Strain_value1,Strain_value2,Strain_value3,Strain_value4,Strain_value5,"
      "AOA,AOS,TAS,dp,dq,dr,INDI_q_des,INDI_q_filt,INDI_dq_des,INDI_dq_used,"
      "INDI_delta_e_cmd_deg,INDI_delta_e_est_deg,INDI_pwm_cmd,"
      "MT6701_raw(deg),MT6701_relative(deg),MT6701_valid"));
  dataFile.flush();
  delay(10);
  USBSerial.print("新建日志文件：");
  USBSerial.println(filename_sd);
}

bool loggerSdReady() { return sdReady; }

unsigned int loggerFileNumber() { return fileNumber; }

// 按日志频率将单机状态写入 SD 文件，并定期刷新缓存。
void loggerSINGLE() {
  float invFreq = 1.0 / logfreq * 1000000.0;
  unsigned long checker = micros();

  if (checker - lastLogTime < invFreq)
    return;
  lastLogTime = checker;

  // dataFile = SD.open(filename_sd, FILE_WRITE);
  // 光打开不close 拔电就没了，但是每次都打开又close很浪费时间。
  // 将本次采样的传感器与控制状态拼接为日志记录。
  dataString =
      String(current_time) + "," + String(q0) + "," + String(q1) + "," +
      String(q2) + "," + String(q3) + "," + String(roll_IMU) + "," +
      String(pitch_IMU) + "," + String(yaw_IMU) + "," + String(roll_des) + "," +
      String(pitch_des_local) + "," + String(yaw_des) + "," +
      String(channel_1_pwm) + "," + String(channel_2_pwm) + "," +
      String(channel_3_pwm) + "," + String(channel_4_pwm) + "," +
      String(channel_5_pwm) + "," + String(ail1_PWM - pwm_channel1_trim) +
      "," + String(ail2_PWM - pwm_channel2_trim) + "," +
      String(ele_PWM - pwm_channel3_trim) + "," + String(thro_PWM) + "," +
      String(rudd_PWM) + "," + String(-GyroX_6050) + "," + String(GyroY_9250) +
      "," + String(GyroZ_9250) + "," + String(Pab) + "," +
      String(relativeAngle_ready) + "," + String(Phiab_des) + "," +
      String(-AccX_9250) + "," + String(AccY_9250) + "," + String(AccZ_9250);
  // USBSerial.println(dataString);
  dataFile.println(dataString);


  // 定期刷新 SD 写入缓存。

  if (millis() - fileCycle > 2000) { // 每2秒刷新缓存
    dataFile.flush();
    fileCycle = millis();
  }
}

// 按日志频率将编队传感器、控制和通信状态写入 SD 文件。
void loggerTEAM() {
  float invFreq = 1.0 / logfreq * 1000000.0;
  unsigned long checker = micros();

  if (checker - lastLogTime < invFreq)
    return;
  lastLogTime = checker;

  float pitch_des_local_log;
#if defined APLANE

#if defined EVEN
  pitch_des_local_log = pitch_des_local + central_pitch;
#elif defined ODD
  pitch_des_local_log = pitch_des_local;
#endif

#else
  pitch_des_local_log = pitch_des_local;

#endif
  A_pitch_sp = pitch_des_local_log;
  // dataFile = SD.open(filename_sd, FILE_WRITE);
  // 光打开不close 拔电就没了，但是每次都打开又close很浪费时间。
  // 将本次采样的传感器与控制状态拼接为日志记录。
  float Phiab_Mea_logger;
  float Phiac_Mea_logger;
  float Phibd_Mea_logger;
  float Phice_Mea_logger;
  float Phidf_Mea_logger;
  float Phieg_Mea_logger;

#if defined userotatesensor
  Phiab_Mea_logger = relativeAngle_ready; // phiab是A机自己测的。
  // Phiab_Mea=rollAB_rad_Qua;
  Phiac_Mea_logger = phiac;
  Phibd_Mea_logger = phibd;
  Phice_Mea_logger = phice;
  Phidf_Mea_logger = phidf;
  Phieg_Mea_logger = phieg;
#else
  Phiab_Mea_logger = phiB_raw - roll_IMU; //
  // Phiab_Mea=rollAB_rad_Qua;
  Phiac_Mea_logger = phiC_raw - roll_IMU; //
  Phibd_Mea_logger = phiD_raw - phiB_raw;
  Phice_Mea_logger = phiE_raw - phiC_raw;
  Phidf_Mea_logger = phiF_raw - phiD_raw;
  Phieg_Mea_logger = phiG_raw - phiE_raw;
#endif

  dataString =
      String(current_time) + "," + String(roll_IMU) + "," + String(roll_eq) +
      "," + String(pitch_IMU) + "," + String(yaw_IMU) + "," + String(roll_des) +
      "," + String(pitch_des_local_log) + "," + String(yaw_des) + "," +
      String(channel_1_pwm) + "," + String(channel_2_pwm) + "," +
      String(channel_3_pwm) + "," + String(channel_4_pwm) + "," +
      String(channel_5_pwm) + "," + String(channel_6_pwm) + "," +
      String(channel_7_pwm) + "," + String(channel_8_pwm) + "," +
      String(ail1_PWM - pwm_channel1_trim) + "," +
      String(ail2_PWM - pwm_channel2_trim) + "," +
      String(ele_PWM - pwm_channel3_trim) + "," + String(thro_PWM) + "," +
      String(rudd_PWM) + "," + String(Bail1_PWM) + "," + String(Bail2_PWM) +
      "," + String(Bthro_PWM) + "," + String(Brudd_PWM) + "," +
      String(Cail1_PWM) + "," + String(Cail2_PWM) + "," + String(Cthro_PWM) +
      "," + String(Crudd_PWM) + "," + String(Dail1_PWM) + "," +
      String(Dail2_PWM) + "," + String(Dthro_PWM) + "," + String(Drudd_PWM) +
      "," + String(Eail1_PWM) + "," + String(Eail2_PWM) + "," +
      String(Ethro_PWM) + "," + String(Erudd_PWM) + "," + String(Fail1_PWM) +
      "," + String(Fail2_PWM) + "," + String(Fthro_PWM) + "," +
      String(Frudd_PWM) + "," + String(Gail1_PWM) + "," + String(Gail2_PWM) +
      "," + String(Gthro_PWM) + "," + String(Grudd_PWM) + "," + String(Pab) +
      "," + String(Pac) + "," + String(Pbd) + "," + String(Pce) + "," +
      String(Pdf) + "," + String(Peg) + "," + String(Phiab_Mea_logger) + "," +
      String(Phiab_des) + "," + String(Phiac_Mea_logger) + "," +
      String(Phibd_Mea_logger) + "," + String(Phice_Mea_logger) + "," +
      String(Phidf_Mea_logger) + "," + String(Phieg_Mea_logger) + "," +
      String(A_pitch_sp) + "," + String(B_pitch_sp) + "," + String(C_pitch_sp) +
      "," + String(D_pitch_sp) + "," + String(E_pitch_sp) + "," +
      String(F_pitch_sp) + "," + String(G_pitch_sp) + "," + String(thetaB_raw) +
      "," + String(thetaC_raw) + "," + String(thetaD_raw) + "," +
      String(thetaE_raw) + "," + String(thetaF_raw) + "," + String(thetaG_raw) +
      "," + String(Bele_PWM) + "," +
      String(Cele_PWM) + "," +
      String(Dele_PWM) + "," +
      String(Eele_PWM) + "," +
      String(Fele_PWM) + "," +
      String(Gele_PWM) + "," + String(rollAB_rad_Qua) +
      "," + String(roll_IMU_EXT) + "," + String(pitch_IMU_EXT) + "," +
      String(yaw_IMU_EXT) + "," + String(-AccX_6050) + "," + String(AccY_6050) +
      "," + String(AccZ_6050) + "," + String(-GyroX_6050) + "," +
      String(GyroY_6050) + "," + String(GyroZ_6050) + "," + String(Gyro_X_EXT) +
      "," + String(Gyro_Y_EXT) + "," + String(Gyro_Z_EXT) + "," +
      String(Phiab_PID) + "," + String(Phiac_PID) + "," + String(Phibd_PID) +
      "," + String(Phice_PID) + "," + String(Phidf_PID) + "," +
      String(Phieg_PID) + "," + String(roll_PID) + "," + String(pitch_PID) +
      "," + String(airspeed_A) + "," + String(Strain_value1) + "," +
      String(Strain_value2) + "," + String(Strain_value3) + "," +
      String(Strain_value4) + "," + String(Strain_value5) + "," +
      String(airdata.aoa) + "," + String(airdata.aos) + "," +
      String(airdata.tas) + "," + String(dp) + "," + String(dq) + "," +
      String(dr) + "," + String(indi_pitch_q_des_log) + "," +
      String(indi_pitch_q_filt_log) + "," + String(indi_pitch_dq_des_log) +
      "," + String(indi_pitch_dq_used_log) + "," +
      String(indi_pitch_delta_e_cmd_deg_log) + "," +
      String(indi_pitch_delta_e_est_deg_log) + "," +
      String(indi_pitch_pwm_cmd_log) + "," +
      String(relativeAngle_raw) + "," + String(relativeAngle_ready) + "," +
      String(rotateSensorValid() ? 1 : 0);
  dataFile.println(dataString);

  // 文件可用时写入本次记录。

  // 定期刷新 SD 写入缓存。

  if (millis() - fileCycle > 2000) { // 每2秒刷新缓存
    dataFile.flush();

    fileCycle = millis();
  }
}

