#include "human_interface.h"
#include "control_state.h"
#include "control_allocation.h"
#include "flight_clock.h"
#include <Adafruit_SSD1306.h>
#include <SD.h>
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


// SETUP OLED
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


// 数传
float configuration_tele[6];
float attitude_tele[3];


// SD log
char filename_sd[20];
File dataFile;
unsigned int fileNumber = 0;
unsigned long sd_counter = 0;
unsigned int logfreq = 50; // 记录频率Hz 注意要是2000的因数。
unsigned long lastLogTime = 0;
unsigned long fileCycle = 0;
String dataString = "";
unsigned long lastdispTime = 0;
unsigned int displayfreq = 10; // 显示屏帧率。
bool isdisplay = 1;
unsigned long lasttelemetryTime = 0;

// 频率
unsigned int Freqtelemetry = 5;     // 数传频率


unsigned long blink_counter, blink_delay;
bool blinkAlternate;

} // namespace

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

void beginHumanInterfaceLinks() {
  Serial.begin(500000);
  Serial8.begin(115200);
}

void initializeHumanInterface() {
  pinMode(32, OUTPUT);
  pinMode(33, OUTPUT);
  pinMode(36, OUTPUT);
  pinMode(36, INPUT_PULLUP);
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
  }
  display.clearDisplay();
  displaythumbsup();

  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("Card failed, or not present");
    displaySD("MISS");
  } else {
    Serial.println("card initialized.");
  }
  findMaxFileNumber();
  sprintf(filename_sd, "datalog%03d.txt", fileNumber);
  dataFile = SD.open(filename_sd, FILE_WRITE);
  if (!dataFile) {
    Serial.println("Error opening datalog.txt");
  }
  dataFile.println(String(
      "TimeStamp(us),ROLL_IMU(deg),ROLL_Eq(deg),PITCH_IMU(deg),YAW_IMU(deg),"
      "ROLL_des(deg),PITCH_des_local(deg),YAW_des(deg),CH1_PWM,CH2_PWM,CH3_PWM,"
      "CH4_PWM,CH5_PWM,CH6_PWM,CH7_PWM,CH8_PWM,Aail1_PWM,Aail2_PWM,Aele_PWM,"
      "Athro_PWM,Arudd_PWM,Bail1_PWM,Bail2_PWM,Bthro_PWM,Brudd_PWM,Cail1_PWM,"
      "Cail2_PWM,Cthro_PWM,Crudd_PWM,Dail1_PWM,Dail2_PWM,Dthro_PWM,Drudd_PWM,"
      "Eail1_PWM,Eail2_PWM,Ethro_PWM,Erudd_PWM,Fail1_PWM,Fail2_PWM,Fthro_PWM,"
      "Frudd_PWM,Gail1_PWM,Gail2_PWM,Gthro_PWM,Grudd_PWM,Pab(deg),Pac(deg),Pbd("
      "deg),Pce(deg),Pdf(deg),Peg(deg),relativeAngle_ready(deg),Phiab_des(deg),"
      "phiac(deg),phibd(deg),phice(deg),phidf(deg),phieg(deg),Apitchsp,"
      "Bpitchsp,Cpitchsp,Dpitchsp,Epitchsp,Fpitchsp,Gpitchsp,Bpitch_raw,Cpitch_"
      "raw,Dpitch_raw,Epitch_raw,Fpitch_raw,Gpitch_raw,Bele_pwm,Cele_pwm,Dele_"
      "pwm,Eele_pwm,Fele_pwm,Gele_pwm,rollAB_rad_Qua,roll_IMU_EXT,pitch_IMU_"
      "EXT,yaw_IMU_EXT,invAccX_6050,AccY_6050,AccZ_6050,Gyro_X_6050,Gyro_Y_"
      "6050,Gyro_Z_6050,Gyro_X_EXT,Gyro_Y_EXT,Gyro_Z_EXT,phiab_PID,phiac_PID,"
      "phibd_PID,phice_PID,phidf_PID,phieg_PID,roll_PID,pitch_PID,airspeed_A,"
      "Strain_value1,Strain_value2,Strain_value3,Strain_value4,Strain_value5,"
      "AOA,AOS,TAS,dp,dq,dr,INDI_q_des,INDI_q_filt,INDI_dq_des,INDI_dq_used,"
      "INDI_delta_e_cmd_deg,INDI_delta_e_est_deg,INDI_pwm_cmd"));
  dataFile.flush();
  delay(10);
  Serial.print("新建日志文件：");
  Serial.println(filename_sd);
  displayfilenum();
}

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
}

void loopBlink() {
  // DESCRIPTION: Blink LED on board to indicate main loop is running
  /*
   * It looks cool.
   */
  if (current_time - blink_counter > blink_delay) {
    blink_counter = micros();
    digitalWrite(13, blinkAlternate); // Pin 13 is built in LED

    if (blinkAlternate == 1) {
      blinkAlternate = 0;
      blink_delay = 100000;
    } else if (blinkAlternate == 0) {
      blinkAlternate = 1;
      blink_delay = 2000000;
    }
  }
}

void setupBlink(int numBlinks, int upTime, int downTime) {
  // DESCRIPTION: Simple function to make LED on board blink as desired
  for (int j = 1; j <= numBlinks; j++) {
    digitalWrite(13, LOW);
    delay(downTime);
    digitalWrite(13, HIGH);
    delay(upTime);
  }
}


void loggerSINGLE() {
  float invFreq = 1.0 / logfreq * 1000000.0;
  unsigned long checker = micros();

  if (checker - lastLogTime < invFreq)
    return;
  lastLogTime = checker;

  // dataFile = SD.open(filename_sd, FILE_WRITE);
  // 光打开不close 拔电就没了，但是每次都打开又close很浪费时间。
  //  read three sensors and append to the string:
  dataString =
      String(current_time) + "," + String(q0) + "," + String(q1) + "," +
      String(q2) + "," + String(q3) + "," + String(roll_IMU) + "," +
      String(pitch_IMU) + "," + String(yaw_IMU) + "," + String(roll_des) + "," +
      String(pitch_des_local) + "," + String(yaw_des) + "," +
      String(channel_1_pwm) + "," + String(channel_2_pwm) + "," +
      String(channel_3_pwm) + "," + String(channel_4_pwm) + "," +
      String(channel_5_pwm) + "," + String(Aail1_PWM - pwm_channel1_trim) +
      "," + String(Aail2_PWM - pwm_channel2_trim) + "," +
      String(Aele_PWM - pwm_channel3_trim) + "," + String(Athro_PWM) + "," +
      String(Arudd_PWM) + "," + String(-GyroX_6050) + "," + String(GyroY_9250) +
      "," + String(GyroZ_9250) + "," + String(Pab) + "," +
      String(relativeAngle_ready) + "," + String(Phiab_des) + "," +
      String(-AccX_9250) + "," + String(AccY_9250) + "," + String(AccZ_9250);
  // Serial.println(dataString);
  dataFile.println(dataString);

  // if the file is available, write to it:
  /*
  if (dataFile) {
    dataFile.println(dataString);
    if(++sd_counter >= 50) { //每50次写入刷新一次
      dataFile.flush();
      sd_counter = 0;
    }
     //dataFile.close();
    // print to the serial port too:
    //Serial.println(dataString);
  }
  */

  // 在loop()最后添加定期关闭/重新打开

  if (millis() - fileCycle > 2000) { // 每2秒重新打开
    dataFile.flush();
    /*
  dataFile.close();
  dataFile = SD.open(filename_sd, FILE_WRITE);
  if(!SD.open(filename_sd, FILE_WRITE))
  {
    display.fillRect(0, 0, 120, 8, SSD1306_BLACK);
    displaySD("BLACK BOX MISS");
    }
    */
    fileCycle = millis();
  }
}

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
  //  read three sensors and append to the string:
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
      String(Aail1_PWM - pwm_channel1_trim) + "," +
      String(Aail2_PWM - pwm_channel2_trim) + "," +
      String(Aele_PWM - pwm_channel3_trim) + "," + String(Athro_PWM) + "," +
      String(Arudd_PWM) + "," + String(Bail1_PWM) + "," + String(Bail2_PWM) +
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
      "," + String(Bele_PWM - pwm_channel3B_trim) + "," +
      String(Cele_PWM - pwm_channel3C_trim) + "," +
      String(Dele_PWM - pwm_channel3D_trim) + "," +
      String(Eele_PWM - pwm_channel3E_trim) + "," +
      String(Fele_PWM - pwm_channel3F_trim) + "," +
      String(Gele_PWM - pwm_channel3G_trim) + "," + String(rollAB_rad_Qua) +
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
      String(indi_pitch_pwm_cmd_log);
  dataFile.println(dataString);

  // if the file is available, write to it:

  // 在loop()最后添加定期关闭/重新打开

  if (millis() - fileCycle > 2000) { // 每2秒重新打开
    dataFile.flush();

    fileCycle = millis();
  }
}

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

//=========================================================================================//

// HELPER FUNCTIONS

void displayID(char *ss) {
  // display.clearDisplay();
  display.setTextSize(2, 2);           // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.setCursor(100, 12);          // Start at top-left corner
  display.println(ss);                 // 直接用指针的形式。
  display.display();
}

void displaySD(char *ss) {
  // display.clearDisplay();
  display.setTextSize(1);              // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.setCursor(100, 0);           // Start at top-left corner
  display.println(ss);                 // 直接用指针的形式。
  // display.println(F("sdsds")); //用字符串
  display.display();
}

void displayfilenum() {
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.setCursor(100, 0);
  display.print(fileNumber);
  display.print((char)247); // 度符号°
}

void displayAttitude() {

  if (!isdisplay) {
    return;
  }
  float invFreq = 1.0 / displayfreq * 1000000.0;
  unsigned long checker = micros();

  if (checker - lastdispTime < invFreq)
    return;
  lastdispTime = checker;

  // 第一行：相对滚转角
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.fillRect(30, 0, 50, 8, SSD1306_BLACK);
  display.setCursor(0, 0);
  display.print("Relat: ");
  display.print(relativeAngle_ready, 1);
  display.print((char)247); // 度符号°

  // 第二行：滚转角
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.fillRect(30, 8, 50, 8, SSD1306_BLACK);
  display.setCursor(0, 8);
  display.print("Roll: ");
  display.print(roll_IMU, 1);
  display.print((char)247); // 度符号°

  // 第三行：俯仰角
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.fillRect(30, 16, 50, 8, SSD1306_BLACK);
  display.setCursor(0, 16);
  display.print("Pitch:");
  display.print(pitch_IMU, 1);
  display.print((char)247);

  // 第四行：偏航角速度
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.fillRect(30, 24, 50, 8, SSD1306_BLACK);
  display.setCursor(0, 24);
  display.print("Yaw:  ");
  display.print(yaw_IMU, 1);
  display.print((char)247);

  // 图形化指示（简易人工地平仪）
  // drawArtificialHorizon(roll, pitch);

  display.display();
}

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

void telemetry() // 主机数传
{
  // 数据包结构：头(0x55) + 类型(0x71) + 8个int16_t(各2字节) + 校验和 = 21字节
  uint8_t buffer[19];
  uint8_t pos = 0;
  uint8_t checksum = 0;

  float invFreq = 1.0 / Freqtelemetry * 1000000.0;
  unsigned long checker3 = micros();

  if (checker3 - lasttelemetryTime < invFreq)
    return;
  lasttelemetryTime = checker3;

#if defined userotatesensor
  configuration_tele[0] = relativeAngle_ready; // phiab是A机自己测的。
  configuration_tele[1] = phiac;
  configuration_tele[2] = phibd;
  configuration_tele[3] = phice;
  configuration_tele[4] = phidf;
  configuration_tele[5] = phieg;
#else
  configuration_tele[0] = phiB_raw - roll_IMU; //
  configuration_tele[1] = phiC_raw - roll_IMU; //
  configuration_tele[2] = phiD_raw - phiB_raw;
  configuration_tele[3] = phiE_raw - phiC_raw;
  configuration_tele[4] = phiF_raw - phiD_raw;
  configuration_tele[5] = phiG_raw - phiE_raw;
#endif

  attitude_tele[0] = roll_eq;
  attitude_tele[1] = pitch_IMU;

  //  float anglerand = random(-200000, 200001) / 1000.0;  // 范围 -200.000 ~
  //  +200.000

  // 1. 数据头
  buffer[pos++] = 0x55;
  checksum += 0x55;
  buffer[pos++] = 0x71;
  checksum += 0x71; // 新类型标识

  // 2. 打包相对转角6个 (转换为int16_t 一位小数)
  for (int i = 0; i < 6; i++) {
    int16_t val = configuration_tele[i] * 10.0f; // 放大10倍保留1位小数
    buffer[pos++] = val & 0xFF;
    checksum += buffer[pos - 1];
    buffer[pos++] = (val >> 8);
    checksum += buffer[pos - 1];
  }

  // 3.打包欧拉角
  for (int i = 0; i < 2; i++) {
    int16_t val = attitude_tele[i] * 10.0f; // 放大10倍保留1位小数
    buffer[pos++] = val & 0xFF;
    checksum += buffer[pos - 1];
    buffer[pos++] = (val >> 8);
    checksum += buffer[pos - 1];
  }

  // 4. 校验和
  buffer[pos] = checksum;
  // 5. 发送
  Serial8.write(buffer, sizeof(buffer));
}


void ProcessButtonState() {

  int buttonState = digitalRead(36);
  int buttonState1 = digitalRead(31);

  // 2. 检测按键按下（下降沿）
  if (buttonState == LOW && !buttonActive) {
    buttonActive = true;
    buttonPressTime = millis();
    delay(DEBOUNCE_DELAY); // 消抖延迟
    Serial.println("按键按下");
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
    Serial.println("按键释放");
  }

  // 4. 检测长按（持续按下）
  if (buttonActive && !longPressActive &&
      (millis() - buttonPressTime >= LONG_PRESS_TIME)) {
    longPressActive = true;
    Serial.println("长按一次");
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
    Serial.println("按键2按下");
  }

  // 3. 检测按键释放（上升沿）
  if (buttonState1 == HIGH && buttonActive1) {
    buttonActive1 = false;
    Serial.println("按键2释放");
  }

  // 4. 检测长按（持续按下）
  if (buttonActive1 && !longPressActive1 &&
      (millis() - buttonPressTime1 >= LONG_PRESS_TIME)) {
    longPressActive1 = true;
    // Serial.println("长按一次");
    //  第一行：相对滚转角
    Serial.println("3s后开始校准IMU");
    // delay(500);
    Serial.println("2s开始校准IMU");
    // delay(500);
    Serial.println("1s开始校准IMU");
    // delay(500);
    calculate_IMU_error();
    Serial.println("校准IMU完成");
    // delay(500);
  }

  // 5. 重置长按状态
  if (buttonState1 == HIGH && longPressActive1) {
    longPressActive1 = false;
  }
}
