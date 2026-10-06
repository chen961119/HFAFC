#include "control_modes.h"
#include "control_state.h"
#include "control_allocation.h"
#include "flight_clock.h"
#include "radioComm.h"
#include "human_interface.h"
#include "interaircraft_comm.h"
#include "math_utils.h"
#include "sensor_processing.h"
#include "flight_config.h"

int channel_1_pwm, channel_2_pwm, channel_3_pwm, channel_4_pwm, channel_5_pwm,
    channel_6_pwm, channel_7_pwm, channel_8_pwm;
static int channel_1_pwm_prev, channel_2_pwm_prev, channel_3_pwm_prev,
    channel_4_pwm_prev;

FlightMode currentMode;
FlightMode lastMode;
bool int_is_valid = false;
bool force_manual = false;

static bool ModeChange = 0;


// 巡航速度
static float V_cruise = 13;


// PWM——OUTPUT

const int pwm_channel1_rev = -1;
const int pwm_channel2_rev = 1;
const int pwm_channel3_rev = -1;
const int pwm_channel4_rev = 1;
const int pwm_channel5_rev = -1;

// A机
const float pwm_channel1_trim = 190 - 30;  // 减少是向上 20是偏置
const float pwm_channel2_trim = -205 + 30; // 减少是向上

// 襟副翼微调
const float pwm_channel1B_trim = 215 - 30;  // 减少是向上
const float pwm_channel2B_trim = -70 + 30;  // 减少是向上
const float pwm_channel1C_trim = 160 - 30;  // 减少是向上
const float pwm_channel2C_trim = -202 + 30; // 减少是向上
const float pwm_channel1D_trim = 132 - 30;  // 减少是向上
const float pwm_channel2D_trim = -213 + 30; // 减少是向上
const float pwm_channel1E_trim = 150 - 30;  // 减少是向上
const float pwm_channel2E_trim = -220 + 30; // 减少是向上
const float pwm_channel1F_trim = 144 - 10;  // 减少是向上
const float pwm_channel2F_trim = -217 + 10; // 减少是向上
const float pwm_channel1G_trim = 180 - 10;  // 减少是向上
const float pwm_channel2G_trim = -200 + 10; // 减少是向上

// 升降舵微调，手动模式用
const float pwm_channel3B_trim = 42;
const float pwm_channel3C_trim = -50; // 对于子机也要修正
const float pwm_channel3D_trim = 150;
const float pwm_channel3E_trim = -50;
const float pwm_channel3F_trim = 9;
const float pwm_channel3G_trim = 180;

// 当前飞机

#if defined APLANE
const float pwm_channel3_trim =
    20; // 减少是向上  对于子机，也要修正,是用来增稳模式的 A是180 载机是10
#elif defined BPLANE
const float pwm_channel3_trim = pwm_channel3B_trim;
#elif defined CPLANE
const float pwm_channel3_trim = pwm_channel3C_trim;
#elif defined DPLANE
const float pwm_channel3_trim = pwm_channel3D_trim;
#elif defined EPLANE
const float pwm_channel3_trim = pwm_channel3E_trim;
#elif defined FPLANE
const float pwm_channel3_trim = pwm_channel3F_trim;
#elif defined GPLANE
const float pwm_channel3_trim = pwm_channel3G_trim;
#endif

const float pwm_channel4_trim = 0;
const float pwm_channel5_trim = 0;

// 载机
#if defined TESTBED
const float pwm_channel1_trim = 170 - 30;  // 减少是向上 20是偏置
const float pwm_channel2_trim = -240 + 30; // 减少是向上
const float pwm_channel3_trim = 10;        // 减少是向上
#endif

static int outputpwm1, outputpwm2, outputpwm3, outputpwm4, outputpwm5;


// Radio failsafe values for every channel in the event that bad reciever data
// is detected. Recommended defaults:
static unsigned long channel_1_fs = 1500; // thro
static unsigned long channel_2_fs = 1500; // ail
static unsigned long channel_3_fs = 1000; // elev
static unsigned long channel_4_fs = 1500; // rudd
static unsigned long channel_5_fs = 1500; // gear, greater than 1500 = throttle cut
static unsigned long channel_6_fs = 2000; // aux1
static unsigned long channel_7_fs = 2000; // aux2
static unsigned long channel_8_fs = 2000; // aux3

static float Aail1_PWM_TRIM = 0.0; // 舵面微调
static float Aail2_PWM_TRIM = 0;   // 舵面微调
static float Aele_PWM_TRIM = 0.0;  // 舵面微调
static float Athro_PWM_TRIM = 0;   // 舵面微调
static float Arudd_PWM_TRIM = 0;   // 舵面微调

#if defined expensive
static float Trim_pitch_angle = 3.0;
#else
static float Trim_pitch_angle = 8.0;
#endif


// Control Variables
//  刹车
float ailBrake_PWM = 0.0, flap2eleratio = 0.0;
// A组控制变量
float Aail1_PWM, Aail2_PWM, Aele_PWM, Athro_PWM, Arudd_PWM, A_pitch_sp = 0.0;
// B组控制变量
float Bail1_PWM, Bail2_PWM, B_pitch_sp, Bthro_PWM, Brudd_PWM,
    B_ele_command_PWM_Manual, B_ele_command_PWM_FF = 0.0f;
// C组控制变量
float Cail1_PWM, Cail2_PWM, C_pitch_sp, Cthro_PWM, Crudd_PWM,
    C_ele_command_PWM_Manual, C_ele_command_PWM_FF = 0.0f;
// D组控制变量
float Dail1_PWM, Dail2_PWM, D_pitch_sp, Dthro_PWM, Drudd_PWM,
    D_ele_command_PWM_Manual, D_ele_command_PWM_FF = 0.0f;
// E组控制变量
float Eail1_PWM, Eail2_PWM, E_pitch_sp, Ethro_PWM, Erudd_PWM,
    E_ele_command_PWM_Manual, E_ele_command_PWM_FF = 0.0f;
// F组控制变量
float Fail1_PWM, Fail2_PWM, F_pitch_sp, Fthro_PWM, Frudd_PWM,
    F_ele_command_PWM_Manual, F_ele_command_PWM_FF = 0.0f;
// G组控制变量
float Gail1_PWM, Gail2_PWM, G_pitch_sp, Gthro_PWM, Grudd_PWM,
    G_ele_command_PWM_Manual, G_ele_command_PWM_FF = 0.0f;


static float Local_pitch_des_last = 8; // 期望当地俯仰角
static int Local_rudd_PWM_last = 1520;
static int Local_thro_PWM_last = 1111;
static int Local_ail2_PWM_last = 1520;
static int Local_ail1_PWM_last = 1520;
static float relativeAngle_ready_prev;


// Controller parameters (take note of defaults before modifying!):

// Clp increase
int Ail_Clp;
static float Clp_PID = 0.0;
static float k_Clp = 0.05;

static float i_limit =
    45.0; // Integrator saturation level, mostly for safety (default 25.0)
static float i_valid = 0.0;   // 积分是否起作用
static float maxRoll = 80.0;  // Max roll angle in degrees for angle mode (maximum ~70
                       // degrees), deg/sec for rate mode
static float maxPitch = 60.0; // Max pitch angle in degrees for angle mode (maximum ~70
                       // degrees), deg/sec for rate mode
static float maxYaw = 160.0;  // Max yaw rate in deg/sec

static float kp_rotate = 0.4;
static float Kp_roll_angle = 0.25; // Roll P-gain - angle mode
static float Ki_roll_angle = 0.0;  // Roll I-gain - angle mode 0.08
static float Kd_roll_angle =
    0.0; // Roll D-gain - angle mode (has no effect on controlANGLE2)
static float B_loop_roll = 1.0; // Roll damping term for controlANGLE2(), lower is more
                         // damping (must be between 0 to 1)
static float Kp_pitch_angle = 0.12; // Pitch P-gain - angle mode
static float Ki_pitch_angle = 0.0;  // Pitch I-gain - angle mode 0.1
static float Kd_pitch_angle =
    0.00; // Pitch D-gain - angle mode (has no effect on controlANGLE2)
static float B_loop_pitch = 1.0; // Pitch damping term for controlANGLE2(), lower is
                          // more damping (must be between 0 to 1)
static float Kp_Flap = 0.2;      // Flap
static float B_loop_FLAP = 0.0;
static float dw4, dw6, dw7, dw10, dw13, dw16, dw19, dw22;

// INDI相关
static double eleprev;                  // 升降舵位置估计值。
static float indi_pitch_q_gain = 25.0f; // q误差转dq_des的比例
static float indi_pitch_effectiveness =
    -113.65f; // 升降舵舵效: dq / delta_e, 单位 deg/s^2 per deg
static float indi_pitch_pwm_to_deg_k =
    0.090909f; // delta_e = k * pwm + b, 1500PWM中位, 1720PWM下偏20deg
static float indi_pitch_pwm_to_deg_b = -136.3636f;
static float indi_pitch_servo_delay_s = 0.03f;       // 舵机纯延迟
static float indi_pitch_servo_tau_s = 0.015f;        // 舵机一阶时间常数
static float indi_pitch_deflection_min_deg = -20.0f; // 最大最小可用偏角 20deg
static float indi_pitch_deflection_max_deg = 20.0f;
static float indi_pitch_rate_limit_deg_s = 800.0f; // 舵偏角速度限幅
static float indi_pitch_cmd_lpf_fc_hz = 6.0f;      // INDI输出舵偏指令低通截止频率 13.0
static float indi_pitch_pwm_min = 1100.0f;
static float indi_pitch_pwm_max = 1920.0f;
float indi_pitch_q_des_log = 0.0f;           // INDI目标俯仰角速度
float indi_pitch_q_filt_log = 0.0f;          // INDI独立滤波后的俯仰角速度
float indi_pitch_dq_des_log = 0.0f;          // INDI目标俯仰角加速度
float indi_pitch_dq_used_log = 0.0f;         // INDI使用的角加速度反馈
float indi_pitch_delta_e_cmd_deg_log = 0.0f; // INDI目标舵偏角
float indi_pitch_delta_e_est_deg_log = 0.0f; // INDI估计的实际舵偏角
float indi_pitch_pwm_cmd_log = 0.0f;         // INDI输出PWM
#if defined SINGLE
static float Kp_roll_rate = 0.045; // Roll P-gain - rate mode 0.06单机
static float Ki_roll_rate = 0.010; // Roll I-gain - rate mode 0.01单机
static float Kd_roll_rate =
    0.0000; // Roll D-gain - rate mode (be careful when increasing too high,
            // motors will begin to overheat!)
static float Kp_pitch_rate = 0.09; // Pitch P-gain - rate mode 0.12
static float Ki_pitch_rate = 0.11; // Pitch I-gain - rate mode 0.25
static float Kd_pitch_rate =
    0.0000; // Pitch D-gain - rate mode (be careful when increasing too high,
            // motors will begin to overheat!)
static float Kff_roll_rate = 0.15;  // Roll FF-gain - rate mode 0.15单机
static float Kff_pitch_rate = 0.20; // 0.2单机
static float Kff_yaw_rate = 0.03;
static float Kff_FLAP_RATE = 0.09;
static float Kp_FLAP_RATE = 0.07;
static float Ki_FLAP_RATE = 0.12;

#elif defined TEAM

static float Kp_roll_rate = 0.15; // Roll P-gain - rate mode 0.06单机
static float Ki_roll_rate = 0.1;  // Roll I-gain - rate mode 0.02单机
static float Kd_roll_rate =
    0.0002; // Roll D-gain - rate mode (be careful when increasing too high,
            // motors will begin to overheat!)
static float Kp_pitch_rate = 0.11; // Pitch P-gain - rate mode
static float Ki_pitch_rate = 0.10; // Pitch I-gain - rate mode 0.25
static float Kd_pitch_rate =
    0.000; // 0.0002 Pitch D-gain - rate mode (be careful when increasing too
           // high, motors will begin to overheat!)
static float Kff_roll_rate = 0.12; // Roll FF-gain - rate mode 0.15单机
static float Kff_pitch_rate = 0.20;
static float Kff_yaw_rate = 0.03;
static float Kff_FLAP_RATE = 0.1;
static float Kp_FLAP_RATE = 0.2;  // 0.09
static float Ki_FLAP_RATE = 0.20; // 0.15

#endif

float roll_eq;

static float Kp_yaw = 0.2;     // Yaw P-gain
static float Ki_yaw = 0.05;    // Yaw I-gain
static float Kd_yaw = 0.00000; // Yaw D-gain (be careful when increasing too high,
                        // motors will begin to overheat!)


static float roll_IMU_prev, pitch_IMU_prev;

// 滚转角控制指令滤波
static float roll_PID_lpf = 0.0f;
static const float roll_pid_lpf_fc = 7.0f; // 7hz指令滤波
static const float TWO_PI_F = 6.28318530718f;


// Normalized desired state:
float thro_des, roll_des, pitch_des, yaw_des, rotate_speed_des, pitch_des_local;
static float thro_des_RAW, roll_des_RAW, pitch_des_RAW, yaw_des_RAW;
static float roll_passthru, pitch_passthru, yaw_passthru;
static float pitch_des_local_last;
static float pitch_des_local_rate;
static float pitch_des_local_rate_lpf_fc = 5.0f;

// Controller:
float error_roll, error_roll_prev, roll_des_prev, integral_roll,
    integral_roll_il, integral_roll_ol, integral_roll_prev,
    integral_roll_prev_il, integral_roll_prev_ol, derivative_roll, roll_PID = 0;
float error_pitch, error_pitch_prev, pitch_des_prev, integral_pitch,
    integral_pitch_il, integral_pitch_ol, integral_pitch_prev,
    integral_pitch_prev_il, integral_pitch_prev_ol, derivative_pitch,
    pitch_PID = 0;
float error_yaw, error_yaw_prev, integral_yaw, integral_yaw_prev,
    derivative_yaw, yaw_PID = 0;
static float rotate_error;
// 滚转方向的前馈
static bool roll_pid_dot_initialized = false;
static float roll_PID_prev = 0.0f;
static float roll_PID_dot = 0.0f;
static float roll_PID_dot_lpf = 0.0f;     // 如果你后面想给前馈用，建议保留一个滤波后的
static float roll_pid_dot_lpf_fc = 10.0f; // 先给个截止频率参数，后面可调

float error_Phiab_RATE, error_Phiab, Phiab_des = 0, Phiab_Mea,
                                     integral_Phiab_ol, integral_Phiab_prev_ol,
                                     Phiab_des_ol, Phiab_des_nf;
float Phiab_des_prev, integral_Phiab_RATE_il, integral_Phiab_RATE_prev_il,
    Phiab_PID;
float error_Phiac, Phiac_des = 0, Phiac_Mea, integral_Phiac_ol,
                   integral_Phiac_prev_ol, Phiac_des_ol, Phiac_des_prev,
                   error_Phiac_RATE, integral_Phiac_RATE_il,
                   integral_Phiac_RATE_prev_il, Phiac_PID, Phiac_des_nf;
float error_Phibd, Phibd_des = 0, Phibd_Mea, integral_Phibd_ol,
                   integral_Phibd_prev_ol, Phibd_des_ol, Phibd_des_prev,
                   error_Phibd_RATE, integral_Phibd_RATE_il,
                   integral_Phibd_RATE_prev_il, Phibd_PID, Phibd_des_nf;
float error_Phice, Phice_des = 0, Phice_Mea, integral_Phice_ol,
                   integral_Phice_prev_ol, Phice_des_ol, Phice_des_prev,
                   error_Phice_RATE, integral_Phice_RATE_il,
                   integral_Phice_RATE_prev_il, Phice_PID, Phice_des_nf;
float error_Phidf, Phidf_des = 0, Phidf_Mea, integral_Phidf_ol,
                   integral_Phidf_prev_ol, Phidf_des_ol, Phidf_des_prev,
                   error_Phidf_RATE, integral_Phidf_RATE_il,
                   integral_Phidf_RATE_prev_il, Phidf_PID, Phidf_des_nf;
float error_Phieg, Phieg_des = 0, Phieg_Mea, integral_Phieg_ol,
                   integral_Phieg_prev_ol, Phieg_des_ol, Phieg_des_prev,
                   error_Phieg_RATE, integral_Phieg_RATE_il,
                   integral_Phieg_RATE_prev_il, Phieg_PID, Phieg_des_nf;

// TEAM
float phiab, phiac, phibd, phice, phidf, phieg; // 直接传入的。FDBACEG
float Pab, Pac, Pbd, Pdf, Pce, Peg;

// Mixer
static float m1_command_scaled, m2_command_scaled, m3_command_scaled,
    m4_command_scaled, m5_command_scaled, m6_command_scaled;
int m1_command_PWM, m2_command_PWM, m3_command_PWM, m4_command_PWM,
    m5_command_PWM, m6_command_PWM;
static float Aail1_scaled, Aail2_scaled, Aele_scaled, Athro_scaled, Arudd_scaled,
    s6_command_scaled, s7_command_scaled;
static float Bail1_scaled, Bail2_scaled, Bele_scaled, Bthro_scaled, Brudd_scaled;
static float Cail1_scaled, Cail2_scaled, Cele_scaled, Cthro_scaled, Crudd_scaled;
static float Dail1_scaled, Dail2_scaled, Dele_scaled, Dthro_scaled, Drudd_scaled;
static float Eail1_scaled, Eail2_scaled, Eele_scaled, Ethro_scaled, Erudd_scaled;
static float Fail1_scaled, Fail2_scaled, Fele_scaled, Fthro_scaled, Frudd_scaled;
static float Gail1_scaled, Gail2_scaled, Gele_scaled, Gthro_scaled, Grudd_scaled;

int s6_command_PWM, s7_command_PWM;

// Flight status
static bool armedFly = false;

float central_pitch = 0.0f;
static const uint8_t num_DSM_channels = 6;

void controlMixer() {
  // DESCRIPTION: Mixes scaled commands from PID controller to actuator outputs
  // based on vehicle configuration
  /*
   * Takes roll_PID, pitch_PID, and yaw_PID computed from the PID controller and
   * appropriately mixes them for the desired vehicle configuration. For example
   * on a quadcopter, the left two motors should have +roll_PID while the right
   * two motors should have -roll_PID. Front two should have -pitch_PID and the
   * back two should have +pitch_PID etc... every motor has normalized (0 to 1)
   * thro_des command for throttle control. Can also apply direct unstabilized
   * commands from the transmitter with roll_passthru, pitch_passthru, and
   * yaw_passthu. mX_command_scaled and sX_command scaled variables are used in
   * scaleCommands() in preparation to be sent to the motor ESCs and servos.
   *
   *Relevant variables:
   *thro_des - direct thottle control
   *roll_PID, pitch_PID, yaw_PID - stabilized axis variables
   *roll_passthru, pitch_passthru, yaw_passthru - direct unstabilized command
   * passthrough channel_6_pwm - free auxillary channel, can be used to toggle
   * things with an 'if' statement
   */

  // 0.5 is centered servo, 0.0 is zero throttle if connecting to ESC for
  // conventional PWM, 1.0 is max throttle

  if (currentMode == MANUAL_MODE) {
    roll_PID = roll_des_RAW / 2.0;
    pitch_PID = pitch_des_RAW / 2.0;
    yaw_PID = yaw_des_RAW / 2.0;
  }

  // 混控在这里

#if defined SINGLE
  // 自己飞
  Aail1_scaled = roll_PID;
  Aail2_scaled = -roll_PID;
  Aele_scaled = pitch_PID;
  Athro_scaled = thro_des;
  Arudd_scaled = yaw_PID;

#elif defined TEAM

/*
//2机一起飞 BA
//策略1

float coeroll=1.0;
if (channel_6_pwm>1600)
{
Bail1_scaled = 0.81*roll_PID+0.5*Phiab_PID;
Bail2_scaled = 0.41*roll_PID-0.5*Phiab_PID;
Bthro_scaled = thro_des-0.8*yaw_PID;
Brudd_scaled = yaw_PID;
B_pitch_sp=pitch_des_local+coeroll*0.07*roll_des;//2机、3机


Aail1_scaled = -0.41*roll_PID-0.5*Phiab_PID;
Aail2_scaled = -0.81*roll_PID+0.5*Phiab_PID;
Aele_scaled = pitch_PID;
Athro_scaled = thro_des+0.8*yaw_PID;
Arudd_scaled = yaw_PID;
central_pitch=-0.07*coeroll*roll_des;
}
else if(channel_6_pwm<1600&&channel_6_pwm>1400)
{

//策略2
Bail1_scaled = 0.5*0.81*roll_PID+0.5*Phiab_PID;
Bail2_scaled = 0.5*0.41*roll_PID-0.5*Phiab_PID;
Bthro_scaled = thro_des-0.8*yaw_PID;
Brudd_scaled = yaw_PID;
B_pitch_sp=pitch_des_local+12.0*roll_PID;//2机、3机


Aail1_scaled = 0.5*-0.41*roll_PID-0.5*Phiab_PID;
Aail2_scaled = 0.5*-0.81*roll_PID+0.5*Phiab_PID;
Aele_scaled = pitch_PID;
Athro_scaled = thro_des+0.8*yaw_PID;
Arudd_scaled = yaw_PID;
central_pitch=-12.0*coeroll*roll_PID;

}
else
{
//策略3
Bail1_scaled = 0.1*0.81*roll_PID+0.5*Phiab_PID;
Bail2_scaled = 0.1*0.41*roll_PID-0.5*Phiab_PID;
Bthro_scaled = thro_des+keeppositive(-1.2*yaw_PID)+keeppositive(0.4*roll_PID);
Brudd_scaled = yaw_PID;
B_pitch_sp=pitch_des_local+45.0*coeroll*roll_PID;//2机、3机


Aail1_scaled = 0.1*-0.41*roll_PID-0.5*Phiab_PID;
Aail2_scaled = 0.1*-0.81*roll_PID+0.5*Phiab_PID;
Aele_scaled = pitch_PID;
Athro_scaled = thro_des+keeppositive(1.2*yaw_PID)+keeppositive(-0.4*roll_PID);
Arudd_scaled = yaw_PID;
central_pitch=-45.0*coeroll*roll_PID;


}

*/
// Serial.println(Aail1_scaled);

// 3机一起飞 BAC
#if defined THREEPLANE

  float coeab, coeac, coeroll;
  if (abs(error_Phiab) > 20) {
    coeab = abs(error_Phiab) / 20.0;
  } else {
    coeab = 0.8;
  }
  if (abs(error_Phiac) > 20) {
    coeac = abs(error_Phiac) / 20.0;
  } else {
    coeac = 0.8;
  }

  coeac = 1.0;

  // 策略1
  if (channel_6_pwm > 1600) {
    Bail1_scaled =
        1.016 * roll_PID + coeab * 1 * Phiab_PID - coeac * 0.68 * Phiac_PID;
    Bail2_scaled = 0.593 * roll_PID - coeab * 0.3392 * Phiab_PID -
                   coeac * 0.12 * Phiac_PID;
    Bthro_scaled = thro_des - 0.8 * yaw_PID;
    Brudd_scaled = yaw_PID;
    B_pitch_sp = pitch_des_local; // 2机、3机

    Aail1_scaled =
        0.1 * roll_PID - coeab * 1.186 * Phiab_PID + coeac * 0.71 * Phiac_PID;
    Aail2_scaled =
        -0.1 * roll_PID - coeab * 0.71 * Phiab_PID + coeac * 1.16 * Phiac_PID;
    Aele_scaled = pitch_PID;
    Athro_scaled = thro_des;
    Arudd_scaled = yaw_PID;

    Cail1_scaled =
        -0.593 * roll_PID + coeab * 0.12 * Phiab_PID + coeac * 0.34 * Phiac_PID;
    Cail2_scaled =
        -1.016 * roll_PID + coeab * 0.68 * Phiab_PID - coeac * 1 * Phiac_PID;
    Cthro_scaled = thro_des + 0.8 * yaw_PID;
    Crudd_scaled = yaw_PID;
    C_pitch_sp = pitch_des_local; // 2机、3机
  }
  // 策略2

  else if (channel_6_pwm < 1600 && channel_6_pwm > 1400) {
    coeroll = 0.2;
    Bail1_scaled = 1.016 * coeroll * roll_PID + coeab * 0.864 * Phiab_PID -
                   coeac * 0.62 * Phiac_PID;
    Bail2_scaled = 0.593 * coeroll * roll_PID - coeab * 0.27 * Phiab_PID -
                   coeac * 0.08 * Phiac_PID;
    Bthro_scaled =
        thro_des + keeppositive(-1.2 * yaw_PID) + keeppositive(0.4 * roll_PID);
    Brudd_scaled = yaw_PID;
    B_pitch_sp = pitch_des_local + 20.0 * roll_PID_lpf; // 2机、3机
    B_ele_command_PWM_FF = 0.0;

    Aail1_scaled =
        0.1 * roll_PID - coeab * 1.0 * Phiab_PID + coeac * 0.65 * Phiac_PID;
    Aail2_scaled =
        -0.1 * roll_PID - coeab * 0.65 * Phiab_PID + coeac * 1.0 * Phiac_PID;
    Aele_scaled = pitch_PID;
    Athro_scaled = thro_des;
    Arudd_scaled = yaw_PID;

    Cail1_scaled = -0.593 * coeroll * roll_PID + coeab * 0.08 * Phiab_PID +
                   coeac * 0.27 * Phiac_PID;
    Cail2_scaled = -1.016 * coeroll * roll_PID + coeab * 0.62 * Phiab_PID -
                   coeac * 0.864 * Phiac_PID;
    Cthro_scaled =
        thro_des + keeppositive(1.2 * yaw_PID) + keeppositive(-0.4 * roll_PID);
    Crudd_scaled = yaw_PID;
    C_pitch_sp = pitch_des_local - 20.0 * roll_PID_lpf; // 2机、3机
    C_ele_command_PWM_FF = 0.0;

  } else {
    coeroll = 0.2;

    Bail1_scaled = 1.016 * coeroll * roll_PID + coeab * 0.864 * Phiab_PID -
                   coeac * 0.62 * Phiac_PID;
    Bail2_scaled = 0.593 * coeroll * roll_PID - coeab * 0.27 * Phiab_PID -
                   coeac * 0.08 * Phiac_PID;
    Bthro_scaled =
        thro_des + keeppositive(-1.2 * yaw_PID) + keeppositive(0.4 * roll_PID);
    Brudd_scaled = yaw_PID;
    B_pitch_sp = pitch_des_local + 30.0 * roll_PID_lpf +
                 6.0 * (Bail1_scaled + Bail2_scaled); // 2机、3机
    B_ele_command_PWM_FF =
        -10.0 * 20.0 * roll_PID_lpf - 10.0 * roll_PID_dot_lpf;

    Aail1_scaled =
        0.1 * roll_PID - coeab * 1.0 * Phiab_PID + coeac * 0.65 * Phiac_PID;
    Aail2_scaled =
        -0.1 * roll_PID - coeab * 0.65 * Phiab_PID + coeac * 1.0 * Phiac_PID;
    Aele_scaled = pitch_PID;
    Athro_scaled = thro_des;
    Arudd_scaled = yaw_PID;

    Cail1_scaled = -0.593 * coeroll * roll_PID + coeab * 0.08 * Phiab_PID +
                   coeac * 0.27 * Phiac_PID;
    Cail2_scaled = -1.016 * coeroll * roll_PID + coeab * 0.62 * Phiab_PID -
                   coeac * 0.864 * Phiac_PID;
    Cthro_scaled =
        thro_des + keeppositive(1.2 * yaw_PID) + keeppositive(-0.4 * roll_PID);
    Crudd_scaled = yaw_PID;
    C_pitch_sp =
        pitch_des_local - 30.0 * roll_PID_lpf +
        6.0 * (Cail1_scaled + Cail2_scaled); // 2机、3机 增加了相对滚转到升降舵
    C_ele_command_PWM_FF = 10.0 * 20.0 * roll_PID_lpf + 10.0 * roll_PID_dot_lpf;
  }

#elif defined FOURPLANE

  // 4机一起飞 DBAC
  float coeab, coeac, coebd,
      coeroll; // bc的系数和de的系数不同 de的系数是ab的0.66倍

  coeroll = 1.5;

  // Serial.println(coeroll);
  if (abs(error_Phiab) > 20) {
    coeab = 1.2 * abs(error_Phiab) / 20.0;
  } else {
    coeab = 1.2;
  }

  if (abs(error_Phiac) > 20) {
    coeac = 1.0 * abs(error_Phiac) / 20.0;
  } else {
    coeac = 1.0;
  }

  if (abs(error_Phibd) > 20) {
    coebd = 1.0 * abs(error_Phibd) / 20.0;
  } else {
    coebd = 1.0;
  }

  // 策略1
  if (channel_6_pwm > 1600) {
    Dail1_scaled = 1.0 * coeroll * roll_PID + 1.0 * coeab * Phiab_PID -
                   0.53 * coeac * Phiac_PID + 1.0 * coebd * Phibd_PID;
    Dail2_scaled = 0.74 * coeroll * roll_PID + 0.325 * coeab * Phiab_PID -
                   0.26 * coeac * Phiac_PID - 0.1 * coebd * Phibd_PID;
    Dthro_scaled = thro_des - 1.2 * yaw_PID;
    Drudd_scaled = yaw_PID;
    D_pitch_sp = pitch_des_local + 0.13 * coeroll * roll_des;

    Bail1_scaled = 0.28 * coeroll * roll_PID - 0.667 * coeab * Phiab_PID +
                   0.19 * coeac * Phiac_PID - 1.0 * coebd * Phibd_PID;
    Bail2_scaled = 0.13 * coeroll * roll_PID - 1.0 * coeab * Phiab_PID +
                   0.39 * coeac * Phiac_PID - 0.736 * coebd * Phibd_PID;
    Bthro_scaled = thro_des - 0.6 * yaw_PID;
    Brudd_scaled = yaw_PID;
    B_pitch_sp = pitch_des_local + 0.07 * coeroll * roll_des;

    Aail1_scaled = -0.13 * coeroll * roll_PID - 1.0 * coeab * Phiab_PID +
                   0.736 * coeac * Phiac_PID - 0.39 * coebd * Phibd_PID;
    Aail2_scaled = -0.28 * coeroll * roll_PID - 0.667 * coeab * Phiab_PID +
                   1.0 * coeac * Phiac_PID - 0.19 * coebd * Phibd_PID;
    Aele_scaled = pitch_PID;
    Athro_scaled = thro_des + 0.6 * yaw_PID;
    ;
    Arudd_scaled = yaw_PID;
    central_pitch = -0.07 * coeroll * roll_des;

    Cail1_scaled = -0.74 * coeroll * roll_PID + 0.325 * coeab * Phiab_PID +
                   0.1 * coeac * Phiac_PID + 0.26 * coebd * Phibd_PID;
    Cail2_scaled = -1.0 * coeroll * roll_PID + 1.0 * coeab * Phiab_PID -
                   1.0 * coeac * Phiac_PID + 0.53 * coebd * Phibd_PID;
    Cthro_scaled = thro_des + 1.2 * yaw_PID;
    Crudd_scaled = yaw_PID;
    C_pitch_sp = pitch_des_local - 0.13 * coeroll * roll_des;

  }

  // 策略2

  else if (channel_6_pwm < 1600 && channel_6_pwm > 1400) {
    coeroll = 0.3;
    Dail1_scaled = 1.0 * coeroll * roll_PID + 1.0 * coeab * Phiab_PID -
                   0.53 * coeac * Phiac_PID + 1.0 * coebd * Phibd_PID;
    Dail2_scaled = 0.74 * coeroll * roll_PID + 0.325 * coeab * Phiab_PID -
                   0.26 * coeac * Phiac_PID - 0.1 * coebd * Phibd_PID;
    Dthro_scaled =
        thro_des + keeppositive(-1.5 * yaw_PID) + keeppositive(0.5 * roll_PID);
    Drudd_scaled = yaw_PID;
    D_pitch_sp = pitch_des_local + 70.0 * roll_PID;

    Bail1_scaled = 0.28 * coeroll * roll_PID - 0.667 * coeab * Phiab_PID +
                   0.19 * coeac * Phiac_PID - 1.0 * coebd * Phibd_PID;
    Bail2_scaled = 0.1 * coeroll * roll_PID - 1.0 * coeab * Phiab_PID +
                   0.39 * coeac * Phiac_PID - 0.736 * coebd * Phibd_PID;
    Bthro_scaled =
        thro_des + keeppositive(-1.0 * yaw_PID) + keeppositive(0.3 * roll_PID);
    Brudd_scaled = yaw_PID;
    B_pitch_sp = pitch_des_local + 30.0 * roll_PID;

    Aail1_scaled = -0.1 * coeroll * roll_PID - 1.0 * coeab * Phiab_PID +
                   0.736 * coeac * Phiac_PID - 0.39 * coebd * Phibd_PID;
    Aail2_scaled = -0.28 * coeroll * roll_PID - 0.667 * coeab * Phiab_PID +
                   1.0 * coeac * Phiac_PID - 0.19 * coebd * Phibd_PID;
    Aele_scaled = pitch_PID;
    Athro_scaled =
        thro_des + keeppositive(1.0 * yaw_PID) + keeppositive(-0.3 * roll_PID);
    Arudd_scaled = yaw_PID;
    central_pitch = -30.0 * roll_PID;

    Cail1_scaled = -0.74 * coeroll * roll_PID + 0.325 * coeab * Phiab_PID +
                   0.1 * coeac * Phiac_PID + 0.26 * coebd * Phibd_PID;
    Cail2_scaled = -1.0 * coeroll * roll_PID + 1.0 * coeab * Phiab_PID -
                   1.0 * coeac * Phiac_PID + 0.53 * coebd * Phibd_PID;
    Cthro_scaled =
        thro_des + keeppositive(1.5 * yaw_PID) + keeppositive(-0.5 * roll_PID);
    Crudd_scaled = yaw_PID;
    C_pitch_sp = pitch_des_local - 70.0 * roll_PID;

  } else {

    coeroll = 0.2;
    Dail1_scaled = 1.0 * coeroll * roll_PID + 1.0 * coeab * Phiab_PID -
                   0.53 * coeac * Phiac_PID + 1.0 * coebd * Phibd_PID;
    Dail2_scaled = 0.74 * coeroll * roll_PID + 0.325 * coeab * Phiab_PID -
                   0.26 * coeac * Phiac_PID - 0.1 * coebd * Phibd_PID;
    Dthro_scaled =
        thro_des + keeppositive(-1.5 * yaw_PID) + keeppositive(0.5 * roll_PID);
    Drudd_scaled = yaw_PID;
    D_pitch_sp = pitch_des_local + 70.0 * roll_PID + 35.0 * Phiab_PID -
                 20.0 * Phiac_PID + 20.0 * Phibd_PID;

    Bail1_scaled = 0.28 * coeroll * roll_PID - 0.667 * coeab * Phiab_PID +
                   0.19 * coeac * Phiac_PID - 1.0 * coebd * Phibd_PID;
    Bail2_scaled = 0.1 * coeroll * roll_PID - 1.0 * coeab * Phiab_PID +
                   0.39 * coeac * Phiac_PID - 0.736 * coebd * Phibd_PID;
    Bthro_scaled =
        thro_des + keeppositive(-1.0 * yaw_PID) + keeppositive(0.3 * roll_PID);
    Brudd_scaled = yaw_PID;
    B_pitch_sp = pitch_des_local + 30.0 * roll_PID - 40.0 * Phiab_PID +
                 15.0 * Phiac_PID - 45.0 * Phibd_PID;

    Aail1_scaled = -0.1 * coeroll * roll_PID - 1.0 * coeab * Phiab_PID +
                   0.736 * coeac * Phiac_PID - 0.39 * coebd * Phibd_PID;
    Aail2_scaled = -0.28 * coeroll * roll_PID - 0.667 * coeab * Phiab_PID +
                   1.0 * coeac * Phiac_PID - 0.19 * coebd * Phibd_PID;
    Aele_scaled = pitch_PID;
    Athro_scaled =
        thro_des + keeppositive(1.0 * yaw_PID) + keeppositive(-0.3 * roll_PID);
    Arudd_scaled = yaw_PID;
    central_pitch = -30.0 * roll_PID - 40.0 * Phiab_PID + 45.0 * Phiac_PID -
                    15.0 * Phibd_PID;

    Cail1_scaled = -0.74 * coeroll * roll_PID + 0.325 * coeab * Phiab_PID +
                   0.1 * coeac * Phiac_PID + 0.26 * coebd * Phibd_PID;
    Cail2_scaled = -1.0 * coeroll * roll_PID + 1.0 * coeab * Phiab_PID -
                   1.0 * coeac * Phiac_PID + 0.53 * coebd * Phibd_PID;
    Cthro_scaled =
        thro_des + keeppositive(1.5 * yaw_PID) + keeppositive(-0.5 * roll_PID);
    Crudd_scaled = yaw_PID;
    C_pitch_sp = pitch_des_local - 70.0 * roll_PID + 35.0 * Phiab_PID -
                 20.0 * Phiac_PID + 20.0 * Phibd_PID;
  }

#elif defined FIVEPLANE

  // 5机一起飞DBACE
  float coeab, coeac, coebd, coece,
      coeroll; // bc的系数和de的系数不同 de的系数是ab的0.66倍
  if (abs(error_Phiab) > 20) {
    coeab = abs(error_Phiab) / 20.0;
  } else {
    coeab = 1.0;
  }

  if (abs(error_Phiac) > 20) {
    coeac = abs(error_Phiac) / 20.0;
  } else {
    coeac = 1.0;
  }

  if (abs(error_Phibd) > 20) {
    coebd = abs(error_Phibd) / 20.0;
  } else {
    coebd = 0.9;
  }

  if (abs(error_Phice) > 20) {
    coece = abs(error_Phice) / 20.0;
  } else {
    coece = 0.9;
  }

  // 策略1
  coeroll = 1.2;
  Dail1_scaled = 1.0 * coeroll * roll_PID + 1.0 * coeab * Phiab_PID -
                 0.72 * coeac * Phiac_PID + 1.0 * coebd * Phibd_PID -
                 0.38 * coece * Phice_PID;
  Dail2_scaled = 0.75 * coeroll * roll_PID + 0.44 * coeab * Phiab_PID -
                 0.42 * coeac * Phiac_PID + 0.06 * coebd * Phibd_PID -
                 0.23 * coece * Phice_PID;
  Dthro_scaled = thro_des - 1.0 * yaw_PID;
  Drudd_scaled = yaw_PID;
  D_pitch_sp = pitch_des_local + 0.3 * coeroll * roll_des;

  Bail1_scaled = 0.417 * coeroll * roll_PID - 0.36 * coeab * Phiab_PID +
                 0.05 * coeac * Phiac_PID - 0.7 * coebd * Phibd_PID -
                 0.01 * coece * Phice_PID;
  Bail2_scaled = 0.33 * coeroll * roll_PID - 0.76 * coeab * Phiab_PID +
                 0.31 * coeac * Phiac_PID - 0.6 * coebd * Phibd_PID +
                 0.08 * coece * Phice_PID;
  Bthro_scaled = thro_des - 0.8 * yaw_PID;
  Brudd_scaled = yaw_PID;
  B_pitch_sp = pitch_des_local + 0.15 * coeroll * roll_des;

  Aail1_scaled = 0.15 * coeroll * roll_PID - 0.88 * coeab * Phiab_PID +
                 0.66 * coeac * Phiac_PID - 0.42 * coebd * Phibd_PID +
                 0.26 * coece * Phice_PID;
  Aail2_scaled = -0.15 * coeroll * roll_PID - 0.66 * coeab * Phiab_PID +
                 0.88 * coeac * Phiac_PID - 0.27 * coebd * Phibd_PID +
                 0.38 * coece * Phice_PID;
  Aele_scaled = pitch_PID;
  Athro_scaled = thro_des;
  Arudd_scaled = yaw_PID;

  Cail1_scaled = -0.33 * coeroll * roll_PID - 0.31 * coeab * Phiab_PID +
                 0.76 * coeac * Phiac_PID - 0.08 * coebd * Phibd_PID +
                 0.6 * coece * Phice_PID;
  Cail2_scaled = -0.417 * coeroll * roll_PID - 0.05 * coeab * Phiab_PID +
                 0.36 * coeac * Phiac_PID - 0.01 * coebd * Phibd_PID +
                 0.7 * coece * Phice_PID;
  Cthro_scaled = thro_des + 0.8 * yaw_PID;
  Crudd_scaled = yaw_PID;
  C_pitch_sp = pitch_des_local - 0.15 * coeroll * roll_des;

  Eail1_scaled = -0.75 * coeroll * roll_PID + 0.42 * coeab * Phiab_PID -
                 coeac * 0.44 * Phiac_PID + 0.23 * coebd * Phibd_PID -
                 0.06 * coece * Phice_PID;
  Eail2_scaled = -1.0 * coeroll * roll_PID + 0.72 * coeab * Phiab_PID -
                 1.0 * coeac * Phiac_PID + 0.38 * coebd * Phibd_PID -
                 1.0 * coece * Phice_PID;
  Ethro_scaled = thro_des + 1.0 * yaw_PID;
  Erudd_scaled = yaw_PID;
  E_pitch_sp = pitch_des_local - 0.3 * coeroll * roll_des;

  /*
   else if (channel_6_pwm < 1600 && channel_6_pwm > 1400) {
     //策略2
     coeroll = 0.5;
     Dail1_scaled = 1.0 * coeroll * roll_PID + 1.0 * coeab * Phiab_PID - 0.72 *
   coeac * Phiac_PID + 1.0 * coebd * Phibd_PID - 0.38 * coece * Phice_PID;
     Dail2_scaled = 0.75 * coeroll * roll_PID + 0.44 * coeab * Phiab_PID - 0.42
   * coeac * Phiac_PID + 0.06 * coebd * Phibd_PID - 0.23 * coece * Phice_PID;
     Dthro_scaled = thro_des + keeppositive(-1.8 * yaw_PID) + keeppositive(0.7 *
   roll_PID); Drudd_scaled = yaw_PID; D_pitch_sp = pitch_des_local + 80.0 *
   roll_PID ;

     Bail1_scaled = 0.417 * coeroll * roll_PID - 0.36 * coeab * Phiab_PID + 0.05
   * coeac * Phiac_PID - 0.7 * coebd * Phibd_PID - 0.01 * coece * Phice_PID;
     Bail2_scaled = 0.33 * coeroll * roll_PID - 0.76 * coeab * Phiab_PID + 0.31
   * coeac * Phiac_PID - 0.6 * coebd * Phibd_PID + 0.08 * coece * Phice_PID;
     Bthro_scaled = thro_des + keeppositive(-1.0 * yaw_PID) + keeppositive(0.4 *
   roll_PID); Brudd_scaled = yaw_PID; B_pitch_sp = pitch_des_local + 30.0 *
   roll_PID ;

     Aail1_scaled = 0.15 * coeroll * roll_PID - 0.88 * coeab * Phiab_PID + 0.66
   * coeac * Phiac_PID - 0.42 * coebd * Phibd_PID + 0.27 * coece * Phice_PID;
     Aail2_scaled = -0.15 * coeroll * roll_PID - 0.66 * coeab * Phiab_PID + 0.88
   * coeac * Phiac_PID - 0.27 * coebd * Phibd_PID + 0.42 * coece * Phice_PID;
     Aele_scaled = pitch_PID;
     Athro_scaled = thro_des;
     Arudd_scaled = yaw_PID;

     Cail1_scaled = -0.33 * coeroll * roll_PID - 0.31 * coeab * Phiab_PID + 0.76
   * coeac * Phiac_PID - 0.08 * coebd * Phibd_PID + 0.6 * coece * Phice_PID;
     Cail2_scaled = -0.417 * coeroll * roll_PID - 0.05 * coeab * Phiab_PID +
   0.36 * coeac * Phiac_PID - 0.01 * coebd * Phibd_PID + 0.7 * coece *
   Phice_PID; Cthro_scaled = thro_des + keeppositive(1.0 * yaw_PID) +
   keeppositive(-0.4 * roll_PID); Crudd_scaled = yaw_PID; C_pitch_sp =
   pitch_des_local - 30.0 * roll_PID;

     Eail1_scaled = -0.75 * coeroll * roll_PID + 0.42 * coeab * Phiab_PID -
   coeac * 0.44 * Phiac_PID + 0.23 * coebd * Phibd_PID - 0.06 * coece *
   Phice_PID; Eail2_scaled = -1.0 * coeroll * roll_PID + 0.72 * coeab *
   Phiab_PID - 1.0 * coeac * Phiac_PID + 0.38 * coebd * Phibd_PID - 1.0 * coece
   * Phice_PID; Ethro_scaled = thro_des + keeppositive(1.8 * yaw_PID) +
   keeppositive(-0.7 * roll_PID); Erudd_scaled = yaw_PID; E_pitch_sp =
   pitch_des_local - 80.0 * roll_PID;
   }

   else {
     //Serial.println("ss");
     //策略3 实时控制分配
     //期望虚拟控制量
     dw4 = roll_PID * 40.0;
     dw6 = yaw_PID * 20.0;
     dw7 = Phiab_PID * 130.0;
     dw10 = Phiac_PID * 130.0;
     dw13 = Phibd_PID * 110.0;
     dw16 = Phice_PID * 110.0;


     dw_att << dw4,
       dw6,
       0.0,
       0.0,
       0.0,
       0.0;

     dw_config << dw7,
       dw10,
       dw13,
       dw16;

     de_config = Bplusminismall_pinv * dw_config;
     de_att = Bplusfullsmall_pinv * dw_att;
     //printFullMatrix(de_config);
     Dail1_scaled = de_att(0) + de_config(0);
     Dail2_scaled = de_att(1) + de_config(1);
     Dthro_scaled = thro_des + keeppositive(-1.4 * yaw_PID) + keeppositive(0.5 *
   roll_PID); Drudd_scaled = yaw_PID; D_pitch_sp = pitch_des_local + 80.0 *
   roll_PID - 1.0;

     Bail1_scaled = de_att(2) + de_config(2);
     Bail2_scaled = de_att(3) + de_config(3);
     Bthro_scaled = thro_des + keeppositive(-0.8 * yaw_PID) + keeppositive(0.3 *
   roll_PID); Brudd_scaled = yaw_PID; B_pitch_sp = pitch_des_local + 30.0 *
   roll_PID - 1.0;

     Aail1_scaled = de_att(4) + de_config(4);
     Aail2_scaled = de_att(5) + de_config(5);
     Aele_scaled = pitch_PID;
     Athro_scaled = thro_des;
     Arudd_scaled = yaw_PID;

     Cail1_scaled = de_att(6) + de_config(6);
     Cail2_scaled = de_att(7) + de_config(7);
     Cthro_scaled = thro_des + keeppositive(0.8 * yaw_PID) + keeppositive(-0.3 *
   roll_PID); Crudd_scaled = yaw_PID; C_pitch_sp = pitch_des_local - 30.0 *
   roll_PID;

     Eail1_scaled = de_att(8) + de_config(8);
     Eail2_scaled = de_att(9) + de_config(9);
     Ethro_scaled = thro_des + keeppositive(1.4 * yaw_PID) + keeppositive(-0.5 *
   roll_PID); Erudd_scaled = yaw_PID; E_pitch_sp = pitch_des_local - 80.0 *
   roll_PID;
   }
     */

#elif defined SEVENPLANE

  // 7机一起飞 FDBACEG
  float coeab, coeac, coebd, coece, coedf, coeeg,
      coeroll; // bc的系数和de的系数不同 de的系数是ab的0.66倍
  if (abs(error_Phiab) > 20) {
    coeab = abs(error_Phiab) / 20.0;
  } else {
    coeab = 1.0;
  }

  if (abs(error_Phiac) > 20) {
    coeac = abs(error_Phiac) / 20.0;
  } else {
    coeac = 1.0;
  }

  if (abs(error_Phibd) > 20) {
    coebd = abs(error_Phibd) / 20.0;
  } else {
    coebd = 0.9;
  }

  if (abs(error_Phice) > 20) {
    coece = abs(error_Phice) / 20.0;
  } else {
    coece = 0.9;
  }

  if (abs(error_Phidf) > 20) {
    coedf = abs(error_Phidf) / 20.0;
  } else {
    coedf = 0.6;
  }

  if (abs(error_Phieg) > 20) {
    coeeg = abs(error_Phieg) / 20.0;
  } else {
    coeeg = 0.6;
  }

  // 策略1
  coeroll = 1.8;

  Fail1_scaled = 1.0 * coeroll * roll_PID + 1.0 * coeab * Phiab_PID -
                 0.77 * coeac * Phiac_PID + 1.0 * coebd * Phibd_PID -
                 0.44 * coece * Phice_PID + 1.0 * coedf * Phidf_PID -
                 0.24 * coeeg * Phieg_PID;
  Fail2_scaled = 0.87 * coeroll * roll_PID + 0.69 * coeab * Phiab_PID -
                 0.56 * coeac * Phiac_PID + 0.54 * coebd * Phibd_PID -
                 0.33 * coece * Phice_PID + 0.19 * coedf * Phidf_PID -
                 0.18 * coeeg * Phieg_PID;
  Fthro_scaled = 0.85 * thro_des - 1.5 * yaw_PID;
  Frudd_scaled = yaw_PID;
  F_pitch_sp = pitch_des_local + 0.5 * coeroll * roll_des;

  Dail1_scaled = 0.72 * coeroll * roll_PID + 0.31 * coeab * Phiab_PID -
                 0.32 * coeac * Phiac_PID + 0.06 * coebd * Phibd_PID -
                 0.21 * coece * Phice_PID - 0.27 * coedf * Phidf_PID -
                 0.12 * coeeg * Phieg_PID;
  Dail2_scaled = 0.55 * coeroll * roll_PID - 0.12 * coeab * Phiab_PID -
                 0.06 * coeac * Phiac_PID - 0.44 * coebd * Phibd_PID -
                 0.07 * coece * Phice_PID - 0.41 * coedf * Phidf_PID -
                 0.05 * coeeg * Phieg_PID;
  Dthro_scaled = 0.92 * thro_des - 1.0 * yaw_PID;
  Drudd_scaled = yaw_PID;
  D_pitch_sp = pitch_des_local + 0.3 * coeroll * roll_des;

  Bail1_scaled = 0.4 * coeroll * roll_PID - 0.50 * coeab * Phiab_PID +
                 0.2 * coeac * Phiac_PID - 0.65 * coebd * Phibd_PID +
                 0.05 * coece * Phice_PID - 0.41 * coedf * Phidf_PID +
                 0.01 * coeeg * Phieg_PID;
  Bail2_scaled = 0.23 * coeroll * roll_PID - 0.83 * coeab * Phiab_PID +
                 0.47 * coeac * Phiac_PID - 0.57 * coebd * Phibd_PID +
                 0.17 * coece * Phice_PID - 0.27 * coedf * Phidf_PID +
                 0.08 * coeeg * Phieg_PID;
  Bthro_scaled = thro_des - 0.8 * yaw_PID;
  Brudd_scaled = yaw_PID;
  B_pitch_sp = pitch_des_local + 0.15 * coeroll * roll_des;

  Aail1_scaled = 0.1 * coeroll * roll_PID - 0.89 * coeab * Phiab_PID +
                 0.70 * coeac * Phiac_PID - 0.46 * coebd * Phibd_PID +
                 0.31 * coece * Phice_PID - 0.18 * coedf * Phidf_PID +
                 0.14 * coeeg * Phieg_PID;
  Aail2_scaled = -0.1 * coeroll * roll_PID - 0.70 * coeab * Phiab_PID +
                 0.89 * coeac * Phiac_PID - 0.31 * coebd * Phibd_PID +
                 0.46 * coece * Phice_PID - 0.14 * coedf * Phidf_PID +
                 0.18 * coeeg * Phieg_PID;
  Aele_scaled = pitch_PID;
  Athro_scaled = thro_des;
  Arudd_scaled = yaw_PID;

  Cail1_scaled = -0.23 * coeroll * roll_PID - 0.47 * coeab * Phiab_PID +
                 0.83 * coeac * Phiac_PID - 0.17 * coebd * Phibd_PID +
                 0.57 * coece * Phice_PID - 0.08 * coedf * Phidf_PID +
                 0.27 * coeeg * Phieg_PID;
  Cail2_scaled = -0.4 * coeroll * roll_PID - 0.2 * coeab * Phiab_PID +
                 0.50 * coeac * Phiac_PID - 0.05 * coebd * Phibd_PID +
                 0.65 * coece * Phice_PID - 0.01 * coedf * Phidf_PID +
                 0.41 * coeeg * Phieg_PID;
  Cthro_scaled = thro_des + 0.8 * yaw_PID;
  Crudd_scaled = yaw_PID;
  C_pitch_sp = pitch_des_local - 0.15 * coeroll * roll_des;

  Eail1_scaled = -0.55 * coeroll * roll_PID + 0.06 * coeab * Phiab_PID +
                 coeac * 0.12 * Phiac_PID + 0.07 * coebd * Phibd_PID +
                 0.44 * coece * Phice_PID + 0.05 * coedf * Phidf_PID +
                 0.41 * coeeg * Phieg_PID;
  Eail2_scaled = -0.72 * coeroll * roll_PID + 0.32 * coeab * Phiab_PID -
                 0.31 * coeac * Phiac_PID + 0.21 * coebd * Phibd_PID -
                 0.06 * coece * Phice_PID + 0.12 * coedf * Phidf_PID +
                 0.27 * coeeg * Phieg_PID;
  Ethro_scaled = 0.92 * thro_des + 1.0 * yaw_PID;
  Erudd_scaled = yaw_PID;
  E_pitch_sp = pitch_des_local - 0.3 * coeroll * roll_des;

  Gail1_scaled = -0.87 * coeroll * roll_PID + 0.56 * coeab * Phiab_PID -
                 coeac * 0.69 * Phiac_PID + 0.33 * coebd * Phibd_PID -
                 0.54 * coece * Phice_PID + 0.18 * coedf * Phidf_PID -
                 0.19 * coeeg * Phieg_PID;
  Gail2_scaled = -1.0 * coeroll * roll_PID + 0.77 * coeab * Phiab_PID -
                 1.0 * coeac * Phiac_PID + 0.44 * coebd * Phibd_PID -
                 1.0 * coece * Phice_PID + 0.24 * coedf * Phidf_PID -
                 1.0 * coeeg * Phieg_PID;
  Gthro_scaled = 0.85 * thro_des + 1.5 * yaw_PID;
  Grudd_scaled = yaw_PID;
  G_pitch_sp = pitch_des_local - 0.5 * coeroll * roll_des;

#endif

#endif

  s6_command_scaled = 0;
  s7_command_scaled = 0;
}

void armedStatus() {
  // DESCRIPTION: Check if the throttle cut is off and the throttle input is low
  // to prepare for flight.
  if ((channel_5_pwm < 1500) && (channel_1_pwm < 1050)) {
    armedFly = true;
  }
}

void getDesState() { // 调整了通道顺序
  // DESCRIPTION: Normalizes desired control values to appropriate values
  /*
   * Updates the desired state variables thro_des, roll_des, pitch_des, and
   * yaw_des. These are computed by using the raw RC pwm commands and scaling
   * them to be within our limits defined in setup. thro_des stays within 0 to 1
   * range. roll_des and pitch_des are scaled to be within max roll/pitch amount
   * in either degrees (angle mode) or degrees/sec (rate mode). yaw_des is
   * scaled to be within max yaw in degrees/sec. Also creates roll_passthru,
   * pitch_passthru, and yaw_passthru variables, to be used in commanding
   * motors/servos with direct unstabilized commands in controlMixer().
   */
  float GyroZ;
#if defined USE_MPU6050_I2C
  GyroZ = GyroZ_6050;
#endif
#if defined USE_MPU9250_SPI
  GyroZ = GyroZ_9250;
#endif

#if defined APLANE // 是主机
  {
    thro_des_RAW = (channel_3_pwm - 1100.0) / 1000.0; // Between 0 and 1
    roll_des_RAW = (channel_1_pwm - 1520.0) / 500.0;  // Between -1 and 1
    pitch_des_RAW = (channel_2_pwm - 1520.0) / 500.0; // Between -1 and 1
    yaw_des_RAW = (channel_4_pwm - 1520.0) / 500.0;   // Between -1 and 1

    /*
    if (channel_6_pwm>1600) //水平构型
    {
      Phiab_des_nf = 0.0;
      Phiac_des_nf = 0.0;
      Phibd_des_nf = 0.0;
      Phice_des_nf = 0.0;
      Phidf_des_nf = 0.0;
      Phieg_des_nf = 0.0;
     }
     else if (channel_6_pwm<1600&&channel_6_pwm>1400) //下反
     {
        Phiab_des_nf = -5.0;
        Phiac_des_nf = 5.0;
        Phibd_des_nf = -5.0;
        Phice_des_nf = 5.0;
        Phidf_des_nf = -5.0;
        Phieg_des_nf = 5.0;
     }
     else //Z字
     {
        Phiab_des_nf = 10.0;
        Phiac_des_nf = 10.0;
        Phibd_des_nf = 10.0;
        Phice_des_nf = 10.0;
        Phidf_des_nf = 10.0;
        Phieg_des_nf = 10.0;
     }

     Phiab_des=0.02*Phiab_des_nf+(1-0.02)*Phiab_des;
     Phiac_des=0.02*Phiac_des_nf+(1-0.02)*Phiac_des;
     Phibd_des=0.02*Phibd_des_nf+(1-0.02)*Phibd_des;
     Phice_des=0.02*Phice_des_nf+(1-0.02)*Phice_des;
     Phidf_des=0.02*Phidf_des_nf+(1-0.02)*Phidf_des;
     Phieg_des=0.02*Phieg_des_nf+(1-0.02)*Phieg_des;
    */

    roll_passthru = roll_des_RAW / 2.0;   // Between -0.5 and 0.5
    pitch_passthru = pitch_des_RAW / 2.0; // Between -0.5 and 0.5
    yaw_passthru = yaw_des_RAW / 2.0;     // Between -0.5 and 0.5

    // Constrain within normalized bounds
    thro_des = constrain(thro_des_RAW, 0.0, 1.0); // Between 0 and 1
    roll_des = constrain(roll_des_RAW, -1.0, 1.0) *
               maxRoll; // Between -maxRoll and +maxRoll
    pitch_des_local = constrain(pitch_des_RAW, -1.0, 1.0) *
                      maxPitch; // Between -maxPitch and +maxPitch
    yaw_des = constrain(yaw_des_RAW, -1.0, 1.0) *
              maxYaw; // Between -maxYaw and +maxYaw
    roll_passthru = constrain(roll_passthru, -0.5, 0.5);
    pitch_passthru = constrain(pitch_passthru, -0.5, 0.5);
    yaw_passthru = constrain(yaw_passthru, -0.5, 0.5);

    // Phiab_des=constrain(Phiab_des, -1.0, 1.0)*15.0; //Between -maxRoll and
    // +maxRoll Phiac_des=constrain(Phiac_des, -1.0, 1.0)*-15.0; //Between
    // -maxRoll and +maxRoll

    pitch_des_local += Trim_pitch_angle;

#if defined TEAM

    // pitch_des_local=pitch_des-0.1*roll_des;//2机
    pitch_des_local = pitch_des_local; // 3机
#endif

    // 1600以上是手动  1400-1600是速率 1400以下是增稳
    if (channel_5_pwm > 1600) {
      currentMode = MANUAL_MODE;
    } else {
      currentMode =
          (channel_5_pwm < 1400) ? STABILIZE_MODE : STABLIZE_MODE_NO_I;
    }

    if (currentMode != lastMode) {
      ModeChange = 1;
    } else {
      ModeChange = 0;
    }
    lastMode = currentMode;
  }
#else // 是从机
  {
    receiveCommandData();
    if (int_is_valid) {
      currentMode = STABILIZE_MODE;
    } else {
      currentMode = STABLIZE_MODE_NO_I;
    }
    if (force_manual) {
      currentMode = MANUAL_MODE;
    }
    if (currentMode != lastMode) {
      ModeChange = 1;
    } else {
      ModeChange = 0;
    }
    lastMode = currentMode;

    thro_des = 0;
    roll_des = 0;
    pitch_des_local = Local_pitch_des;
    // Serial.println(pitch_des_local);
    yaw_des = 0;
  }
#endif
}

void controlANGLE() {
  // DESCRIPTION: Computes control commands based on state error (angle)
  /*
   * Basic PID control to stablize on angle setpoint based on desired states
   * roll_des, pitch_des, and yaw_des computed in getDesState(). Error is simply
   * the desired state minus the actual state (ex. roll_des - roll_IMU). Two
   * safety features are implimented here regarding the I terms. The I terms are
   * saturated within specified limits on startup to prevent excessive buildup.
   * This can be seen by holding the vehicle at an angle and seeing the motors
   * ramp up on one side until they've maxed out throttle...saturating I to a
   * specified limit fixes this. The second feature defaults the I terms to 0 if
   * the throttle is at the minimum setting. This means the motors will not
   * start spooling up on the ground, and the I terms will always start from 0
   * on takeoff. This function updates the variables roll_PID, pitch_PID, and
   * yaw_PID which can be thought of as 1-D stablized signals. They are mixed to
   * the configuration of the vehicle in controlMixer().
   */
  float GyroZ;
  float GyroY;
  float GyroX;
#if defined USE_MPU6050_I2C
  GyroZ = GyroZ_6050;
  GyroY = GyroY_6050;
  GyroX = -GyroX_6050; // 安装位置
#endif
#if defined USE_MPU9250_SPI
  GyroZ = GyroZ_9250;
  GyroY = GyroY_9250;
  GyroX = -GyroX_9250;
#endif
  // rotate_speed

  rotate_error = rotate_speed_des * 1000.0 - GyroZ;
  rotate_error = constrain(rotate_error, -100, 100); // 100度每秒
  // thro_des=0.01*kp_rotate*rotate_error;//0-1

  // Roll
  error_roll = roll_des - roll_IMU;
  integral_roll = integral_roll_prev + error_roll * dt;
  if (channel_3_pwm <
      1160) { // Don't let integrator build if throttle is too low
    integral_roll = 0;
  }
  integral_roll =
      constrain(integral_roll, -i_limit,
                i_limit); // Saturate integrator to prevent unsafe buildup
  derivative_roll = GyroX;
  roll_PID =
      0.01 *
      (Kp_roll_angle * error_roll + Ki_roll_angle * integral_roll -
       Kd_roll_angle *
           derivative_roll); // Scaled by .01 to bring within -1 to 1 range

  // Pitch
  error_pitch = pitch_des - pitch_IMU;
  integral_pitch = integral_pitch_prev + error_pitch * dt;
  if (channel_3_pwm <
      1160) { // Don't let integrator build if throttle is too low
    integral_pitch = 0;
  }
  integral_pitch =
      constrain(integral_pitch, -i_limit,
                i_limit); // Saturate integrator to prevent unsafe buildup
  derivative_pitch = GyroY;
  pitch_PID =
      .01 *
      (Kp_pitch_angle * error_pitch + Ki_pitch_angle * integral_pitch -
       Kd_pitch_angle *
           derivative_pitch); // Scaled by .01 to bring within -1 to 1 range

  // Yaw, stablize on rate from GyroZ

  // yaw_des=yaw_des;
  error_yaw =
      yaw_des -
      57.3 * (9.8 / V_cruise * tan(roll_des / 57.3) * cos(pitch_des / 57.3)) -
      GyroZ;
  integral_yaw = integral_yaw_prev + error_yaw * dt;
  if (channel_3_pwm <
      1160) { // Don't let integrator build if throttle is too low
    integral_yaw = 0;
  }
  integral_yaw =
      constrain(integral_yaw, -i_limit,
                i_limit); // Saturate integrator to prevent unsafe buildup
  derivative_yaw = (error_yaw - error_yaw_prev) / dt;
  yaw_PID =
      .01 *
      (Kp_yaw * error_yaw + Ki_yaw * integral_yaw +
       Kd_yaw * derivative_yaw); // Scaled by .01 to bring within -1 to 1 range

  // Update roll variables
  integral_roll_prev = integral_roll;
  // Update pitch variables
  integral_pitch_prev = integral_pitch;
  // Update yaw variables
  error_yaw_prev = error_yaw;
  integral_yaw_prev = integral_yaw;
}

void increase_Clp() {
  float GyroX;
#if defined EXTIMU
  GyroX = Gyro_X_EXT;

#endif
#if defined INTIMU

  GyroX = -GyroX_6050; // 安装位置
#endif

  Clp_PID = .01 * k_Clp * (0 - GyroX);
  Ail_Clp = Clp_PID * 125.0;
}

void controlANGLE2() {
  // DESCRIPTION: Computes control commands based on state error (angle) in
  // cascaded scheme
  /*
   * Gives better performance than controlANGLE() but requires much more tuning.
   * Not reccommended for first-time setup. See the documentation for tuning
   * this controller.
   */
  float GyroZ;
  float GyroY;
  float GyroX;
#if defined USE_MPU6050_I2C
  GyroZ = GyroZ_6050;
  GyroY = GyroY_6050;
  GyroX = -GyroX_6050; // 安装位置

  GyroZ = gyroFiltZ.apply(GyroZ_6050);
  GyroY = gyroFiltY.apply(GyroY_6050);
  GyroX = -gyroFiltX.apply(GyroX_6050); // 安装位置

  // Serial.print(GyroY_6050);
  // Serial.print(" ");
  // Serial.println(GyroY);
#endif
#if defined USE_MPU9250_SPI
  GyroZ = GyroZ_9250;
  GyroY = GyroY_9250;
  GyroX = -GyroX_9250;
#endif

#if defined EXTIMU
  GyroX = Gyro_X_EXT;
  GyroY = Gyro_Y_EXT;
  GyroZ = Gyro_Z_EXT;
  roll_IMU = roll_IMU_EXT;
  pitch_IMU = pitch_IMU_EXT;
  yaw_IMU = yaw_IMU_EXT;
#endif

  // Outer loop - PID on angle
  float roll_des_ol, pitch_des_ol;
// Roll
#if defined SINGLE
  roll_eq = roll_IMU;
#else

  float phiA, phiB, phiC, phiD, phiE, phiF, phiG;

#if defined userotatesensor
  phiA = roll_IMU / 57.3;
  phiB = (roll_IMU + relativeAngle_ready) / 57.3;
  phiC = (roll_IMU + phiac) / 57.3;
  phiD = (roll_IMU + relativeAngle_ready + phibd) / 57.3;
  phiE = (roll_IMU + phiac + phice) / 57.3;
#else
  // 直接测量的方式
  phiA = roll_IMU / 57.3;
  phiB = phiB_raw / 57.3;
  phiC = phiC_raw / 57.3;
  phiD = phiD_raw / 57.3;
  phiE = phiE_raw / 57.3;
  phiF = phiF_raw / 57.3;
  phiG = phiG_raw / 57.3;

#endif

// roll_eq=(roll_IMU+roll_IMU+relativeAngle_ready)/2.0; //2机等效姿态
#if defined THREEPLANE
  roll_eq = atan((sin(phiA) + sin(phiB) + sin(phiC)) /
                 (cos(phiA) + cos(phiB) + cos(phiC))) *
            57.3; // 3机等效姿态
#elif defined FOURPLANE
  roll_eq = atan((sin(phiA) + sin(phiB) + sin(phiC) + sin(phiD)) /
                 (cos(phiA) + cos(phiB) + cos(phiC) + cos(phiD))) *
            57.3; // 4机等效姿态
#elif defined FIVEPLANE
  roll_eq = atan((sin(phiA) + sin(phiB) + sin(phiC) + sin(phiD) + sin(phiE)) /
                 (cos(phiA) + cos(phiB) + cos(phiC) + cos(phiD) + cos(phiE))) *
            57.3; // 5机等效姿态
#elif defined SEVENPLANE
  roll_eq = atan((sin(phiA) + sin(phiB) + sin(phiC) + sin(phiD) + sin(phiE) +
                  sin(phiF) + sin(phiG)) /
                 (cos(phiA) + cos(phiB) + cos(phiC) + cos(phiD) + cos(phiE) +
                  cos(phiF) + cos(phiG))) *
            57.3; // 7机等效姿态
#endif
#endif

  error_roll = roll_des - roll_eq;
  integral_roll_ol = integral_roll_prev_ol + error_roll * dt; // I
  if (channel_1_pwm <
      1060) { // Don't let integrator build if throttle is too low
    integral_roll_ol = 0;
  }
  integral_roll_ol =
      constrain(integral_roll_ol, -i_limit,
                i_limit); // Saturate integrator to prevent unsafe buildup
  if (ModeChange == 1) {
    integral_roll_ol = 0;
  }
  derivative_roll = (roll_IMU - roll_IMU_prev) / dt;
  roll_des_ol = Kp_roll_angle * error_roll +
                i_valid * Ki_roll_angle *
                    integral_roll_ol; // - Kd_roll_angle*derivative_roll;

  // Pitch
  // Serial.println(pitch_des_local);

#if defined APLANE

#if defined EVEN
  error_pitch = pitch_des_local + central_pitch - pitch_IMU;
#elif defined ODD
  error_pitch = pitch_des_local - pitch_IMU;
#endif

#else
  error_pitch = pitch_des_local - pitch_IMU;
#endif

  // Serial.println(error_pitch);
  // error_pitch = pitch_des_local - pitch_IMU;
  integral_pitch_ol = integral_pitch_prev_ol + error_pitch * dt;
  if (channel_1_pwm <
      1060) { // Don't let integrator build if throttle is too low
    integral_pitch_ol = 0;
  }
  integral_pitch_ol =
      constrain(integral_pitch_ol, -i_limit,
                i_limit); // saturate integrator to prevent unsafe buildup
  if (ModeChange == 1) {
    integral_pitch_ol = 0;
  }
  derivative_pitch = (pitch_IMU - pitch_IMU_prev) / dt;
  pitch_des_ol = Kp_pitch_angle * error_pitch +
                 i_valid * Ki_pitch_angle *
                     integral_pitch_ol; // - Kd_pitch_angle*derivative_pitch;

  // Apply loop gain, constrain, and LP filter for artificial damping
  float Kl = 30.0;
  roll_des_ol = Kl * roll_des_ol;
  pitch_des_ol = Kl * pitch_des_ol;
  pitch_des_ol = (pitch_des_ol + sin(roll_IMU / 57.3) * -GyroZ) /
                 constrain(cos(roll_IMU / 57.3), 0.5, 1);
  roll_des_ol = constrain(roll_des_ol, -240.0, 240.0);
  pitch_des_ol = constrain(pitch_des_ol, -240.0, 240.0);
  roll_des_ol = (1.0 - B_loop_roll) * roll_des_prev + B_loop_roll * roll_des_ol;
  pitch_des_ol =
      (1.0 - B_loop_pitch) * pitch_des_prev + B_loop_pitch * pitch_des_ol;

  // Inner loop - PID on rate
  // Roll
  float Rollrate;
#if defined SINGLE
  Rollrate = GyroX;
#else
// Rollrate=(GYRO_X_B+GyroX)/2.0;
#if defined THREEPLANE
  Rollrate = (GYRO_X_B + GYRO_X_C + GyroX) / 3.0;
#elif defined FIVEPLANE
  Rollrate = (GYRO_X_B + GYRO_X_C + GYRO_X_D + GyroX) / 4.0;
#elif defined FIVEPLANE
  Rollrate = (GYRO_X_B + GYRO_X_C + GYRO_X_D + GYRO_X_E + GyroX) / 5.0;
#elif defined SEVENPLANE
  Rollrate = (GYRO_X_B + GYRO_X_C + GYRO_X_D + GYRO_X_E + GYRO_X_F + GYRO_X_G +
              GyroX) /
             7.0; // 7机等效滚转角速度
#endif
#endif
  // Serial.println(Rollrate);

  error_roll = roll_des_ol - Rollrate;
  integral_roll_il = integral_roll_prev_il + error_roll * dt;
  if (channel_1_pwm <
      1060) { // Don't let integrator build if throttle is too low
    integral_roll_il = 0;
  }
  integral_roll_il =
      constrain(integral_roll_il, -i_limit,
                i_limit); // Saturate integrator to prevent unsafe buildup
  if (ModeChange == 1) {
    integral_roll_il = 0;
  }
  derivative_roll = (error_roll - error_roll_prev) / dt;
  roll_PID =
      .01 *
      (Kff_roll_rate * roll_des_ol + Kp_roll_rate * error_roll +
       i_valid * Ki_roll_rate * integral_roll_il +
       Kd_roll_rate *
           derivative_roll); // Scaled by .01 to bring within -1 to 1 range

  float tau_roll_pid = 1.0f / (6.28f * roll_pid_lpf_fc);
  float alpha_roll_pid = dt / (tau_roll_pid + dt);
  roll_PID_lpf = roll_PID_lpf + alpha_roll_pid * (roll_PID - roll_PID_lpf);

  // 指令的变化率
  if (!roll_pid_dot_initialized) {
    roll_PID_prev = roll_PID;
    roll_PID_dot = 0.0f;
    roll_PID_dot_lpf = 0.0f;
    roll_pid_dot_initialized = true;
  } else {
    if (dt > 1.0e-6f) {
      roll_PID_dot = (roll_PID - roll_PID_prev) / dt;
    } else {
      roll_PID_dot = 0.0f;
    }

    if (roll_pid_dot_lpf_fc > 0.0f && dt > 1.0e-6f) {
      float tau_dot = 1.0f / (6.28f * roll_pid_dot_lpf_fc);
      float alpha_dot = dt / (tau_dot + dt);
      roll_PID_dot_lpf =
          roll_PID_dot_lpf + alpha_dot * (roll_PID_dot - roll_PID_dot_lpf);
    } else {
      roll_PID_dot_lpf = roll_PID_dot;
    }
    roll_PID_prev = roll_PID;
  }

  // Serial.println(roll_PID_dot_lpf);

  // Pitch
  error_pitch = pitch_des_ol - GyroY;
  integral_pitch_il = integral_pitch_prev_il + error_pitch * dt;
  if (channel_1_pwm <
      1060) { // Don't let integrator build if throttle is too low
    integral_pitch_il = 0;
  }
  integral_pitch_il =
      constrain(integral_pitch_il, -i_limit,
                i_limit); // Saturate integrator to prevent unsafe buildup
  if (ModeChange == 1) {
    integral_pitch_il = 0;
  }
  derivative_pitch = (error_pitch - error_pitch_prev) / dt;
  pitch_PID =
      .01 *
      (Kff_pitch_rate * pitch_des_ol + Kp_pitch_rate * error_pitch +
       i_valid * Ki_pitch_rate * integral_pitch_il +
       Kd_pitch_rate *
           derivative_pitch); // Scaled by .01 to bring within -1 to 1 range
  // Serial.println(integral_pitch_il);
  // Yaw
  error_yaw =
      yaw_des -
      57.3 * (9.8 / V_cruise * tan(roll_des / 57.3) * cos(pitch_des / 57.3)) -
      GyroZ;
  integral_yaw = integral_yaw_prev + error_yaw * dt;
  if (channel_1_pwm <
      1060) { // Don't let integrator build if throttle is too low
    integral_yaw = 0;
  }
  integral_yaw =
      constrain(integral_yaw, -i_limit,
                i_limit); // Saturate integrator to prevent unsafe buildup
  if (ModeChange == 1) {
    integral_yaw = 0;
  }
  derivative_yaw = (error_yaw - error_yaw_prev) / dt;
  yaw_PID =
      .01 *
      (Kff_yaw_rate * (yaw_des - 57.3 * (9.8 / V_cruise * tan(roll_des / 57.3) *
                                         cos(pitch_des / 57.3))) +
       Kp_yaw * error_yaw + Ki_yaw * integral_yaw +
       Kd_yaw * derivative_yaw); // Scaled by .01 to bring within -1 to 1 range

  // Update roll variables
  integral_roll_prev_ol = integral_roll_ol;
  integral_roll_prev_il = integral_roll_il;
  error_roll_prev = error_roll;
  roll_IMU_prev = roll_IMU;
  roll_des_prev = roll_des_ol;
  // Update pitch variables
  integral_pitch_prev_ol = integral_pitch_ol;
  integral_pitch_prev_il = integral_pitch_il;
  error_pitch_prev = error_pitch;
  pitch_IMU_prev = pitch_IMU;
  pitch_des_prev = pitch_des_ol;
  // Update yaw variables
  error_yaw_prev = error_yaw;
  integral_yaw_prev = integral_yaw;
}

void controlRATE() {
  // DESCRIPTION: Computes control commands based on state error (rate)
  /*
   * See explanation for controlANGLE(). Everything is the same here except the
   * error is now the desired rate - raw gyro reading.
   */

  float GyroZ;
  float GyroY;
  float GyroX;
#if defined USE_MPU6050_I2C
  GyroZ = GyroZ_6050;
  GyroY = GyroY_6050;
  GyroX = -GyroX_6050;
#endif
#if defined USE_MPU9250_SPI
  GyroZ = GyroZ_9250;
  GyroY = GyroY_9250;
  GyroX = -GyroX_9250;
#endif
#if defined EXTIMU
  GyroX = Gyro_X_EXT;
  GyroY = Gyro_Y_EXT;
  GyroZ = Gyro_Z_EXT;
#endif

  // Roll
  error_roll = roll_des * 3.0 - GyroX;
  integral_roll = integral_roll_prev + error_roll * dt;
  if (channel_1_pwm <
      1060) { // Don't let integrator build if throttle is too low
    integral_roll = 0;
  }
  integral_roll =
      constrain(integral_roll, -i_limit,
                i_limit); // Saturate integrator to prevent unsafe buildup
  derivative_roll = (error_roll - error_roll_prev) / dt;
  roll_PID =
      .01 *
      (Kff_roll_rate * roll_des + Kp_roll_rate * error_roll +
       Kd_roll_rate *
           derivative_roll); // Scaled by .01 to bring within -1 to 1 range

  // Pitch
  error_pitch = pitch_des_local * 3.0 - GyroY;
  integral_pitch = integral_pitch_prev + error_pitch * dt;
  if (channel_1_pwm <
      1060) { // Don't let integrator build if throttle is too low
    integral_pitch = 0;
  }
  integral_pitch =
      constrain(integral_pitch, -i_limit,
                i_limit); // Saturate integrator to prevent unsafe buildup
  derivative_pitch = (error_pitch - error_pitch_prev) / dt;
  pitch_PID =
      .01 *
      (Kff_pitch_rate * pitch_des + Kp_pitch_rate * error_pitch +
       Kd_pitch_rate *
           derivative_pitch); // Scaled by .01 to bring within -1 to 1 range

  // Yaw, stablize on rate from GyroZ
  error_yaw = yaw_des * 3.0 - GyroZ;
  integral_yaw = integral_yaw_prev + error_yaw * dt;
  if (channel_1_pwm <
      1060) { // Don't let integrator build if throttle is too low
    integral_yaw = 0;
  }
  integral_yaw =
      constrain(integral_yaw, -i_limit,
                i_limit); // Saturate integrator to prevent unsafe buildup
  derivative_yaw = (error_yaw - error_yaw_prev) / dt;
  yaw_PID =
      .01 *
      (Kff_yaw_rate * yaw_des + Kp_yaw * error_yaw + Ki_yaw * integral_yaw +
       Kd_yaw * derivative_yaw); // Scaled by .01 to bring within -1 to 1 range

  // Update roll variables
  error_roll_prev = error_roll;
  integral_roll_prev = integral_roll;
  // GyroX_prev = GyroX;//这行有什么用啊 似乎没用 而且不会干扰getimudata吗
  // Update pitch variables
  error_pitch_prev = error_pitch;
  integral_pitch_prev = integral_pitch;
  // GyroY_prev = GyroY;//这行有什么用啊 似乎没用 而且不会干扰getimudata吗
  // Update yaw variables
  error_yaw_prev = error_yaw;
  integral_yaw_prev = integral_yaw;
}

int PITCH_INDI_control() {
  // 1. 读取当前俯仰角速度原始值。
  // 这里先只统一到当前工程已经在用的角速度定义，不改变原有轴系约定。
  float GyroY_raw;
#if defined USE_MPU6050_I2C
  GyroY_raw = GyroY_6050;
#endif
#if defined USE_MPU9250_SPI
  GyroY_raw = GyroY_9250;
#endif
#if defined EXTIMU
  GyroY_raw = Gyro_Y_EXT;
#endif

  // 2. 使用独立的低通滤波器得到 INDI 专用的俯仰角速度 q。
  // 这样做有两个目的：
  // - 不复用现有 controlANGLE2() 里的滤波状态，避免互相串扰；
  // - 让 q 与 dq 的噪声口径更接近，减小 INDI 中 q 和 dq 不一致的问题。
  float GyroY_filt = 0.0f;
  const int queue_len = 64;
  static bool indi_initialized = false;
  static float delayed_pwm_queue[queue_len];
  static int queue_head = 0;
  static float delta_e_est_deg = 0.0f;
  static float delta_e_cmd_prev_deg = 0.0f;
  static float delta_e_cmd_lpf_prev_deg = 0.0f;

  // 3. 如果线性映射或舵效系数还没有填写，直接返回一个安全中位 PWM。
  // 这里是保护逻辑，避免参数未填完时函数输出发散。
  if (fabsf(indi_pitch_pwm_to_deg_k) < 1.0e-6f ||
      fabsf(indi_pitch_effectiveness) < 1.0e-6f) {
    indi_pitch_q_des_log = 0.0f;
    indi_pitch_q_filt_log = 0.0f;
    indi_pitch_dq_des_log = 0.0f;
    indi_pitch_dq_used_log = dq;
    indi_pitch_delta_e_cmd_deg_log = 0.0f;
    indi_pitch_delta_e_est_deg_log = eleprev;
    indi_pitch_pwm_cmd_log = 1520.0f + pwm_channel3_trim;
    return constrain((int)(1520 + pwm_channel3_trim), (int)indi_pitch_pwm_min,
                     (int)indi_pitch_pwm_max);
  }

  // 4. 计算 trim 对应的舵偏角。
  // 后面所有舵机状态初始化都以这个配平点为基准，而不是默认 0 度舵偏。
  const float pwm_trim_center = 1520.0f + pwm_channel3_trim;
  const float delta_e_trim_deg =
      indi_pitch_pwm_to_deg_k * pwm_trim_center + indi_pitch_pwm_to_deg_b;

  // 5. 第一次进入函数时：
  // - 初始化独立滤波器的内部状态
  // - 初始化纯延迟队列
  // - 初始化实际舵偏估计值和上一次指令值
  if (!indi_initialized) {
    gyroFiltYIndi.reset(GyroY_raw);
    for (int i = 0; i < queue_len; i++) {
      delayed_pwm_queue[i] = pwm_trim_center;
    }
    delta_e_est_deg = delta_e_trim_deg;
    delta_e_cmd_prev_deg = delta_e_trim_deg;
    delta_e_cmd_lpf_prev_deg = delta_e_trim_deg;
    eleprev = delta_e_trim_deg;
    indi_initialized = true;
  }
  GyroY_filt = gyroFiltYIndi.apply(GyroY_raw);

  // 6. 先计算俯仰角误差，再由角度误差生成期望俯仰角速度 q_des。
  // 这里做的是“外环角度误差 -> 内环角速度目标”的转换，
  // 所以不能直接拿 pitch_des_local 本身当作角速度目标，必须减去当前俯仰角。

  float pitch_des_local_rate_raw;
  static bool pitch_des_local_rate_lpf_initialized = false;
  if (dt > 1.0e-6f) {
    pitch_des_local_rate_raw = (pitch_des_local - pitch_des_local_last) / dt;
  } else {
    pitch_des_local_rate_raw = 0.0f;
  }
  pitch_des_local_last = pitch_des_local;
  pitch_des_local_rate_raw = constrain(pitch_des_local_rate_raw, -10.0, 10.0);

  if (!pitch_des_local_rate_lpf_initialized) {
    pitch_des_local_rate = pitch_des_local_rate_raw;
    pitch_des_local_rate_lpf_initialized = true;
  } else if (pitch_des_local_rate_lpf_fc > 0.0f && dt > 1.0e-6f) {
    float tau_pitch_des_rate =
        1.0f / (6.28318530718f * pitch_des_local_rate_lpf_fc);
    float alpha_pitch_des_rate = dt / (tau_pitch_des_rate + dt);
    pitch_des_local_rate = pitch_des_local_rate +
                           alpha_pitch_des_rate * (pitch_des_local_rate_raw -
                                                   pitch_des_local_rate);
  } else {
    pitch_des_local_rate = pitch_des_local_rate_raw;
  }

  float pitch_error_deg = pitch_des_local - pitch_IMU;
  float rate_des_deg_s = pitch_error_deg * 3.0f + pitch_des_local_rate;
  float q_des_deg_s = (rate_des_deg_s + sin(roll_IMU / 57.3) *
                                            -gyroFiltZIndi.apply(GyroZ_6050)) /
                      cos(roll_IMU / 57.3);
  q_des_deg_s = constrain(q_des_deg_s, -120, 120);

  // 7. 由速率误差生成期望角加速度 dq_des。
  // 这一步是 INDI 的外层：先问“我希望产生多大的俯仰角加速度”。
  float dq_des_deg_s2 = indi_pitch_q_gain * (q_des_deg_s - GyroY_filt);
  // Serial.println(dq_des_deg_s2);
  //  8. INDI 增量控制律。
  //  舵效 indi_pitch_effectiveness 的物理意义是：
  //    dq / delta_e
  //  所以这里用 (dq_des - dq_meas) / 舵效，得到还需要增加多少舵偏角。
  float delta_e_indi_deg =
      0.7f * (dq_des_deg_s2 - dq) / indi_pitch_effectiveness;

  // 9. 以“当前实际舵偏估计值”为基准，加上 INDI 算出的增量，得到新的舵偏目标。
  float delta_e_cmd_deg = delta_e_est_deg + delta_e_indi_deg;

  // 10. 对舵偏变化率做限幅。
  // 这一步的目的是避免 INDI 因 dq
  // 噪声或舵效估计误差，一次跳出非常大的舵偏命令。
  float max_delta_deg = indi_pitch_rate_limit_deg_s * dt;
  float delta_step_deg = delta_e_cmd_deg - delta_e_cmd_prev_deg;
  delta_step_deg = constrain(delta_step_deg, -max_delta_deg, max_delta_deg);
  delta_e_cmd_deg = delta_e_cmd_prev_deg + delta_step_deg;

  // 11. 对舵偏角本身做限幅。
  // 这里建议你后续直接填真实可用的升降舵最大/最小偏角。
  delta_e_cmd_deg = constrain(delta_e_cmd_deg, indi_pitch_deflection_min_deg,
                              indi_pitch_deflection_max_deg);

  // 12. 对最终输出舵偏指令再做一阶低通。
  // 这一步压的是“打给舵机的命令带宽”，不是状态反馈带宽。
  // 这样可以减少 INDI 因 dq 噪声带来的高频抖舵。
  if (indi_pitch_cmd_lpf_fc_hz > 0.0f && dt > 1.0e-6f) {
    float tau_cmd = 1.0f / (6.28318530718f * indi_pitch_cmd_lpf_fc_hz);
    float alpha_cmd = dt / (tau_cmd + dt);
    delta_e_cmd_deg = delta_e_cmd_lpf_prev_deg +
                      alpha_cmd * (delta_e_cmd_deg - delta_e_cmd_lpf_prev_deg);
  }
  delta_e_cmd_deg = constrain(delta_e_cmd_deg, indi_pitch_deflection_min_deg,
                              indi_pitch_deflection_max_deg);
  delta_e_cmd_prev_deg = delta_e_cmd_deg;
  delta_e_cmd_lpf_prev_deg = delta_e_cmd_deg;

  // 13. 将目标舵偏角通过你测得的线性映射，反算成舵机 PWM。
  // 若你的关系是 delta_e = k * pwm + b，则反解为 pwm = (delta_e - b) / k。
  float pwm_cmd =
      (delta_e_cmd_deg - indi_pitch_pwm_to_deg_b) / indi_pitch_pwm_to_deg_k;
  pwm_cmd = constrain(pwm_cmd, indi_pitch_pwm_min, indi_pitch_pwm_max);

  // 14. 将“刚刚发出的 PWM 命令”压入纯延迟队列。
  // 后面实际舵偏估计器不会立刻看到这个命令，而是等 delay_samples
  // 个周期以后才看到。
  delayed_pwm_queue[queue_head] = pwm_cmd;
  queue_head = (queue_head + 1) % queue_len;

  // 15. 将你测到的 0.03s 舵机纯延迟，离散成若干个采样周期。
  int delay_samples = 0;
  if (dt > 1.0e-6f) {
    delay_samples = (int)(indi_pitch_servo_delay_s / dt + 0.5f);
  }
  delay_samples = constrain(delay_samples, 0, queue_len - 1);

  // 16. 从延迟队列中取出“经过纯延迟后才真正进入舵机的一条 PWM 命令”。
  int delayed_index = queue_head - 1 - delay_samples;
  if (delayed_index < 0) {
    delayed_index += queue_len;
  }
  float pwm_delayed = delayed_pwm_queue[delayed_index];
  float delta_e_delayed_deg =
      indi_pitch_pwm_to_deg_k * pwm_delayed + indi_pitch_pwm_to_deg_b;

  // 17. 用一阶舵机动态模型更新“当前实际舵偏估计值”。
  // 这就是你测到的 tau = 0.015s 所对应的执行器模型。
  float tau = indi_pitch_servo_tau_s;
  if (tau < 1.0e-4f) {
    delta_e_est_deg = delta_e_delayed_deg;
  } else {
    float alpha_servo = dt / (tau + dt);
    delta_e_est_deg += alpha_servo * (delta_e_delayed_deg - delta_e_est_deg);
  }

  // 18. eleprev 保留为“当前估计的实际升降舵偏角”，方便你后续打 log 或直接观察。
  eleprev = delta_e_est_deg;
  indi_pitch_q_des_log = q_des_deg_s;
  indi_pitch_q_filt_log = GyroY_filt;
  indi_pitch_dq_des_log = dq_des_deg_s2;
  indi_pitch_dq_used_log = dq;
  indi_pitch_delta_e_cmd_deg_log = delta_e_cmd_deg;
  indi_pitch_delta_e_est_deg_log = delta_e_est_deg;
  indi_pitch_pwm_cmd_log = pwm_cmd;

  // 18. 函数最终只返回一个独立计算好的升降舵 PWM。
  // 目前它不会自动接入现有控制链，只有你显式调用它时才会生效。
  return (int)(pwm_cmd + 0.5f);
}

void scaleCommands() {
  // DESCRIPTION: Scale normalized actuator commands to values for ESC/Servo
  // protocol
  /*
   * mX_command_scaled variables from the mixer function are scaled to 125-250us
   * for OneShot125 protocol. sX_command_scaled variables from the mixer
   * function are scaled to 0-180 for the servo library using standard PWM.
   * mX_command_PWM are updated here which are used to command the motors in
   * commandMotors(). sX_command_PWM are updated which are used to command the
   * servos.
   */

  // Scaled to 1100-1940  mid 1520 for servo library
  Aail1_PWM = 1520 + 1000 * (Aail1_scaled);
  Aail2_PWM = 1520 + 1000 * (Aail2_scaled);
  Aele_PWM = 1520 + 1000 * (Aele_scaled);
  Athro_PWM = 1100 + 1000 * (Athro_scaled);
  Arudd_PWM = 1520 + 1000 * (Arudd_scaled);

  Bail1_PWM = 1520 + 1000 * (Bail1_scaled);
  Bail2_PWM = 1520 + 1000 * (Bail2_scaled);
  Bthro_PWM = 1100 + 1000 * (Bthro_scaled);
  Brudd_PWM = 1520 + 1000 * (Brudd_scaled);
  B_ele_command_PWM_Manual = 1520 + 1000 * (Aele_scaled) +
                             0.5 * 1000 * (Bail1_scaled + Bail2_scaled) / 2;

  Cail1_PWM = 1520 + 1000 * (Cail1_scaled);
  Cail2_PWM = 1520 + 1000 * (Cail2_scaled);
  Cthro_PWM = 1100 + 1000 * (Cthro_scaled);
  Crudd_PWM = 1520 + 1000 * (Crudd_scaled);
  C_ele_command_PWM_Manual = 1520 + 1000 * (Aele_scaled) +
                             0.5 * 1000 * (Cail1_scaled + Cail2_scaled) / 2;

  Dail1_PWM = 1520 + 1000 * (Dail1_scaled);
  Dail2_PWM = 1520 + 1000 * (Dail2_scaled);
  Dthro_PWM = 1100 + 1000 * (Dthro_scaled);
  Drudd_PWM = 1520 + 1000 * (Drudd_scaled);
  D_ele_command_PWM_Manual = 1520 + 1000 * (Aele_scaled) +
                             0.5 * 1000 * (Dail1_scaled + Dail2_scaled) / 2;

  Eail1_PWM = 1520 + 1000 * (Eail1_scaled);
  Eail2_PWM = 1520 + 1000 * (Eail2_scaled);
  Ethro_PWM = 1100 + 1000 * (Ethro_scaled);
  Erudd_PWM = 1520 + 1000 * (Erudd_scaled);
  E_ele_command_PWM_Manual = 1520 + 1000 * (Aele_scaled) +
                             0.5 * 1000 * (Eail1_scaled + Eail2_scaled) / 2;

  Fail1_PWM = 1520 + 1000 * (Fail1_scaled);
  Fail2_PWM = 1520 + 1000 * (Fail2_scaled);
  Fthro_PWM = 1100 + 1000 * (Fthro_scaled);
  Frudd_PWM = 1520 + 1000 * (Frudd_scaled);
  F_ele_command_PWM_Manual = 1520 + 1000 * (Aele_scaled) +
                             0.5 * 1000 * (Fail1_scaled + Fail2_scaled) / 2;

  Gail1_PWM = 1520 + 1000 * (Gail1_scaled);
  Gail2_PWM = 1520 + 1000 * (Gail2_scaled);
  Gthro_PWM = 1100 + 1000 * (Gthro_scaled);
  Grudd_PWM = 1520 + 1000 * (Grudd_scaled);
  G_ele_command_PWM_Manual = 1520 + 1000 * (Aele_scaled) +
                             0.5 * 1000 * (Gail1_scaled + Gail2_scaled) / 2;

  s6_command_PWM = s6_command_scaled * 180;
  s7_command_PWM = s7_command_scaled * 180;
  // Constrain commands to servos within servo library bounds
  Aail1_PWM = constrain(Aail1_PWM, 1100, 1920);
  Aail2_PWM = constrain(Aail2_PWM, 1100, 1920);
  Aele_PWM = constrain(Aele_PWM, 1100, 1920); // 贵的飞机限幅
  Athro_PWM = constrain(Athro_PWM, 1100, 1920);
  Arudd_PWM = constrain(Arudd_PWM, 1100, 1920);
  s6_command_PWM = constrain(s6_command_PWM, 0, 180);
  s7_command_PWM = constrain(s7_command_PWM, 0, 180);
}

void getCommands() {
  // DESCRIPTION: Get raw PWM values for every channel from the radio
  /*
   * Updates radio PWM commands in loop based on current available commands.
   * channel_x_pwm is the raw command used in the rest of the loop. If using a
   * PWM or PPM receiver, the radio commands are retrieved from a function in
   * the readPWM file separate from this one which is running a bunch of
   * interrupts to continuously update the radio readings. If using an SBUS
   * receiver, the alues are pulled from the SBUS library directly. The raw
   * radio commands are filtered with a first order low-pass filter to eliminate
   * any really high frequency noise.
   */

#if defined USE_PPM_RX || defined USE_PWM_RX
  channel_1_pwm = getRadioPWM(1);
  channel_2_pwm = getRadioPWM(2);
  channel_3_pwm = getRadioPWM(3);
  channel_4_pwm = getRadioPWM(4);
  channel_5_pwm = getRadioPWM(5);
  channel_6_pwm = getRadioPWM(6);

#elif defined USE_SBUS_RX
  if (sbus.read(&sbusChannels[0], &sbusFailSafe, &sbusLostFrame)) {
    // sBus scaling below is for Taranis-Plus and X4R-SB
    float scale = 0.615;
    float bias = 895.0;
    channel_1_pwm = sbusChannels[0] * scale + bias;
    channel_2_pwm = sbusChannels[1] * scale + bias;
    channel_3_pwm = sbusChannels[2] * scale + bias;
    channel_4_pwm = sbusChannels[3] * scale + bias;
    channel_5_pwm = sbusChannels[4] * scale + bias;
    channel_6_pwm = sbusChannels[5] * scale + bias;
    channel_7_pwm = sbusChannels[6] * scale + bias;
    channel_8_pwm = sbusChannels[7] * scale + bias;
  }

#elif defined USE_DSM_RX
  if (DSM.timedOut(micros())) {
    // Serial.println("*** DSM RX TIMED OUT ***");
  } else if (DSM.gotNewFrame()) {
    uint16_t values[num_DSM_channels];
    DSM.getChannelValues(values, num_DSM_channels);

    channel_1_pwm = values[0];
    channel_2_pwm = values[1];
    channel_3_pwm = values[2];
    channel_4_pwm = values[3];
    channel_5_pwm = values[4];
    channel_6_pwm = values[5];
  }
#endif

  // Low-pass the critical commands and update previous values
  float b = 0.7; // Lower=slower, higher=noiser
  channel_1_pwm = (1.0 - b) * channel_1_pwm_prev + b * channel_1_pwm;
  channel_2_pwm = (1.0 - b) * channel_2_pwm_prev + b * channel_2_pwm;
  channel_3_pwm = (1.0 - b) * channel_3_pwm_prev + b * channel_3_pwm;
  channel_4_pwm = (1.0 - b) * channel_4_pwm_prev + b * channel_4_pwm;
  channel_1_pwm_prev = channel_1_pwm;
  channel_2_pwm_prev = channel_2_pwm;
  channel_3_pwm_prev = channel_3_pwm;
  channel_4_pwm_prev = channel_4_pwm;
}

void failSafe() {
  // DESCRIPTION: If radio gives garbage values, set all commands to default
  // values
  /*
   * Radio connection failsafe used to check if the getCommands() function is
   * returning acceptable pwm values. If any of the commands are lower than 800
   * or higher than 2200, then we can be certain that there is an issue with the
   * radio connection (most likely hardware related). If any of the channels
   * show this failure, then all of the radio commands channel_x_pwm are set to
   * default failsafe values specified in the setup. Comment out this function
   * when troubleshooting your radio connection in case any extreme values are
   * triggering this function to overwrite the printed variables.
   */
  unsigned minVal = 800;
  unsigned maxVal = 2200;
  int check1 = 0;
  int check2 = 0;
  int check3 = 0;
  int check4 = 0;
  int check5 = 0;
  int check6 = 0;
  int check7 = 0;
  int check8 = 0;
  int check9 = 0;
  int check10 = 0;

  // Triggers for failure criteria
  if (channel_1_pwm > maxVal || channel_1_pwm < minVal)
    check1 = 1;
  if (channel_2_pwm > maxVal || channel_2_pwm < minVal)
    check2 = 1;
  if (channel_3_pwm > maxVal || channel_3_pwm < minVal)
    check3 = 1;
  if (channel_4_pwm > maxVal || channel_4_pwm < minVal)
    check4 = 1;
  if (channel_5_pwm > maxVal || channel_5_pwm < minVal)
    check5 = 1;
  if (channel_6_pwm > maxVal || channel_6_pwm < minVal)
    check6 = 1;
  if (channel_7_pwm > maxVal || channel_7_pwm < minVal)
    check7 = 1;
  if (channel_8_pwm > maxVal || channel_8_pwm < minVal)
    check8 = 1;

  // If any failures, set to default failsafe values
  if ((check1 + check2 + check3 + check4 + check5 + check6 + check7 + check8) >
      0) {
    channel_1_pwm = channel_1_fs;
    channel_2_pwm = channel_2_fs;
    channel_3_pwm = channel_3_fs;
    channel_4_pwm = channel_4_fs;
    channel_5_pwm = channel_5_fs;
    channel_6_pwm = channel_6_fs;
    channel_7_pwm = channel_7_fs;
    channel_8_pwm = channel_8_fs;
  }
}

void commandMotors() {}

void armMotors() {}

void calibrateESCs() {}

void switchRollYaw(int reverseRoll, int reverseYaw) {
  // DESCRIPTION: Switches roll_des and yaw_des variables for tailsitter-type
  // configurations
  /*
   * Takes in two integers (either 1 or -1) corresponding to the desired
   * reversing of the roll axis and yaw axis, respectively. Reversing of the
   * roll or yaw axis may be needed when switching between the two for some
   * dynamic configurations. Inputs of 1, 1 does not reverse either of them,
   * while -1, 1 will reverse the output corresponding to the new roll axis.
   * This function may be replaced in the future by a function that switches the
   * IMU data instead (so that angle can also be estimated with the IMU tilted
   * 90 degrees from default level).
   */
  float switch_holder;

  switch_holder = yaw_des;
  yaw_des = reverseYaw * roll_des;
  roll_des = reverseRoll * switch_holder;
}

void throttleCut() {}


void controlFlapMotion() {
  // DESCRIPTION: Computes control commands based on state error (angle) in
  // cascaded scheme FlapMotion!
  float GyroX;
#if defined USE_MPU6050_I2C
  GyroX = -GyroX_6050; // 安装位置
#endif
#if defined USE_MPU9250_SPI
  GyroX = -GyroX_9250;
#endif

#if defined EXTIMU
  GyroX = Gyro_X_EXT;
#endif

#if defined userotatesensor
  Phiab_Mea = relativeAngle_ready; // phiab是A机自己测的。
  // Phiab_Mea=rollAB_rad_Qua;
  Phiac_Mea = phiac;
  Phibd_Mea = phibd;
  Phice_Mea = phice;
#else
  Phiab_Mea = phiB_raw - roll_IMU; //
  // Phiab_Mea=rollAB_rad_Qua;
  Phiac_Mea = phiC_raw - roll_IMU; //
  Phibd_Mea = phiD_raw - phiB_raw;
  Phice_Mea = phiE_raw - phiC_raw;
  Phidf_Mea = phiF_raw - phiD_raw;
  Phieg_Mea = phiG_raw - phiE_raw;
#endif

  Pab = GYRO_X_B - (GyroX);
  Pac = GYRO_X_C - (GyroX);
  Pbd = GYRO_X_D - GYRO_X_B;
  Pce = GYRO_X_E - GYRO_X_C;
  Pdf = GYRO_X_F - GYRO_X_D;
  Peg = GYRO_X_G - GYRO_X_E;

  // Outer loop - P
  // float Phiab_des_ol, Phiac_des_ol;  //outerloop
  // Phiab
  error_Phiab = Phiab_des - Phiab_Mea;
  Phiab_des_ol =
      Kp_Flap * error_Phiab; //+ Ki_Phiab_angle*integral_FlapMotion_ol;// -
                             // Kd_roll_angle*derivative_roll;

  // Phiac
  error_Phiac = Phiac_des - Phiac_Mea;
  Phiac_des_ol = Kp_Flap * error_Phiac; // + Ki_pitch_angle*integral_pitch_ol;//
                                        // - Kd_pitch_angle*derivative_pitch;
  // Phibd
  error_Phibd = Phibd_des - Phibd_Mea;
  Phibd_des_ol = Kp_Flap * error_Phibd;
  // Phice
  error_Phice = Phice_des - Phice_Mea;
  Phice_des_ol = Kp_Flap * error_Phice;
  // Phidf
  error_Phidf = Phidf_des - Phidf_Mea;
  Phidf_des_ol = Kp_Flap * error_Phidf;
  // Phieg
  error_Phieg = Phieg_des - Phieg_Mea;
  Phieg_des_ol = Kp_Flap * error_Phieg;

  // Serial.println(Phibd_Mea);

  // Apply loop gain, constrain, and LP filter for artificial damping
  float Kl = 30.0;
  Phiab_des_ol = Kl * Phiab_des_ol;
  Phiac_des_ol = Kl * Phiac_des_ol;
  Phibd_des_ol = Kl * Phibd_des_ol;
  Phice_des_ol = Kl * Phice_des_ol;
  Phidf_des_ol = Kl * Phidf_des_ol;
  Phieg_des_ol = Kl * Phieg_des_ol;
  Phiab_des_ol = constrain(Phiab_des_ol, -240.0, 240.0);
  Phiac_des_ol = constrain(Phiac_des_ol, -240.0, 240.0);
  Phibd_des_ol = constrain(Phibd_des_ol, -240.0, 240.0);
  Phice_des_ol = constrain(Phice_des_ol, -240.0, 240.0);
  Phidf_des_ol = constrain(Phidf_des_ol, -240.0, 240.0);
  Phieg_des_ol = constrain(Phieg_des_ol, -240.0, 240.0);

  // 内环 PI
  // Phiab
  float coeffconfiguration =
      1.0 + constrain((channel_7_pwm - 1520) / 500.0, -1, 1);
  error_Phiab_RATE = Phiab_des_ol - Pab;
  integral_Phiab_RATE_il = integral_Phiab_RATE_prev_il + error_Phiab_RATE * dt;
  if (channel_1_pwm <
      1060) { // Don't let integrator build if throttle is too low
    integral_Phiab_RATE_il = 0;
  }
  integral_Phiab_RATE_il =
      constrain(integral_Phiab_RATE_il, -i_limit,
                i_limit); // Saturate integrator to prevent unsafe buildup
  // derivative_roll = (error_roll - error_roll_prev)/dt;
  Phiab_PID = coeffconfiguration * .01 *
              (Kff_FLAP_RATE * Phiab_des_ol + Kp_FLAP_RATE * error_Phiab_RATE +
               i_valid * Ki_FLAP_RATE *
                   integral_Phiab_RATE_il); // Scaled by .01 to bring within -1
                                            // to 1 range

  // Phiac
  error_Phiac_RATE = Phiac_des_ol - Pac;
  integral_Phiac_RATE_il = integral_Phiac_RATE_prev_il + error_Phiac_RATE * dt;
  if (channel_1_pwm <
      1060) { // Don't let integrator build if throttle is too low
    integral_Phiac_RATE_il = 0;
  }
  integral_Phiac_RATE_il =
      constrain(integral_Phiac_RATE_il, -i_limit,
                i_limit); // Saturate integrator to prevent unsafe buildup
  // derivative_pitch = (error_pitch - error_pitch_prev)/dt;
  Phiac_PID = coeffconfiguration * .01 *
              (Kff_FLAP_RATE * Phiac_des_ol + Kp_FLAP_RATE * error_Phiac_RATE +
               i_valid * Ki_FLAP_RATE *
                   integral_Phiac_RATE_il); // Scaled by .01 to bring within -1
                                            // to 1 range

  // Phibd
  error_Phibd_RATE = Phibd_des_ol - Pbd;
  integral_Phibd_RATE_il = integral_Phibd_RATE_prev_il + error_Phibd_RATE * dt;
  if (channel_1_pwm <
      1060) { // Don't let integrator build if throttle is too low
    integral_Phibd_RATE_il = 0;
  }
  integral_Phibd_RATE_il =
      constrain(integral_Phibd_RATE_il, -i_limit,
                i_limit); // Saturate integrator to prevent unsafe buildup
  // derivative_pitch = (error_pitch - error_pitch_prev)/dt;
  Phibd_PID = coeffconfiguration * .01 *
              (Kff_FLAP_RATE * Phibd_des_ol + Kp_FLAP_RATE * error_Phibd_RATE +
               i_valid * Ki_FLAP_RATE *
                   integral_Phibd_RATE_il); // Scaled by .01 to bring within -1
                                            // to 1 range

  // Phice
  error_Phice_RATE = Phice_des_ol - Pce;
  integral_Phice_RATE_il = integral_Phice_RATE_prev_il + error_Phice_RATE * dt;
  if (channel_1_pwm <
      1060) { // Don't let integrator build if throttle is too low
    integral_Phice_RATE_il = 0;
  }
  integral_Phice_RATE_il =
      constrain(integral_Phice_RATE_il, -i_limit,
                i_limit); // Saturate integrator to prevent unsafe buildup
  // derivative_pitch = (error_pitch - error_pitch_prev)/dt;
  Phice_PID = coeffconfiguration * .01 *
              (Kff_FLAP_RATE * Phice_des_ol + Kp_FLAP_RATE * error_Phice_RATE +
               i_valid * Ki_FLAP_RATE *
                   integral_Phice_RATE_il); // Scaled by .01 to bring within -1
                                            // to 1 range

  // Phidf
  error_Phidf_RATE = Phidf_des_ol - Pdf;
  integral_Phidf_RATE_il = integral_Phidf_RATE_prev_il + error_Phidf_RATE * dt;
  if (channel_1_pwm <
      1060) { // Don't let integrator build if throttle is too low
    integral_Phice_RATE_il = 0;
  }
  integral_Phidf_RATE_il =
      constrain(integral_Phidf_RATE_il, -i_limit,
                i_limit); // Saturate integrator to prevent unsafe buildup
  // derivative_pitch = (error_pitch - error_pitch_prev)/dt;
  Phidf_PID = coeffconfiguration * .01 *
              (Kff_FLAP_RATE * Phidf_des_ol + Kp_FLAP_RATE * error_Phidf_RATE +
               i_valid * Ki_FLAP_RATE *
                   integral_Phidf_RATE_il); // Scaled by .01 to bring within -1
                                            // to 1 range

  // Phieg
  error_Phieg_RATE = Phieg_des_ol - Peg;
  integral_Phieg_RATE_il = integral_Phieg_RATE_prev_il + error_Phieg_RATE * dt;
  if (channel_1_pwm <
      1060) { // Don't let integrator build if throttle is too low
    integral_Phieg_RATE_il = 0;
  }
  integral_Phieg_RATE_il =
      constrain(integral_Phieg_RATE_il, -i_limit,
                i_limit); // Saturate integrator to prevent unsafe buildup
  // derivative_pitch = (error_pitch - error_pitch_prev)/dt;
  Phieg_PID = coeffconfiguration * .01 *
              (Kff_FLAP_RATE * Phieg_des_ol + Kp_FLAP_RATE * error_Phieg_RATE +
               i_valid * Ki_FLAP_RATE *
                   integral_Phieg_RATE_il); // Scaled by .01 to bring within -1
                                            // to 1 range

  // Update Flap variables
  Phiab_des_prev = Phiab_des_ol;
  Phiac_des_prev = Phiac_des_ol;
  Phibd_des_prev = Phibd_des_ol;
  Phice_des_prev = Phice_des_ol;
  Phidf_des_prev = Phidf_des_ol;
  Phieg_des_prev = Phieg_des_ol;
  integral_Phiab_RATE_prev_il = integral_Phiab_RATE_il;
  integral_Phiac_RATE_prev_il = integral_Phiac_RATE_il;
  integral_Phibd_RATE_prev_il = integral_Phibd_RATE_il;
  integral_Phice_RATE_prev_il = integral_Phice_RATE_il;
  integral_Phidf_RATE_prev_il = integral_Phidf_RATE_il;
  integral_Phieg_RATE_prev_il = integral_Phieg_RATE_il;
  relativeAngle_ready_prev = relativeAngle_ready;
}

void initializeRadioFailsafeChannels() {
  channel_1_pwm = channel_1_fs;
  channel_2_pwm = channel_2_fs;
  channel_3_pwm = channel_3_fs;
  channel_4_pwm = channel_4_fs;
  channel_5_pwm = channel_5_fs;
  channel_6_pwm = channel_6_fs;
}

void runSelectedControlMode() {
  if (currentMode == STABILIZE_MODE) // 增稳
  {
    i_valid = 1.0;
    int_is_valid = true;
    force_manual = false;
    controlANGLE2(); // Stabilize on angle setpoint using cascaded method. Rate
                     // controller must be tuned well first!
    displayFlightModeIndicators(currentMode);
    controlFlapMotion();
  } else if (currentMode == STABLIZE_MODE_NO_I) // 增稳
  {
    int_is_valid = false;
    force_manual = false;
    i_valid = 0.0;
    controlANGLE2();        // Stabilize on angle no I
    displayFlightModeIndicators(currentMode);
    controlFlapMotion();
  } else if (currentMode == MANUAL_MODE) {
    force_manual = true;
    displayFlightModeIndicators(currentMode);
    Phiab_PID = 0.0;
    Phiac_PID = 0.0;
    Phibd_PID = 0.0;
    Phice_PID = 0.0;
    Phidf_PID = 0.0;
    Phieg_PID = 0.0;
  }

}
