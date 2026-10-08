#include "serial_ports.h"
#include "control_modes.h"
#include "parameter_registry.h"
#include "debug_print.h"
#include "control_state.h"
#include "control_allocation.h"
#include "flight_clock.h"
#include "human_interface.h"
#include "interaircraft_comm.h"
#include "math_utils.h"
#include "sensor_processing.h"
#include "flight_config.h"

int channel_1_pwm, channel_2_pwm, channel_3_pwm, channel_4_pwm, channel_5_pwm,
    channel_6_pwm, channel_7_pwm, channel_8_pwm;

FlightMode currentMode;
FlightMode lastMode;

// 上电时设置初始控制模式。
void initializeInitialControlMode() {
  currentMode = MANUAL_MODE;
}
bool int_is_valid = false;
bool force_manual = false;

static bool ModeChange = 0;


// 巡航速度
static float V_cruise = 13;


// PWM——OUTPUT

int32_t pwm_channel1_rev = -1;
int32_t pwm_channel2_rev = 1;
int32_t pwm_channel3_rev = -1;
int32_t pwm_channel4_rev = 1;
int32_t pwm_channel5_rev = -1;

// 相比旧基准，舵面安装 trim 增加 20 μs，以保持机械中位。
// A机
#if defined TESTBED
float pwm_channel1_trim = 160;
float pwm_channel2_trim = -190;
#else
float pwm_channel1_trim = 180;  // 减少是向上 安装偏置
float pwm_channel2_trim = -155; // 减少是向上
#endif

// 襟副翼微调
float pwm_channel1B_trim = 205;  // 减少是向上
float pwm_channel2B_trim = -20;  // 减少是向上
float pwm_channel1C_trim = 150;  // 减少是向上
float pwm_channel2C_trim = -152; // 减少是向上
float pwm_channel1D_trim = 122;  // 减少是向上
float pwm_channel2D_trim = -163; // 减少是向上
float pwm_channel1E_trim = 140;  // 减少是向上
float pwm_channel2E_trim = -170; // 减少是向上
float pwm_channel1F_trim = 154;  // 减少是向上
float pwm_channel2F_trim = -187; // 减少是向上
float pwm_channel1G_trim = 190;  // 减少是向上
float pwm_channel2G_trim = -170; // 减少是向上

// 升降舵微调，手动模式用
float pwm_channel3B_trim = 62;
float pwm_channel3C_trim = -30; // 对于子机也要修正
float pwm_channel3D_trim = 170;
float pwm_channel3E_trim = -30;
float pwm_channel3F_trim = 29;
float pwm_channel3G_trim = 200;

#if defined TESTBED
float pwm_channel3A_trim = 30;
#else
float pwm_channel3A_trim = 40;
#endif

// 当前飞机

#if defined TESTBED || defined APLANE
float &pwm_channel3_trim =
    pwm_channel3A_trim; // A 机升降舵安装微调，增稳模式使用。
#elif defined BPLANE
float &pwm_channel3_trim = pwm_channel3B_trim;
#elif defined CPLANE
float &pwm_channel3_trim = pwm_channel3C_trim;
#elif defined DPLANE
float &pwm_channel3_trim = pwm_channel3D_trim;
#elif defined EPLANE
float &pwm_channel3_trim = pwm_channel3E_trim;
#elif defined FPLANE
float &pwm_channel3_trim = pwm_channel3F_trim;
#elif defined GPLANE
float &pwm_channel3_trim = pwm_channel3G_trim;
#endif

// 油门以 1100 μs 为起点，输出中位相消，trim 保持原值。
float pwm_channel4_trim = 0;
float pwm_channel5_trim = 20;

static int outputpwm1, outputpwm2, outputpwm3, outputpwm4, outputpwm5;


// 遥控信号异常时写入各通道的安全 PWM 值（单位：μs）。
static unsigned long channel_1_fs = 1500; // 滚转
static unsigned long channel_2_fs = 1500; // 俯仰
static unsigned long channel_3_fs = 1000; // 油门
static unsigned long channel_4_fs = 1500; // 方向舵
static unsigned long channel_5_fs = 1500; // 油门切断开关
static unsigned long channel_6_fs = 2000; // 辅助通道 1
static unsigned long channel_7_fs = 2000; // 辅助通道 2
static unsigned long channel_8_fs = 2000; // 辅助通道 3

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


// 控制输出与各机指令状态。
//  刹车
float ailBrake_PWM = 0.0, flap2eleratio = 0.0;
// 主机分发的俯仰目标（度）及升降舵逻辑前馈偏移（μs）。
float A_pitch_sp, B_pitch_sp, C_pitch_sp, D_pitch_sp, E_pitch_sp, F_pitch_sp, G_pitch_sp;
float B_ele_control_ff_us, C_ele_control_ff_us, D_ele_control_ff_us,
    E_ele_control_ff_us, F_ele_control_ff_us, G_ele_control_ff_us;

static float Local_pitch_des_last = 8; // 期望当地俯仰角
static int Local_rudd_PWM_last = PWM_CENTER_US;
static int Local_thro_PWM_last = 1111;
static int Local_ail2_PWM_last = PWM_CENTER_US;
static int Local_ail1_PWM_last = PWM_CENTER_US;
static float relativeAngle_ready_prev;


// 控制器增益与限幅参数；修改后需重新验证闭环响应。
// 注意：积分清零条件检查 channel_1_pwm，而 getDesState() 将通道 1 映射为滚转；
// 若原意是低油门清零，需核对接收机通道映射后再修改判断条件。
// 等效滚转阻尼补偿。
int Ail_Clp;
static float Clp_PID = 0.0;
static float k_Clp = 0.05;

static float i_limit =
    45.0; // 积分项限幅，防止长时间误差导致指令过大。
static float i_valid = 0.0;   // 积分是否起作用
static float maxRoll = 80.0;  // 最大滚转期望；角度模式为 °，角速度模式为 °/s。
static float maxPitch = 60.0; // 最大俯仰期望；单位随控制模式变化。
static float maxYaw = 160.0;  // 最大偏航角速度期望，单位 °/s。

static float kp_rotate = 0.4;
static float Kp_roll_angle = 0.25; // 滚转角比例增益。
static float Ki_roll_angle = 0.0;  // 滚转角积分增益。
static float Kd_roll_angle =
    0.0; // 滚转角微分增益；controlANGLE2() 不使用该参数。
static float B_loop_roll = 1.0; // 滚转外环阻尼系数，范围 0～1。
static float Kp_pitch_angle = 0.12; // 俯仰角比例增益。
static float Ki_pitch_angle = 0.0;  // 俯仰角积分增益。
static float Kd_pitch_angle =
    0.00; // 俯仰角微分增益；controlANGLE2() 不使用该参数。
static float B_loop_pitch = 1.0; // 俯仰外环阻尼系数，范围 0～1。
static float Kp_Flap = 0.2;      // 襟翼角度比例增益。
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
static float Kp_roll_rate = 0.045; // 滚转角速度比例增益。
static float Ki_roll_rate = 0.010; // 滚转角速度积分增益。
static float Kd_roll_rate =
    0.0000; // 滚转角速度微分增益；增大前应检查输出噪声。
static float Kp_pitch_rate = 0.09; // 俯仰角速度比例增益。
static float Ki_pitch_rate = 0.11; // 俯仰角速度积分增益。
static float Kd_pitch_rate =
    0.0000; // 俯仰角速度微分增益；增大前应检查输出噪声。
static float Kff_roll_rate = 0.15;  // 滚转角速度前馈增益。
static float Kff_pitch_rate = 0.20; // 0.2单机
static float Kff_yaw_rate = 0.03;
static float Kff_FLAP_RATE = 0.09;
static float Kp_FLAP_RATE = 0.07;
static float Ki_FLAP_RATE = 0.12;

#elif defined TEAM

static float Kp_roll_rate = 0.15; // 滚转角速度比例增益。
static float Ki_roll_rate = 0.1;  // 滚转角速度积分增益。
static float Kd_roll_rate =
    0.0002; // 滚转角速度微分增益；增大前应检查输出噪声。
static float Kp_pitch_rate = 0.11; // 俯仰角速度比例增益。
static float Ki_pitch_rate = 0.10; // 俯仰角速度积分增益。
static float Kd_pitch_rate =
    0.000; // 俯仰角速度微分增益；当前设为零。
static float Kff_roll_rate = 0.12; // 滚转角速度前馈增益。
static float Kff_pitch_rate = 0.20;
static float Kff_yaw_rate = 0.03;
static float Kff_FLAP_RATE = 0.1;
static float Kp_FLAP_RATE = 0.2;  // 0.09
static float Ki_FLAP_RATE = 0.20; // 0.15

#endif

float roll_eq;

static float Kp_yaw = 0.2;     // 偏航角速度比例增益。
static float Ki_yaw = 0.05;    // 偏航角速度积分增益。
static float Kd_yaw = 0.00000; // 偏航角速度微分增益；当前设为零。



static float roll_IMU_prev, pitch_IMU_prev;

// 滚转角控制指令滤波
static float roll_PID_lpf = 0.0f;
static float roll_pid_lpf_fc = 7.0f; // 7hz指令滤波
static const float TWO_PI_F = 6.28318530718f;


// 遥控输入映射后的归一化期望状态。
float thro_des, roll_des, pitch_des, yaw_des, rotate_speed_des, pitch_des_local;
static float thro_des_RAW, roll_des_RAW, pitch_des_RAW, yaw_des_RAW;
static float roll_passthru, pitch_passthru, yaw_passthru;
static float pitch_des_local_last;
static float pitch_des_local_rate;
static float pitch_des_local_rate_lpf_fc = 5.0f;

// 控制器误差、积分及输出状态。
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

// 混控后的归一化执行器指令。
static float m1_command_scaled, m2_command_scaled, m3_command_scaled,
    m4_command_scaled, m5_command_scaled, m6_command_scaled;
int m1_command_PWM, m2_command_PWM, m3_command_PWM, m4_command_PWM,
    m5_command_PWM, m6_command_PWM;
float Aail1_scaled, Aail2_scaled, Aele_scaled, Athro_scaled, Arudd_scaled,
    s6_command_scaled, s7_command_scaled;
float Bail1_scaled, Bail2_scaled, Bele_scaled, Bthro_scaled, Brudd_scaled;
float Cail1_scaled, Cail2_scaled, Cele_scaled, Cthro_scaled, Crudd_scaled;
float Dail1_scaled, Dail2_scaled, Dele_scaled, Dthro_scaled, Drudd_scaled;
float Eail1_scaled, Eail2_scaled, Eele_scaled, Ethro_scaled, Erudd_scaled;
float Fail1_scaled, Fail2_scaled, Fele_scaled, Fthro_scaled, Frudd_scaled;
float Gail1_scaled, Gail2_scaled, Gele_scaled, Gthro_scaled, Grudd_scaled;

int s6_command_PWM, s7_command_PWM;

// 飞行解锁状态。
static bool armedFly = false;

float central_pitch = 0.0f;

// 按单机或编队构型，把手动指令或姿态控制量映射到各机归一化舵量。
const FlightParameter *controlParameterTable(size_t &count) {
  // Keep registration beside the private variables, without exporting them.
  static const FlightParameter parameters[] = {
      {"Kp_roll_angle", &Kp_roll_angle, 0, 10, "Attitude", "Roll angle P"},
      {"Ki_roll_angle", &Ki_roll_angle, 0, 10, "Attitude", "Roll angle I"},
      {"Kd_roll_angle", &Kd_roll_angle, 0, 1, "Attitude", "Roll angle D (controlANGLE only)"},
      {"Kp_pitch_angle", &Kp_pitch_angle, 0, 10, "Attitude", "Pitch angle P"},
      {"Ki_pitch_angle", &Ki_pitch_angle, 0, 10, "Attitude", "Pitch angle I"},
      {"Kd_pitch_angle", &Kd_pitch_angle, 0, 1, "Attitude", "Pitch angle D (controlANGLE only)"},
      {"B_loop_roll", &B_loop_roll, 0, 1, "Attitude", "Roll outer-loop filter coefficient"},
      {"B_loop_pitch", &B_loop_pitch, 0, 1, "Attitude", "Pitch outer-loop coefficient"},
      {"Kp_roll_rate", &Kp_roll_rate, 0, 10, "Attitude", "Roll rate P"},
      {"Ki_roll_rate", &Ki_roll_rate, 0, 10, "Attitude", "Roll rate I"},
      {"Kd_roll_rate", &Kd_roll_rate, 0, 1, "Attitude", "Roll rate D"},
      {"Kff_roll_rate", &Kff_roll_rate, 0, 10, "Attitude", "Roll rate feedforward"},
      {"Kp_pitch_rate", &Kp_pitch_rate, 0, 10, "Attitude", "Pitch rate P"},
      {"Ki_pitch_rate", &Ki_pitch_rate, 0, 10, "Attitude", "Pitch rate I"},
      {"Kd_pitch_rate", &Kd_pitch_rate, 0, 1, "Attitude", "Pitch rate D"},
      {"Kff_pitch_rate", &Kff_pitch_rate, 0, 10, "Attitude", "Pitch rate feedforward"},
      {"Kp_yaw", &Kp_yaw, 0, 10, "Attitude", "Yaw rate P"},
      {"Ki_yaw", &Ki_yaw, 0, 10, "Attitude", "Yaw rate I"},
      {"Kd_yaw", &Kd_yaw, 0, 1, "Attitude", "Yaw rate D"},
      {"Kff_yaw_rate", &Kff_yaw_rate, 0, 10, "Attitude", "Yaw rate feedforward"},
      {"Kp_Flap", &Kp_Flap, 0, 10, "Configuration", "Relative angle P"},
      {"Kp_FLAP_RATE", &Kp_FLAP_RATE, 0, 10, "Configuration", "Relative rate P"},
      {"Ki_FLAP_RATE", &Ki_FLAP_RATE, 0, 10, "Configuration", "Relative rate I"},
      {"Kff_FLAP_RATE", &Kff_FLAP_RATE, 0, 10, "Configuration", "Relative rate feedforward"},
      {"B_loop_FLAP", &B_loop_FLAP, 0, 1, "Configuration", "Relative outer-loop filter coefficient"},
      {"indi_pitch_q_gain", &indi_pitch_q_gain, 0.01f, 1000, "INDI", "Pitch rate error gain (1/s)"},
      {"indi_pitch_effectiveness", &indi_pitch_effectiveness, -10000, -0.001f, "INDI", "Elevator effectiveness (deg/s^2 per deg)"},
      {"usb_throttle_debug", &localThrottleDebugSetting(), 0, 1, "Debug", "Throttle USB log: 0 off, 1 on (10 Hz)"},
      {"k_Clp", &k_Clp, 0, 10, "Control", "Roll damping gain"},
      {"i_limit", &i_limit, 0, 1000, "Control", "Controller integral limit"},
      {"maxRoll", &maxRoll, 0, 180, "Control", "Maximum roll command (deg or deg/s)"},
      {"maxPitch", &maxPitch, 0, 180, "Control", "Maximum pitch command (deg or deg/s)"},
      {"maxYaw", &maxYaw, 0, 720, "Control", "Maximum yaw rate command (deg/s)"},
      {"Trim_pitch_angle", &Trim_pitch_angle, -30, 30, "Control", "Local pitch command trim (deg)"},
      {"roll_pid_lpf_fc", &roll_pid_lpf_fc, 0.1, 200, "Control", "Roll command filter cutoff (Hz)"},
      {"roll_pid_dot_lpf_fc", &roll_pid_dot_lpf_fc, 0.1, 200, "Control", "Roll command derivative filter cutoff (Hz)"},
      {"pitch_des_local_rate_lpf_fc", &pitch_des_local_rate_lpf_fc, 0.1, 200, "Control", "Local pitch command rate filter cutoff (Hz)"},
      {"indi_pitch_pwm_to_deg_k", &indi_pitch_pwm_to_deg_k, 1e-05, 1, "INDI", "Elevator PWM-to-angle slope (deg/us)"},
      {"indi_pitch_pwm_to_deg_b", &indi_pitch_pwm_to_deg_b, -1000, 1000, "INDI", "Elevator PWM-to-angle intercept (deg)"},
      {"indi_pitch_servo_delay_s", &indi_pitch_servo_delay_s, 0, 0.2, "INDI", "Servo pure delay (s; delay line clamps at 63 samples)"},
      {"indi_pitch_servo_tau_s", &indi_pitch_servo_tau_s, 0, 1, "INDI", "Servo time constant (s)"},
      {"indi_pitch_deflection_min_deg", &indi_pitch_deflection_min_deg, -90, 0, "INDI", "Minimum elevator deflection (deg)"},
      {"indi_pitch_deflection_max_deg", &indi_pitch_deflection_max_deg, 0, 90, "INDI", "Maximum elevator deflection (deg)"},
      {"indi_pitch_rate_limit_deg_s", &indi_pitch_rate_limit_deg_s, 1, 5000, "INDI", "Elevator deflection rate limit (deg/s)"},
      {"indi_pitch_cmd_lpf_fc_hz", &indi_pitch_cmd_lpf_fc_hz, 0, 200, "INDI", "Elevator command cutoff (Hz; 0 disables)"},
      {"indi_pitch_pwm_min", &indi_pitch_pwm_min, 900, 1500, "INDI", "Minimum elevator PWM (us)"},
      {"indi_pitch_pwm_max", &indi_pitch_pwm_max, 1500, 2100, "INDI", "Maximum elevator PWM (us)"},
      {"pwm_channel1_rev", &pwm_channel1_rev, -1, 1, "Actuator", "Output direction: -1 reverse, +1 normal; zero rejected"},
      {"pwm_channel2_rev", &pwm_channel2_rev, -1, 1, "Actuator", "Output direction: -1 reverse, +1 normal; zero rejected"},
      {"pwm_channel3_rev", &pwm_channel3_rev, -1, 1, "Actuator", "Output direction: -1 reverse, +1 normal; zero rejected"},
      {"pwm_channel4_rev", &pwm_channel4_rev, -1, 1, "Actuator", "Output direction: -1 reverse, +1 normal; zero rejected"},
      {"pwm_channel5_rev", &pwm_channel5_rev, -1, 1, "Actuator", "Output direction: -1 reverse, +1 normal; zero rejected"},
      {"pwm_channel1_trim", &pwm_channel1_trim, -500, 500, "Actuator", "Local/A channel mechanical trim (us)"},
      {"pwm_channel2_trim", &pwm_channel2_trim, -500, 500, "Actuator", "Local/A channel mechanical trim (us)"},
      {"pwm_channel4_trim", &pwm_channel4_trim, -500, 500, "Actuator", "Local/A channel mechanical trim (us)"},
      {"pwm_channel5_trim", &pwm_channel5_trim, -500, 500, "Actuator", "Local/A channel mechanical trim (us)"},
      {"pwm_channel3A_trim", &pwm_channel3A_trim, -500, 500, "Actuator", "A elevator mechanical trim (us)"},
      {"pwm_channel1B_trim", &pwm_channel1B_trim, -500, 500, "Actuator", "B channel 1 mechanical trim (us)"},
      {"pwm_channel2B_trim", &pwm_channel2B_trim, -500, 500, "Actuator", "B channel 2 mechanical trim (us)"},
      {"pwm_channel3B_trim", &pwm_channel3B_trim, -500, 500, "Actuator", "B channel 3 mechanical trim (us)"},
      {"pwm_channel1C_trim", &pwm_channel1C_trim, -500, 500, "Actuator", "C channel 1 mechanical trim (us)"},
      {"pwm_channel2C_trim", &pwm_channel2C_trim, -500, 500, "Actuator", "C channel 2 mechanical trim (us)"},
      {"pwm_channel3C_trim", &pwm_channel3C_trim, -500, 500, "Actuator", "C channel 3 mechanical trim (us)"},
      {"pwm_channel1D_trim", &pwm_channel1D_trim, -500, 500, "Actuator", "D channel 1 mechanical trim (us)"},
      {"pwm_channel2D_trim", &pwm_channel2D_trim, -500, 500, "Actuator", "D channel 2 mechanical trim (us)"},
      {"pwm_channel3D_trim", &pwm_channel3D_trim, -500, 500, "Actuator", "D channel 3 mechanical trim (us)"},
      {"pwm_channel1E_trim", &pwm_channel1E_trim, -500, 500, "Actuator", "E channel 1 mechanical trim (us)"},
      {"pwm_channel2E_trim", &pwm_channel2E_trim, -500, 500, "Actuator", "E channel 2 mechanical trim (us)"},
      {"pwm_channel3E_trim", &pwm_channel3E_trim, -500, 500, "Actuator", "E channel 3 mechanical trim (us)"},
      {"pwm_channel1F_trim", &pwm_channel1F_trim, -500, 500, "Actuator", "F channel 1 mechanical trim (us)"},
      {"pwm_channel2F_trim", &pwm_channel2F_trim, -500, 500, "Actuator", "F channel 2 mechanical trim (us)"},
      {"pwm_channel3F_trim", &pwm_channel3F_trim, -500, 500, "Actuator", "F channel 3 mechanical trim (us)"},
      {"pwm_channel1G_trim", &pwm_channel1G_trim, -500, 500, "Actuator", "G channel 1 mechanical trim (us)"},
      {"pwm_channel2G_trim", &pwm_channel2G_trim, -500, 500, "Actuator", "G channel 2 mechanical trim (us)"},
      {"pwm_channel3G_trim", &pwm_channel3G_trim, -500, 500, "Actuator", "G channel 3 mechanical trim (us)"},
  };
  count = sizeof(parameters) / sizeof(parameters[0]);
  return parameters;
}


void controlMixer() {
  // 按单机或编队构型，将姿态控制量分配到各机舵面和油门。
  // 手动模式直接使用归一化遥控指令；舵机归一化中位为 0.5，油门范围为 0～1。

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
// USBSerial.println(Aail1_scaled);

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
    B_ele_control_ff_us = 0.0;

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
    C_ele_control_ff_us = 0.0;

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
    B_ele_control_ff_us =
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
    C_ele_control_ff_us = 10.0 * 20.0 * roll_PID_lpf + 10.0 * roll_PID_dot_lpf;
  }

#elif defined FOURPLANE

  // 4机一起飞 DBAC
  float coeab, coeac, coebd,
      coeroll; // bc的系数和de的系数不同 de的系数是ab的0.66倍

  coeroll = 1.5;

  // USBSerial.println(coeroll);
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
     //USBSerial.println("ss");
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

// 根据遥控通道 5 与通道 1 的阈值设置解锁标志；当前通道映射需与实际接线核对。
void armedStatus() {
  // 通道 5 低于 1500 μs 且通道 1 低于 1050 μs 时标记为已解锁。
  // 当前映射下通道 1 是滚转；若本意是低油门解锁，需核对该判断条件。
  if ((channel_5_pwm < 1500) && (channel_1_pwm < 1050)) {
    armedFly = true;
  }
}

// 将遥控 PWM 归一化并限幅，生成油门、姿态角和角速度期望及手动直通量。
void getDesState() { // 调整了通道顺序
  // 将接收机 PWM 映射为油门、姿态角和角速度期望值，并限制在配置范围内。
  // RAW 与 passthru 变量保留未增稳的归一化输入，供手动混控使用。
  float GyroZ;
#if defined USE_MPU6050_I2C
  GyroZ = GyroZ_6050;
#endif
#if defined USE_MPU9250_SPI
  GyroZ = GyroZ_9250;
#endif

#if defined APLANE // 是主机
  {
    thro_des_RAW = (channel_3_pwm - 1100.0) / 1000.0; // 范围 0～1。
    roll_des_RAW = (channel_1_pwm - PWM_CENTER_US) / 500.0;  // 范围 -1～1。
    pitch_des_RAW = (channel_2_pwm - PWM_CENTER_US) / 500.0; // 范围 -1～1。
    yaw_des_RAW = (channel_4_pwm - PWM_CENTER_US) / 500.0;   // 范围 -1～1。

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

    roll_passthru = roll_des_RAW / 2.0;   // 范围 -0.5～0.5。
    pitch_passthru = pitch_des_RAW / 2.0; // 范围 -0.5～0.5。
    yaw_passthru = yaw_des_RAW / 2.0;     // 范围 -0.5～0.5。

    // 将各轴指令限制在归一化或配置的最大范围内。
    thro_des = constrain(thro_des_RAW, 0.0, 1.0); // 范围 0～1。
    roll_des = constrain(roll_des_RAW, -1.0, 1.0) *
               maxRoll; // 范围为 ±maxRoll。
    pitch_des_local = constrain(pitch_des_RAW, -1.0, 1.0) *
                      maxPitch; // 范围为 ±maxPitch。
    yaw_des = constrain(yaw_des_RAW, -1.0, 1.0) *
              maxYaw; // 范围为 ±maxYaw。
    roll_passthru = constrain(roll_passthru, -0.5, 0.5);
    pitch_passthru = constrain(pitch_passthru, -0.5, 0.5);
    yaw_passthru = constrain(yaw_passthru, -0.5, 0.5);

    // Phiab_des=constrain(Phiab_des, -1.0, 1.0)*15.0; // 历史试验方案已停用。

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
    // USBSerial.println(pitch_des_local);
    yaw_des = 0;
  }
#endif
}

// 单环控制：滚转、俯仰使用角度误差，偏航使用陀螺仪角速度误差。
// void controlANGLE() {
//   // 单环姿态 PID：用期望角与测量角之差计算滚转、俯仰控制量。
//   // 积分项限幅；通道 1 低于阈值时清零，输出供 controlMixer() 分配。
//   float GyroZ;
//   float GyroY;
//   float GyroX;
// #if defined USE_MPU6050_I2C
//   GyroZ = GyroZ_6050;
//   GyroY = GyroY_6050;
//   GyroX = -GyroX_6050; // 安装位置
// #endif
// #if defined USE_MPU9250_SPI
//   GyroZ = GyroZ_9250;
//   GyroY = GyroY_9250;
//   GyroX = -GyroX_9250;
// #endif
//   // rotate_speed

//   rotate_error = rotate_speed_des * 1000.0 - GyroZ;
//   rotate_error = constrain(rotate_error, -100, 100); // 100度每秒
//   // thro_des=0.01*kp_rotate*rotate_error;//0-1

//   // 滚转通道。
//   error_roll = roll_des - roll_IMU;
//   integral_roll = integral_roll_prev + error_roll * dt;
//   if (channel_3_pwm <
//       1160) { // 通道 1 低于阈值时清零积分项。
//     integral_roll = 0;
//   }
//   integral_roll =
//       constrain(integral_roll, -i_limit,
//                 i_limit); // 对积分项限幅，防止持续饱和。
//   derivative_roll = GyroX;
//   roll_PID =
//       0.01 *
//       (Kp_roll_angle * error_roll + Ki_roll_angle * integral_roll -
//        Kd_roll_angle *
//            derivative_roll); // 按控制器约定缩放输出量。

//   // 俯仰通道。
//   error_pitch = pitch_des - pitch_IMU;
//   integral_pitch = integral_pitch_prev + error_pitch * dt;
//   if (channel_3_pwm <
//       1160) { // 通道 1 低于阈值时清零积分项。
//     integral_pitch = 0;
//   }
//   integral_pitch =
//       constrain(integral_pitch, -i_limit,
//                 i_limit); // 对积分项限幅，防止持续饱和。
//   derivative_pitch = GyroY;
//   pitch_PID =
//       .01 *
//       (Kp_pitch_angle * error_pitch + Ki_pitch_angle * integral_pitch -
//        Kd_pitch_angle *
//            derivative_pitch); // 按控制器约定缩放输出量。

//   // 偏航通道使用 Z 轴角速度反馈。

//   // yaw_des=yaw_des;
//   error_yaw =
//       yaw_des -
//       57.3 * (9.8 / V_cruise * tan(roll_des / 57.3) * cos(pitch_des / 57.3)) -
//       GyroZ;
//   integral_yaw = integral_yaw_prev + error_yaw * dt;
//   if (channel_3_pwm <
//       1160) { // 通道 1 低于阈值时清零积分项。
//     integral_yaw = 0;
//   }
//   integral_yaw =
//       constrain(integral_yaw, -i_limit,
//                 i_limit); // 对积分项限幅，防止持续饱和。
//   derivative_yaw = (error_yaw - error_yaw_prev) / dt;
//   yaw_PID =
//       .01 *
//       (Kp_yaw * error_yaw + Ki_yaw * integral_yaw +
//        Kd_yaw * derivative_yaw); // 按控制器约定缩放输出量。

//   // 保存滚转状态，供下一周期计算。
//   integral_roll_prev = integral_roll;
//   // 保存俯仰状态，供下一周期计算。
//   integral_pitch_prev = integral_pitch;
//   // 保存偏航状态，供下一周期计算。
//   error_yaw_prev = error_yaw;
//   integral_yaw_prev = integral_yaw;
// }

// 用当前滚转角速度生成等效滚转阻尼补偿，并换算为副翼 PWM 修正量。
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

// 串级姿态控制：外环产生角速度目标，内环 PID 生成舵面控制量。
void controlANGLE2() {
  // 串级姿态控制：外环角度误差生成角速度期望，内环跟踪角速度。
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

  // USBSerial.print(GyroY_6050);
  // USBSerial.print(" ");
  // USBSerial.println(GyroY);
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

  // 外环：角度误差生成角速度期望。
  float roll_des_ol, pitch_des_ol;
// 滚转通道。
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
      1060) { // 通道 1 低于阈值时清零积分项。
    integral_roll_ol = 0;
  }
  integral_roll_ol =
      constrain(integral_roll_ol, -i_limit,
                i_limit); // 对积分项限幅，防止持续饱和。
  if (ModeChange == 1) {
    integral_roll_ol = 0;
  }
  derivative_roll = (roll_IMU - roll_IMU_prev) / dt;
  roll_des_ol = Kp_roll_angle * error_roll +
                i_valid * Ki_roll_angle *
                    integral_roll_ol; // - Kd_roll_angle*derivative_roll;

  // 俯仰通道。
  // USBSerial.println(pitch_des_local);

#if defined APLANE

#if defined EVEN
  error_pitch = pitch_des_local + central_pitch - pitch_IMU;
#elif defined ODD
  error_pitch = pitch_des_local - pitch_IMU;
#endif

#else
  error_pitch = pitch_des_local - pitch_IMU;
#endif

  // USBSerial.println(error_pitch);
  // error_pitch = pitch_des_local - pitch_IMU;
  integral_pitch_ol = integral_pitch_prev_ol + error_pitch * dt;
  if (channel_1_pwm <
      1060) { // 通道 1 低于阈值时清零积分项。
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

  // 对外环指令限幅并低通，抑制相邻机体运动振荡。
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

  // 内环：角速度误差 PID。
  // 滚转通道。
  float Rollrate;
#if defined SINGLE
  Rollrate = GyroX;
#else
// 历史方案：可将相邻机体角速度用于等效滚转角速度估计。
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
  // USBSerial.println(Rollrate);

  error_roll = roll_des_ol - Rollrate;
  integral_roll_il = integral_roll_prev_il + error_roll * dt;
  if (channel_1_pwm <
      1060) { // 通道 1 低于阈值时清零积分项。
    integral_roll_il = 0;
  }
  integral_roll_il =
      constrain(integral_roll_il, -i_limit,
                i_limit); // 对积分项限幅，防止持续饱和。
  if (ModeChange == 1) {
    integral_roll_il = 0;
  }
  derivative_roll = (error_roll - error_roll_prev) / dt;
  roll_PID =
      .01 *
      (Kff_roll_rate * roll_des_ol + Kp_roll_rate * error_roll +
       i_valid * Ki_roll_rate * integral_roll_il +
       Kd_roll_rate *
           derivative_roll); // 按控制器约定缩放输出量。

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

  // USBSerial.println(roll_PID_dot_lpf);

  // 俯仰通道。
  error_pitch = pitch_des_ol - GyroY;
  integral_pitch_il = integral_pitch_prev_il + error_pitch * dt;
  if (channel_1_pwm <
      1060) { // 通道 1 低于阈值时清零积分项。
    integral_pitch_il = 0;
  }
  integral_pitch_il =
      constrain(integral_pitch_il, -i_limit,
                i_limit); // 对积分项限幅，防止持续饱和。
  if (ModeChange == 1) {
    integral_pitch_il = 0;
  }
  derivative_pitch = (error_pitch - error_pitch_prev) / dt;
  pitch_PID =
      .01 *
      (Kff_pitch_rate * pitch_des_ol + Kp_pitch_rate * error_pitch +
       i_valid * Ki_pitch_rate * integral_pitch_il +
       Kd_pitch_rate *
           derivative_pitch); // 按控制器约定缩放输出量。
  // USBSerial.println(integral_pitch_il);
  // 偏航通道。
  error_yaw =
      yaw_des -
      57.3 * (9.8 / V_cruise * tan(roll_des / 57.3) * cos(pitch_des / 57.3)) -
      GyroZ;
  integral_yaw = integral_yaw_prev + error_yaw * dt;
  if (channel_1_pwm <
      1060) { // 通道 1 低于阈值时清零积分项。
    integral_yaw = 0;
  }
  integral_yaw =
      constrain(integral_yaw, -i_limit,
                i_limit); // 对积分项限幅，防止持续饱和。
  if (ModeChange == 1) {
    integral_yaw = 0;
  }
  derivative_yaw = (error_yaw - error_yaw_prev) / dt;
  yaw_PID =
      .01 *
      (Kff_yaw_rate * (yaw_des - 57.3 * (9.8 / V_cruise * tan(roll_des / 57.3) *
                                         cos(pitch_des / 57.3))) +
       Kp_yaw * error_yaw + Ki_yaw * integral_yaw +
       Kd_yaw * derivative_yaw); // 按控制器约定缩放输出量。

  // 保存滚转状态，供下一周期计算。
  integral_roll_prev_ol = integral_roll_ol;
  integral_roll_prev_il = integral_roll_il;
  error_roll_prev = error_roll;
  roll_IMU_prev = roll_IMU;
  roll_des_prev = roll_des_ol;
  // 保存俯仰状态，供下一周期计算。
  integral_pitch_prev_ol = integral_pitch_ol;
  integral_pitch_prev_il = integral_pitch_il;
  error_pitch_prev = error_pitch;
  pitch_IMU_prev = pitch_IMU;
  pitch_des_prev = pitch_des_ol;
  // 保存偏航状态，供下一周期计算。
  error_yaw_prev = error_yaw;
  integral_yaw_prev = integral_yaw;
}

// 角速度模式 PID：用期望角速度和陀螺仪读数计算三轴控制量。
// void controlRATE() {
//   // 角速度模式：以期望角速度与陀螺仪读数之差计算控制量。

//   float GyroZ;
//   float GyroY;
//   float GyroX;
// #if defined USE_MPU6050_I2C
//   GyroZ = GyroZ_6050;
//   GyroY = GyroY_6050;
//   GyroX = -GyroX_6050;
// #endif
// #if defined USE_MPU9250_SPI
//   GyroZ = GyroZ_9250;
//   GyroY = GyroY_9250;
//   GyroX = -GyroX_9250;
// #endif
// #if defined EXTIMU
//   GyroX = Gyro_X_EXT;
//   GyroY = Gyro_Y_EXT;
//   GyroZ = Gyro_Z_EXT;
// #endif

//   // 滚转通道。
//   error_roll = roll_des * 3.0 - GyroX;
//   integral_roll = integral_roll_prev + error_roll * dt;
//   if (channel_1_pwm <
//       1060) { // 通道 1 低于阈值时清零积分项。
//     integral_roll = 0;
//   }
//   integral_roll =
//       constrain(integral_roll, -i_limit,
//                 i_limit); // 对积分项限幅，防止持续饱和。
//   derivative_roll = (error_roll - error_roll_prev) / dt;
//   roll_PID =
//       .01 *
//       (Kff_roll_rate * roll_des + Kp_roll_rate * error_roll +
//        Kd_roll_rate *
//            derivative_roll); // 按控制器约定缩放输出量。

//   // 俯仰通道。
//   error_pitch = pitch_des_local * 3.0 - GyroY;
//   integral_pitch = integral_pitch_prev + error_pitch * dt;
//   if (channel_1_pwm <
//       1060) { // 通道 1 低于阈值时清零积分项。
//     integral_pitch = 0;
//   }
//   integral_pitch =
//       constrain(integral_pitch, -i_limit,
//                 i_limit); // 对积分项限幅，防止持续饱和。
//   derivative_pitch = (error_pitch - error_pitch_prev) / dt;
//   pitch_PID =
//       .01 *
//       (Kff_pitch_rate * pitch_des + Kp_pitch_rate * error_pitch +
//        Kd_pitch_rate *
//            derivative_pitch); // 按控制器约定缩放输出量。

//   // 偏航通道使用 Z 轴角速度反馈。
//   error_yaw = yaw_des * 3.0 - GyroZ;
//   integral_yaw = integral_yaw_prev + error_yaw * dt;
//   if (channel_1_pwm <
//       1060) { // 通道 1 低于阈值时清零积分项。
//     integral_yaw = 0;
//   }
//   integral_yaw =
//       constrain(integral_yaw, -i_limit,
//                 i_limit); // 对积分项限幅，防止持续饱和。
//   derivative_yaw = (error_yaw - error_yaw_prev) / dt;
//   yaw_PID =
//       .01 *
//       (Kff_yaw_rate * yaw_des + Kp_yaw * error_yaw + Ki_yaw * integral_yaw +
//        Kd_yaw * derivative_yaw); // 按控制器约定缩放输出量。

//   // 保存滚转状态，供下一周期计算。
//   error_roll_prev = error_roll;
//   integral_roll_prev = integral_roll;
//   // GyroX_prev = GyroX;//这行有什么用啊 似乎没用 而且不会干扰getimudata吗
//   // 保存俯仰状态，供下一周期计算。
//   error_pitch_prev = error_pitch;
//   integral_pitch_prev = integral_pitch;
//   // GyroY_prev = GyroY;//这行有什么用啊 似乎没用 而且不会干扰getimudata吗
//   // 保存偏航状态，供下一周期计算。
//   error_yaw_prev = error_yaw;
//   integral_yaw_prev = integral_yaw;
// }

// 使用俯仰角速度、角加速度和升降舵舵效计算 INDI 升降舵指令。
// 返回值为经限幅后的 PWM 微秒数；函数维护独立滤波与舵机估计状态。
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
    indi_pitch_pwm_cmd_log = PWM_CENTER_US + pwm_channel3_trim;
    return constrain((int)(PWM_CENTER_US + pwm_channel3_trim), (int)indi_pitch_pwm_min,
                     (int)indi_pitch_pwm_max);
  }

  // 4. 计算 trim 对应的舵偏角。
  // 后面所有舵机状态初始化都以这个配平点为基准，而不是默认 0 度舵偏。
  const float pwm_trim_center = PWM_CENTER_US + pwm_channel3_trim;
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
  // USBSerial.println(dq_des_deg_s2);
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

// 检测遥控 PWM 是否越过设定范围；任一通道异常时回退到安全值。
void failSafe() {
  // 任一遥控通道超出 800～2200 μs 时，将全部通道置为预设安全值。
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

  // 检查遥控信号越界条件。
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

  // 任一通道异常时，全部通道恢复为预设安全值。
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

// 预留的电机输出接口；当前函数体为空，不会驱动电机。
void commandMotors() {}

// 预留的电机解锁接口；当前函数体为空。
void armMotors() {}

// 预留的电调标定接口；当前函数体为空。
void calibrateESCs() {}

// 交换滚转和偏航期望值，并按输入的 1 或 -1 决定各轴方向。
void switchRollYaw(int reverseRoll, int reverseYaw) {
  // 交换滚转和偏航期望值，并分别按输入符号（1 或 -1）决定方向。
  float switch_holder;

  switch_holder = yaw_des;
  yaw_des = reverseYaw * roll_des;
  roll_des = reverseRoll * switch_holder;
}

// 预留的油门切断接口；当前函数体为空。
void throttleCut() {}


// 按机间相对角与角速度误差计算六个构型通道的控制量。
void controlFlapMotion() {
  // 根据相邻机体角度误差和角速度误差计算襟翼构型控制量。
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

  // 外环：相邻机体夹角比例控制。
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

  // USBSerial.println(Phibd_Mea);

  // 对外环指令限幅并低通，抑制相邻机体运动振荡。
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
      1.0 + constrain((channel_7_pwm - PWM_CENTER_US) / 500.0, -1, 1);
  error_Phiab_RATE = Phiab_des_ol - Pab;
  integral_Phiab_RATE_il = integral_Phiab_RATE_prev_il + error_Phiab_RATE * dt;
  if (channel_1_pwm <
      1060) { // 通道 1 低于阈值时清零积分项。
    integral_Phiab_RATE_il = 0;
  }
  integral_Phiab_RATE_il =
      constrain(integral_Phiab_RATE_il, -i_limit,
                i_limit); // 对积分项限幅，防止持续饱和。
  // derivative_roll = (error_roll - error_roll_prev)/dt;
  Phiab_PID = coeffconfiguration * .01 *
              (Kff_FLAP_RATE * Phiab_des_ol + Kp_FLAP_RATE * error_Phiab_RATE +
               i_valid * Ki_FLAP_RATE *
                   integral_Phiab_RATE_il); // 按 0.01 系数缩放为混控量。

  // Phiac
  error_Phiac_RATE = Phiac_des_ol - Pac;
  integral_Phiac_RATE_il = integral_Phiac_RATE_prev_il + error_Phiac_RATE * dt;
  if (channel_1_pwm <
      1060) { // 通道 1 低于阈值时清零积分项。
    integral_Phiac_RATE_il = 0;
  }
  integral_Phiac_RATE_il =
      constrain(integral_Phiac_RATE_il, -i_limit,
                i_limit); // 对积分项限幅，防止持续饱和。
  // derivative_pitch = (error_pitch - error_pitch_prev)/dt;
  Phiac_PID = coeffconfiguration * .01 *
              (Kff_FLAP_RATE * Phiac_des_ol + Kp_FLAP_RATE * error_Phiac_RATE +
               i_valid * Ki_FLAP_RATE *
                   integral_Phiac_RATE_il); // 按 0.01 系数缩放为混控量。

  // Phibd
  error_Phibd_RATE = Phibd_des_ol - Pbd;
  integral_Phibd_RATE_il = integral_Phibd_RATE_prev_il + error_Phibd_RATE * dt;
  if (channel_1_pwm <
      1060) { // 通道 1 低于阈值时清零积分项。
    integral_Phibd_RATE_il = 0;
  }
  integral_Phibd_RATE_il =
      constrain(integral_Phibd_RATE_il, -i_limit,
                i_limit); // 对积分项限幅，防止持续饱和。
  // derivative_pitch = (error_pitch - error_pitch_prev)/dt;
  Phibd_PID = coeffconfiguration * .01 *
              (Kff_FLAP_RATE * Phibd_des_ol + Kp_FLAP_RATE * error_Phibd_RATE +
               i_valid * Ki_FLAP_RATE *
                   integral_Phibd_RATE_il); // 按 0.01 系数缩放为混控量。

  // Phice
  error_Phice_RATE = Phice_des_ol - Pce;
  integral_Phice_RATE_il = integral_Phice_RATE_prev_il + error_Phice_RATE * dt;
  if (channel_1_pwm <
      1060) { // 通道 1 低于阈值时清零积分项。
    integral_Phice_RATE_il = 0;
  }
  integral_Phice_RATE_il =
      constrain(integral_Phice_RATE_il, -i_limit,
                i_limit); // 对积分项限幅，防止持续饱和。
  // derivative_pitch = (error_pitch - error_pitch_prev)/dt;
  Phice_PID = coeffconfiguration * .01 *
              (Kff_FLAP_RATE * Phice_des_ol + Kp_FLAP_RATE * error_Phice_RATE +
               i_valid * Ki_FLAP_RATE *
                   integral_Phice_RATE_il); // 按 0.01 系数缩放为混控量。

  // Phidf
  error_Phidf_RATE = Phidf_des_ol - Pdf;
  integral_Phidf_RATE_il = integral_Phidf_RATE_prev_il + error_Phidf_RATE * dt;
  if (channel_1_pwm <
      1060) { // 通道 1 低于阈值时清零积分项。
    integral_Phice_RATE_il = 0;
  }
  integral_Phidf_RATE_il =
      constrain(integral_Phidf_RATE_il, -i_limit,
                i_limit); // 对积分项限幅，防止持续饱和。
  // derivative_pitch = (error_pitch - error_pitch_prev)/dt;
  Phidf_PID = coeffconfiguration * .01 *
              (Kff_FLAP_RATE * Phidf_des_ol + Kp_FLAP_RATE * error_Phidf_RATE +
               i_valid * Ki_FLAP_RATE *
                   integral_Phidf_RATE_il); // 按 0.01 系数缩放为混控量。

  // Phieg
  error_Phieg_RATE = Phieg_des_ol - Peg;
  integral_Phieg_RATE_il = integral_Phieg_RATE_prev_il + error_Phieg_RATE * dt;
  if (channel_1_pwm <
      1060) { // 通道 1 低于阈值时清零积分项。
    integral_Phieg_RATE_il = 0;
  }
  integral_Phieg_RATE_il =
      constrain(integral_Phieg_RATE_il, -i_limit,
                i_limit); // 对积分项限幅，防止持续饱和。
  // derivative_pitch = (error_pitch - error_pitch_prev)/dt;
  Phieg_PID = coeffconfiguration * .01 *
              (Kff_FLAP_RATE * Phieg_des_ol + Kp_FLAP_RATE * error_Phieg_RATE +
               i_valid * Ki_FLAP_RATE *
                   integral_Phieg_RATE_il); // 按 0.01 系数缩放为混控量。

  // 保存襟翼控制状态供下一周期使用。
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

// 上电时把遥控通道 1～6 初始化为预设安全 PWM。
void initializeRadioFailsafeChannels() {
  channel_1_pwm = channel_1_fs;
  channel_2_pwm = channel_2_fs;
  channel_3_pwm = channel_3_fs;
  channel_4_pwm = channel_4_fs;
  channel_5_pwm = channel_5_fs;
  channel_6_pwm = channel_6_fs;
}

// 根据当前模式选择带积分增稳、无积分增稳或手动，并更新模式标志和指示灯。
void runSelectedControlMode() {
  if (currentMode == STABILIZE_MODE) // 增稳
  {
    i_valid = 1.0;
    int_is_valid = true;
    force_manual = false;
    controlANGLE2(); // 使用串级角度控制器；内环角速度增益需先完成整定。
    displayFlightModeIndicators(currentMode);
    controlFlapMotion();
  } else if (currentMode == STABLIZE_MODE_NO_I) // 增稳
  {
    int_is_valid = false;
    force_manual = false;
    i_valid = 0.0;
    controlANGLE2();        // 使用无积分的角度增稳模式。
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
