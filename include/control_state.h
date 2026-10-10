#pragma once

#include "flight_config.h"
#include <stdint.h>

// 仅声明日志、执行器、通信等模块需要读取的控制状态；控制器内部量留在 control_modes.cpp。
// PWM 单位为微秒，遥控通道值也按 PWM 微秒数表示。
extern float central_pitch;
extern int channel_1_pwm, channel_2_pwm, channel_3_pwm, channel_4_pwm;
extern int channel_5_pwm, channel_6_pwm, channel_7_pwm, channel_8_pwm;
// 各通道方向系数与各机安装偏置。
extern int32_t pwm_channel1_rev, pwm_channel2_rev, pwm_channel3_rev,
    pwm_channel4_rev, pwm_channel5_rev;
extern float pwm_channel4_trim, pwm_channel5_trim;
extern float ailBrake_PWM;
extern float D_ele_command_PWM_FF, E_ele_command_PWM_FF;
extern float F_ele_command_PWM_FF, G_ele_command_PWM_FF;
extern FlightMode currentMode;
extern bool int_is_valid;
extern bool force_manual;
extern float pwm_channel1_trim;
extern float pwm_channel2_trim;
extern float pwm_channel3_trim;
// controlMixer() 输出的归一化控制量，供 actuator_output 缩放。
extern float A_ail1_scaled, A_ail2_scaled, A_ele_scaled, A_thro_scaled, A_rudd_scaled;
extern float B_ail1_scaled, B_ail2_scaled, B_thro_scaled, B_rudd_scaled;
extern float C_ail1_scaled, C_ail2_scaled, C_thro_scaled, C_rudd_scaled;
extern float D_ail1_scaled, D_ail2_scaled, D_thro_scaled, D_rudd_scaled;
extern float E_ail1_scaled, E_ail2_scaled, E_thro_scaled, E_rudd_scaled;
extern float F_ail1_scaled, F_ail2_scaled, F_thro_scaled, F_rudd_scaled;
extern float G_ail1_scaled, G_ail2_scaled, G_thro_scaled, G_rudd_scaled;
// extern float s6_command_scaled, s7_command_scaled;
// 未反向的升降舵前馈控制偏移（μs），不含机械中位。
extern float B_ele_control_ff_us, C_ele_control_ff_us, D_ele_control_ff_us, E_ele_control_ff_us, F_ele_control_ff_us, G_ele_control_ff_us;

// A* 是 prepare 阶段生成的本机最终物理 PWM（所有机型通用）。
extern float ail1_PWM;
extern float ail2_PWM;
extern float ele_PWM;
extern float thro_PWM;
extern float rudd_PWM;
extern float A_pitch_sp;
// B～G 的 *_PWM 是发送给子机的控制偏移（μs），不含中位、rev 或 trim。
extern float B_ail1_PWM;
extern float B_ail2_PWM;
extern float B_pitch_sp;
extern float B_thro_PWM;
extern float B_rudd_PWM;
extern float B_ele_command_PWM_Manual;
extern float B_ele_command_PWM_FF;
extern float C_ail1_PWM;
extern float C_ail2_PWM;
extern float C_pitch_sp;
extern float C_thro_PWM;
extern float C_rudd_PWM;
extern float C_ele_command_PWM_Manual;
extern float C_ele_command_PWM_FF;
extern float D_ail1_PWM;
extern float D_ail2_PWM;
extern float D_pitch_sp;
extern float D_thro_PWM;
extern float D_rudd_PWM;
extern float D_ele_command_PWM_Manual;
extern float E_ail1_PWM;
extern float E_ail2_PWM;
extern float E_pitch_sp;
extern float E_thro_PWM;
extern float E_rudd_PWM;
extern float E_ele_command_PWM_Manual;
extern float F_ail1_PWM;
extern float F_ail2_PWM;
extern float F_pitch_sp;
extern float F_thro_PWM;
extern float F_rudd_PWM;
extern float F_ele_command_PWM_Manual;
extern float G_ail1_PWM;
extern float G_ail2_PWM;
extern float G_pitch_sp;
extern float G_thro_PWM;
extern float G_rudd_PWM;
extern float G_ele_command_PWM_Manual;
extern int Ail_Clp;
extern float indi_pitch_q_des_log;
extern float indi_pitch_q_filt_log;
extern float indi_pitch_dq_des_log;
extern float indi_pitch_dq_used_log;
extern float indi_pitch_delta_e_cmd_deg_log;
extern float indi_pitch_delta_e_est_deg_log;
extern float indi_pitch_pwm_cmd_log;
extern float roll_eq;
extern float thro_des;
extern float roll_des;
extern float pitch_des;
extern float yaw_des;
extern float pitch_des_local;
extern float roll_PID;
extern float pitch_PID;
extern float yaw_PID;
extern float Phiab_des;
extern float Phiab_Mea;
extern float Phiab_PID;
extern float Phiac_Mea;
extern float Phiac_PID;
extern float Phibd_Mea;
extern float Phibd_PID;
extern float Phice_Mea;
extern float Phice_PID;
extern float Phidf_Mea;
extern float Phidf_PID;
extern float Phieg_Mea;
extern float Phieg_PID;
extern float phiab;
extern float phiac;
extern float phibd;
extern float phice;
extern float phidf;
extern float phieg;
extern float Pab;
extern float Pac;
extern float Pbd;
extern float Pdf;
extern float Pce;
extern float Peg;
extern int m1_command_PWM;
extern int m2_command_PWM;
extern int m3_command_PWM;
extern int m4_command_PWM;
extern int m5_command_PWM;
extern int m6_command_PWM;
// extern int s6_command_PWM;
// extern int s7_command_PWM;
