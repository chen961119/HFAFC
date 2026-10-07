// Host checks of the production actuator pipeline; no flight hardware.
#include <cassert>
#include <cmath>
#include <iostream>
#include "control_state.h"
#undef APLANE
#undef BPLANE
#undef CPLANE
#undef DPLANE
#undef EPLANE
#undef FPLANE
#undef GPLANE
#ifdef TEST_MASTER
#define APLANE
#else
#define FPLANE
#endif
#define TESTINDI
#include "interaircraft_comm.h"
constexpr int direction = TEST_DIRECTION;
int32_t pwm_channel1_rev = direction, pwm_channel2_rev = direction,
    pwm_channel3_rev = direction, pwm_channel4_rev = direction, pwm_channel5_rev = direction;
float pwm_channel1_trim = 180, pwm_channel2_trim = -155,
    pwm_channel4_trim = 0, pwm_channel5_trim = 20;
float localElevatorTrim = 29;
float &pwm_channel3_trim = localElevatorTrim;
int channel_8_pwm = 1800, Ail_Clp;
bool int_is_valid;
FlightMode currentMode;
float ailBrake_PWM;
int Local_ail1_PWM, Local_ail2_PWM, Local_ele_PWM, Local_ele_ff_PWM,
    Local_thro_PWM, Local_rudd_PWM;
int s6_command_PWM, s7_command_PWM;
float s6_command_scaled, s7_command_scaled;
float Aail1_scaled, Aail2_scaled, Aele_scaled, Athro_scaled, Arudd_scaled;
float A_pitch_sp;
float Bail1_scaled, Bail2_scaled, Bthro_scaled, Brudd_scaled;
float B_pitch_sp;
float pwm_channel1B_trim = 120, pwm_channel2B_trim = -70, pwm_channel3B_trim = 42;
float B_ele_control_ff_us;
float Cail1_scaled, Cail2_scaled, Cthro_scaled, Crudd_scaled;
float C_pitch_sp;
float pwm_channel1C_trim = 120, pwm_channel2C_trim = -70, pwm_channel3C_trim = 42;
float C_ele_control_ff_us;
float Dail1_scaled, Dail2_scaled, Dthro_scaled, Drudd_scaled;
float D_pitch_sp;
float pwm_channel1D_trim = 120, pwm_channel2D_trim = -70, pwm_channel3D_trim = 42;
float D_ele_control_ff_us;
float Eail1_scaled, Eail2_scaled, Ethro_scaled, Erudd_scaled;
float E_pitch_sp;
float pwm_channel1E_trim = 120, pwm_channel2E_trim = -70, pwm_channel3E_trim = 42;
float E_ele_control_ff_us;
float Fail1_scaled, Fail2_scaled, Fthro_scaled, Frudd_scaled;
float F_pitch_sp;
float pwm_channel1F_trim = 120, pwm_channel2F_trim = -70, pwm_channel3F_trim = 42;
float F_ele_control_ff_us;
float Gail1_scaled, Gail2_scaled, Gthro_scaled, Grudd_scaled;
float G_pitch_sp;
float pwm_channel1G_trim = 120, pwm_channel2G_trim = -70, pwm_channel3G_trim = 42;
float G_ele_control_ff_us;
int childWrites, indiCalls, sends, forwards, feedback;
struct Child { int a1, a2, throttle, rudder, manual, ff; } children[6];
void setLeftChildCommand(unsigned i, int a1, int a2, int t, int r, float, int e, int ff) {
  ++childWrites; children[2*i] = {a1,a2,t,r,e,ff};
}
void setRightChildCommand(unsigned i, int a1, int a2, int t, int r, float, int e, int ff) {
  ++childWrites; children[2*i+1] = {a1,a2,t,r,e,ff};
}
void sendPreparedChildCommands(bool) { ++sends; }
void forwardReceivedChildCommands(bool) { ++forwards; }
void sendGYROxANGLE() { ++feedback; }
int PITCH_INDI_control() { ++indiCalls; return 1631; }
#include "../src/actuator_output.cpp"
void close(float a, float b) { assert(std::fabs(a-b) < 0.01f); }
int main() {
  currentMode = STABLIZE_MODE_NO_I;
  // Scaling never overwrites a final output from the previous cycle.
  Aail1_PWM = 1777; Aele_PWM = 1666;
  scaleCommands();
  assert(Aail1_PWM == 1777 && Aele_PWM == 1666);
  Local_ail1_PWM = 1620; Local_ail2_PWM = 1430;
  Local_ele_PWM = 1570; Local_ele_ff_PWM = 0;
  Local_thro_PWM = 1170; Local_rudd_PWM = 1507;
  prepareActuatorCommands();
#ifdef TEST_MASTER
  close(Aail1_PWM, 1680); close(Aail2_PWM, 1345); close(Aele_PWM, 1529);
  assert(childWrites == 6);
  for (const auto &c : children) { assert(c.a1 == 1620 && c.a2 == 1430 && c.manual == 1542); }
#else
  // Mechanical trim must not create an aileron compensation at zero control.
  close(Aail1_PWM, 1620); close(Aail2_PWM, 1430); close(Aele_PWM, 1529);
  assert(childWrites == 0);
#endif
  Aele_scaled = 0.15f; Ail_Clp = 11;
  B_ele_control_ff_us = 17; C_ele_control_ff_us = -19;
  scaleCommands(); prepareActuatorCommands();
#ifdef TEST_MASTER
  // 150 us elevator control / 6 * 0.4 = 10 us logical flap compensation.
  close(Aail1_PWM, 1680 + direction*21); close(Aail2_PWM, 1345 + direction*21);
  assert(children[0].ff == direction*17 && children[1].ff == direction*-19);
  assert(children[0].manual == 1542 + direction*150);
#else
  close(Aail1_PWM, 1620 + direction*21); close(Aail2_PWM, 1430 + direction*21);
  close(Aele_PWM, 1529 + direction*150);
  Local_ele_ff_PWM = direction*17;
  prepareActuatorCommands(); close(Aele_PWM, 1529 + direction*167);
  currentMode = MANUAL_MODE;
  prepareActuatorCommands(); close(Aele_PWM, 1570);
#endif
  // Apply uses the prepared snapshot even if upstream inputs change afterwards.
  const int expected[] = {int(Aail1_PWM),int(Aail2_PWM),int(Aele_PWM),int(Athro_PWM),int(Arudd_PWM)};
  Ail_Clp = 900; Local_ail1_PWM = Local_ele_PWM = 999; Local_ele_ff_PWM = 900;
  applyAndTransmitActuatorCommands();
  assert(servo1.value == expected[0] && servo2.value == expected[1] && servo3.value == expected[2]);
  assert(servo4.value == expected[3] && servo5.value == expected[4]);
#ifdef TEST_MASTER
  assert(sends == 1 && forwards == 0);
#else
  assert(forwards == 1 && feedback == 1);
#endif
  // The measured INDI physical PWM is used exactly once, with no extra rev/trim.
  Ail_Clp = 0; Local_ele_ff_PWM = direction*17; currentMode = STABILIZE_MODE;
  prepareActuatorCommands(); assert(indiCalls == 1);
#ifdef TEST_MASTER
  close(Aele_PWM, 1631);
#else
  close(Aele_PWM, 1631 + direction*17);
#endif
  std::cout << "PASS: stage separation, neutral trim, direction, child FF, manual/INDI and pure apply\n";
}
