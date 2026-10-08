#pragma once

// 舵机与电调上电时的安全操作；调用顺序由 setup() 决定。
void prepareActuatorPower();
void attachActuators();
void commandSafeActuatorPositions();

// 将归一化控制指令转换为 *_control_us 逻辑 PWM（μs），尚未应用 rev/trim。
void convertControlCommandsToPWM();
// 主机生成本机和子机 PWM；从机叠加本地补偿并选择最终升降舵 PWM。
void prepareActuatorCommands();
// 直接输出已准备的 PWM，并发送或转发命令。
void applyAndTransmitActuatorCommands();
