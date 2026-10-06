#pragma once

// 舵机与电调上电时的安全操作；调用顺序由 setup() 决定。
void prepareActuatorPower();
void attachActuators();
void commandSafeActuatorPositions();

// 将控制器输出换算为各机 PWM，并写入子机命令缓冲区；最后输出本机舵机并发送命令。
void prepareActuatorCommands();
void applyAndTransmitActuatorCommands();
