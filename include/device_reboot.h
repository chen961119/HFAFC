#pragma once
// Only accepted while locked. Reset waits for the USB reply to drain.
bool requestDeviceReboot();
bool deviceRebootPending();
void pollDeviceReboot(bool responsePending);
