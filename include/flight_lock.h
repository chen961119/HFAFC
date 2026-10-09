#pragma once

// 上电锁定。A 机由显示键控制；从机可由上级命令同步锁定状态。
bool isFlightLocked();
void setFlightLocked(bool locked);
