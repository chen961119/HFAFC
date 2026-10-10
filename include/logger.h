#pragma once

// SD 日志初始化、状态查询和周期记录接口。
void initializeLogger();
bool loggerSdReady();
void flushLogger();
unsigned int loggerFileNumber();
void loggerSINGLE();
void loggerTEAM();
