#pragma once
class Servo { public: int value=1500; void attach(int,int,int) {} void write(int v) { value=900+v*1200/180; } void writeMicroseconds(int v) { value=v<900?900:(v>2100?2100:v); } };
