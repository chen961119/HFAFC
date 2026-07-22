#ifndef MS4525_h
#define MS4525_h

#include "Arduino.h"
#include "Wire.h"    // I2C library

#define  I2C_ADDR   0x28//I2C地址
#define Pmax 1.0f
#define Pmin -1.0f
#define COUNTS_MAX 13106.4f//80%*16383
#define COUNTS_MIN 1638.3f//10%*16383
#define PSI_to_Pa 6894.7f//psi转Pa

typedef struct
{
    float pressure;
    float airspeed;
    float temperature;
    float pressure_drift;
}MS4525_paramt;

class MS4525read_Task
{
    protected:
    uint8_t buf[4];
    TwoWire* MS4525_wire;
    public:
    MS4525_paramt MS4525_param;
    public:
    MS4525read_Task(TwoWire* wire) : MS4525_wire(wire)
    {
        if (MS4525_wire) {MS4525_wire->begin();}
    }
    bool readMS4525();
    void calib();
    /*获取空速*/
    float getAirspeed()
    {
        this->readMS4525();
        
        float dp=MS4525_param.pressure-MS4525_param.pressure_drift;
        
        if(dp<0)dp=0.0f;
        MS4525_param.airspeed=sqrtf((2*dp*PSI_to_Pa)/1.225);
        
        return MS4525_param.airspeed;
    }
};
#endif