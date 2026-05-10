#ifndef __PID_H__
#define __PID_H__

#include "headfile.h"

typedef struct
{
    float Kp;
    float Ki;
    float Kd;
    float Error0;//本次误差
    float Error1;//上次误差
    float ErrorInt;
    float OutMax;
    float OutMin;
    float Target;
    float Actual;
    float Out;
} PID;

typedef struct
{
    float Kp;
    float Ki;
    float Kd;
    int Error0;//本次误差
    int Error1;//上次误差
    float ErrorInt;
    float OutMax;
    float OutMin;
    int Target;
    int Actual;
    float Out;
} PID_JY901S;

extern int16_t Speed, Location;

void PID_Init(PID *pid,int mode);
void PID_Update(PID *pid);
void PID_JY901S_Update(PID_JY901S *pid);
void PID_Sensor_Update(PID *pid,uint8_t *SensorData);

#endif
