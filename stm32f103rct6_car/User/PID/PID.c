#include "PID.h"
#include <math.h>

#define EPSILON 0.0001f

int16_t Speed, Location;

void PID_Init(PID *pid,int mode)
{
    /*
    内环kp=0.35,ki=0.3,kd=0
    外环kp=0.35,ki=0,kd=0.45
    */
    switch(mode)
    {
        case 0:
        //内环
        pid->Kp = 1.5f;
        pid->Ki = 0.01f;
        pid->Kd = 0.0f;
        pid->Error0 = 0.0f;
        pid->Error1 = 0.0f;
        pid->ErrorInt = 0.0f;
        pid->OutMax = 150.0f;
        pid->OutMin = -150.0f;
        pid->Target = 50.0f;
        pid->Actual = 0.0f;
        pid->Out = 0.0f;
            break;
        case 1:
        //外环
            pid->Kp = 0.35;
            pid->Ki = 0;
            pid->Kd = 0.45;
            pid->OutMax = 50;
            pid->OutMin = -50;
            break;
        default:
            break;
    }
    pid->Error0 = 0;
    pid->Error1 = 0;
    pid->ErrorInt = 0;
    pid->Target = 0;
    pid->Actual = 0;
    pid->Out = 0;
}

void PID_Update(PID *pid)
{
    pid->Error1 = pid->Error0;
    pid->Error0 = pid->Target - pid->Actual;
    /*变速积分*/
    float C = 1 / (0.2 * fabs(pid->Error0) + 1);
    if(fabs(pid->Error0) < 20)
    {
        pid->ErrorInt += C * pid->Error0;
    }
    else
    {
        pid->ErrorInt = 0;
    }

    /*防止Ki从0变为非0时，出现积分饱和*/
    if(fabs(pid->Ki) > EPSILON)
    {
        pid->ErrorInt += pid->Error0;
    }
    else
    {
        pid->ErrorInt = 0;
    }

    pid->Out = pid->Kp * pid->Error0
             + pid->Ki * pid->ErrorInt
             + pid->Kd * (pid->Error0 - pid->Error1);

    if (pid->Out > pid->OutMax)
        pid->Out = pid->OutMax;
    else if (pid->Out < pid->OutMin)
        pid->Out = pid->OutMin;
}

void PID_JY901S_Update(PID_JY901S *pid)
{
    pid->Error1 = pid->Error0;
    pid->Error0 = pid->Target - pid->Actual;
    /*变速积分*/
    float C = 1 / (0.2 * pid->Error0 + 1);
    if(pid->Error0 < 3)
    {
        pid->ErrorInt += C * pid->Error0;
    }
    else
    {
        pid->ErrorInt = 0;
    }

    /*防止Ki从0变为非0时，出现积分饱和*/
    if(fabs(pid->Ki) > EPSILON)
    {
        pid->ErrorInt += pid->Error0;
    }
    else
    {
        pid->ErrorInt = 0;
    }

    pid->Out = pid->Kp * pid->Error0
             + pid->Ki * pid->ErrorInt
             + pid->Kd * (pid->Error0 - pid->Error1);
    
    if(pid->Target > 0){
        pid->Out = -pid->Out;
    }

    if (pid->Out > pid->OutMax)
        pid->Out = pid->OutMax;
    else if (pid->Out < pid->OutMin)
        pid->Out = pid->OutMin;
}

void PID_Sensor_Update(PID *pid,uint8_t *SensorData)
{
    pid->Error1 = pid->Error0;
    pid->Error0 = 0;
    // 8路传感器的权重配置
    // 从左到右：传感器0-7，中间位置在3.5处
    static const float weights[8] = {
        -70.0f, -50.0f, -30.0f, -10.0f, 
         10.0f,  30.0f,  50.0f,  70.0f
    };
    
    // 计算位置偏差
    uint8_t active_count = 0;
    for (uint8_t i = 0; i < 8; i++)
    {
        if (SensorData[i] == 1)
        {
            pid->Error0 += weights[i];
            active_count++;
        }
    }
    
    // 如果没有检测到黑线，使用上一次的误差值
    if (active_count == 0) {
        pid->Error0 = 0;
    }
    // 如果检测到多个传感器，取平均值
    else if (active_count > 1) {
        pid->Error0 /= active_count;
    }


    /*变速积分*/
    float C = 1 / (0.2 * fabs(pid->Error0) + 1);
    if(fabs(pid->Error0) < 3)
    {
        pid->ErrorInt += C * pid->Error0;
    }
    else
    {
        pid->ErrorInt = 0;
    }

    /*防止Ki从0变为非0时，出现积分饱和*/
    if(fabs(pid->Ki) > EPSILON)
    {
        pid->ErrorInt += pid->Error0;
    }
    else
    {
        pid->ErrorInt = 0;
    }

    pid->Out = pid->Kp * pid->Error0
             + pid->Ki * pid->ErrorInt
             + pid->Kd * (pid->Error0 - pid->Error1);

    if (pid->Out > pid->OutMax)
        pid->Out = pid->OutMax;
    else if (pid->Out < pid->OutMin)
        pid->Out = pid->OutMin;
}
