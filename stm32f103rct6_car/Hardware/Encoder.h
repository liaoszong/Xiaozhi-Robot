#ifndef _ENCODER_H
#define _ENCODER_H

#include "headfile.h"

int16_t GetLeftEncoder(void);
int16_t GetRightEncoder(void);
void SetMotor(int left_speed, int right_speed);


typedef struct {
    int forward_speed;
    int turn_speed;
    uint16_t default_duration_ms;
    uint8_t auto_stop;
} CarMotionParams_t;

void CarMotion_SetForwardSpeed(int speed);
void CarMotion_SetTurnSpeed(int speed);
void CarMotion_SetDefaultDuration(uint16_t duration_ms);
void CarMotion_SetAutoStop(uint8_t enabled);
CarMotionParams_t CarMotion_GetParams(void);
// 运动控制函数
void MoveForward(void);
void MoveBackward(void);
void MoveLeft(void);
void MoveRight(void);
void MoveLeftUp(void);
void MoveRightUp(void);
void MoveLeftDown(void);
void MoveRightDown(void);
void MoveStop(void);

void TurnLeft();
void TurnRight();
void MOVESTOP();

extern int Count_turn;

#endif
