#include "headfile.h"

int Count_turn = 0;

static CarMotionParams_t car_motion_params = {
    .forward_speed = 15,
    .turn_speed = 10,
    .default_duration_ms = 1000,
    .auto_stop = 1
};

static int ClampInt(int value, int min_value, int max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

void CarMotion_SetForwardSpeed(int speed)
{
    car_motion_params.forward_speed = ClampInt(speed, 0, 80);
}

void CarMotion_SetTurnSpeed(int speed)
{
    car_motion_params.turn_speed = ClampInt(speed, 0, 70);
}

void CarMotion_SetDefaultDuration(uint16_t duration_ms)
{
    car_motion_params.default_duration_ms = (uint16_t)ClampInt(duration_ms, 200, 30000);
}

void CarMotion_SetAutoStop(uint8_t enabled)
{
    car_motion_params.auto_stop = enabled ? 1 : 0;
}

CarMotionParams_t CarMotion_GetParams(void)
{
    return car_motion_params;
}
// 定义速度�?
#define DIAGONAL_SCALE_DIVISOR 2

int16_t GetLeftEncoder(void)
{
    int16_t temp;
    temp = __HAL_TIM_GetCounter(&htim3);
    __HAL_TIM_SetCounter(&htim3,0);
    return -temp;
}

int16_t GetRightEncoder(void)
{
    int16_t temp;
    temp = __HAL_TIM_GetCounter(&htim1);
    __HAL_TIM_SetCounter(&htim1,0);
    return temp;
}

// 设置电机方向和速度
void SetMotor(int left_speed, int right_speed)
{
    // 左电机控�?
    if(left_speed >= 0) {
        // 正转
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
    } else {
        // 反转
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
        left_speed = -left_speed;
    }
    
    // 右电机控�?
    if(right_speed >= 0) {
        // 正转
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
    } else {
        // 反转
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
        right_speed = -right_speed;
    }
    
    // 限制速度�?-100范围�?
    if(left_speed > 100) left_speed = 100;
    if(right_speed > 100) right_speed = 100;
    
    // 设置PWM
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, left_speed);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, right_speed);
}

// 前进
void MoveForward(void)
{
    SetMotor(car_motion_params.forward_speed, car_motion_params.forward_speed);
}

// 后退
void MoveBackward(void)
{
    SetMotor(-car_motion_params.forward_speed, -car_motion_params.forward_speed);
}

// 左转
void MoveLeft(void)
{
    SetMotor(-car_motion_params.turn_speed, car_motion_params.turn_speed);
}

// 右转
void MoveRight(void)
{
    SetMotor(car_motion_params.turn_speed, -car_motion_params.turn_speed);
}

// 左上
void MoveLeftUp(void)
{
    SetMotor(car_motion_params.forward_speed / DIAGONAL_SCALE_DIVISOR, car_motion_params.forward_speed);
}

// 右上
void MoveRightUp(void)
{
    SetMotor(car_motion_params.forward_speed, car_motion_params.forward_speed / DIAGONAL_SCALE_DIVISOR);
}

// 左下
void MoveLeftDown(void)
{
    SetMotor(-(car_motion_params.forward_speed / DIAGONAL_SCALE_DIVISOR), -car_motion_params.forward_speed);
}

// 右下
void MoveRightDown(void)
{
    SetMotor(-car_motion_params.forward_speed, -(car_motion_params.forward_speed / DIAGONAL_SCALE_DIVISOR));
}

// 停止
void MoveStop(void)
{
    SetMotor(0, 0);
}

void TurnLeft()
{
    MoveLeft();
}

void TurnRight()
{
    MoveRight();
}

void MOVESTOP()
{
    MoveStop();
}
