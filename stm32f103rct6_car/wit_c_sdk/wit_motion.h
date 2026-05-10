#ifndef __WIT_MOTION_H
#define __WIT_MOTION_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include "communication.h"
#include "wit_c_sdk.h"

// 角度数据结构
typedef struct {
    float roll;
    float pitch;
    float yaw;
} WitAngleData;

// 初始化陀螺仪
void WitMotion_Init(UART_HandleTypeDef *huart);

// 处理接收数据
void WitMotion_ProcessData(uint8_t *data, uint16_t size);

// 获取角度数据
WitAngleData WitMotion_GetAngle(void);
uint8_t WitMotion_IsDataUpdated(void);

// 获取原始角度�?
float WitMotion_GetRawRoll(void);
float WitMotion_GetRawPitch(void);
float WitMotion_GetRawYaw(void);

// 配置函数
HAL_StatusTypeDef WitMotion_ConfigOutputRate(uint8_t rate);
HAL_StatusTypeDef WitMotion_ConfigContent(uint32_t content);
HAL_StatusTypeDef WitMotion_CalibrateAccel(void);

#ifdef __cplusplus
}
#endif

#endif
