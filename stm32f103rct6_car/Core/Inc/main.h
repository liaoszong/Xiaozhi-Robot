/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

// 工作模式定义
typedef enum {
    MODE_BLUETOOTH = 0,    // 蓝牙控制模式
    MODE_LINE_FOLLOWING,   // 巡线模式
    MODE_STOP              // 停止模式
} WorkMode_t;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PWMA_Pin GPIO_PIN_2
#define PWMA_GPIO_Port GPIOA
#define PWMB_Pin GPIO_PIN_3
#define PWMB_GPIO_Port GPIOA
#define AIN2_Pin GPIO_PIN_4
#define AIN2_GPIO_Port GPIOA
#define AIN1_Pin GPIO_PIN_5
#define AIN1_GPIO_Port GPIOA
#define E1A_Pin GPIO_PIN_6
#define E1A_GPIO_Port GPIOA
#define E1B_Pin GPIO_PIN_7
#define E1B_GPIO_Port GPIOA
#define SDA_Pin GPIO_PIN_6
#define SDA_GPIO_Port GPIOC
#define SCK_Pin GPIO_PIN_7
#define SCK_GPIO_Port GPIOC
#define E2A_Pin GPIO_PIN_8
#define E2A_GPIO_Port GPIOA
#define E2B_Pin GPIO_PIN_9
#define E2B_GPIO_Port GPIOA
#define BIN2_Pin GPIO_PIN_11
#define BIN2_GPIO_Port GPIOA
#define BIN1_Pin GPIO_PIN_12
#define BIN1_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
