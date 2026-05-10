/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "headfile.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SensorNum 5
#define TARGET_SPEED 15.0f

// USART3/ESP32 safety gate:
// 1) Ignore USART3 input shortly after boot, avoiding ESP32 boot logs/noise.
// 2) Only accept framed car commands, not raw single characters.
#define USART3_STARTUP_IGNORE_MS 2000U
#define ACCEPT_LEGACY_SINGLE_CHAR_CMD 0

// Yaw closed-loop hold for straight forward/backward movement.
// If the car becomes more biased after correction, flip
// YAW_HOLD_FORWARD_REVERSE_CORRECTION between 1 and 0 first.
#define YAW_HOLD_TEST_DISABLED 0
#define YAW_HOLD_DEADBAND_DEG 1.5f
#define YAW_HOLD_CORRECTION_MAX 5.0f
#define YAW_HOLD_INTEGRAL_MAX 80.0f
#define YAW_HOLD_KP 0.30f
#define YAW_HOLD_KI 0.02f
#define YAW_HOLD_KD 0.06f
#define YAW_HOLD_FORWARD_REVERSE_CORRECTION 1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// 全局变量定义
uint8_t SensorData[8] = {0, 0, 0, 0, 0, 0, 0, 0};
WitAngleData angle;
unsigned char rx_buff[256] = {0};
unsigned short Anolog[8] = {0};
unsigned short white[8] = {1988,1681,1753,1489,2098,2494,2098,1395};
unsigned short black[8] = {225,291,230,294,234,323,239,296};
unsigned short Normal[8];
unsigned char Digtal;

uint8_t KeyNum;
WorkMode_t current_mode = MODE_STOP;  // 当前工作模式
char current_cmd = 'Z';               // 蓝牙当前命令
char display_info[32] = "Stop";       // 显示信息
volatile uint8_t line_follow_update_pending = 0;
volatile uint8_t yaw_hold_update_pending = 0;
volatile uint8_t yaw_hold_enabled = 0;
float yaw_hold_target = 0.0f;
float yaw_hold_last_error = 0.0f;
float yaw_hold_integral = 0.0f;
uint32_t yaw_hold_last_tick = 0;

// PID控制�?
PID Innerpid_L = {
    .Kp = 1.5f, .Ki = 0.01f, .Kd = 0.0f,
    .Error0 = 0.0f, .Error1 = 0.0f, .ErrorInt = 0.0f,
    .OutMax = 150.0f, .OutMin = -150.0f,
    .Target = TARGET_SPEED, .Actual = 0.0f, .Out = 0.0f
};

PID Innerpid_R = {
    .Kp = 1.5f, .Ki = 0.01f, .Kd = 0.0f,
    .Error0 = 0.0f, .Error1 = 0.0f, .ErrorInt = 0.0f,
    .OutMax = 150.0f, .OutMin = -150.0f,
    .Target = TARGET_SPEED, .Actual = 0.0f, .Out = 0.0f
};

PID Outerpid = {
    .Kp = 2.0f, .Ki = 0.0f, .Kd = 0.5f,
    .Error0 = 0.0f, .Error1 = 0.0f, .ErrorInt = 0.0f,
    .OutMax = 30.0f, .OutMin = -30.0f,
    .Target = 0.0f, .Actual = 0.0f, .Out = 0.0f
};

PID imupid = {
    .Kp = 0.5f, .Ki = 0.0f, .Kd = 0.3f,
    .Error0 = 0, .Error1 = 0, .ErrorInt = 0.0f,
    .OutMax = 3.0f, .OutMin = -3.0f,
    .Target = -113.0f, .Actual = -113.0f, .Out = 0.0f
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void ProcessBluetoothCommand(char cmd);
void ExecuteCommand(char cmd);
void ExecuteLastCommand(void);
void UpdateLineFollowing(void);
void UpdateDisplay(void);
uint8_t TryProcessProtocolFrame(void);
uint8_t TryExtractCarCommand(const char *frame, char *cmd);
void SendCarParams(void);
void SendCarState(char cmd);
const char* GetCommandStateName(char cmd);
void StartYawHold(void);
void StopYawHold(void);
uint8_t IsYawHoldCommand(char cmd);
float NormalizeYawError(float error);
void UpdateYawHoldControl(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    /* USER CODE BEGIN 1 */
    uint32_t usart3_accept_tick = 0;
    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/
    HAL_Init();
    SystemClock_Config();

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_TIM5_Init();
    MX_USART1_UART_Init();
    MX_ADC1_Init();
    MX_USART3_UART_Init();

    /* USER CODE BEGIN 2 */
    // 初始化编码器
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
    
    // 初始化PWM输出
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);
    
    // 初始化OLED
    OLED_Init();
    OLED_Update();
    
    // 初始化串口和蓝牙
    Com_usart_Init();
    
    // 初始化陀螺仪
    WitMotion_Init(&huart1);
    angle = WitMotion_GetAngle();
    imupid.Target = angle.yaw;
    WitMotion_ConfigOutputRate(RRATE_20HZ);
    WitMotion_ConfigContent(RSW_ANGLE | RSW_GYRO);
    
    // 设置电机方向
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
    
    // 初始化占空比
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, 0);
    
    // 初始化灰度传感器
    No_MCU_Sensor sensor;
    No_MCU_Ganv_Sensor_Init(&sensor, white, black);
    HAL_Delay(100);
    No_Mcu_Ganv_Sensor_Task_Without_tick(&sensor);
    Digtal = Get_Digtal_For_User(&sensor);
    
    // 初始化蜂鸣器
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
    
    // 启动定时�?
    HAL_TIM_Base_Start_IT(&htim2);
    
    // 初始状态停止，并丢弃上电阶段 USART3 噪声/ESP32 启动日志
    current_mode = MODE_STOP;
    current_cmd = CMD_STOP;
    StopYawHold();
    MoveStop();
    Bluetooth_SetLastValidCommand(CMD_STOP);
    Bluetooth_ClearFlag();
    usart3_accept_tick = HAL_GetTick() + USART3_STARTUP_IGNORE_MS;

    /* USER CODE END 2 */

    /* Infinite loop */
    while (1)
    {
        // 按键处理
        KeyNum = Key_GetNum();
        if (KeyNum == 1) {
            // 切换到停止模�?
            current_mode = MODE_STOP;
            MoveStop();
            strcpy(display_info, "Stop Mode");
            Bluetooth_SendString("Mode: Stop\r\n");
        } else if (KeyNum == 2) {
            // 手动进入蓝牙控制模式：清掉历史/噪声命令，避免一进模式就误动作。
            current_mode = MODE_BLUETOOTH;
            current_cmd = CMD_STOP;
            StopYawHold();
            MoveStop();
            Bluetooth_SetLastValidCommand(CMD_STOP);
            Bluetooth_ClearFlag();
            strcpy(display_info, "Bluetooth Mode");
            Bluetooth_SendString("Mode: Bluetooth Control\r\n");
        } else if (KeyNum == 3) {
            // 切换到巡线模�?
            current_mode = MODE_LINE_FOLLOWING;
            strcpy(display_info, "Line Following");
            Bluetooth_SendString("Mode: Line Following\r\n");
        }
        
        // USART3 command processing.
        // - Framed commands such as CAR:F / #F are accepted in any mode.
        // - Legacy raw single-character commands are accepted only after Key 2
        //   manually enters MODE_BLUETOOTH. This keeps manual Bluetooth usable
        //   while preventing boot noise from auto-moving the car.
        if (Bluetooth_DataReady()) {
            if (HAL_GetTick() < usart3_accept_tick) {
                // ESP32 may print boot logs or leave TX unstable during startup.
                // Drop everything before the guard time expires.
                Bluetooth_ClearFlag();
            } else {
                if (TryProcessProtocolFrame()) {
                    Bluetooth_ClearFlag();
                } else {
                    if (current_mode == MODE_BLUETOOTH && Bluetooth_ProcessCommands()) {
                        while (Bluetooth_HasPendingCommands()) {
                            char command = Bluetooth_GetNextCommand();
                            if (command != 0) {
                                ExecuteCommand(command);
                            }
                        }
                    }
                    Bluetooth_ClearFlag();
                }
            }
        }
        
        // 灰度传感器数据处�?
        No_Mcu_Ganv_Sensor_Task_Without_tick(&sensor);
        Digtal = Get_Digtal_For_User(&sensor);
        
        // 陀螺仪数据处理
        if (WitMotion_IsDataUpdated()) {
            angle = WitMotion_GetAngle();
            // 可以在这里添加陀螺仪数据处理逻辑
        }
        
        // 持续执行蓝牙命令（如果没有收到停止命令）
#if 0
        // Disabled for safety: do not repeat the previous motion command
        // after a stop, otherwise a stale/noisy command may move the car again.
        if (current_mode == MODE_BLUETOOTH && 
            Bluetooth_GetLastValidCommand() != CMD_STOP && 
            current_cmd == CMD_STOP) {
            ExecuteLastCommand();
        }
#endif
        
        if (current_mode == MODE_BLUETOOTH && yaw_hold_update_pending) {
            yaw_hold_update_pending = 0;
            UpdateYawHoldControl();
        }

        // Run line-following control in the main loop when TIM2 schedules it.
        if (current_mode == MODE_LINE_FOLLOWING && line_follow_update_pending) {
            line_follow_update_pending = 0;
            UpdateLineFollowing();
        }
        
        // 更新显示
        UpdateDisplay();
        
        HAL_Delay(20);
    }
}

void SystemClock_Config(void)
{
    // 系统时钟配置
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
    PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
    HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);
}

/* USER CODE BEGIN 4 */

/**
  * @brief 处理蓝牙命令
  */
void StartYawHold(void)
{
    // Lock the current heading as the straight-line target.
    yaw_hold_target = angle.yaw;
    yaw_hold_last_error = 0.0f;
    yaw_hold_integral = 0.0f;
    yaw_hold_last_tick = HAL_GetTick();
    yaw_hold_enabled = 1;
}

void StopYawHold(void)
{
    yaw_hold_enabled = 0;
    yaw_hold_update_pending = 0;
    yaw_hold_last_error = 0.0f;
    yaw_hold_integral = 0.0f;
    yaw_hold_last_tick = 0;
}

uint8_t IsYawHoldCommand(char cmd)
{
    return (cmd == CMD_FORWARD || cmd == CMD_BACKWARD);
}

float NormalizeYawError(float error)
{
    while (error > 180.0f) error -= 360.0f;
    while (error < -180.0f) error += 360.0f;
    return error;
}

void UpdateYawHoldControl(void)
{
    if (!yaw_hold_enabled || !IsYawHoldCommand(current_cmd)) {
        return;
    }

    CarMotionParams_t params = CarMotion_GetParams();
    int base_speed = params.forward_speed;

    uint32_t now = HAL_GetTick();
    float dt = 0.1f;
    if (yaw_hold_last_tick != 0 && now >= yaw_hold_last_tick) {
        dt = (float)(now - yaw_hold_last_tick) / 1000.0f;
        if (dt < 0.01f) dt = 0.01f;
        if (dt > 0.5f) dt = 0.1f;
    }
    yaw_hold_last_tick = now;

    float error = NormalizeYawError(yaw_hold_target - angle.yaw);

    // Small deadband avoids continuous left-right hunting around the target.
    if (error > -YAW_HOLD_DEADBAND_DEG && error < YAW_HOLD_DEADBAND_DEG) {
        yaw_hold_last_error = error;
        yaw_hold_integral = 0.0f;
        if (current_cmd == CMD_FORWARD) {
            SetMotor(base_speed, base_speed);
        } else if (current_cmd == CMD_BACKWARD) {
            SetMotor(-base_speed, -base_speed);
        }
        return;
    }

    yaw_hold_integral += error * dt;
    if (yaw_hold_integral > YAW_HOLD_INTEGRAL_MAX) yaw_hold_integral = YAW_HOLD_INTEGRAL_MAX;
    if (yaw_hold_integral < -YAW_HOLD_INTEGRAL_MAX) yaw_hold_integral = -YAW_HOLD_INTEGRAL_MAX;

    float derivative = (error - yaw_hold_last_error) / dt;
    yaw_hold_last_error = error;

    float correction = YAW_HOLD_KP * error +
                       YAW_HOLD_KI * yaw_hold_integral +
                       YAW_HOLD_KD * derivative;

    if (correction > YAW_HOLD_CORRECTION_MAX) correction = YAW_HOLD_CORRECTION_MAX;
    if (correction < -YAW_HOLD_CORRECTION_MAX) correction = -YAW_HOLD_CORRECTION_MAX;

    int correction_pwm = (correction >= 0.0f) ?
                         (int)(correction + 0.5f) :
                         (int)(correction - 0.5f);

    if (current_cmd == CMD_FORWARD) {
#if YAW_HOLD_FORWARD_REVERSE_CORRECTION
        // Current test direction: positive correction reduces left wheel and
        // increases right wheel. If the car corrects farther away from target,
        // set YAW_HOLD_FORWARD_REVERSE_CORRECTION to 0.
        SetMotor(base_speed - correction_pwm, base_speed + correction_pwm);
#else
        SetMotor(base_speed + correction_pwm, base_speed - correction_pwm);
#endif
    } else if (current_cmd == CMD_BACKWARD) {
        // Backward keeps the previous correction direction because it was
        // reported to be more stable than forward.
        SetMotor(-base_speed - correction_pwm, -base_speed + correction_pwm);
    }
}
uint8_t TryExtractCarCommand(const char *frame, char *cmd)
{
    const char *p = frame;

    if (frame == NULL || cmd == NULL) {
        return 0;
    }

    while (*p == ' ' || *p == '	') {
        p++;
    }

    // Preferred frames from ESP32:
    //   CAR:F\n  CAR:B\n  CAR:L\n  CAR:R\n  CAR:S\n
    if (strncmp(p, "CAR:", 4) == 0) {
        p += 4;
    } else if (*p == '#') {
        // Optional compact frames:
        //   #F\n  #B\n  #L\n  #R\n  #S\n
        p += 1;
    } else {
#if ACCEPT_LEGACY_SINGLE_CHAR_CMD
        // Compatibility mode is intentionally disabled by default because
        // raw characters are too easy to trigger by noise or boot logs.
#else
        return 0;
#endif
    }

    if (p[0] == '\0') {
        return 0;
    }

    // A valid command frame must contain exactly one command character
    // after the prefix, allowing only trailing spaces/tabs.
    char c = p[0];
    p++;
    while (*p == ' ' || *p == '	') {
        p++;
    }
    if (*p != '\0') {
        return 0;
    }

    switch (c) {
        case CMD_FORWARD:
        case CMD_BACKWARD:
        case CMD_LEFT:
        case CMD_RIGHT:
        case CMD_STOP:
            *cmd = c;
            return 1;
        default:
            return 0;
    }
}

uint8_t TryProcessProtocolFrame(void)
{
    char frame[BLUETOOTH_RX_BUFFER_SIZE];
    uint16_t size;

    __disable_irq();
    size = hbluetooth.process_size;
    if (size >= sizeof(frame)) {
        size = sizeof(frame) - 1;
    }
    memcpy(frame, hbluetooth.process_buffer, size);
    __enable_irq();

    frame[size] = '\0';
    for (uint16_t i = 0; i < size; i++) {
        if (frame[i] == '\r' || frame[i] == '\n') {
            frame[i] = '\0';
            break;
        }
    }

    char cmd = 0;
    if (TryExtractCarCommand(frame, &cmd)) {
        if (current_mode != MODE_BLUETOOTH) {
            current_mode = MODE_BLUETOOTH;
            strcpy(display_info, "Voice Control");
        }
        ExecuteCommand(cmd);
        return 1;
    }

    if (strncmp(frame, "GET:PARAM", 9) == 0) {
        SendCarParams();
        return 1;
    }

    int value = 0;
    if (sscanf(frame, "SET:FS=%d", &value) == 1) {
        CarMotion_SetForwardSpeed(value);
        Bluetooth_Printf("ACK:SET:FS=%d\r\n", CarMotion_GetParams().forward_speed);
        SendCarParams();
        return 1;
    }
    if (sscanf(frame, "SET:TS=%d", &value) == 1) {
        CarMotion_SetTurnSpeed(value);
        Bluetooth_Printf("ACK:SET:TS=%d\r\n", CarMotion_GetParams().turn_speed);
        SendCarParams();
        return 1;
    }
    if (sscanf(frame, "SET:DUR=%d", &value) == 1) {
        CarMotion_SetDefaultDuration((uint16_t)value);
        Bluetooth_Printf("ACK:SET:DUR=%d\r\n", (int)CarMotion_GetParams().default_duration_ms);
        SendCarParams();
        return 1;
    }
    if (sscanf(frame, "SET:AUTO=%d", &value) == 1) {
        CarMotion_SetAutoStop(value ? 1 : 0);
        Bluetooth_Printf("ACK:SET:AUTO=%d\r\n", (int)CarMotion_GetParams().auto_stop);
        SendCarParams();
        return 1;
    }

    return 0;
}

void SendCarParams(void)
{
    CarMotionParams_t params = CarMotion_GetParams();
    Bluetooth_Printf("PARAM:FS=%d,TS=%d,DUR=%d,AUTO=%d\r\n",
                     params.forward_speed,
                     params.turn_speed,
                     (int)params.default_duration_ms,
                     (int)params.auto_stop);
}

const char* GetCommandStateName(char cmd)
{
    switch (cmd) {
        case CMD_FORWARD: return "FORWARD";
        case CMD_BACKWARD: return "BACKWARD";
        case CMD_LEFT: return "LEFT";
        case CMD_RIGHT: return "RIGHT";
        case CMD_LEFT_UP: return "LEFT_FORWARD";
        case CMD_RIGHT_UP: return "RIGHT_FORWARD";
        case CMD_LEFT_DOWN: return "LEFT_BACKWARD";
        case CMD_RIGHT_DOWN: return "RIGHT_BACKWARD";
        case CMD_STOP: return "STOP";
        default: return "UNKNOWN";
    }
}

void SendCarState(char cmd)
{
    CarMotionParams_t params = CarMotion_GetParams();
    Bluetooth_Printf("ACK:%c\r\n", cmd);
    Bluetooth_Printf("STATE:%s,FS=%d,TS=%d,YAW=%d,TARGET=%d,HOLD=%d\r\n",
                     GetCommandStateName(cmd),
                     params.forward_speed,
                     params.turn_speed,
                     (int)angle.yaw,
                     (int)yaw_hold_target,
                     (int)yaw_hold_enabled);
}
void ProcessBluetoothCommand(char cmd)
{
    switch(cmd) {
        case CMD_FORWARD:
#if YAW_HOLD_TEST_DISABLED
            StopYawHold();
#else
            StartYawHold();
#endif
            MoveForward();
            strcpy(display_info, "Forward");
            break;
        case CMD_STOP:
            StopYawHold();
            MoveStop();
            strcpy(display_info, "Stop");
            Bluetooth_SetLastValidCommand(CMD_STOP);
            break;
        case CMD_LEFT:
            StopYawHold();
            MoveLeft();
            strcpy(display_info, "Left");
            break;
        case CMD_RIGHT:
            StopYawHold();
            MoveRight();
            strcpy(display_info, "Right");
            break;
        case CMD_BACKWARD:
#if YAW_HOLD_TEST_DISABLED
            StopYawHold();
#else
            StartYawHold();
#endif
            MoveBackward();
            strcpy(display_info, "Backward");
            break;
        case CMD_LEFT_UP:
            StopYawHold();
            MoveLeftUp();
            strcpy(display_info, "Left Up");
            break;
        case CMD_RIGHT_UP:
            StopYawHold();
            MoveRightUp();
            strcpy(display_info, "Right Up");
            break;
        case CMD_LEFT_DOWN:
            StopYawHold();
            MoveLeftDown();
            strcpy(display_info, "Left Down");
            break;
        case CMD_RIGHT_DOWN:
            StopYawHold();
            MoveRightDown();
            strcpy(display_info, "Right Down");
            break;
        default:
            break;
    }
    
    if(cmd != CMD_STOP) {
        Bluetooth_SetLastValidCommand(cmd);
    }
}

/**
  * @brief 执行蓝牙命令
  */
void ExecuteCommand(char cmd)
{
    current_cmd = cmd;
    ProcessBluetoothCommand(cmd);
    
    char response[64];
    snprintf(response, sizeof(response), "Exec: %c - %s\r\n", cmd, display_info);
    Bluetooth_SendString(response);
    SendCarState(cmd);
}

/**
  * @brief 执行最后一个有效命�?
  */
void ExecuteLastCommand(void)
{
    char last_cmd = Bluetooth_GetLastValidCommand();
    if(last_cmd != CMD_STOP) {
        current_cmd = last_cmd;
        ProcessBluetoothCommand(last_cmd);
    }
}

/**
  * @brief 更新巡线控制
  */
void UpdateLineFollowing(void)
{
    // 获取灰度传感器数�?
    for(int i = 0; i < 8; i++) {
        SensorData[i] = (Digtal >> i) & 0x01;
    }
    
    // 更新PID控制�?
    imupid.Actual = angle.yaw;
    PID_Sensor_Update(&Outerpid, SensorData);
    PID_Update(&imupid);
    
    Innerpid_L.Target = TARGET_SPEED - Outerpid.Out;
    Innerpid_R.Target = TARGET_SPEED + Outerpid.Out;
    
    Innerpid_L.Actual = GetLeftEncoder();
    Innerpid_R.Actual = GetRightEncoder();
    
    PID_Update(&Innerpid_L);
    PID_Update(&Innerpid_R);
    
    // SetMotor handles direction pins and clamps signed PID output to PWM range.
    SetMotor((int)Innerpid_L.Out, (int)Innerpid_R.Out);
}

/**
  * @brief 更新显示
  */
void UpdateDisplay(void)
{
    OLED_Clear();
    
    // 显示模式信息
    switch(current_mode) {
        case MODE_BLUETOOTH:
            OLED_Printf(0, 0, OLED_8X16, "Mode: Bluetooth");
            OLED_Printf(0, 16, OLED_8X16, "Cmd: %c", current_cmd);
            OLED_Printf(0, 32, OLED_8X16, "State: %s", display_info);
            break;
        case MODE_LINE_FOLLOWING:
            OLED_Printf(0, 0, OLED_8X16, "Mode: Line Follow");
            OLED_Printf(0, 16, OLED_8X16, "Sensor: %d%d%d%d", 
                       SensorData[0], SensorData[1], SensorData[2], SensorData[3]);
            break;
        case MODE_STOP:
            OLED_Printf(0, 0, OLED_8X16, "Mode: Stop");
            break;
    }
    
    // 显示传感器数�?
    OLED_Printf(0, 48, OLED_6X8, "Yaw: %.1f", angle.yaw);
    
    OLED_Update();
}

/**
  * @brief 定时器中断回调函�?
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM2) {
        Key_Tick();
        
        static int Count_10ms = 0;
        Count_10ms++;
        
        if(Count_10ms >= 10) {
            Count_10ms = 0;
            
            // Schedule control work outside the interrupt to avoid PID re-entry.
            if(current_mode == MODE_LINE_FOLLOWING) {
                line_follow_update_pending = 1;
            } else if(current_mode == MODE_BLUETOOTH && yaw_hold_enabled && IsYawHoldCommand(current_cmd)) {
                yaw_hold_update_pending = 1;
            }
    }
}
}

/* USER CODE END 4 */

void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif
