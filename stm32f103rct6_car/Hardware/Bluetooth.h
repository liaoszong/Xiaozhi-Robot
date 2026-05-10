#ifndef _BLUETOOTH_H
#define _BLUETOOTH_H

#include "main.h"
#include <stdint.h>
#include <stdarg.h>
#include <string.h>

// 蓝牙接收缓冲区大�?
#define BLUETOOTH_RX_BUFFER_SIZE 256
#define BLUETOOTH_CMD_BUFFER_SIZE 8

// 有效命令定义
typedef enum {
    CMD_FORWARD = 'A',
    CMD_BACKWARD = 'E',
    CMD_LEFT = 'G',
    CMD_RIGHT = 'C',
    CMD_STOP = 'Z',
    CMD_LEFT_UP = 'H',
    CMD_RIGHT_UP = 'B',
    CMD_LEFT_DOWN = 'F',
    CMD_RIGHT_DOWN = 'D'
} Bluetooth_Command_t;

// 蓝牙状态结构体
typedef struct {
    volatile uint8_t rx_flag;
    volatile uint16_t rx_size;
    uint8_t rx_buffer[BLUETOOTH_RX_BUFFER_SIZE];
    uint8_t process_buffer[BLUETOOTH_RX_BUFFER_SIZE];
    volatile uint16_t process_size;
    char cmd_buffer[BLUETOOTH_CMD_BUFFER_SIZE];
    uint8_t cmd_index;
    uint8_t read_index;
    char last_valid_cmd;
} Bluetooth_HandleTypeDef;

// 初始化函�?
void Bluetooth_Init(UART_HandleTypeDef *huart);
void Bluetooth_DeInit(UART_HandleTypeDef *huart);

// 数据收发函数
uint8_t Bluetooth_DataReady(void);
void Bluetooth_ClearFlag(void);
void Bluetooth_SendString(const char* data);
void Bluetooth_Printf(const char *format, ...);

// 命令处理函数
uint8_t Bluetooth_ProcessCommands(void);
char Bluetooth_GetNextCommand(void);
uint8_t Bluetooth_HasPendingCommands(void);
char Bluetooth_GetLastValidCommand(void);
void Bluetooth_SetLastValidCommand(char cmd);

// 回调函数
void Bluetooth_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);

// 外部变量声明
extern Bluetooth_HandleTypeDef hbluetooth;

#endif
