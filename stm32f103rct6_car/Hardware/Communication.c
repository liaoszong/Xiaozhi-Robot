#include "headfile.h"
#include "Bluetooth.h"

uint8_t receiveData[50];

void Com_usart_Init(void)
{    
    // 初始化USART1 (陀螺仪)
    HAL_UART_DMAStop(&huart1);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, receiveData, sizeof(receiveData));
    __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
    
    // 初始化蓝牙模块
    Bluetooth_Init(&huart3);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if(huart->Instance == USART1) {
        // 将接收到的数据交给陀螺仪模块处理
        WitMotion_ProcessData(receiveData, Size);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, receiveData, sizeof(receiveData));
    }
    if(huart->Instance == USART3) {
        // 蓝牙数据接收回调
        Bluetooth_RxEventCallback(huart, Size);
    }
}

void Serial_Printf(const char *format, ...)
{
    va_list args;
    va_start(args, format);
    char buffer[256];
    int len = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    if (len > 0) {
        HAL_UART_Transmit(&huart3, (uint8_t*)buffer, len, HAL_MAX_DELAY);
    }
}
