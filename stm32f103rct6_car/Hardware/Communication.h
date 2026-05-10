#ifndef _COMMUNICATION_H
#define _COMMUNICATION_H

#include <stdint.h>
#include <stdarg.h>

void Com_usart_Init(void);
void Serial_Printf(const char *format, ...);

// 声明全局变量
extern uint8_t receiveData[50];

#endif
