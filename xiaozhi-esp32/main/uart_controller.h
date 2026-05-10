#ifndef UART_CONTROLLER_H
#define UART_CONTROLLER_H

#include <driver/uart.h>
#include <string>

class UartController {
public:
    static UartController& GetInstance();

    bool Initialize();
    bool SendCommand(char command);
    bool SendCommand(const std::string& command);
    bool SendCarCommand(char command);

private:
    UartController() = default;

    bool initialized_ = false;
    uart_port_t uart_num_ = UART_NUM_1;
};

// Compatibility helpers for older call sites.
void UART_Send(char command);
void UART_Send(const char* data);

#endif
