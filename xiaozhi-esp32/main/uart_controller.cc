#include "uart_controller.h"

#include <driver/gpio.h>
#include <esp_err.h>
#include <esp_log.h>

#define TAG "UART"

// DNESP32S3 P4 serial header: U0_TXD/U0_RXD are GPIO43/GPIO44 on ESP32-S3.
// Wire ESP TX to STM32 RX, ESP RX to STM32 TX, and keep both sides at 115200 8N1.
#define CAR_UART_NUM UART_NUM_1
#define CAR_UART_TX_PIN GPIO_NUM_43
#define CAR_UART_RX_PIN GPIO_NUM_44
#define CAR_UART_BAUD_RATE 115200
#define CAR_UART_BUF_SIZE 256

UartController& UartController::GetInstance() {
    static UartController instance;
    return instance;
}

bool UartController::Initialize() {
    if (initialized_) {
        return true;
    }

    uart_config_t uart_config = {
        .baud_rate = CAR_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    esp_err_t ret = uart_param_config(CAR_UART_NUM, &uart_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "uart_param_config failed: %s", esp_err_to_name(ret));
        return false;
    }

    ret = uart_set_pin(CAR_UART_NUM, CAR_UART_TX_PIN, CAR_UART_RX_PIN,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "uart_set_pin failed: %s", esp_err_to_name(ret));
        return false;
    }

    ret = uart_driver_install(CAR_UART_NUM, CAR_UART_BUF_SIZE, 0, 0, nullptr, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "uart_driver_install failed: %s", esp_err_to_name(ret));
        return false;
    }

    uart_num_ = CAR_UART_NUM;
    initialized_ = true;
    ESP_LOGI(TAG, "car uart ready: uart%d tx=%d rx=%d baud=%d",
             uart_num_, CAR_UART_TX_PIN, CAR_UART_RX_PIN, CAR_UART_BAUD_RATE);
    return true;
}

bool UartController::SendCommand(char command) {
    if (!Initialize()) {
        return false;
    }

    // STM32 Bluetooth_ProcessCommands() extracts isolated command characters.
    // Send exactly one byte here; no newline or wrapper is required.
    int written = uart_write_bytes(uart_num_, &command, 1);
    if (written != 1) {
        ESP_LOGE(TAG, "failed to send car command: %c", command);
        return false;
    }

    ESP_LOGI(TAG, "sent car command: %c", command);
    return true;
}

bool UartController::SendCarCommand(char command) {
    // The STM32 safe receiver accepts framed commands in any mode, so do not
    // rely on legacy raw bytes. Example: CAR:A\n means forward.
    std::string frame = "CAR:";
    frame.push_back(command);
    frame.push_back('\n');
    return SendCommand(frame);
}

bool UartController::SendCommand(const std::string& command) {
    if (!Initialize()) {
        return false;
    }

    int written = uart_write_bytes(uart_num_, command.data(), command.size());
    if (written != static_cast<int>(command.size())) {
        ESP_LOGE(TAG, "failed to send uart data: %s", command.c_str());
        return false;
    }

    ESP_LOGI(TAG, "sent uart data: %s", command.c_str());
    return true;
}

void UART_Send(char command) {
    UartController::GetInstance().SendCommand(command);
}

void UART_Send(const char* data) {
    if (data != nullptr) {
        UartController::GetInstance().SendCommand(std::string(data));
    }
}
