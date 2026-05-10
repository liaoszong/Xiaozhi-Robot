#include "wit_motion.h"

// 私有变量
static UART_HandleTypeDef *wit_huart = NULL;
static volatile uint8_t data_updated = 0;
static WitAngleData current_angle = {0};

// 私有函数声明
static void SensorUartSend(uint8_t *p_data, uint32_t uiSize);
static void SensorDataUpdate(uint32_t uiReg, uint32_t uiRegNum);
static void Delayms(uint16_t ucMs);

// 初始化陀螺仪
void WitMotion_Init(UART_HandleTypeDef *huart) {
    wit_huart = huart;
    
    // 初始化WIT协议
    WitInit(WIT_PROTOCOL_NORMAL, 0x50);
    WitSerialWriteRegister(SensorUartSend);
    WitRegisterCallBack(SensorDataUpdate);
    WitDelayMsRegister(Delayms);
    
    // 默认配置
    WitMotion_ConfigOutputRate(RRATE_10HZ);
    WitMotion_ConfigContent(RSW_ANGLE);
}

// 处理接收到的数据
void WitMotion_ProcessData(uint8_t *data, uint16_t size) {
    for(uint16_t i = 0; i < size; i++) {
        WitSerialDataIn(data[i]);
    }
}

// 获取角度数据
WitAngleData WitMotion_GetAngle(void) {
    WitAngleData result;
    
    // 原子操作复制数据
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    result = current_angle;
    data_updated = 0;  // 清除更新标志
    __set_PRIMASK(primask);
    
    return result;
}

// 检查数据是否更新
uint8_t WitMotion_IsDataUpdated(void) {
    return data_updated;
}

// 配置输出速率
HAL_StatusTypeDef WitMotion_ConfigOutputRate(uint8_t rate) {
    if (rate < RRATE_02HZ || rate > RRATE_200HZ) return HAL_ERROR;
    return (WitSetOutputRate(rate) == WIT_HAL_OK) ? HAL_OK : HAL_ERROR;
}

// 配置输出内容
HAL_StatusTypeDef WitMotion_ConfigContent(uint32_t content) {
    if (content > RSW_MASK) return HAL_ERROR;
    return (WitSetContent(content) == WIT_HAL_OK) ? HAL_OK : HAL_ERROR;
}

// 加速度校准
HAL_StatusTypeDef WitMotion_CalibrateAccel(void) {
    return (WitStartAccCali() == WIT_HAL_OK) ? HAL_OK : HAL_ERROR;
}

//------------------ 私有函数 ------------------//
static void SensorUartSend(uint8_t *p_data, uint32_t uiSize) {
    // 通过UART1发送数据到陀螺仪
    HAL_UART_Transmit(wit_huart, p_data, uiSize, HAL_MAX_DELAY);
}

static void Delayms(uint16_t ucMs) {
    HAL_Delay(ucMs);
}

static void SensorDataUpdate(uint32_t uiReg, uint32_t uiRegNum) {
    for(uint32_t i = 0; i < uiRegNum; i++) {
        switch(uiReg) {
            case Roll:
            case Pitch:
            case Yaw:
                // 更新角度数据
                current_angle.roll = sReg[Roll] / 32768.0f * 180.0f;
                current_angle.pitch = sReg[Pitch] / 32768.0f * 180.0f;
                current_angle.yaw = sReg[Yaw] / 32768.0f * 180.0f;
                data_updated = 1;  // 设置数据更新标志
                break;
        }
        uiReg++;
    }
}
