#include "bsp.hpp"
#include "bsp_can.hpp"
#include "bsp_usart.hpp"
#include "bsp_pwm.hpp"
HAL_StatusTypeDef bsp_Init(void) {
    if (__get_IPSR() || __get_PRIMASK()) return HAL_ERROR;
    HAL_StatusTypeDef status = PWM_Init();
    if (status != HAL_OK) return status;
    status = USART_Init();
    if (status != HAL_OK) return status;
    return CAN_Init();
}
