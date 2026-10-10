#include "bsp_i2c.hpp"
#include <cstring>
namespace {
bool valid(I2C_HandleTypeDef *h, uint8_t address, const uint8_t *data, uint16_t n) {
    return ((h == &hi2c1 && h->Instance == I2C1 && __HAL_RCC_I2C1_IS_CLK_ENABLED()) ||
            (h == &hi2c3 && h->Instance == I2C3 && __HAL_RCC_I2C3_IS_CLK_ENABLED())) &&
           h->Init.AddressingMode == I2C_ADDRESSINGMODE_7BIT &&
           HAL_I2C_GetState(h) == HAL_I2C_STATE_READY && address > 0 && address < 0x78 && data &&
           n > 0 && n <= 32 && !__get_IPSR() && !__get_PRIMASK();
}
} // namespace
HAL_StatusTypeDef I2C_ReadRegister(I2C_HandleTypeDef *h, uint8_t a, uint8_t reg, uint8_t *p,
                                   uint16_t n) {
    if (!valid(h, a, p, n))
        return HAL_ERROR;
    return HAL_I2C_Mem_Read(h, uint16_t(a) << 1, reg, I2C_MEMADD_SIZE_8BIT, p, n, 10);
}
HAL_StatusTypeDef I2C_WriteRegister(I2C_HandleTypeDef *h, uint8_t a, uint8_t reg, const uint8_t *p,
                                    uint16_t n) {
    if (!valid(h, a, p, n))
        return HAL_ERROR;
    uint8_t copy[32];
    std::memcpy(copy, p, n);
    return HAL_I2C_Mem_Write(h, uint16_t(a) << 1, reg, I2C_MEMADD_SIZE_8BIT, copy, n, 10);
}
