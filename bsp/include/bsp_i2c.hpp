#pragma once
#include "i2c.h"
// Bounded, blocking, thread/main-context transactions; address is unshifted 7-bit.
HAL_StatusTypeDef I2C_ReadRegister(I2C_HandleTypeDef *, uint8_t address, uint8_t reg, uint8_t *,
                                   uint16_t length);
HAL_StatusTypeDef I2C_WriteRegister(I2C_HandleTypeDef *, uint8_t address, uint8_t reg,
                                    const uint8_t *, uint16_t length);
