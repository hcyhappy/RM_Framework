#pragma once
#include "main.h"
#define I2C1 ((void *)2)
#define I2C3 ((void *)3)
#define I2C_ADDRESSINGMODE_7BIT 0
#define I2C_MEMADD_SIZE_8BIT 1
struct I2C_HandleTypeDef {
    void *Instance;
    struct {
        int AddressingMode;
    } Init;
};
extern I2C_HandleTypeDef hi2c1, hi2c3;
enum HAL_I2C_StateTypeDef { HAL_I2C_STATE_RESET, HAL_I2C_STATE_READY };
HAL_I2C_StateTypeDef HAL_I2C_GetState(I2C_HandleTypeDef *);
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *, uint16_t, uint16_t, uint16_t, uint8_t *,
                                   uint16_t, uint32_t);
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *, uint16_t, uint16_t, uint16_t, uint8_t *,
                                    uint16_t, uint32_t);
