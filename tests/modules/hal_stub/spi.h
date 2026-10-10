#pragma once
#include "main.h"
#define SPI1 ((void *)1)
struct SPI_HandleTypeDef {
    void *Instance;
};
extern SPI_HandleTypeDef hspi1;
enum HAL_SPI_StateTypeDef { HAL_SPI_STATE_RESET, HAL_SPI_STATE_READY };
HAL_SPI_StateTypeDef HAL_SPI_GetState(SPI_HandleTypeDef *);
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *, uint8_t *, uint8_t *, uint16_t,
                                          uint32_t);
