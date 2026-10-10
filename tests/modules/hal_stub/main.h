#pragma once
#include <cstdint>
enum HAL_StatusTypeDef { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT };
enum GPIO_PinState { GPIO_PIN_RESET, GPIO_PIN_SET };
inline uint32_t __get_IPSR() { return 0; }
inline uint32_t __get_PRIMASK() { return 0; }
inline void __disable_irq() {}
inline void __set_PRIMASK(uint32_t) {}
inline bool clockEnabled = true;
#define __HAL_RCC_SPI1_IS_CLK_ENABLED() clockEnabled
#define __HAL_RCC_GPIOA_IS_CLK_ENABLED() clockEnabled
#define __HAL_RCC_GPIOB_IS_CLK_ENABLED() clockEnabled
#define __HAL_RCC_GPIOG_IS_CLK_ENABLED() clockEnabled
#define __HAL_RCC_GPIOH_IS_CLK_ENABLED() clockEnabled
#define __HAL_RCC_I2C1_IS_CLK_ENABLED() clockEnabled
#define __HAL_RCC_I2C3_IS_CLK_ENABLED() clockEnabled
using GPIO_TypeDef = int;
inline int ga, gb, gg, gh;
#define GPIOH (&gh)
#define CS1_ACCEL_GPIO_Port (&ga)
#define CS1_GYRO_GPIO_Port (&gb)
#define RSTN_IST8310_GPIO_Port (&gg)
#define CS1_ACCEL_Pin 16
#define CS1_GYRO_Pin 1
#define RSTN_IST8310_Pin 64
#define LED_B_Pin 1024
#define LED_G_Pin 2048
#define LED_R_Pin 4096
void HAL_GPIO_WritePin(GPIO_TypeDef *, uint16_t, GPIO_PinState);
void HAL_GPIO_TogglePin(GPIO_TypeDef *, uint16_t);
void HAL_Delay(uint32_t);
uint32_t HAL_GetTick();
