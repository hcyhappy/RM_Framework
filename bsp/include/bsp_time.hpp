#pragma once
#include <cstdint>
uint32_t BSP_GetTickMs(); // HAL TIM6 millisecond clock; unsigned subtraction handles rollover.
