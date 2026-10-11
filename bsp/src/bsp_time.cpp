#include "bsp_time.hpp"
#include "main.h"
uint32_t BSP_GetTickMs() { return HAL_GetTick(); }
