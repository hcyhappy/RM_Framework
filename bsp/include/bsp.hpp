#ifndef RM_BSP_HPP
#define RM_BSP_HPP
#include "main.h"
#ifdef __cplusplus
extern "C" {
#endif
/* Call once after all MX_* initializers, with IRQs enabled. */
HAL_StatusTypeDef bsp_Init(void);
#ifdef __cplusplus
}
#endif
#endif
