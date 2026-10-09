#ifndef BSP_CAN_HPP
#define BSP_CAN_HPP
#include "can.h"
struct CAN_RxFrame { uint32_t id; uint8_t length; uint8_t data[8]; };
struct CAN_Stats { uint32_t received, dropped, errors; };
HAL_StatusTypeDef CAN_Init(void);
/* Standard data frames only; length 0..8. HAL_OK means mailbox accepted. */
HAL_StatusTypeDef CAN_Transmit(CAN_HandleTypeDef *, uint32_t id, const uint8_t *, uint16_t length);
/* Nonblocking; HAL_BUSY means empty. 16 queued frames per bus. */
HAL_StatusTypeDef CAN_Read(CAN_HandleTypeDef *, CAN_RxFrame *);
HAL_StatusTypeDef CAN_GetStats(CAN_HandleTypeDef *, CAN_Stats *);
#endif
