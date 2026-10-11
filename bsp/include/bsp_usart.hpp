#ifndef BSP_USART_HPP
#define BSP_USART_HPP
#include "usart.h"
enum USART_Mode : uint8_t { USART_MODE_BLOCK = 0, USART_MODE_DMA = 1, USART_MODE_IT = 2 };
struct USART_Stats {
    uint32_t received_bytes, dropped_bytes, errors, last_error;
    HAL_StatusTypeDef receive_status;
};
/* After MX_DMA_Init + MX_USART*_UART_Init. Starts 256-byte idle DMA RX. */
HAL_StatusTypeDef USART_Init(void);
HAL_StatusTypeDef USART1_Init(void);
HAL_StatusTypeDef USART6_Init(void);
/* DMA/IT copy <=256 bytes into BSP storage; caller can reuse its buffer.
 * BLOCK uses a finite timeout, never in ISR. HAL_BUSY requires caller policy. */
HAL_StatusTypeDef USART_Transmit(UART_HandleTypeDef *, const uint8_t *, uint16_t,
                               USART_Mode, uint32_t timeout_ms = 100);
/* Stop RX first. Custom persistent DMA buffer <=256 B, ordinary SRAM only.
 * Auto-rearmed; caller must keep storage alive until StopReceive succeeds. */
HAL_StatusTypeDef USART_Receive(UART_HandleTypeDef *, uint8_t *, uint16_t);
HAL_StatusTypeDef USART_StopReceive(UART_HandleTypeDef *);
/* Call periodically from the RX owner thread to retry after HAL errors. */
HAL_StatusTypeDef USART_Service(UART_HandleTypeDef *);
/* Nonblocking stream read: at most 256 bytes/call; ring capacity 511 bytes. */
uint16_t USART_Read(UART_HandleTypeDef *, uint8_t *, uint16_t capacity);
HAL_StatusTypeDef USART_GetStats(UART_HandleTypeDef *, USART_Stats *);
#endif
