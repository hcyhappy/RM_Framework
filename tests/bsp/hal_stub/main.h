#pragma once
#include <cstdint>
using IRQn_Type = int;
enum HAL_StatusTypeDef { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT };
enum { HAL_CAN_STATE_RESET, HAL_CAN_STATE_READY, HAL_CAN_STATE_LISTENING };
enum { HAL_UART_STATE_RESET, HAL_UART_STATE_READY, HAL_UART_STATE_BUSY_TX, HAL_UART_STATE_BUSY_RX, HAL_UART_STATE_ERROR };
enum { HAL_DMA_STATE_RESET, HAL_DMA_STATE_READY, HAL_DMA_STATE_BUSY, HAL_DMA_STATE_ERROR };
enum { HAL_TIM_STATE_RESET, HAL_TIM_STATE_READY };
enum { HAL_TIM_CHANNEL_STATE_RESET, HAL_TIM_CHANNEL_STATE_READY, HAL_TIM_CHANNEL_STATE_BUSY };
enum { USART1_IRQn, USART6_IRQn, DMA2_Stream1_IRQn, DMA2_Stream2_IRQn, DMA2_Stream6_IRQn, DMA2_Stream7_IRQn, CAN1_RX0_IRQn, CAN2_RX0_IRQn };
constexpr uint32_t ENABLE=1, CAN_FILTERMODE_IDMASK=0, CAN_FILTERSCALE_32BIT=1, CAN_RX_FIFO0=0;
constexpr uint32_t CAN_IT_RX_FIFO0_MSG_PENDING=1, CAN_ID_STD=0, CAN_RTR_DATA=0, CAN_ESR_BOFF=4;
constexpr uint32_t UART_WORDLENGTH_8B=0, UART_PARITY_NONE=0, UART_MODE_TX_RX=3, HAL_MAX_DELAY=0xffffffff;
constexpr uint32_t DMA_CHANNEL_4=4, DMA_CHANNEL_5=5, DMA_PERIPH_TO_MEMORY=0, DMA_MEMORY_TO_PERIPH=1;
constexpr uint32_t DMA_NORMAL=0, DMA_PINC_DISABLE=0, DMA_MINC_ENABLE=1, DMA_PDATAALIGN_BYTE=0, DMA_MDATAALIGN_BYTE=0, DMA_IT_HT=4;
constexpr uint32_t HAL_UART_RXEVENT_HT=0, HAL_UART_RXEVENT_TC=1, HAL_UART_RXEVENT_IDLE=2;
constexpr uint32_t TIM_CHANNEL_1=0, TIM_CHANNEL_2=4, TIM_CHANNEL_3=8, TIM_CHANNEL_4=12;
constexpr uint32_t TIM_COUNTERMODE_UP=0, RCC_HCLK_DIV1=1, TIM_CR1_CEN=1, TIM_EGR_UG=1, TIM_FLAG_UPDATE=1;
struct CAN_TypeDef { uint32_t ESR; };
struct USART_TypeDef {};
struct DMA_Stream_TypeDef {};
struct TIM_TypeDef { uint32_t PSC, ARR, CR1, EGR, CNT, CCR[4]; };
extern CAN_TypeDef can_regs[2];
extern USART_TypeDef uart_regs[2];
extern DMA_Stream_TypeDef dma_regs[4];
extern TIM_TypeDef tim_regs[2];
#define CAN1 (&can_regs[0])
#define CAN2 (&can_regs[1])
#define USART1 (&uart_regs[0])
#define USART6 (&uart_regs[1])
#define DMA2_Stream2 (&dma_regs[0])
#define DMA2_Stream7 (&dma_regs[1])
#define DMA2_Stream1 (&dma_regs[2])
#define DMA2_Stream6 (&dma_regs[3])
#define TIM1 (&tim_regs[0])
#define TIM8 (&tim_regs[1])
struct CAN_HandleTypeDef { CAN_TypeDef *Instance; int State; };
struct CAN_FilterTypeDef {
    uint32_t FilterIdHigh, FilterIdLow, FilterMaskIdHigh, FilterMaskIdLow;
    uint32_t FilterFIFOAssignment, FilterBank, FilterMode, FilterScale, FilterActivation, SlaveStartFilterBank;
};
struct CAN_TxHeaderTypeDef { uint32_t StdId, ExtId, IDE, RTR, DLC, TransmitGlobalTime; };
struct CAN_RxHeaderTypeDef { uint32_t StdId, IDE, RTR, DLC; };
struct DMA_InitTypeDef { uint32_t Channel, Direction, Mode, PeriphInc, MemInc, PeriphDataAlignment, MemDataAlignment; };
struct DMA_HandleTypeDef { DMA_Stream_TypeDef *Instance; void *Parent; int State; DMA_InitTypeDef Init; };
struct UART_InitTypeDef { uint32_t WordLength, Parity, Mode; };
struct UART_HandleTypeDef {
    USART_TypeDef *Instance; int gState, RxState; UART_InitTypeDef Init;
    DMA_HandleTypeDef *hdmarx, *hdmatx;
    uint32_t ErrorCode, RxEventType;
    uint8_t *rx; const uint8_t *tx; uint16_t rx_size, tx_size;
};
struct TIM_InitTypeDef { uint32_t CounterMode, Period; };
struct TIM_HandleTypeDef { TIM_TypeDef *Instance; int State; TIM_InitTypeDef Init; int channels[4]; };
struct RCC_ClkInitTypeDef { uint32_t APB2CLKDivider; };
extern CAN_HandleTypeDef hcan1, hcan2;
extern UART_HandleTypeDef huart1, huart6;
extern TIM_HandleTypeDef htim1, htim8;
extern DMA_HandleTypeDef mock_dma[4];
extern bool clocks[7], enabled[8];
extern uint32_t primask, ipsr;
inline uint32_t __get_PRIMASK() { return primask; }
inline uint32_t __get_IPSR() { return ipsr; }
inline void __disable_irq() { primask=1; }
inline void __set_PRIMASK(uint32_t n) { primask=n; }
inline bool NVIC_GetEnableIRQ(int n) { return enabled[n]; }
#define __HAL_RCC_CAN1_IS_CLK_ENABLED() clocks[0]
#define __HAL_RCC_CAN2_IS_CLK_ENABLED() clocks[1]
#define __HAL_RCC_USART1_IS_CLK_ENABLED() clocks[2]
#define __HAL_RCC_USART6_IS_CLK_ENABLED() clocks[3]
#define __HAL_RCC_DMA2_IS_CLK_ENABLED() clocks[4]
#define __HAL_RCC_TIM1_IS_CLK_ENABLED() clocks[5]
#define __HAL_RCC_TIM8_IS_CLK_ENABLED() clocks[6]
#define __HAL_DMA_DISABLE_IT(dma, irq) ((void)(dma), (void)(irq))
#define __HAL_TIM_GET_AUTORELOAD(t) ((t)->Instance->ARR)
#define __HAL_TIM_SET_AUTORELOAD(t,v) ((t)->Instance->ARR=(v))
#define __HAL_TIM_GET_COMPARE(t,c) ((t)->Instance->CCR[(c)/4])
#define __HAL_TIM_SET_COMPARE(t,c,v) ((t)->Instance->CCR[(c)/4]=(v))
#define __HAL_TIM_SET_COUNTER(t,v) ((t)->Instance->CNT=(v))
#define __HAL_TIM_CLEAR_FLAG(t,f) ((void)(t), (void)(f))
extern "C" {
int HAL_CAN_GetState(CAN_HandleTypeDef *);
HAL_StatusTypeDef HAL_CAN_ConfigFilter(CAN_HandleTypeDef *, const CAN_FilterTypeDef *);
HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef *);
HAL_StatusTypeDef HAL_CAN_Stop(CAN_HandleTypeDef *);
HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef *, uint32_t);
uint32_t HAL_CAN_GetTxMailboxesFreeLevel(CAN_HandleTypeDef *);
HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef *, const CAN_TxHeaderTypeDef *, const uint8_t *, uint32_t *);
uint32_t HAL_CAN_GetRxFifoFillLevel(CAN_HandleTypeDef *, uint32_t);
HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef *, uint32_t, CAN_RxHeaderTypeDef *, uint8_t *);
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *, uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *, const uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *, const uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *, const uint8_t *, uint16_t, uint32_t);
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *);
HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *);
uint32_t HAL_UARTEx_GetRxEventType(UART_HandleTypeDef *);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *);
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *, uint16_t);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *);
void HAL_RCC_GetClockConfig(RCC_ClkInitTypeDef *, uint32_t *);
uint32_t HAL_RCC_GetPCLK2Freq();
int HAL_TIM_GetChannelState(TIM_HandleTypeDef *, uint32_t);
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *, uint32_t);
HAL_StatusTypeDef HAL_TIM_PWM_Stop(TIM_HandleTypeDef *, uint32_t);
}
