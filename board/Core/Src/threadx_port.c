#include "main.h"
#include "tx_api.h"
#include "usart.h"

extern VOID *_tx_initialize_unused_memory;
extern VOID *_tx_thread_system_stack_ptr;
extern VOID _tx_timer_interrupt(VOID);
extern const uint32_t g_pfnVectors[];

VOID _tx_initialize_low_level(VOID)
{
    __disable_irq();
    SCB->VTOR = (uint32_t)g_pfnVectors;
    _tx_thread_system_stack_ptr = (VOID *)g_pfnVectors[0];
    _tx_initialize_unused_memory = TX_NULL; // Application uses a static byte pool.

    SystemCoreClockUpdate();
    if (SysTick_Config(SystemCoreClock / TX_TIMER_TICKS_PER_SECOND) != 0U)
        Error_Handler();
    HAL_NVIC_SetPriority(SVCall_IRQn, 15, 0);
    HAL_NVIC_SetPriority(PendSV_IRQn, 15, 0);
    HAL_NVIC_SetPriority(SysTick_IRQn, 4, 0);
    // ThreadX enables interrupts when its scheduler starts.
}

void SysTick_Handler(void)
{
    _tx_timer_interrupt();
}

void USART1_IRQHandler(void) { HAL_UART_IRQHandler(&huart1); }
void USART6_IRQHandler(void) { HAL_UART_IRQHandler(&huart6); }
