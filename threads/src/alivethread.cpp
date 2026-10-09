#include "threads.hpp"
#include "main.h"

void alive_thread_entry(ULONG)
{
    for (;;)
    {
#if RM_ENABLE_HEARTBEAT_DEMO
        const UINT status = tx_semaphore_get(&heartbeat_semaphore,
                                             TX_TIMER_TICKS_PER_SECOND);
        if (status == TX_SUCCESS)
        {
            ++app_alive_count;
            HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
            HAL_GPIO_TogglePin(LED_G_GPIO_Port, LED_G_Pin);
        }
        else
        {
            HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
        }
#else
        ++app_alive_count;
        HAL_GPIO_TogglePin(LED_G_GPIO_Port, LED_G_Pin);
        tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND / 5);
#endif
    }
}
