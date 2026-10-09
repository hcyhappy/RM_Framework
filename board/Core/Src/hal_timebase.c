#include "main.h"

// HAL timebase is independent of ThreadX: available before tx_kernel_enter().
TIM_HandleTypeDef htim6;

HAL_StatusTypeDef HAL_InitTick(uint32_t priority)
{
    RCC_ClkInitTypeDef clocks;
    uint32_t latency;
    if (priority >= (1UL << __NVIC_PRIO_BITS))
        return HAL_ERROR;
    __HAL_RCC_TIM6_CLK_ENABLE();
    HAL_RCC_GetClockConfig(&clocks, &latency);
    const uint32_t timer_clock = HAL_RCC_GetPCLK1Freq() *
        (clocks.APB1CLKDivider == RCC_HCLK_DIV1 ? 1U : 2U);
    htim6.Instance = TIM6;
    htim6.Init.Prescaler = timer_clock / 1000000U - 1U;
    htim6.Init.Period = 999U;
    htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim6.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
        return HAL_ERROR;
    __HAL_TIM_CLEAR_FLAG(&htim6, TIM_FLAG_UPDATE);
    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, priority, 0);
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
    uwTickPrio = priority;
    return HAL_TIM_Base_Start_IT(&htim6);
}

void HAL_SuspendTick(void)
{
    __HAL_TIM_DISABLE_IT(&htim6, TIM_IT_UPDATE);
}

void HAL_ResumeTick(void)
{
    __HAL_TIM_ENABLE_IT(&htim6, TIM_IT_UPDATE);
}

void TIM6_DAC_IRQHandler(void)
{
    if (__HAL_TIM_GET_FLAG(&htim6, TIM_FLAG_UPDATE) != RESET &&
        __HAL_TIM_GET_IT_SOURCE(&htim6, TIM_IT_UPDATE) != RESET)
    {
        __HAL_TIM_CLEAR_IT(&htim6, TIM_IT_UPDATE);
        HAL_IncTick();
    }
}
