#include "bsp_pwm.hpp"
#include "bsp_critical.hpp"
#include <cmath>

namespace {
bool initialized[2]{};
int timer_index(TIM_HandleTypeDef *htim) {
    if (htim == &htim1 && htim->Instance == TIM1) return 0;
    if (htim == &htim8 && htim->Instance == TIM8) return 1;
    return -1;
}
bool hardware_ready(TIM_HandleTypeDef *htim) {
    const int index = timer_index(htim);
    return index >= 0 &&
        (index == 0 ? __HAL_RCC_TIM1_IS_CLK_ENABLED() : __HAL_RCC_TIM8_IS_CLK_ENABLED()) &&
        htim->State == HAL_TIM_STATE_READY &&
        htim->Init.CounterMode == TIM_COUNTERMODE_UP;
}
bool ready(TIM_HandleTypeDef *htim) {
    const int index = timer_index(htim);
    return index >= 0 && initialized[index] && hardware_ready(htim);
}
bool valid_channel(TIM_HandleTypeDef *htim, uint32_t channel) {
    return channel == TIM_CHANNEL_1 || channel == TIM_CHANNEL_2 ||
           channel == TIM_CHANNEL_3 || (htim == &htim1 && channel == TIM_CHANNEL_4);
}
double tick_hz(TIM_HandleTypeDef *htim) {
    RCC_ClkInitTypeDef clock{};
    uint32_t latency;
    HAL_RCC_GetClockConfig(&clock, &latency);
    const uint32_t timer_clock = HAL_RCC_GetPCLK2Freq() *
        (clock.APB2CLKDivider == RCC_HCLK_DIV1 ? 1U : 2U);
    return static_cast<double>(timer_clock) / (htim->Instance->PSC + 1U);
}
}
HAL_StatusTypeDef PWM_Init(void) {
    BspCritical lock;
    if (!hardware_ready(&htim1) || !hardware_ready(&htim8)) return HAL_ERROR;
    initialized[0] = initialized[1] = true;
    return HAL_OK;
}
HAL_StatusTypeDef PWM_Start(TIM_HandleTypeDef *htim, uint32_t channel) {
    BspCritical lock;
    if (!ready(htim) || !valid_channel(htim, channel)) return HAL_ERROR;
    if (HAL_TIM_GetChannelState(htim, channel) != HAL_TIM_CHANNEL_STATE_READY) return HAL_BUSY;
    return HAL_TIM_PWM_Start(htim, channel);
}
HAL_StatusTypeDef PWM_Stop(TIM_HandleTypeDef *htim, uint32_t channel) {
    BspCritical lock;
    if (!ready(htim) || !valid_channel(htim, channel)) return HAL_ERROR;
    if (HAL_TIM_GetChannelState(htim, channel) == HAL_TIM_CHANNEL_STATE_READY) return HAL_OK;
    return HAL_TIM_PWM_Stop(htim, channel);
}
HAL_StatusTypeDef PWM_SetPeriod(TIM_HandleTypeDef *htim, float period_s) {
    if (!std::isfinite(period_s) || period_s <= 0.0f) return HAL_ERROR;
    BspCritical lock;
    if (!ready(htim)) return HAL_ERROR;
    if (htim->Instance->CR1 & TIM_CR1_CEN) return HAL_BUSY;
    const double desired = period_s * tick_hz(htim);
    if (!std::isfinite(desired) || desired < 1.5 || desired >= 65535.5) return HAL_ERROR;
    const uint32_t ticks = static_cast<uint32_t>(desired + 0.5);
    const uint32_t old_ticks = __HAL_TIM_GET_AUTORELOAD(htim) + 1U;
    const unsigned count = htim == &htim1 ? 4 : 3;
    for (unsigned i = 0; i < count; ++i) {
        const uint32_t channel = TIM_CHANNEL_1 + i * 4U;
        const double scaled = static_cast<double>(__HAL_TIM_GET_COMPARE(htim, channel)) *
                              ticks / old_ticks;
        const uint32_t value = scaled >= ticks ? ticks : static_cast<uint32_t>(scaled + 0.5);
        __HAL_TIM_SET_COMPARE(htim, channel, value);
    }
    __HAL_TIM_SET_AUTORELOAD(htim, ticks - 1U);
    htim->Init.Period = ticks - 1U;
    __HAL_TIM_SET_COUNTER(htim, 0);
    htim->Instance->EGR = TIM_EGR_UG; // Latch shadow registers while stopped.
    __HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_UPDATE);
    return HAL_OK;
}
HAL_StatusTypeDef PWM_SetDutyRatio(TIM_HandleTypeDef *htim, float ratio, uint32_t channel) {
    if (!std::isfinite(ratio) || ratio < 0.0f || ratio > 1.0f) return HAL_ERROR;
    BspCritical lock;
    if (!ready(htim) || !valid_channel(htim, channel)) return HAL_ERROR;
    const uint32_t ticks = __HAL_TIM_GET_AUTORELOAD(htim) + 1U;
    if (ticks > 65535U) return HAL_ERROR;
    __HAL_TIM_SET_COMPARE(htim, channel, static_cast<uint32_t>(ratio * ticks + 0.5));
    return HAL_OK;
}
HAL_StatusTypeDef PWM_SetPulseUs(TIM_HandleTypeDef *htim, float pulse_us, uint32_t channel) {
    if (!std::isfinite(pulse_us) || pulse_us < 0.0f) return HAL_ERROR;
    BspCritical lock;
    if (!ready(htim) || !valid_channel(htim, channel)) return HAL_ERROR;
    const uint32_t ticks = __HAL_TIM_GET_AUTORELOAD(htim) + 1U;
    const double desired = pulse_us * tick_hz(htim) / 1000000.0;
    if (!std::isfinite(desired) || desired > ticks || ticks > 65535U) return HAL_ERROR;
    __HAL_TIM_SET_COMPARE(htim, channel, static_cast<uint32_t>(desired + 0.5));
    return HAL_OK;
}
