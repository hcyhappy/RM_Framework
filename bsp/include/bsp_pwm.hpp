#ifndef BSP_PWM_HPP
#define BSP_PWM_HPP
#include "tim.h"
/* Validate MX_TIM1/8 initialization; no channel is automatically started. */
HAL_StatusTypeDef PWM_Init(void);
HAL_StatusTypeDef PWM_Start(TIM_HandleTypeDef *, uint32_t channel);
HAL_StatusTypeDef PWM_Stop(TIM_HandleTypeDef *, uint32_t channel);
/* Seconds. Timer must be stopped; changes all channel periods and preserves duty.
 * Supported period is 2..65535 counter ticks (16-bit CCR supports 100% duty). */
HAL_StatusTypeDef PWM_SetPeriod(TIM_HandleTypeDef *, float period_s);
HAL_StatusTypeDef PWM_SetDutyRatio(TIM_HandleTypeDef *, float ratio, uint32_t channel);
/* Servo convenience: microseconds, not percent. Reject pulses beyond period. */
HAL_StatusTypeDef PWM_SetPulseUs(TIM_HandleTypeDef *, float pulse_us, uint32_t channel);
#endif
