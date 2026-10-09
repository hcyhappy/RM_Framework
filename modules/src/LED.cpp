#include "LED.hpp"
#include "main.h"

namespace {
uint16_t pin_for(LED::Color color) {
    switch (color) {
    case LED::Color::Blue: return LED_B_Pin;
    case LED::Color::Green: return LED_G_Pin;
    case LED::Color::Red: return LED_R_Pin;
    }
    return 0;
}
}
void LED::Set(Color color, bool on) {
    const uint16_t pin = pin_for(color);
    if (pin && __HAL_RCC_GPIOH_IS_CLK_ENABLED())
        HAL_GPIO_WritePin(GPIOH, pin, on ? GPIO_PIN_RESET : GPIO_PIN_SET);
}
void LED::Toggle(Color color) {
    const uint16_t pin = pin_for(color);
    if (pin && __HAL_RCC_GPIOH_IS_CLK_ENABLED()) HAL_GPIO_TogglePin(GPIOH, pin);
}
