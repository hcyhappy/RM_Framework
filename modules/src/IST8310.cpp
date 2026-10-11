#include "IST8310.hpp"
#include "bsp_time.hpp"
#include "main.h"
#include "tx_api.h"
namespace {
void delay(unsigned ms) {
    if (tx_thread_identify())
        tx_thread_sleep((ms * TX_TIMER_TICKS_PER_SECOND + 999) / 1000);
    else
        HAL_Delay(ms);
}
int signedLE(const uint8_t *p) {
    int v = p[0] | (unsigned(p[1]) << 8);
    return v >= 32768 ? v - 65536 : v;
}
} // namespace
HAL_StatusTypeDef IST8310::trigger() {
    uint8_t mode = 1;
    auto s = I2C_WriteRegister(&hi2c1, 0x0e, 0x0a, &mode, 1);
    pending_ = s == HAL_OK;
    if (pending_)
        triggerTick_ = BSP_GetTickMs();
    return s;
}
HAL_StatusTypeDef IST8310::Init() {
    ready_ = pending_ = false;
    if (__get_IPSR() || __get_PRIMASK() || !__HAL_RCC_GPIOG_IS_CLK_ENABLED())
        return HAL_ERROR;
    HAL_GPIO_WritePin(RSTN_IST8310_GPIO_Port, RSTN_IST8310_Pin, GPIO_PIN_RESET);
    delay(2);
    HAL_GPIO_WritePin(RSTN_IST8310_GPIO_Port, RSTN_IST8310_Pin, GPIO_PIN_SET);
    delay(10);
    uint8_t id = 0;
    auto s = I2C_ReadRegister(&hi2c1, 0x0e, 0, &id, 1);
    if (s != HAL_OK)
        return s;
    if (id != 0x10)
        return HAL_ERROR;
    s = trigger();
    ready_ = s == HAL_OK;
    return s;
}
HAL_StatusTypeDef IST8310::Read(MagneticField *out) {
    if (!out || !ready_)
        return HAL_ERROR;
    if (!pending_) {
        auto s = trigger();
        return s == HAL_OK ? HAL_BUSY : s;
    }
    if (uint32_t(BSP_GetTickMs() - triggerTick_) < 5)
        return HAL_BUSY;
    uint8_t raw[8]{};
    auto s = I2C_ReadRegister(&hi2c1, 0x0e, 0x02, raw, 1);
    if (s != HAL_OK)
        return s;
    if (!(raw[0] & 1))
        return HAL_BUSY;
    // Data read clears DRDY; include STAT2 to complete the read sequence.
    s = I2C_ReadRegister(&hi2c1, 0x0e, 0x03, raw + 1, 7);
    if (s != HAL_OK) {
        pending_ = false;
        return s;
    }
    pending_ = false;
    // DOR means a sample was overwritten. STAT2 bit3 is INT, not overflow.
    if (raw[0] & 2) {
        trigger();
        return HAL_ERROR;
    }
    MagneticField next{signedLE(raw + 1) * 0.3f, signedLE(raw + 3) * 0.3f,
                       signedLE(raw + 5) * 0.3f};
    s = trigger();
    if (s != HAL_OK)
        return s;
    field = next;
    *out = next;
    return HAL_OK;
}
