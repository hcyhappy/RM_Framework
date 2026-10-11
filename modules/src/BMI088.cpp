#include "BMI088.hpp"
#include "bsp_critical.hpp"
#include "tx_api.h"
#include <cmath>
#include <cstring>
namespace BMI088 {
namespace {
bool busy = false;
bool context() { return __get_IPSR() == 0 && __get_PRIMASK() == 0; }
void waitMs(ULONG ms) {
    if (tx_thread_identify())
        tx_thread_sleep((ms * TX_TIMER_TICKS_PER_SECOND + 999) / 1000);
    else
        HAL_Delay(ms);
}
int16_t signedLE(const uint8_t *p) {
    int32_t v = p[0] | (uint32_t(p[1]) << 8);
    return static_cast<int16_t>(v >= 32768 ? v - 65536 : v);
}
HAL_StatusTypeDef transfer(BMI088_SENSOR sensor, uint8_t addr, uint8_t *data, const uint8_t *out,
                           uint8_t n) {
    if (!context() || (sensor != BMI088_CS_ACC && sensor != BMI088_CS_GYRO) || !n || n > 30 ||
        addr > 0x7f || unsigned(addr) + n > 0x80 || (!data && !out) || hspi1.Instance != SPI1 ||
        HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY || !__HAL_RCC_SPI1_IS_CLK_ENABLED() ||
        !__HAL_RCC_GPIOA_IS_CLK_ENABLED() || !__HAL_RCC_GPIOB_IS_CLK_ENABLED())
        return HAL_ERROR;
    {
        BspCritical guard;
        if (busy)
            return HAL_BUSY;
        busy = true;
    }
    uint8_t tx[32]{}, rx[32]{};
    const unsigned offset = out ? 1 : (sensor == BMI088_CS_ACC ? 2 : 1);
    tx[0] = out ? (addr & 0x7f) : (addr | 0x80);
    if (out)
        std::memcpy(tx + 1, out, n);
    auto port = sensor == BMI088_CS_ACC ? CS1_ACCEL_GPIO_Port : CS1_GYRO_GPIO_Port;
    auto pin = sensor == BMI088_CS_ACC ? CS1_ACCEL_Pin : CS1_GYRO_Pin;
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
    const auto status = HAL_SPI_TransmitReceive(&hspi1, tx, rx, n + offset, 10);
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
    if (status == HAL_OK && data)
        std::memcpy(data, rx + offset, n);
    {
        BspCritical guard;
        busy = false;
    }
    return status;
}
} // namespace
HAL_StatusTypeDef cBMI088::ReadReg(BMI088_SENSOR cs, uint8_t addr, uint8_t *data, uint8_t len) {
    return status_ = transfer(cs, addr, data, nullptr, len);
}
HAL_StatusTypeDef cBMI088::WriteReg(BMI088_SENSOR cs, uint8_t addr, const uint8_t *data,
                                    uint8_t len) {
    return status_ = transfer(cs, addr, nullptr, data, len);
}
void cBMI088::VerifyAccChipID() {
    uint8_t id = 0;
    self_test.ACC_CHIP_ID_ERR = ReadReg(BMI088_CS_ACC, 0, &id, 1) != HAL_OK || id != 0x1e;
    if (self_test.ACC_CHIP_ID_ERR) {
        ready_ = false;
        self_test.INIT_ERR = true;
        if (status_ == HAL_OK)
            status_ = HAL_ERROR;
    }
}
void cBMI088::VerifyGyroChipID() {
    uint8_t id = 0;
    self_test.GYRO_CHIP_ID_ERR = ReadReg(BMI088_CS_GYRO, 0, &id, 1) != HAL_OK || id != 0x0f;
    if (self_test.GYRO_CHIP_ID_ERR) {
        ready_ = false;
        self_test.INIT_ERR = true;
        if (status_ == HAL_OK)
            status_ = HAL_ERROR;
    }
}
void cBMI088::Config() {
    ready_ = false;
    self_test.INIT_ERR = true;
    self_test.ACC_DATA_ERR = self_test.GYRO_DATA_ERR = true;
    if (!context()) {
        status_ = HAL_ERROR;
        return;
    }
    waitMs(50);
    // The first accelerometer read switches its interface from I2C to SPI.
    uint8_t ignored;
    if (ReadReg(BMI088_CS_ACC, 0, &ignored, 1) != HAL_OK)
        return;
    uint8_t reset = 0xb6;
    if (WriteReg(BMI088_CS_ACC, 0x7e, &reset, 1) != HAL_OK)
        return;
    waitMs(50);
    if (ReadReg(BMI088_CS_ACC, 0, &ignored, 1) != HAL_OK)
        return;
    VerifyAccChipID();
    if (self_test.ACC_CHIP_ID_ERR) {
        status_ = HAL_ERROR;
        return;
    }
    if (WriteReg(BMI088_CS_GYRO, 0x14, &reset, 1) != HAL_OK)
        return;
    waitMs(50);
    VerifyGyroChipID();
    if (self_test.GYRO_CHIP_ID_ERR) {
        status_ = HAL_ERROR;
        return;
    }
    struct Setting {
        BMI088_SENSOR sensor;
        uint8_t reg, value;
    };
    const Setting settings[] = {
        {BMI088_CS_ACC, 0x7c, 0x00},  {BMI088_CS_ACC, 0x7d, 0x04},  {BMI088_CS_ACC, 0x41, 0x01},
        {BMI088_CS_ACC, 0x40, 0xab},  {BMI088_CS_ACC, 0x53, 0x08},  {BMI088_CS_ACC, 0x58, 0x04},
        {BMI088_CS_GYRO, 0x0f, 0x00}, {BMI088_CS_GYRO, 0x10, 0x02}, {BMI088_CS_GYRO, 0x11, 0x00},
        {BMI088_CS_GYRO, 0x15, 0x80}, {BMI088_CS_GYRO, 0x16, 0x00}, {BMI088_CS_GYRO, 0x18, 0x01}};
    for (const auto &s : settings) {
        if (WriteReg(s.sensor, s.reg, &s.value, 1) != HAL_OK)
            return;
        waitMs(5);
        uint8_t actual;
        if (ReadReg(s.sensor, s.reg, &actual, 1) != HAL_OK)
            return;
        const uint8_t mask = s.sensor == BMI088_CS_GYRO && s.reg == 0x10 ? 0x7f : 0xff;
        if ((actual & mask) != (s.value & mask)) {
            status_ = HAL_ERROR;
            return;
        }
    }
    waitMs(50);
    ready_ = true;
    self_test.INIT_ERR = false;
    status_ = HAL_OK;
}
void cBMI088::ReadAccData(acc_data_t *data) {
    self_test.ACC_DATA_ERR = true;
    if (!data || !ready_) {
        status_ = HAL_ERROR;
        return;
    }
    uint8_t raw[6];
    if (ReadReg(BMI088_CS_ACC, 0x12, raw, 6) != HAL_OK)
        return;
    constexpr float scale = 6.0f * 9.80665f / 32768.0f;
    data->x = signedLE(raw) * scale;
    data->y = signedLE(raw + 2) * scale;
    data->z = signedLE(raw + 4) * scale;
    acc_data.x = data->x;
    acc_data.y = data->y;
    acc_data.z = data->z;
    self_test.ACC_DATA_ERR = false;
}
void cBMI088::ReadGyroData(gyro_data_t *data) {
    self_test.GYRO_DATA_ERR = true;
    if (!data || !ready_) {
        status_ = HAL_ERROR;
        return;
    }
    uint8_t raw[6];
    if (ReadReg(BMI088_CS_GYRO, 2, raw, 6) != HAL_OK)
        return;
    constexpr float scale = 2000.0f * 0.0174532925199433f / 32768.0f;
    data->x = signedLE(raw) * scale - Gyro_offset[0];
    data->y = signedLE(raw + 2) * scale - Gyro_offset[1];
    data->z = signedLE(raw + 4) * scale - Gyro_offset[2];
    gyro_data = *data;
    self_test.GYRO_DATA_ERR = false;
}
void cBMI088::ReadAccTemperature(float *temp) {
    if (!temp || !ready_) {
        status_ = HAL_ERROR;
        return;
    }
    uint8_t raw[2];
    if (ReadReg(BMI088_CS_ACC, 0x22, raw, 2) != HAL_OK)
        return;
    int value = (int(raw[0]) << 3) | (raw[1] >> 5);
    if (value == 1024) {
        status_ = HAL_ERROR;
        return;
    } // Datasheet invalid-temperature sentinel.
    if (value >= 1024)
        value -= 2048;
    *temp = value * 0.125f + 23.0f;
    acc_data.temperature = *temp;
}
void cBMI088::VerifyAccData() { ReadAccData(&acc_data); }
void cBMI088::VerifyGyroData() { ReadGyroData(&gyro_data); }
void cBMI088::Calibrate() {
    self_test.CALIBRATE_ERR = true;
    if (!ready_ || !context() || !tx_thread_identify())
        return;
    // Stationary gyro-only calibration. No offsets copied from another physical board.
    float old[3];
    std::memcpy(old, Gyro_offset, sizeof old);
    std::memset(Gyro_offset, 0, sizeof Gyro_offset);
    double sum[3]{};
    bool good = true;
    for (unsigned i = 0; i < 1000; ++i) {
        gyro_data_t sample{};
        ReadGyroData(&sample);
        if (self_test.GYRO_DATA_ERR || std::fabs(sample.x) > 0.1f || std::fabs(sample.y) > 0.1f ||
            std::fabs(sample.z) > 0.1f) {
            good = false;
            break;
        }
        sum[0] += sample.x;
        sum[1] += sample.y;
        sum[2] += sample.z;
        waitMs(2);
    }
    if (good) {
        for (unsigned i = 0; i < 3; ++i)
            Gyro_offset[i] = sum[i] / 1000;
        self_test.CALIBRATE_ERR = false;
    } else
        std::memcpy(Gyro_offset, old, sizeof old);
}
void cBMI088::TemperatureControl(float) { self_test.TEMP_CTRL_ERR = true; } // TIM10 not configured.
} // namespace BMI088
