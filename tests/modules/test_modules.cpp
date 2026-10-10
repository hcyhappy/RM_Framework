#include "BMI088.hpp"
#include "DJIMotorHandler.hpp"
#include "IST8310.hpp"
#include "LED.hpp"
#include "M2006.hpp"
#include "tx_api.h"
#include <array>
#include <cassert>
#include <cmath>
#include <cstring>
#include <iostream>
#include <vector>
SPI_HandleTypeDef hspi1{SPI1};
I2C_HandleTypeDef hi2c1{I2C1, {0}}, hi2c3{I2C3, {0}};
CAN_HandleTypeDef hcan1{1}, hcan2{2};
uint32_t tick = 0;
uint8_t regmap[2][256]{}, mag[256]{};
int selected = -1;
bool failSPI = false, failI2C = false;
unsigned spiLength = 0;
uint16_t ledPin = 0;
GPIO_PinState ledState{};
uint32_t HAL_GetTick() { return tick; }
void HAL_Delay(uint32_t n) { tick += n; }
void tx_thread_sleep(ULONG n) { tick += n; }
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state) {
    if (port == &ga || port == &gb) {
        if (state == GPIO_PIN_RESET) {
            assert(selected == -1);
            selected = port == &ga ? 0 : 1;
        } else
            selected = -1;
    }
    if (port == &gh) {
        ledPin = pin;
        ledState = state;
    }
}
void HAL_GPIO_TogglePin(GPIO_TypeDef *, uint16_t pin) { ledPin = pin; }
HAL_SPI_StateTypeDef HAL_SPI_GetState(SPI_HandleTypeDef *) { return HAL_SPI_STATE_READY; }
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *, uint8_t *tx, uint8_t *rx, uint16_t n,
                                          uint32_t) {
    assert(selected >= 0);
    spiLength = n;
    if (failSPI)
        return HAL_TIMEOUT;
    if (tx[0] & 0x80) {
        unsigned offset = selected == 0 ? 2 : 1;
        rx[0] = 0xaa;
        if (offset == 2)
            rx[1] = 0xee;
        for (unsigned i = offset; i < n; ++i)
            rx[i] = regmap[selected][(tx[0] & 0x7f) + i - offset] |
                    (selected == 1 && (tx[0] & 0x7f) + i - offset == 0x10 ? 0x80 : 0);
    } else
        for (unsigned i = 1; i < n; ++i)
            regmap[selected][tx[0] + i - 1] = tx[i];
    return HAL_OK;
}
HAL_I2C_StateTypeDef HAL_I2C_GetState(I2C_HandleTypeDef *) { return HAL_I2C_STATE_READY; }
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *, uint16_t addr, uint16_t reg, uint16_t,
                                   uint8_t *p, uint16_t n, uint32_t) {
    assert(addr == 0x1c);
    if (failI2C)
        return HAL_TIMEOUT;
    std::memcpy(p, mag + reg, n);
    return HAL_OK;
}
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *, uint16_t addr, uint16_t reg, uint16_t,
                                    uint8_t *p, uint16_t n, uint32_t) {
    assert(addr == 0x1c);
    if (failI2C)
        return HAL_TIMEOUT;
    std::memcpy(mag + reg, p, n);
    return HAL_OK;
}
struct Packet {
    int bus;
    uint32_t id;
    std::array<uint8_t, 8> bytes;
};
std::vector<Packet> packets;
HAL_StatusTypeDef sendStatus = HAL_OK;
HAL_StatusTypeDef CAN_Transmit(CAN_HandleTypeDef *h, uint32_t id, const uint8_t *p, uint16_t n) {
    assert(n == 8);
    Packet q{h->bus, id, {}};
    std::memcpy(q.bytes.data(), p, 8);
    packets.push_back(q);
    return sendStatus;
}
std::vector<CAN_RxFrame> incoming[2];
HAL_StatusTypeDef CAN_Read(CAN_HandleTypeDef *h, CAN_RxFrame *out) {
    auto &q = incoming[h->bus - 1];
    if (q.empty())
        return HAL_BUSY;
    *out = q.front();
    q.erase(q.begin());
    return HAL_OK;
}
uint32_t drops[2]{};
HAL_StatusTypeDef CAN_GetStats(CAN_HandleTypeDef *h, CAN_Stats *s) {
    *s = {0, drops[h->bus - 1], 0};
    return HAL_OK;
}
unsigned unhandled = 0;
void dispatch(CAN_HandleTypeDef *, const CAN_RxFrame &) { ++unhandled; }
void close(float a, float b) { assert(std::fabs(a - b) < 0.0001f); }
void be(uint8_t *p, int v) {
    unsigned u = uint16_t(v);
    p[0] = u >> 8;
    p[1] = u;
}
int main() {
    regmap[0][0] = 0x1e;
    regmap[1][0] = 0x0f;
    BMI088::cBMI088 imu;
    uint8_t v = 0;
    assert(imu.ReadReg(BMI088::BMI088_CS_ACC, 0, &v, 1) == HAL_OK);
    assert(v == 0x1e && spiLength == 3 && selected == -1);
    assert(imu.ReadReg(BMI088::BMI088_CS_GYRO, 0, &v, 1) == HAL_OK);
    assert(v == 0x0f && spiLength == 2);
    failSPI = true;
    v = 77;
    assert(imu.ReadReg(BMI088::BMI088_CS_ACC, 0, &v, 1) == HAL_TIMEOUT);
    assert(v == 77 && selected == -1);
    failSPI = false;
    assert(imu.ReadReg(BMI088::BMI088_CS_ACC, 0, nullptr, 1) == HAL_ERROR);
    imu.Config();
    assert(imu.Ready() && !imu.self_test.INIT_ERR);
    assert(regmap[1][0x10] == 2);
    regmap[0][0x12] = 0;
    regmap[0][0x13] = 0x80;
    regmap[0][0x14] = 0;
    regmap[0][0x15] = 0x40;
    acc_data_t a{};
    imu.ReadAccData(&a);
    close(a.x, -6 * 9.80665f);
    close(a.y, 3 * 9.80665f);
    regmap[1][2] = 0;
    regmap[1][3] = 0x80;
    gyro_data_t g{};
    imu.ReadGyroData(&g);
    close(g.x, -2000 * 0.0174532925199433f);
    regmap[0][0x22] = 0xff;
    regmap[0][0x23] = 0xe0;
    float temp = 0;
    imu.ReadAccTemperature(&temp);
    close(temp, 22.875f);
    regmap[0][0x22] = 0x80;
    regmap[0][0x23] = 0;
    imu.ReadAccTemperature(&temp);
    close(temp, 22.875f);
    assert(imu.LastStatus() == HAL_ERROR);
    failSPI = true;
    float previous = a.x;
    imu.ReadAccData(&a);
    assert(imu.self_test.ACC_DATA_ERR && a.x == previous);
    failSPI = false;
    regmap[1][0] = 0;
    imu.Config();
    assert(!imu.Ready() && imu.self_test.GYRO_CHIP_ID_ERR);
    regmap[1][0] = 0x0f;
    IST8310 compass;
    mag[0] = 0x10;
    assert(compass.Init() == HAL_OK);
    MagneticField f{7, 8, 9};
    assert(compass.Read(&f) == HAL_BUSY);
    tick += 5;
    mag[2] = 1;
    mag[3] = 100;
    mag[4] = 0;
    mag[5] = 0x9c;
    mag[6] = 0xff;
    mag[7] = 0;
    mag[8] = 0x80;
    mag[9] = 8;
    assert(compass.Read(&f) == HAL_OK);
    close(f.x, 30);
    close(f.y, -30);
    close(f.z, -9830.4f); // STAT2 INT is valid.
    tick += 5;
    failI2C = true;
    assert(compass.Read(&f) == HAL_TIMEOUT);
    close(f.x, 30);
    failI2C = false;
    tick += 5;
    mag[2] = 3;
    assert(compass.Read(&f) == HAL_ERROR);
    close(f.x, 30);
    mag[0] = 0;
    assert(compass.Init() == HAL_ERROR);
    DJIMotorHandler h;
    M2006 m, other, m2;
    assert(h.registerMotor(&m, &hcan1, 0x201));
    assert(!h.registerMotor(&other, &hcan1, 0x201));
    assert(!h.registerMotor(&m, &hcan2, 0x201));
    assert(h.registerMotor(&m2, &hcan2, 0x208));
    assert(!h.registerMotor(&other, &hcan1, 0x209));
    uint8_t data[8]{};
    be(data, 8190);
    be(data + 2, -360);
    be(data + 4, -1234);
    data[6] = 99;
    h.updateFeedback(&hcan1, data, 0);
    assert(m.motorFeedback.ecd == 8190);
    close(m.motorFeedback.speedFdb, -6.283185307f / 6);
    close(m.motorFeedback.currentFdb, -1234);
    assert(std::isnan(m.motorFeedback.temperatureFdb));
    assert(m.ZeroPosition());
    tick++;
    be(data, 2);
    be(data + 2, 0);
    h.updateFeedback(&hcan1, data, 0);
    assert(m.encoderCounts == 4);
    be(data, 8190);
    h.updateFeedback(&hcan1, data, 0);
    assert(m.encoderCounts == 0);
    m.speedPid = PID(1000, 0, 0, 10000, 0);
    m.speedSet = 2;
    m.controlMode = DJIMotor::SPD_MODE;
    m.setOutput();
    assert(m.currentSet == 2000);
    m.positionPid = PID(10, 0, 0, 2, 0);
    m.positionSet = 1;
    m.controlMode = DJIMotor::POS_MODE;
    m.setOutput();
    assert(m.currentSet == 2000);
    m.currentSet = -20000;
    assert(h.sendControlData() == HAL_OK);
    assert(packets.size() == 2);
    assert(packets[0].id == 0x200);
    assert(packets[0].bytes[0] == 0xd8 && packets[0].bytes[1] == 0xf0);
    assert(packets[1].bus == 2 && packets[1].id == 0x1ff);
    for (unsigned i = 2; i < 8; i++)
        assert(packets[0].bytes[i] == 0);
    tick += 101;
    m.setOutput();
    assert(m.currentSet == 0 && m.controlMode == DJIMotor::RELAX_MODE && !m.positionValid);
    h.updateFeedback(&hcan1, data, 0);
    assert(m.controlMode == DJIMotor::RELAX_MODE && !m.positionValid);
    h.unhandledFrame = dispatch;
    incoming[0].push_back({0x100, 8, {}});
    incoming[0].push_back({0x201, 7, {}});
    CAN_RxFrame rx{0x208, 8, {}};
    be(rx.data, 123);
    incoming[1].push_back(rx);
    h.PollFeedback();
    assert(unhandled == 2 && m2.motorFeedback.ecd == 123);
    M2006 third;
    assert(h.registerMotor(&third, &hcan1, 0x205));
    h.updateFeedback(&hcan1, data, 4);
    third.controlMode = DJIMotor::SPD_MODE;
    third.currentSet = 30000;
    packets.clear();
    assert(h.sendControlData() == HAL_OK);
    assert(packets.size() == 3);
    assert(packets[1].id == 0x1ff);
    assert(packets[1].bytes[0] == 0x27 && packets[1].bytes[1] == 0x10);
    drops[0]++;
    h.PollFeedback();
    assert(third.currentSet == 0 && third.controlMode == DJIMotor::RELAX_MODE &&
           !third.positionValid);
    sendStatus = HAL_BUSY;
    assert(h.sendControlData() == HAL_BUSY);
    LED::Set(LED::Color::Red, true);
    assert(ledPin == 4096 && ledState == GPIO_PIN_RESET);
    LED::Set(LED::Color::Blue, false);
    assert(ledPin == 1024 && ledState == GPIO_PIN_SET);
    LED::Toggle(LED::Color::Green);
    assert(ledPin == 2048);
    clockEnabled = false;
    LED::Set(LED::Color::Red, true);
    assert(ledPin == 2048);
    clockEnabled = true;
    std::cout << "Module protocol/control tests passed\n";
}
