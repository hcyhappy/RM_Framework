#pragma once
#include "bsp_i2c.hpp"
struct MagneticField {
    float x{}, y{}, z{};
}; // microtesla, native sensor axes
class IST8310 {
  public:
    HAL_StatusTypeDef Init();
    // Nonblocking conversion polling; HAL_BUSY means not ready. Output preserved on error.
    HAL_StatusTypeDef Read(MagneticField *field);
    MagneticField field{};

  private:
    bool ready_ = false;
    bool pending_ = false;
    uint32_t triggerTick_ = 0;
    HAL_StatusTypeDef trigger();
};
