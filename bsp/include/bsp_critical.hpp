#ifndef BSP_CRITICAL_HPP
#define BSP_CRITICAL_HPP
#include "main.h"
/* Bounded register/queue work only. Never wrap blocking HAL calls. */
class BspCritical {
public:
    BspCritical() : saved_(__get_PRIMASK()) { __disable_irq(); }
    ~BspCritical() { __set_PRIMASK(saved_); }
    BspCritical(const BspCritical &) = delete;
    BspCritical &operator=(const BspCritical &) = delete;
private:
    uint32_t saved_;
};
#endif
