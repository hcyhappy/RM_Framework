#pragma once
#include "threads.hpp"
void AppCheck(UINT status);
bool MotorConfigValid();
class AppLock {
public:
    AppLock() { AppCheck(tx_mutex_get(&app_data_mutex, TX_WAIT_FOREVER)); }
    ~AppLock() { AppCheck(tx_mutex_put(&app_data_mutex)); }
    AppLock(const AppLock &) = delete;
    AppLock &operator=(const AppLock &) = delete;
};
