#pragma once
using ULONG = unsigned long;
#define TX_TIMER_TICKS_PER_SECOND 1000
inline void *tx_thread_identify() { return nullptr; }
void tx_thread_sleep(ULONG);
