#include "threads.hpp"
#include "main.h"
#include "LED.hpp"

void alive_thread_entry(ULONG)
{
    for (;;)
    {
#if RM_ENABLE_HEARTBEAT_DEMO
        const UINT status = tx_semaphore_get(&heartbeat_semaphore,
                                             TX_TIMER_TICKS_PER_SECOND);
        if (status == TX_SUCCESS)
        {
            ++app_alive_count;
            LED::Set(LED::Color::Red, false);
            LED::Toggle(LED::Color::Green);
        }
        else
        {
            LED::Set(LED::Color::Green, false);
            LED::Set(LED::Color::Red, true);
        }
#else
        ++app_alive_count;
        LED::Toggle(LED::Color::Green);
        tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND / 5);
#endif
    }
}
