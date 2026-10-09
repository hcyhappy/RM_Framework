#include "app_threadx.h"
#include "threads.hpp"
#include "main.h"

namespace {
TX_BYTE_POOL thread_pool;
TX_THREAD alive_thread;
#if RM_ENABLE_HEARTBEAT_DEMO
TX_THREAD heartbeat_thread;
#endif
alignas(8) UCHAR pool_storage[8192];
constexpr ULONG alive_stack_bytes = 1024;
constexpr ULONG heartbeat_stack_bytes = 1024;

void check(UINT status)
{
    if (status != TX_SUCCESS)
    {
        app_init_status = status;
        HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
        Error_Handler();
    }
}

void stack_error(TX_THREAD *)
{
    app_stack_error = 1;
    HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
    Error_Handler();
}

#if RM_ENABLE_HEARTBEAT_DEMO
void heartbeat_thread_entry(ULONG)
{
    for (;;)
    {
        check(tx_semaphore_put(&heartbeat_semaphore));
        ++app_heartbeat_count;
        tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND / 5);
    }
}
#endif
} // namespace

TX_SEMAPHORE heartbeat_semaphore;
volatile ULONG app_heartbeat_count = 0;
volatile ULONG app_alive_count = 0;
volatile UINT app_init_status = TX_NOT_DONE;
volatile UINT app_stack_error = 0;

extern "C" void tx_application_define(void *first_unused_memory)
{
    // Static pool storage has an explicit bound; do not allocate from _end.
    (void)first_unused_memory;
    check(tx_thread_stack_error_notify(stack_error));
    check(tx_byte_pool_create(&thread_pool, const_cast<CHAR *>("thread stacks"),
                              pool_storage, sizeof(pool_storage)));
    check(tx_semaphore_create(&heartbeat_semaphore,
                              const_cast<CHAR *>("demo heartbeat"), 0));
    void *stack = nullptr;
    check(tx_byte_allocate(&thread_pool, &stack, alive_stack_bytes, TX_NO_WAIT));
    check(tx_thread_create(&alive_thread, const_cast<CHAR *>("alive"),
                           alive_thread_entry, 0, stack, alive_stack_bytes,
                           15, 15, TX_NO_TIME_SLICE, TX_AUTO_START));
#if RM_ENABLE_HEARTBEAT_DEMO
    check(tx_byte_allocate(&thread_pool, &stack, heartbeat_stack_bytes, TX_NO_WAIT));
    check(tx_thread_create(&heartbeat_thread, const_cast<CHAR *>("heartbeat demo"),
                           heartbeat_thread_entry, 0, stack, heartbeat_stack_bytes,
                           10, 10, TX_NO_TIME_SLICE, TX_AUTO_START));
#endif
    app_init_status = TX_SUCCESS;
}

extern "C" void app_threadx_start(void)
{
    tx_kernel_enter();
    Error_Handler(); // The scheduler must not return.
}
