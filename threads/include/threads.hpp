#pragma once
#include "tx_api.h"

// Exposed for debugger inspection; incremented by the two demo threads.
extern TX_SEMAPHORE heartbeat_semaphore;
extern volatile ULONG app_heartbeat_count;
extern volatile ULONG app_alive_count;
extern volatile UINT app_init_status;
extern volatile UINT app_stack_error;
void alive_thread_entry(ULONG input);
