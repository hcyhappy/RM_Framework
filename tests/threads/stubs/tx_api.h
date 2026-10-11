#pragma once
#include <cstdint>
using ULONG=uint32_t; using UINT=unsigned; using UCHAR=unsigned char; using CHAR=char;
#define TX_TIMER_TICKS_PER_SECOND 1000
constexpr UINT TX_SUCCESS=0, TX_NOT_DONE=0x20, TX_NO_INSTANCE=0x0d;
constexpr ULONG TX_NO_WAIT=0, TX_WAIT_FOREVER=0xffffffff, TX_NO_TIME_SLICE=0;
constexpr UINT TX_DONT_START=0, TX_INHERIT=1;
struct TX_THREAD { void (*entry)(ULONG){}; ULONG bytes{}; UINT priority{}; bool resumed{}; };
struct TX_SEMAPHORE { unsigned count{}; };
struct TX_MUTEX { bool created{}, locked{}; };
struct TX_BYTE_POOL { UCHAR *storage{}; ULONG size{}, used{}; };
TX_THREAD *tx_thread_identify();
UINT tx_thread_sleep(ULONG);
ULONG tx_time_get();
UINT tx_semaphore_create(TX_SEMAPHORE *,CHAR *,ULONG);
UINT tx_semaphore_put(TX_SEMAPHORE *);
UINT tx_semaphore_get(TX_SEMAPHORE *,ULONG);
UINT tx_mutex_create(TX_MUTEX *,CHAR *,UINT);
UINT tx_mutex_get(TX_MUTEX *,ULONG);
UINT tx_mutex_put(TX_MUTEX *);
UINT tx_byte_pool_create(TX_BYTE_POOL *,CHAR *,void *,ULONG);
UINT tx_byte_allocate(TX_BYTE_POOL *,void **,ULONG,ULONG);
UINT tx_thread_stack_error_notify(void (*)(TX_THREAD *));
UINT tx_thread_create(TX_THREAD *,CHAR *,void (*)(ULONG),ULONG,void *,ULONG,UINT,UINT,ULONG,UINT);
UINT tx_thread_resume(TX_THREAD *);
void tx_kernel_enter();

void Error_Handler();
