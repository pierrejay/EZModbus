#pragma once
#include "pico.h"
#include <stddef.h>


// ===================================================================================
// PICO-SPECIFIC CONFIG
// ===================================================================================

/* SMP/monocore selection */
#if PICO_NO_HARDWARE
  #define configNUMBER_OF_CORES           1
#else
  #if defined(PICO_RP2350) || defined(PICO_RP2040)
    #define configNUMBER_OF_CORES         2
    #define configUSE_CORE_AFFINITY       1
    #define configRUN_MULTIPLE_PRIORITIES 1
    #define configUSE_PASSIVE_IDLE_HOOK   0
  #else
    #define configNUMBER_OF_CORES         1
  #endif
#endif
#define configTICK_CORE 0

/* CPU / FPU / TrustZone */
#if defined(PICO_RP2350)
  #define configCPU_CLOCK_HZ              150000000
  #define configENABLE_TRUSTZONE          0
  #define configENABLE_MPU                0
  #define configENABLE_FPU                1
  #define configRUN_FREERTOS_SECURE_ONLY 1
  #define configMAX_SYSCALL_INTERRUPT_PRIORITY 16
#elif defined(PICO_RP2040)
  #define configCPU_CLOCK_HZ              133000000
#endif

/* ISR vector remap */
#define vPortSVCHandler     isr_svcall
#define xPortPendSVHandler  isr_pendsv
#define xPortSysTickHandler isr_systick

#define configSUPPORT_PICO_SYNC_INTEROP         1
#define configSUPPORT_PICO_TIME_INTEROP         1


// ===================================================================================
// COMMON FREERTOS CONFIG
// ===================================================================================

/* Scheduler Related */
#define configUSE_PREEMPTION                    1
#define configUSE_TIME_SLICING                  1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION 0
#define configUSE_TICKLESS_IDLE                 0
#define configMAX_PRIORITIES                    32
#define configIDLE_SHOULD_YIELD                 1
#define configUSE_TASK_NOTIFICATIONS            1
#define configTASK_NOTIFICATION_ARRAY_ENTRIES   3

/* System */
#define configSTACK_DEPTH_TYPE                  uint32_t
#define configMESSAGE_BUFFER_LENGTH_TYPE        size_t

/* Memory allocation related definitions. */
#define configSUPPORT_STATIC_ALLOCATION         1  // Set to 1
#define configSUPPORT_DYNAMIC_ALLOCATION        1
#define configTOTAL_HEAP_SIZE                   (128*1024)
#define configAPPLICATION_ALLOCATED_HEAP        0

/* Hook function related definitions. */
#define configUSE_IDLE_HOOK                     0
#define configUSE_TICK_HOOK                     0
#define configCHECK_FOR_STACK_OVERFLOW          2 // Set to 2
#define configUSE_MALLOC_FAILED_HOOK            1 // Set to 1
#define configUSE_DAEMON_TASK_STARTUP_HOOK      0

/* Run time and task stats gathering related definitions. */
#define configGENERATE_RUN_TIME_STATS           0
#define configUSE_TRACE_FACILITY                1
#define configUSE_STATS_FORMATTING_FUNCTIONS    0

/* Possible values are portCLEAN_UP_TCB, portPRE_DELETION_HOOK_TCB. */
#define configTASK_DELETION_SUPPORT             0

/* Co-routine related definitions. */
#define configUSE_CO_ROUTINES                   0
#define configMAX_CO_ROUTINE_PRIORITIES         1

/* Software timer related definitions. */
#define configUSE_TIMERS                        1
#define configTIMER_TASK_PRIORITY               ( configMAX_PRIORITIES - 1 )
#define configTIMER_QUEUE_LENGTH                10
#define configTIMER_TASK_STACK_DEPTH            1024

/* Interrupt nesting behaviour configuration. */
/*
#define configKERNEL_INTERRUPT_PRIORITY         [dependent of processor]
#define configMAX_SYSCALL_INTERRUPT_PRIORITY    [dependent on processor and application]
#define configMAX_API_CALL_INTERRUPT_PRIORITY   [dependent on processor and application]
*/

#include <assert.h>
/* Define to trap errors during development. */
#define configASSERT(x)                         assert(x)

/* Define the sizes and usage. */
#define configTICK_RATE_HZ                      1000
#define configMAX_TASK_NAME_LEN                 16
#define configMINIMAL_STACK_SIZE                128
#define configUSE_16_BIT_TICKS                  0

/* Mutexes */
#define configUSE_MUTEXES                       1
#define configUSE_RECURSIVE_MUTEXES             1
#define configUSE_APPLICATION_TASK_TAG          0
#define configUSE_COUNTING_SEMAPHORES           1
#define configQUEUE_REGISTRY_SIZE               8
#define configUSE_QUEUE_SETS                    1
#define configUSE_NEWLIB_REENTRANT              0
#define configENABLE_BACKWARD_COMPATIBILITY     0
#define configNUM_THREAD_LOCAL_STORAGE_POINTERS 5

/* Optional functions - most linkers will remove unused functions anyway. */
#define INCLUDE_vTaskPrioritySet                1
#define INCLUDE_uxTaskPriorityGet               1
#define INCLUDE_vTaskDelete                     1
#define INCLUDE_vTaskSuspend                    1
#define INCLUDE_xResumeFromISR                  1
#define INCLUDE_vTaskDelayUntil                 1
#define INCLUDE_vTaskDelay                      1
#define INCLUDE_xTaskGetSchedulerState          1
#define INCLUDE_xTaskGetCurrentTaskHandle       1
#define INCLUDE_uxTaskGetStackHighWaterMark     1
#define INCLUDE_xTaskGetIdleTaskHandle          1
#define INCLUDE_eTaskGetState                   1
#define INCLUDE_xEventGroupSetBitFromISR        1
#define INCLUDE_xTimerPendFunctionCall          1
#define INCLUDE_xTaskAbortDelay                 1
#define INCLUDE_xTaskGetHandle                  1
#define INCLUDE_xTaskResumeFromISR              1