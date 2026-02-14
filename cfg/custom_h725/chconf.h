/*
    ACS4 Flight Computer - ChibiOS/RT Kernel Configuration
    Target: STM32H725VGT6 (ACS4 custom PCB)

    ChibiOS/RT (full RTOS) with:
    - 10 kHz system tick (100 μs resolution)
    - Mutexes, semaphores, events, mailboxes
    - Debug checks enabled for development
*/

#ifndef CHCONF_H
#define CHCONF_H

#define _CHIBIOS_RT_CONF_
#define _CHIBIOS_RT_CONF_VER_8_0_

/*===========================================================================*/
/* System settings                                                           */
/*===========================================================================*/

#define CH_CFG_SMP_MODE                     FALSE
#define CH_CFG_HARDENING_LEVEL              0

/*===========================================================================*/
/* System timers settings                                                    */
/*===========================================================================*/

#define CH_CFG_ST_RESOLUTION                32
#define CH_CFG_ST_FREQUENCY                 10000
#define CH_CFG_INTERVALS_SIZE               32
#define CH_CFG_TIME_TYPES_SIZE              32
#define CH_CFG_ST_TIMEDELTA                 2

/*===========================================================================*/
/* Kernel parameters and options                                             */
/*===========================================================================*/

#define CH_CFG_TIME_QUANTUM                 0
#define CH_CFG_NO_IDLE_THREAD               FALSE

/*===========================================================================*/
/* Performance options                                                       */
/*===========================================================================*/

#define CH_CFG_OPTIMIZE_SPEED               TRUE

/*===========================================================================*/
/* Subsystem options                                                         */
/*===========================================================================*/

#define CH_CFG_USE_TM                       TRUE
#define CH_CFG_USE_TIMESTAMP                TRUE
#define CH_CFG_USE_REGISTRY                 TRUE
#define CH_CFG_USE_WAITEXIT                 TRUE
#define CH_CFG_USE_SEMAPHORES               TRUE
#define CH_CFG_USE_SEMAPHORES_PRIORITY      FALSE
#define CH_CFG_USE_MUTEXES                  TRUE
#define CH_CFG_USE_MUTEXES_RECURSIVE        FALSE
#define CH_CFG_USE_CONDVARS                 TRUE
#define CH_CFG_USE_CONDVARS_TIMEOUT         TRUE
#define CH_CFG_USE_EVENTS                   TRUE
#define CH_CFG_USE_EVENTS_TIMEOUT           TRUE
#define CH_CFG_USE_MESSAGES                 TRUE
#define CH_CFG_USE_MESSAGES_PRIORITY        FALSE
#define CH_CFG_USE_DYNAMIC                  TRUE

/*===========================================================================*/
/* OSLIB options                                                             */
/*===========================================================================*/

#define CH_CFG_USE_MAILBOXES                TRUE
#define CH_CFG_USE_MEMCHECKS                TRUE
#define CH_CFG_USE_MEMCORE                  TRUE
#define CH_CFG_MEMCORE_SIZE                 0
#define CH_CFG_USE_HEAP                     TRUE
#define CH_CFG_USE_MEMPOOLS                 TRUE
#define CH_CFG_USE_OBJ_FIFOS                TRUE
#define CH_CFG_USE_PIPES                    TRUE
#define CH_CFG_USE_OBJ_CACHES              TRUE
#define CH_CFG_USE_DELEGATES                TRUE
#define CH_CFG_USE_JOBS                     TRUE

/*===========================================================================*/
/* Objects factory options                                                   */
/*===========================================================================*/

#define CH_CFG_USE_FACTORY                  FALSE
#define CH_CFG_FACTORY_MAX_NAMES_LENGTH     8
#define CH_CFG_FACTORY_OBJECTS_REGISTRY     FALSE
#define CH_CFG_FACTORY_GENERIC_BUFFERS      FALSE
#define CH_CFG_FACTORY_SEMAPHORES           FALSE
#define CH_CFG_FACTORY_MAILBOXES            FALSE
#define CH_CFG_FACTORY_OBJ_FIFOS            FALSE
#define CH_CFG_FACTORY_PIPES                FALSE

/*===========================================================================*/
/* Debug options                                                             */
/*===========================================================================*/

#define CH_DBG_STATISTICS                   FALSE
#define CH_DBG_SYSTEM_STATE_CHECK           TRUE
#define CH_DBG_ENABLE_CHECKS                TRUE
#define CH_DBG_ENABLE_ASSERTS               TRUE
#define CH_DBG_TRACE_MASK                   CH_DBG_TRACE_MASK_DISABLED
#define CH_DBG_TRACE_BUFFER_SIZE            128
#define CH_DBG_ENABLE_STACK_CHECK           TRUE
#define CH_DBG_FILL_THREADS                 TRUE
#define CH_DBG_THREADS_PROFILING            FALSE

/*===========================================================================*/
/* Kernel hooks                                                              */
/*===========================================================================*/

#define CH_CFG_SYSTEM_EXTRA_FIELDS                                          \
  /* Add system custom fields here.*/

#define CH_CFG_SYSTEM_INIT_HOOK() do {                                      \
  /* Add system initialization code here.*/                                 \
} while (false)

#define CH_CFG_OS_INSTANCE_EXTRA_FIELDS                                     \
  /* Add OS instance custom fields here.*/

#define CH_CFG_OS_INSTANCE_INIT_HOOK(oip) do {                              \
  /* Add OS instance initialization code here.*/                            \
} while (false)

#define CH_CFG_THREAD_EXTRA_FIELDS                                          \
  /* Add threads custom fields here.*/

#define CH_CFG_THREAD_INIT_HOOK(tp) do {                                    \
  /* Add threads initialization code here.*/                                \
} while (false)

#define CH_CFG_THREAD_EXIT_HOOK(tp) do {                                    \
  /* Add threads finalization code here.*/                                  \
} while (false)

#define CH_CFG_CONTEXT_SWITCH_HOOK(ntp, otp) do {                           \
  /* Context switch code here.*/                                            \
} while (false)

#define CH_CFG_IRQ_PROLOGUE_HOOK() do {                                     \
  /* IRQ prologue code here.*/                                              \
} while (false)

#define CH_CFG_IRQ_EPILOGUE_HOOK() do {                                     \
  /* IRQ epilogue code here.*/                                              \
} while (false)

#define CH_CFG_IDLE_ENTER_HOOK() do {                                       \
  /* Idle-enter code here.*/                                                \
} while (false)

#define CH_CFG_IDLE_LEAVE_HOOK() do {                                       \
  /* Idle-leave code here.*/                                                \
} while (false)

#define CH_CFG_IDLE_LOOP_HOOK() do {                                        \
  /* Idle loop code here.*/                                                 \
} while (false)

#define CH_CFG_SYSTEM_TICK_HOOK() do {                                      \
  /* System tick event code here.*/                                         \
} while (false)

#define CH_CFG_SYSTEM_HALT_HOOK(reason) do {                                \
  /* System halt code here.*/                                               \
} while (false)

#define CH_CFG_TRACE_HOOK(tep) do {                                         \
  /* Trace code here.*/                                                     \
} while (false)

#define CH_CFG_RUNTIME_FAULTS_HOOK(mask) do {                               \
  /* Faults handling code here.*/                                           \
} while (false)

#define CH_CFG_SAFETY_CHECK_HOOK(l, f) do {                                 \
  /* Safety handling code here.*/                                           \
  chSysHalt(f);                                                             \
} while (false)

#endif  /* CHCONF_H */
