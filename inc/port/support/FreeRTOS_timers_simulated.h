/**
 * @file FreeRTOS_timers_simulated.h
 * @author Marcos Dominguez
 *
 * @brief Port for Custom timers API
 *
 * @version 0.1
 * @date 2023-12-15
 */

#ifndef _PORT_TIMERS_CUSTOM_H
#define _PORT_TIMERS_CUSTOM_H

#ifdef  __cplusplus
extern "C" {
#endif

/*========= [DEPENDENCIES] =====================================================*/

/// \cond
#include "utils.h"
/// \endcond
/// \cond
#include "data_types.h"
/// \endcond
#include "FreeRTOS_simulated.h"
#include "timer_manager.h"

/*========= [PUBLIC MACRO AND CONSTANTS] =======================================*/

#define configTIMER_TASK_STACK_DEPTH            256 /**< Depth of the stack for the timer task */

/*========= [PUBLIC DATA TYPE] =================================================*/

typedef timer_index_t StaticTimer_t;                            /**< Type definition for a static timer. */

typedef StaticTimer_t *TimerHandle_t;                           /**< Type definition for timer handle. */

typedef void (*TimerCallbackFunction_t)(TimerHandle_t xTimer);  ///<Type definition for the timer callback function.

/*========= [PUBLIC FUNCTION DECLARATIONS] =====================================*/

/**
 * @brief Creates a static timer.
 *
 * @param pcTimerName Name of the timer.
 * @param xTimerPeriodInTicks Period of the timer in ticks.
 * @param xAutoReload Whether the timer should auto-reload.
 * @param pvTimerID Pointer to the timer ID.
 * @param pxCallbackFunction Pointer to the callback function.
 * @param pxTimerBuffer Pointer to the static timer structure.
 * @return Timer handle on success, NULL on failure.
 */
TimerHandle_t xTimerCreateStatic(const char *const pcTimerName, const TickType_t xTimerPeriodInTicks, const BaseType_t xAutoReload, void *const pvTimerID, TimerCallbackFunction_t pxCallbackFunction, StaticTimer_t *pxTimerBuffer);

/**
 * @brief Resets a timer.
 *
 * @param xTimer Timer handle.
 * @param xTicksToWait Ticks to wait for the operation.
 * @return pdPASS if successful, pdFAIL otherwise.
 */
BaseType_t xTimerReset(TimerHandle_t xTimer, TickType_t xTicksToWait);

/**
 * @brief Starts a timer.
 *
 * @param xTimer Timer handle.
 * @param xTicksToWait Ticks to wait for the operation.
 * @return pdPASS if successful, pdFAIL otherwise.
 */
BaseType_t xTimerStart(TimerHandle_t xTimer, TickType_t xTicksToWait);

/**
 * @brief Stops a timer.
 *
 * @param xTimer Timer handle.
 * @param xTicksToWait Ticks to wait for the operation.
 * @return pdPASS if successful, pdFAIL otherwise.
 */
BaseType_t xTimerStop(TimerHandle_t xTimer, TickType_t xTicksToWait);

#ifdef  __cplusplus
}

#endif

#endif  /* _PORT_TIMERS_CUSTOM_H */
