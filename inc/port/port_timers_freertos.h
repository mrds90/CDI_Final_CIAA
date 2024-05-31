/**
 * @file port_timers_freertos.h
 * @author Marcos Dominguez
 *
 * @brief Port for FreeRTOS timers API.
 *
 * @version 0.1
 * @date 2023-12-15
 */

#ifndef _PORT_TIMERS_FREERTOS_H
#define _PORT_TIMERS_FREERTOS_H

#ifdef __cplusplus
extern "C" {
#endif

/*========= [DEPENDENCIES] =====================================================*/

#include "port_freertos.h"
#ifndef TEST
#include "timers.h"
#else
#include "FreeRTOS_timers_simulated.h"
#endif

/// \cond
#include "utils.h"
#include "data_types.h"
/// \endcond
#include "timer_manager.h"

/*========= [PUBLIC MACRO AND CONSTANTS] =======================================*/

/**
 * @brief Resets a FreeRTOS timer.
 *
 * This macro resets a FreeRTOS timer by calling xTimerReset with a timeout of 50 ticks.
 *
 * @param handler The handle of the timer to be reset.
 */
#define PORT_TIMERS_Reset(handler) xTimerReset(((port_timers_handler_t)(handler)), 50)

/**
 * @brief Starts a FreeRTOS timer.
 *
 * This macro starts a FreeRTOS timer by calling xTimerStart with a timeout of 50 ticks.
 *
 * @param handler The handle of the timer to be started.
 */
#define PORT_TIMERS_Start(handler) xTimerStart(((port_timers_handler_t)(handler)), 50)

/**
 * @brief Stops a FreeRTOS timer.
 *
 * This macro stops a FreeRTOS timer by calling xTimerStop with a timeout of 50 ticks.
 *
 * @param handler The handle of the timer to be stopped.
 */
#define PORT_TIMERS_Stop(handler)  xTimerStop(((port_timers_handler_t)(handler)), 50)

/*========= [PUBLIC DATA TYPE] =================================================*/

typedef StaticTimer_t port_timers_holder_t;  /**< Type definition for the static timer holder. */

typedef TimerHandle_t port_timers_handler_t; /**< Type definition for the FreeRTOS timer handle. */

/**
 * @brief Structure for FreeRTOS timer callback.
 *
 * This structure holds the callback function and context to be used when the timer expires.
 */
typedef struct {
    UtilsCallback_t callback; /**< The callback function to be executed when the timer expires. */
    void *context;           /**< Context to be passed to the callback function. */
} PortSWTimerCallback_t;

/*========= [PUBLIC FUNCTION DECLARATIONS] =====================================*/

/**
 * @brief Create a FreeRTOS timer.
 *
 * This function creates a FreeRTOS timer with the provided name, time period, and callback function.
 *
 * @param name       The name of the timer.
 * @param time       The period of the timer in ticks.
 * @param repeat     Whether the timer should repeat or not.
 * @param Callback   The callback function to be executed when the timer expires.
 * @param holder_ptr The pointer to the timer holder.
 * @param index      The index of the timer in the timers_active array.
 *
 * @return The handle of the created timer.
 */
port_timers_handler_t PORT_TIMER_Create(const char *name, port_tick_t time, bool_t repeat, PortSWTimerCallback_t *Callback, port_timers_holder_t *holder_ptr, timer_index_t index);

#ifdef __cplusplus
}

#endif

#endif  /* _PORT_TIMERS_FREERTOS_H */
