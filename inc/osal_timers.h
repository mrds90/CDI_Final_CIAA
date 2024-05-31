/**
 * @file osal_timers.h
 * @author Marcos Dominguez
 *
 * @brief OS abstraction for timer APIs.
 *
 * @version 0.1
 * @date 2023-11-24
 */

#ifndef _OSAL_TIMERS_H
#define _OSAL_TIMERS_H

#ifdef  __cplusplus
extern "C" {
#endif

/*========= [DEPENDENCIES] =====================================================*/

#include "osal_global.h"

#if (OS_USED == OS_FREERTOS)
#include "port_timers_freertos.h"
#endif

/// \cond
#include "data_types.h"
/// \endcond

#include "timer_manager.h"

/*========= [PUBLIC MACRO AND CONSTANTS] =======================================*/

/**
 * @brief Macro to reset a timer.
 */
#define OSAL_TIMERS_Reset(timer)    PORT_TIMERS_Reset(((osal_timer_t *)(timer))->handler)

/**
 * @brief Macro to start a timer.
 */
#define OSAL_TIMERS_Start(timer)    PORT_TIMERS_Start(((osal_timer_t *)(timer))->handler)

/**
 * @brief Macro to stop a timer.
 */
#define OSAL_TIMERS_Stop(timer)     PORT_TIMERS_Stop(((osal_timer_t *)(timer))->handler)


/*========= [PUBLIC DATA TYPE] =================================================*/

/**
 * @brief Callback function type for timers.
 */
typedef PortSWTimerCallback_t OSAL_TimerCallback_t;

/**
 * @brief Type definition for the holder of timer structures.
 */
typedef port_timers_holder_t osal_timer_holder_t;

/**
 * @brief Type definition for the handler of timer structures.
 */
typedef port_timers_handler_t osal_timer_handler_t;

/**
 * @brief Structure to hold information about a timer.
 */
typedef struct {
    OSAL_TimerCallback_t callback;   /**< Callback function to be executed when the timer expires. */
    osal_timer_handler_t handler;    /**< Timer handler. */
    osal_timer_holder_t *holder_ptr; /**< Pointer to the memory region where the timer is held. */
    bool_t repeat;                   /**< Flag indicating whether the timer should repeat. */
    port_tick_t time;                /**< Time interval for the timer. */
    char *name;                      /**< Name of the timer. */
} osal_timer_t;


/*========= [PUBLIC FUNCTION DECLARATIONS] =====================================*/

/**
 * @brief Load the timer struct with information about its configuration.
 *
 * @param timer             Pointer to structure that hold the information about the timer.
 * @param name              Name of the timer.
 * @param time              Time interval for the timer.
 * @param repeat            Flag indicating whether the timer should repeat.
 * @param Callback          Callback function to be executed when the timer expires.
 * @param context           Context for the callback function.
 * @param holder_ptr        Pointer to the memory region where the timer is held.
 * @return bool_t           TRUE: Operation success - FALSE: Operation fail.
 */
bool_t OSAL_TIMERS_LoadStruct(osal_timer_t *timer, char *name, osal_tick_t time, bool_t repeat, UtilsCallback_t Callback, void *context, osal_timer_holder_t *holder_ptr);

/**
 * @brief Create a timer.
 *
 * @param timer             Pointer to structure that hold the information about the timer.
 * @param index             Timer index.
 * @return bool_t           TRUE: Creation success - FALSE: Creation fail.
 */
bool_t OSAL_TIMERS_Create(osal_timer_t *timer, timer_index_t index);

#ifdef  __cplusplus
}

#endif

#endif  /* _OSAL_TIMERS_H */
