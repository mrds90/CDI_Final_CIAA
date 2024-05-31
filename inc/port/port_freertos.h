/**
 * @file port_freertos.h
 * @author Marcos Dominguez
 *
 * @brief Port for FreeRTOS top API.
 *
 * @version 0.1
 * @date 2023-11-24
 */

#ifndef _PORT_FREERTOS_H
#define _PORT_FREERTOS_H

#ifdef __cplusplus
extern "C" {
#endif

/*========= [DEPENDENCIES] =====================================================*/

#ifndef TEST
#include "FreeRTOS.h"
#else
#include "FreeRTOS_simulated.h"
#endif

/*========= [PUBLIC MACRO AND CONSTANTS] =======================================*/

typedef TickType_t port_tick_t; /**< Type definition for the FreeRTOS tick type. */

#define OSAL_MAX_DELAY             portMAX_DELAY                    /**< Maximum delay value for OSAL functions (wait forever). */
#define OSAL_MS_TO_TICKS(ms)       pdMS_TO_TICKS(ms)                /**< Macro to convert milliseconds to ticks. */
#define OSAL_ConfigASSERT(x)       configASSERT(x)                  /**< Macro to assert a configuration condition (block). */
#define OSAL_ATOMIC_TASK_SIZE      configMINIMAL_STACK_SIZE         /**< Size of the stack for atomic tasks. */
#define OSAL_PORT_YIELD(condition) portEND_SWITCHING_ISR(condition) /**< Macro to yield from an ISR. */

/*========= [PUBLIC DATA TYPE] =================================================*/

/*========= [PUBLIC FUNCTION DECLARATIONS] =====================================*/

#ifdef __cplusplus
}

#endif

#endif /* _PORT_FREERTOS_H */
