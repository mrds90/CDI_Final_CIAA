/**
 * @file FreeRTOS_simulated.h
 * @author Marcos Dominguez
 *
 * @brief Port for FreeRTOS top API.
 *
 * This file provides a simulated implementation of the top-level FreeRTOS API.
 * It includes definitions for data types, macros, and functions that are essential
 * for FreeRTOS functionality.
 *
 * @version 0.1
 * @date 2023-11-24
 */

#ifndef _PORT_CUSTOM_H
#define _PORT_CUSTOM_H

#ifdef  __cplusplus
extern "C" {
#endif

/*========= [DEPENDENCIES] =====================================================*/

#include <stdint.h>
#include <stdio.h>

/*========= [PUBLIC MACRO AND CONSTANTS] =======================================*/

typedef uint32_t TickType_t; /**< Type for time-related values in FreeRTOS. */

typedef uint32_t UBaseType_t; /**< Type for unsigned base values in FreeRTOS. */

typedef int32_t BaseType_t; /**< Type for base values in FreeRTOS. */

#define portMAX_DELAY                     ((TickType_t) 0xFFFFFFFF) /**< Maximum time delay value (wait forever). */

#define pdMS_TO_TICKS(ms)                 (ms) /**< Macro to convert milliseconds to ticks. */

#define configASSERT(x)                   if ((x) == 0) {printf("%s - line: %d\n", __FILE__, __LINE__); return FALSE;}

#define configMINIMAL_STACK_SIZE          (32) /**< Minimum stack size for FreeRTOS tasks. */

#define portEND_SWITCHING_ISR(condition)  (condition) /**< End switching interrupt macro. */

#ifndef pdFALSE
#define pdFALSE                                  ((BaseType_t) 0) /**< False value for FreeRTOS. */
#endif

#ifndef pdTRUE
#define pdTRUE                                   ((BaseType_t) 1) /**< True value for FreeRTOS. */
#endif

#ifndef pdPASS
#define pdPASS                                   (pdTRUE) /**< Pass value for FreeRTOS. */
#endif

#ifndef pdFAIL
#define pdFAIL                                   (pdFALSE) /**< Fail value for FreeRTOS. */
#endif

/*========= [PUBLIC DATA TYPE] =================================================*/

/**
 * @brief Structure representing a static queue in FreeRTOS.
 */
typedef struct {
    uint8_t *queue_storage; /**< Pointer to the storage array of the queue. */
    uint8_t *pop_ptr;       /**< Pointer to the location where elements are popped. */
    uint8_t *push_ptr;      /**< Pointer to the location where elements are pushed. */
    uint16_t queue_length;  /**< Maximum length of the queue. */
    uint32_t data_size;     /**< Size of each data element in the queue. */
    uint16_t used_elements; /**< Number of elements currently in the queue. */
} StaticQueue_t;

/*========= [PUBLIC FUNCTION DECLARATIONS] =====================================*/

#ifdef  __cplusplus
}

#endif

#endif  /* _PORT_CUSTOM_H */
