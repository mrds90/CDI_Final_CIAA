/**
 * @file port_task_freertos.h
 * @author Marcos Dominguez
 *
 * @brief Port for FreeRTOS task API.
 *
 * @version 0.1
 * @date 2023-11-24
 */

#ifndef _PORT_TASK_FREERTOS_H
#define _PORT_TASK_FREERTOS_H

#ifdef __cplusplus
extern "C" {
#endif

/*========= [DEPENDENCIES] =====================================================*/

#ifndef TEST
#include "FreeRTOS.h"
#include "task.h"
#else
#include "FreeRTOS_task_simulated.h"
#endif

/*========= [PUBLIC MACRO AND CONSTANTS] =======================================*/

#define OSAL_TASK_Delay(delay_ticks) vTaskDelay(delay_ticks) /**< Macro to delay the task by the specified number of ticks. */

#define OSAL_TASK_DelayUntil(LastWakeTime, delay_ticks) vTaskDelayUntil(LastWakeTime, delay_ticks) /**< Macro to delay the task until the specified time. */

#define OSAL_MAX_TASK_NAME_LEN configMAX_TASK_NAME_LEN /**< Maximum length of a task name. */

#define OSAL_TASK_GetTickCount() xTaskGetTickCount() /**< Macro to get the current tick count. */

#define OSAL_TASK_GetTaskName(X) pcTaskGetTaskName(X) /**< Macro to get the name of a task. */

/*========= [PUBLIC DATA TYPE] =================================================*/

typedef TaskHandle_t osal_task_handler_t; /**< Type definition for the FreeRTOS task handle. */

typedef StaticTask_t osal_task_holder_t; /**< Type definition for the static task holder. */

typedef StackType_t osal_stack_holder_t; /**< Type definition for the static stack holder. */

typedef TaskFunction_t OSAL_TASK_Callback_t; /**< Type definition for the task callback function. */

/*========= [PUBLIC FUNCTION DECLARATIONS] =====================================*/

/**
 * @brief Create a static FreeRTOS task.
 *
 * This function creates a FreeRTOS task with the provided parameters.
 *
 * @param function      The task callback function.
 * @param name          The name of the task.
 * @param size          The size of the task stack.
 * @param context       The context to be passed to the task callback function.
 * @param priority      The priority of the task.
 * @param stack_ptr     The pointer to the stack holder.
 * @param task_hold_ptr The pointer to the task holder.
 *
 * @return The handle of the created task.
 */
osal_task_handler_t PORT_TASK_CreateStaticTask(OSAL_TASK_Callback_t function, char *name, uint16_t size, void *context, uint8_t priority, osal_stack_holder_t *stack_ptr, osal_task_holder_t *task_hold_ptr);

#ifdef __cplusplus
}

#endif

#endif  /* _PORT_TASK_FREERTOS_H */
