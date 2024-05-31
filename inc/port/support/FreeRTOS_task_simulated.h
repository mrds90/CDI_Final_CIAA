/**
 * @file FreeRTOS_task_simulated.h
 * @author Marcos Dominguez
 *
 * @brief Port for FreeRTOS task API
 *
 * @version 0.1
 * @date 2023-12-11
 */

#ifndef _PORT_TASK_CUSTOM_H
#define _PORT_TASK_CUSTOM_H

#ifdef  __cplusplus
extern "C" {
#endif

/*========= [DEPENDENCIES] =====================================================*/

#include "FreeRTOS_simulated.h"

/// \cond
#include "utils.h"
/// \endcond
/// \cond
#include "data_types.h"
/// \endcond

/*========= [PUBLIC MACRO AND CONSTANTS] =======================================*/

#define configMAX_TASK_NAME_LEN   (35U)                 /**< Maximum length of a task name */

#define tskIDLE_PRIORITY          (( UBaseType_t ) 0U)  /**< Priority value for the idle task */

/*========= [PUBLIC DATA TYPE] =================================================*/

/**
 * @brief Function pointer type for task code
 */
typedef void (*TaskFunction_t)(void *);


/**
 * @brief Structure representing a static task
 */
typedef struct {
    TaskFunction_t TaskCallback;    /**< Pointer to the task function */
    void *context;                  /**< Context or parameters for the task */
} StaticTask_t;

typedef uint32_t StackType_t;       /**< Type definition for stack */

typedef StaticTask_t *TaskHandle_t; /**< Type definition for task handle */

/*========= [SHARED VARIABLES] =================================================*/

extern char *simulated_task_name;   /**< Name of the simulated task */

extern bool_t task_create_success;  /**< Flag indicating the success of task creation */

TESTED_VARIABLE uint32_t so_tick_count;

/*========= [PUBLIC FUNCTION DECLARATIONS] =====================================*/

/**
 * @brief Creates a static task.
 *
 * @param pxTaskCode Pointer to the task function.
 * @param pcName Name of the task.
 * @param ulStackDepth Depth of the task's stack.
 * @param pvParameters Task parameters or context.
 * @param uxPriority Priority of the task.
 * @param puxStackBuffer Pointer to the stack buffer.
 * @param pxTaskBuffer Pointer to the static task structure.
 * @return Task handle on success, NULL on failure.
 */
TaskHandle_t xTaskCreateStatic(TaskFunction_t pxTaskCode, const char *const pcName, const uint32_t ulStackDepth, void *const pvParameters, UBaseType_t uxPriority, StackType_t *const puxStackBuffer, StaticTask_t *const pxTaskBuffer);

/**
 * @brief Gets the tick count.
 *
 * @return Current tick count.
 */
TickType_t xTaskGetTickCount(void);

/**
 * @brief Holds the handler for a simulated task.
 *
 * @param handle_addr Pointer to the task handle.
 */
void Task_Simulated_HoldHandler(TaskHandle_t *handle_addr);

/**
 * @brief Delays the task for a specified number of ticks.
 *
 * @param delay_ticks Number of ticks to delay the task.
 */
void vTaskDelay(TickType_t delay_ticks);

/**
 * @brief Delays the task until a specified time.
 *
 * @param previous_time Pointer to the previous time.
 * @param delay_ticks Number of ticks to delay the task.
 */
void vTaskDelayUntil(TickType_t *previous_time, TickType_t delay_ticks);

/**
 * @brief Gets the name of a task.
 *
 * @param xTaskToQuery Task handle to query.
 * @return Task name.
 */
char*pcTaskGetTaskName(TaskHandle_t xTaskToQuery);

#ifdef  __cplusplus
}

#endif

#endif  /* _PORT_TASK_CUSTOM_H */
