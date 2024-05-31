/**
 * @file FreeRTOS_task_simulated.c
 * @author Marcos Dominguez
 *
 * @brief Port for FreeRTOS task API.
 *
 * @version 0.1
 * @date 2023-11-24
 */

/*========= [DEPENDENCIES] =====================================================*/

#include "FreeRTOS_task_simulated.h"

/*========= [PRIVATE MACROS AND CONSTANTS] =====================================*/

/*========= [PRIVATE DATA TYPES] ===============================================*/

/*========= [TASK DECLARATIONS] ================================================*/

/*========= [PRIVATE FUNCTION DECLARATIONS] ====================================*/

/*========= [INTERRUPT FUNCTION DECLARATIONS] ==================================*/

/*========= [LOCAL VARIABLES] ==================================================*/

bool_t task_create_success = TRUE; /**< Indicates whether task creation will be successful. */

TaskHandle_t *handler_to_save = NULL; /**< Pointer to save the created task handler. */

uint32_t so_tick_count = 0; /**< Counter used for simulated tick count. */

char *simulated_task_name; /**< Simulated task name. */

/*========= [STATE FUNCTION POINTERS] ==========================================*/

/*========= [PUBLIC FUNCTION IMPLEMENTATION] ===================================*/

TaskHandle_t __attribute__((weak)) xTaskCreateStatic(TaskFunction_t pxTaskCode, const char *const pcName, const uint32_t ulStackDepth, void *const pvParameters, UBaseType_t uxPriority, StackType_t *const puxStackBuffer, StaticTask_t *const pxTaskBuffer) {
    if (task_create_success) {
        pxTaskBuffer->TaskCallback = pxTaskCode;
        pxTaskBuffer->context = pvParameters;
        if (handler_to_save != NULL) {
            *handler_to_save = pxTaskBuffer;
            handler_to_save = NULL;
        }

        return pxTaskBuffer;
    }
    else {
        return NULL;
    }
}

TickType_t __attribute__((weak)) xTaskGetTickCount(void) {
    return so_tick_count;
}

void __attribute__((weak)) vTaskDelay(TickType_t delay_ticks) {
    so_tick_count += delay_ticks;
}

void __attribute__((weak)) vTaskDelayUntil(TickType_t *previous_time, TickType_t delay_ticks) {
    so_tick_count += delay_ticks;
}

char *__attribute__((weak)) pcTaskGetTaskName(TaskHandle_t xTaskToQuery) {
    return simulated_task_name;
}

void Task_Simulated_HoldHandler(TaskHandle_t *handle_addr) {
    handler_to_save = handle_addr;
}

/*========= [PRIVATE FUNCTION IMPLEMENTATION] ==================================*/

/*========= [INTERRUPT FUNCTION IMPLEMENTATION] ================================*/
