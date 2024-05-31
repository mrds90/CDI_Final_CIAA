/**
 * @file port_queue_freertos.c
 * @author Marcos Dominguez
 *
 * @brief Port for FreeRTOS queue API.
 *
 * @version 0.1
 * @date 2023-11-24
 */

/*========= [DEPENDENCIES] =====================================================*/

#include "port_semaphore_freertos.h"

/*========= [PRIVATE MACROS AND CONSTANTS] =====================================*/

/*========= [PRIVATE DATA TYPES] ===============================================*/

/*========= [TASK DECLARATIONS] ================================================*/

/*========= [PRIVATE FUNCTION DECLARATIONS] ====================================*/

/*========= [INTERRUPT FUNCTION DECLARATIONS] ==================================*/

/*========= [LOCAL VARIABLES] ==================================================*/

/*========= [STATE FUNCTION POINTERS] ==========================================*/

/*========= [PUBLIC FUNCTION IMPLEMENTATION] ===================================*/

bool_t PORT_SEMAPHORE_Give(osal_semaphore_handle_t handler) {
    bool_t ret = FALSE;
    if (xSemaphoreGive(handler)) {
        ret = TRUE;
    }
    return ret;
}

bool_t PORT_SEMAPHORE_Take(osal_semaphore_handle_t handler, port_tick_t wait_time) {
    bool_t ret = FALSE;
    if (xSemaphoreTake(handler, wait_time) == pdTRUE) {
        ret = TRUE;
    }
    return ret;
}

bool_t PORT_SEMAPHORE_GiveFromISR(osal_semaphore_handle_t handler) {
    static BaseType_t HigherPriorityTaskWoken;
    bool_t ret = FALSE;
    xSemaphoreGiveFromISR(handler, &HigherPriorityTaskWoken);
    if (HigherPriorityTaskWoken == pdTRUE) {
        ret = TRUE;
    }
    return ret;
}

bool_t PORT_SEMAPHORE_TakeFromISR(osal_semaphore_handle_t handler) {
    static BaseType_t HigherPriorityTaskWoken;
    bool_t ret = FALSE;
    xSemaphoreTakeFromISR(handler, &HigherPriorityTaskWoken);
    if (HigherPriorityTaskWoken == pdTRUE) {
        ret = TRUE;
    }
    return ret;
}

/*========= [PRIVATE FUNCTION IMPLEMENTATION] ==================================*/

/*========= [INTERRUPT FUNCTION IMPLEMENTATION] ================================*/
