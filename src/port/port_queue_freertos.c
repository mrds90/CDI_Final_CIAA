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

#include "port_queue_freertos.h"

/*========= [PRIVATE MACROS AND CONSTANTS] =====================================*/

/*========= [PRIVATE DATA TYPES] ===============================================*/

/*========= [TASK DECLARATIONS] ================================================*/

/*========= [PRIVATE FUNCTION DECLARATIONS] ====================================*/

/*========= [INTERRUPT FUNCTION DECLARATIONS] ==================================*/

/*========= [LOCAL VARIABLES] ==================================================*/

/*========= [STATE FUNCTION POINTERS] ==========================================*/

/*========= [PUBLIC FUNCTION IMPLEMENTATION] ===================================*/

osal_queue_handle_t PORT_QUEUE_Create(uint16_t queue_length, uint32_t data_size, uint8_t *queue_storage, osal_queue_holder_t *queue_struct) {
    return xQueueCreateStatic(queue_length, data_size, queue_storage, queue_struct);
}

bool_t PORT_QUEUE_Send(osal_queue_handle_t handler, void *data, port_tick_t wait_time) {
    bool_t ret = FALSE;
    if (xQueueSend(handler, data, wait_time) == pdTRUE) {
        ret = TRUE;
    }
    return ret;
}

bool_t PORT_QUEUE_Receive(osal_queue_handle_t handler, void *data, port_tick_t wait_time) {
    bool_t ret = FALSE;
    if (xQueueReceive(handler, data, wait_time) == pdTRUE) {
        ret = TRUE;
    }
    return ret;
}

bool_t PORT_QUEUE_SendFromISR(osal_queue_handle_t handler, void *data, bool_t *yield_need) {
    bool_t ret = FALSE;
    *yield_need = FALSE;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (xQueueSendFromISR(handler, data, &xHigherPriorityTaskWoken) == pdTRUE) {
        if (xHigherPriorityTaskWoken) {
            *yield_need = TRUE;
        }
        ret = TRUE;
    }
    return ret;
}

bool_t PORT_QUEUE_ReceiveFromISR(osal_queue_handle_t handler, void *data, bool_t *yield_need) {
    bool_t ret = FALSE;
    *yield_need = FALSE;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (xQueueReceiveFromISR(handler, data, &xHigherPriorityTaskWoken) == pdTRUE) {
        if (xHigherPriorityTaskWoken) {
            *yield_need = TRUE;
        }
        ret = TRUE;
    }
    return ret;
}

/*========= [PRIVATE FUNCTION IMPLEMENTATION] ==================================*/

/*========= [INTERRUPT FUNCTION IMPLEMENTATION] ================================*/
