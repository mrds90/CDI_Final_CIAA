/**
 * @file osal_queue.c
 * @author Marcos Dominguez
 *
 * @brief OS abstraction for queue API.
 *
 * @version 0.1
 * @date 2023-11-24
 */

/*========= [DEPENDENCIES] =====================================================*/


#include "osal_queue.h"

/*========= [PRIVATE MACROS AND CONSTANTS] =====================================*/

/*========= [PRIVATE DATA TYPES] ===============================================*/

/*========= [TASK DECLARATIONS] ================================================*/

/*========= [PRIVATE FUNCTION DECLARATIONS] ====================================*/

/*========= [INTERRUPT FUNCTION DECLARATIONS] ==================================*/

/*========= [LOCAL VARIABLES] ==================================================*/

/*========= [STATE FUNCTION POINTERS] ==========================================*/

/*========= [PUBLIC FUNCTION IMPLEMENTATION] ===================================*/

bool_t OSAL_QUEUE_LoadStruct(osal_queue_t *queue_ptr, osal_queue_holder_t *holder_ptr, uint8_t *queue_storage, uint32_t data_size, uint16_t queue_length) {
    bool_t ret = FALSE;
    if (queue_ptr != NULL) {
        if (holder_ptr != NULL) {
            if (queue_storage != NULL) {
                if ((data_size > 0) && (queue_length > 0)) {
                    ret = TRUE;
                    queue_ptr->data_size = data_size;
                    queue_ptr->holder_ptr = holder_ptr;
                    queue_ptr->queue_length = queue_length;
                    queue_ptr->queue_storage = queue_storage;
                }
            }
        }
    }
    return ret;
}

bool_t OSAL_QUEUE_Create(osal_queue_t *queue_ptr) {
    bool_t ret = FALSE;
    if (queue_ptr != NULL) {
        if (queue_ptr->data_size > 0) {
            queue_ptr->handler = PORT_QUEUE_Create(queue_ptr->queue_length, queue_ptr->data_size, queue_ptr->queue_storage, queue_ptr->holder_ptr);
            if (queue_ptr->handler != NULL) {
                ret = TRUE;
            }
        }
    }
    return ret;
}

bool_t OSAL_QUEUE_Send(osal_queue_t *queue_ptr, void *data, osal_tick_t wait_time) {
    bool_t ret = FALSE;
    if (queue_ptr != NULL) {
        if (queue_ptr->handler != NULL)
            ret = PORT_QUEUE_Send(queue_ptr->handler, data, wait_time);
    }
    return ret;
}

bool_t OSAL_QUEUE_Receive(osal_queue_t *queue_ptr, void *const data, osal_tick_t wait_time) {
    bool_t ret = FALSE;
    if (queue_ptr != NULL) {
        if (queue_ptr->handler != NULL)
            ret = PORT_QUEUE_Receive(queue_ptr->handler, data, wait_time);
    }
    return ret;
}

bool_t OSAL_QUEUE_SendFromISR(osal_queue_t *queue_ptr, void *data, bool_t *yield_need) {
    bool_t ret = FALSE;
    *yield_need = FALSE;
    if (queue_ptr != NULL) {
        if (queue_ptr->handler != NULL)
            ret = PORT_QUEUE_SendFromISR(queue_ptr->handler, data, yield_need);
    }
    return ret;
}

bool_t OSAL_QUEUE_ReceiveFromISR(osal_queue_t *queue_ptr, void *const data, bool_t *yield_need) {
    bool_t ret = FALSE;
    *yield_need = FALSE;
    if (queue_ptr != NULL) {
        if (queue_ptr->handler != NULL)
            ret = PORT_QUEUE_ReceiveFromISR(queue_ptr->handler, data, yield_need);
    }
    return ret;
}

/*========= [PRIVATE FUNCTION IMPLEMENTATION] ==================================*/

/*========= [INTERRUPT FUNCTION IMPLEMENTATION] ================================*/
