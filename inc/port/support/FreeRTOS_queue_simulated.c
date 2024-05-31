/**
 * @file FreeRTOS_queue_simulated.c
 * @author Marcos Dominguez
 *
 * @brief Port for FreeRTOS queue API.
 *
 * @version 0.1
 * @date 2023-11-24
 */

/*========= [DEPENDENCIES] =====================================================*/

#include <string.h>
#include "FreeRTOS_queue_simulated.h"

/*========= [PRIVATE MACROS AND CONSTANTS] =====================================*/

/**
 * @brief Calculates the total number of bytes needed for the queue storage.
 */
#define BYTES_OF_QUEUE(data_size, element_qty) (data_size * element_qty)

/*========= [PRIVATE DATA TYPES] ===============================================*/

/*========= [TASK DECLARATIONS] ================================================*/

/*========= [PRIVATE FUNCTION DECLARATIONS] ====================================*/

/*========= [INTERRUPT FUNCTION DECLARATIONS] ==================================*/

/*========= [LOCAL VARIABLES] ==================================================*/

/**
 * @brief Indicates whether queue creation will be successful.
 */
bool_t queue_create_success = TRUE;

/**
 * @brief Indicates whether queue sending will be successful.
 */
bool_t queue_send_success = TRUE;

/**
 * @brief Indicates whether queue receiving will be successful.
 */
bool_t queue_receive_success = TRUE;

/*========= [STATE FUNCTION POINTERS] ==========================================*/

/*========= [PUBLIC FUNCTION IMPLEMENTATION] ===================================*/

/*========= [STATE FUNCTION POINTERS] ==========================================*/

/*========= [PUBLIC FUNCTION IMPLEMENTATION] ===================================*/

QueueHandle_t __attribute__((weak)) xQueueCreateStatic(const UBaseType_t queue_length, const UBaseType_t data_size, uint8_t *queue_storage, StaticQueue_t *queue_struct) {
    if (queue_create_success) {
        queue_struct->queue_storage = queue_storage;
        queue_struct->pop_ptr = queue_storage;
        queue_struct->push_ptr = queue_storage;
        queue_struct->queue_length = queue_length;
        queue_struct->data_size = data_size;
        queue_struct->used_elements = 0;
        return queue_struct;
    }
    else {
        return NULL;
    }
}

UBaseType_t __attribute__((weak)) xQueueSend(QueueHandle_t handler, void *data, TickType_t wait_time) {
    if (queue_send_success) {
        if ((handler->used_elements < handler->queue_length)) {
            memcpy(handler->push_ptr, data, handler->data_size);
            handler->push_ptr += handler->data_size;
            if (handler->push_ptr - (handler->queue_storage - 1) >= BYTES_OF_QUEUE(handler->data_size, handler->queue_length)) {
                handler->push_ptr = handler->queue_storage;
            }
            handler->used_elements++;

            return pdTRUE;
        }
        else {
            return pdFALSE;
        }
    }
    else {
        return pdFALSE;
    }
}

UBaseType_t __attribute__((weak)) xQueueReceive(QueueHandle_t handler, void *data, TickType_t wait_time) {
    if (queue_receive_success) {
        if ((handler->used_elements > 0) && (data != NULL)) {
            memcpy(data, handler->pop_ptr, handler->data_size);
            handler->pop_ptr += handler->data_size;
            if (handler->pop_ptr - (handler->queue_storage - 1) >= BYTES_OF_QUEUE(handler->data_size, handler->queue_length)) {
                handler->pop_ptr = handler->queue_storage;
            }
            handler->used_elements--;
            return pdTRUE;
        }
        else {
            return pdFALSE;
        }
    }
    else {
        return pdFALSE;
    }
}

BaseType_t __attribute__((weak)) xQueueSendFromISR(QueueHandle_t handler, void *data, BaseType_t *const pxHigherPriorityTaskWoken) {
    if (queue_send_success) {
        if ((handler->used_elements < handler->queue_length)) {
            memcpy(handler->push_ptr, data, handler->data_size);
            handler->push_ptr += handler->data_size;
            if (handler->push_ptr - (handler->queue_storage - 1) >= BYTES_OF_QUEUE(handler->data_size, handler->queue_length)) {
                handler->push_ptr = handler->queue_storage;
            }
            handler->used_elements++;
            *pxHigherPriorityTaskWoken = pdTRUE;
            return pdTRUE;
        }
        else {
            *pxHigherPriorityTaskWoken = pdFALSE;
            return pdFALSE;
        }
    }
    else {
        *pxHigherPriorityTaskWoken = pdFALSE;
        return pdFALSE;
    }
}

BaseType_t __attribute__((weak)) xQueueReceiveFromISR(QueueHandle_t handler, void *const data, BaseType_t *const pxHigherPriorityTaskWoken) {
    if (queue_receive_success) {
        if ((handler->used_elements > 0) && (data != NULL)) {
            memcpy(data, handler->pop_ptr, handler->data_size);
            handler->pop_ptr += handler->data_size;
            if (handler->pop_ptr - (handler->queue_storage - 1) >= BYTES_OF_QUEUE(handler->data_size, handler->queue_length)) {
                handler->pop_ptr = handler->queue_storage;
            }
            handler->used_elements--;
            *pxHigherPriorityTaskWoken = pdTRUE;
            return pdTRUE;
        }
        else {
            *pxHigherPriorityTaskWoken = pdFALSE;
            return pdFALSE;
        }
    }
    else {
        *pxHigherPriorityTaskWoken = pdFALSE;
        return pdFALSE;
    }
}

/*========= [PRIVATE FUNCTION IMPLEMENTATION] ==================================*/

/*========= [INTERRUPT FUNCTION IMPLEMENTATION] ================================*/
