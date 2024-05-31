/**
 * @file FreeRTOS_queue_simulated.h
 * @author Marcos Dominguez
 *
 * @brief Port for FreeRTOS queue API
 *
 * @version 0.1
 * @date 2023-11-24
 */

#ifndef _PORT_QUEUE_CUSTOM_H
#define _PORT_QUEUE_CUSTOM_H

#ifdef  __cplusplus
extern "C" {
#endif

/*========= [DEPENDENCIES] =====================================================*/

#include "FreeRTOS_simulated.h"
/// \cond
#include "data_types.h"
/// \endcond

/*========= [PUBLIC MACRO AND CONSTANTS] =======================================*/

/*========= [PUBLIC DATA TYPE] =================================================*/

/**
 * @brief Type definition for a FreeRTOS queue handle.
 */
typedef StaticQueue_t *QueueHandle_t;

/*========= [SHARED VARIABLES] =================================================*/

/**
 * @brief Indicates whether queue creation will be successful.
 */
extern bool_t queue_create_success;

/**
 * @brief Indicates whether queue sending will be successful.
 */
extern bool_t queue_send_success;

/**
 * @brief Indicates whether queue receiving will be successful.
 */
extern bool_t queue_receive_success;

/*========= [PUBLIC FUNCTION DECLARATIONS] =====================================*/

/**
 * @brief Creates a static FreeRTOS queue.
 *
 * @param queue_length The maximum number of items the queue can hold.
 * @param data_size The size of each item in the queue.
 * @param queue_storage Pointer to the storage area for the queue.
 * @param queue_struct Pointer to the static queue structure.
 * @return A handle to the created queue, or NULL if creation fails.
 */
QueueHandle_t xQueueCreateStatic(const UBaseType_t queue_length, const UBaseType_t data_size, uint8_t *queue_storage, StaticQueue_t *queue_struct);

/**
 * @brief Sends an item to the queue.
 *
 * @param handler The handle of the queue.
 * @param data Pointer to the data to be sent.
 * @param wait_time The maximum time to wait (in ticks) for space to be available in the queue.
 * @return pdTRUE if the item is sent successfully, pdFALSE otherwise.
 */
UBaseType_t xQueueSend(QueueHandle_t handler, void *data, TickType_t wait_time);

/**
 * @brief Receives an item from the queue.
 *
 * @param handler The handle of the queue.
 * @param data Pointer to where the received data will be stored.
 * @param wait_time The maximum time to wait (in ticks) for an item to be available in the queue.
 * @return pdTRUE if the item is received successfully, pdFALSE otherwise.
 */
UBaseType_t xQueueReceive(QueueHandle_t handler, void *data, TickType_t wait_time);

/**
 * @brief Sends an item to the queue from an ISR.
 *
 * @param handler The handle of the queue.
 * @param data Pointer to the data to be sent.
 * @param pxHigherPriorityTaskWoken A pointer to a variable that will be set to pdTRUE if a task was woken.
 * @return pdTRUE if the item is sent successfully, pdFALSE otherwise.
 */
BaseType_t xQueueSendFromISR(QueueHandle_t handler, void *data, BaseType_t *const pxHigherPriorityTaskWoken);

/**
 * @brief Receives an item from the queue in an ISR.
 *
 * @param handler The handle of the queue.
 * @param data Pointer to where the received data will be stored.
 * @param pxHigherPriorityTaskWoken A pointer to a variable that will be set to pdTRUE if a task was woken.
 * @return pdTRUE if the item is received successfully, pdFALSE otherwise.
 */
BaseType_t xQueueReceiveFromISR(QueueHandle_t handler, void *const data, BaseType_t *const pxHigherPriorityTaskWoken);

#ifdef  __cplusplus
}

#endif

#endif  /* _PORT_QUEUE_CUSTOM_H */
