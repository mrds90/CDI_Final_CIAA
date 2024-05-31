/**
 * @file port_queue_freertos.h
 * @author Marcos Dominguez
 *
 * @brief Port for FreeRTOS queue API.
 *
 * @version 0.1
 * @date 2023-11-24
 */

#ifndef _PORT_QUEUE_FREERTOS_H
#define _PORT_QUEUE_FREERTOS_H

#ifdef __cplusplus
extern "C" {
#endif

/*========= [DEPENDENCIES] =====================================================*/

#include "port_freertos.h"
#ifndef TEST
#include "queue.h"
#else
#include "FreeRTOS_queue_simulated.h"
#endif
/// \cond
#include "data_types.h"
/// \endcond

/*========= [PUBLIC MACRO AND CONSTANTS] =======================================*/

/*========= [PUBLIC DATA TYPE] =================================================*/

typedef QueueHandle_t osal_queue_handle_t; /**< Type definition for the FreeRTOS queue handle. */

typedef StaticQueue_t osal_queue_holder_t; /**< Type definition for the static queue holder. */

/*========= [PUBLIC FUNCTION DECLARATIONS] =====================================*/

/**
 * @brief Create a FreeRTOS queue.
 *
 * This function creates a FreeRTOS queue with the provided parameters and holder.
 *
 * @param queue_length  The length of the queue.
 * @param data_size     The size of each item in the queue.
 * @param queue_storage The storage for the queue.
 * @param queue_struct  The pointer to the queue holder.
 *
 * @return The handle of the created queue.
 */
osal_queue_handle_t PORT_QUEUE_Create(uint16_t queue_length, uint32_t data_size, uint8_t *queue_storage, osal_queue_holder_t *queue_struct);

/**
 * @brief Send data to a queue.
 *
 * This function sends data to the queue with a specified wait time.
 *
 * @param handler   The handle of the queue.
 * @param data      The pointer to the data to be sent.
 * @param wait_time The wait time in ticks.
 *
 * @return TRUE if the operation is successful, FALSE otherwise.
 */
bool_t PORT_QUEUE_Send(osal_queue_handle_t handler, void *data, port_tick_t wait_time);

/**
 * @brief Receive data from a queue.
 *
 * This function receives data from the queue with a specified wait time.
 *
 * @param handler   The handle of the queue.
 * @param data      The pointer to store the received data.
 * @param wait_time The wait time in ticks.
 *
 * @return TRUE if the operation is successful, FALSE otherwise.
 */
bool_t PORT_QUEUE_Receive(osal_queue_handle_t handler, void *data, port_tick_t wait_time);

/**
 * @brief Send data to a queue from an ISR.
 *
 * This function sends data to the queue from an ISR.
 *
 * @param handler    The handle of the queue.
 * @param data       The pointer to the data to be sent.
 * @param yield_need The pointer to a flag indicating if a context switch is needed.
 *
 * @return TRUE if the operation is successful, FALSE otherwise.
 */
bool_t PORT_QUEUE_SendFromISR(osal_queue_handle_t handler, void *data, bool_t *yield_need);

/**
 * @brief Receive data from a queue from an ISR.
 *
 * This function receives data from the queue from an ISR.
 *
 * @param handler    The handle of the queue.
 * @param data       The pointer to store the received data.
 * @param yield_need The pointer to a flag indicating if a context switch is needed.
 *
 * @return TRUE if the operation is successful, FALSE otherwise.
 */
bool_t PORT_QUEUE_ReceiveFromISR(osal_queue_handle_t handler, void *data, bool_t *yield_need);

#ifdef __cplusplus
}

#endif

#endif /* _PORT_QUEUE_FREERTOS_H */
