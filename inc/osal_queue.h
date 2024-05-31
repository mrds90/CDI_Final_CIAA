/**
 * @file osal_queue.h
 * @author Marcos Dominguez
 *
 * @brief OS abstraction for queue APIs.
 *
 * @version 0.1
 * @date 2023-11-24
 */

#ifndef _OSAL_QUEUE_H
#define _OSAL_QUEUE_H

#ifdef  __cplusplus
extern "C" {
#endif

/*========= [DEPENDENCIES] =====================================================*/

#include "osal_global.h"
#if (OS_USED == OS_FREERTOS)
#include "port_queue_freertos.h"
#elif (OS_USED == OS_CUSTOM)
#include "port_queue_custom.h"
#endif

/// \cond
#include "data_types.h"
/// \endcond

/*========= [PUBLIC MACRO AND CONSTANTS] =======================================*/

/**
 * @brief Calculate the required stack size for a queue based on data size and queue length.
 */
#define OSAL_QUEUE_StackSize(data_size, queue_length)  (data_size * queue_length)

/*========= [PUBLIC DATA TYPE] =================================================*/

/**
 * @brief Structure to hold information about a queue.
 */
typedef struct {
    osal_queue_handle_t handler;        /**< Queue handler. */
    osal_queue_holder_t *holder_ptr;    /**< Pointer to the memory region where the semaphore is held.*/
    uint8_t *queue_storage;             /**< Pointer to the buffer where queued elements will be stored. */
    uint32_t data_size;                 /**< Size of each element in the queue. */
    uint16_t queue_length;              /**< Maximum number of elements the queue can hold. */
} osal_queue_t;

/*========= [PUBLIC FUNCTION DECLARATIONS] =====================================*/

/**
 * @brief Load the queue struct with information about its configuration.
 *
 * @param queue_ptr         Pointer to structure that hold the information about the queue.
 * @param holder_ptr        Pointer to the memory region where the semaphore is held.
 * @param queue_storage     Pointer to the buffer where queued elements will be stored.
 * @param data_size         Size of each element in the queue.
 * @param queue_length      Maximum number of elements the queue can hold.
 * @return bool_t           TRUE: Operation success - FALSE: Operation fail.
 */
bool_t OSAL_QUEUE_LoadStruct(osal_queue_t *queue_ptr, osal_queue_holder_t *holder_ptr, uint8_t *queue_storage, uint32_t data_size, uint16_t queue_length);

/**
 * @brief Create a queue.
 *
 * @param queue_ptr         Pointer to structure that hold the information about the queue.
 * @return bool_t           TRUE: Creation success - FALSE: Creation fail.
 */
bool_t OSAL_QUEUE_Create(osal_queue_t *queue_ptr);

/**
 * @brief Send data to the queue.
 *
 * @param queue_ptr         Pointer to structure that hold the information about the queue.
 * @param data              Pointer to the data to be sent.
 * @param wait_time         Maximum time to wait for the queue to have space.
 * @return bool_t           TRUE: Operation success - FALSE: Operation fail.
 */
bool_t OSAL_QUEUE_Send(osal_queue_t *queue_ptr, void *data, osal_tick_t wait_time);

/**
 * @brief Receive data from the queue.
 *
 * @param queue_ptr         Pointer to structure that hold the information about the queue.
 * @param data              Pointer to store the received data.
 * @param wait_time         Maximum time to wait for the queue to have data.
 * @return bool_t           TRUE: Operation success - FALSE: Operation fail.
 */
bool_t OSAL_QUEUE_Receive(osal_queue_t *queue_ptr, void *data, osal_tick_t wait_time);

/**
 * @brief Send data to the queue from ISR.
 *
 * @param queue_ptr         Pointer to structure that hold the information about the queue.
 * @param data              Pointer to the data to be sent.
 * @param yield_need              Pointer where the function report if a change of context is needed.
 *
 * @return bool_t           TRUE: Operation success - FALSE: Operation fail.
 */
bool_t OSAL_QUEUE_SendFromISR(osal_queue_t *queue_ptr, void *data, bool_t *yield_need);


/**
 * @brief Receive data from the queue from ISR.
 *
 * @param queue_ptr         Pointer to structure that hold the information about the queue.
 * @param data              Pointer to store the received data.
 * @param yield_need              Pointer where the function report if a change of context is needed.
 * @return bool_t           TRUE: Operation success - FALSE: Operation fail.
 */
bool_t OSAL_QUEUE_ReceiveFromISR(osal_queue_t *queue_ptr, void *const data, bool_t *yield_need);

#ifdef  __cplusplus
}
#endif

#endif  /* _OSAL_QUEUE_H */
