/**
 * @file osal_semaphore.h
 * @author Marcos Dominguez
 *
 * @brief OS abstraction layer for semaphores
 *
 * @version 0.1
 * @date 2023-12-11
 */

#ifndef _OSAL_SEMAPHORE_H
#define _OSAL_SEMAPHORE_H

#ifdef  __cplusplus
extern "C" {
#endif

/*========= [DEPENDENCIES] =====================================================*/

#include <stdint.h>

/*========= [PUBLIC MACRO AND CONSTANTS] =======================================*/

#if (OS_USED == OS_FREERTOS)
#include "port_semaphore_freertos.h"
#endif

#include "osal_global.h"

/*========= [PUBLIC DATA TYPE] =================================================*/

/**
 * @brief Structure to hold information about a semaphore.
 */
typedef struct {
    osal_semaphore_handle_t handler; /**< Semaphore handler. */
    osal_semaphore_holder_t *holder_ptr; /**< Pointer to the semaphore holder. */
    uint16_t max_count; /**< Maximum count of the semaphore. */
    uint16_t initial_count; /**< Initial count of the semaphore. */
    bool_t mutex_type; /**< Flag indicating whether the semaphore is a mutex. */
} osal_semaphore_t;

/*========= [PUBLIC FUNCTION DECLARATIONS] =====================================*/

/**
 * @brief Load the semaphore struct with information about its configuration.
 *
 * @param semaphore_ptr     Pointer to structure that hold the information about the semaphore.
 * @param holder_ptr        Pointer to the memory region where the semaphore is held.
 * @param max_count         Maximum count of the semaphore.
 * @param initial_count     Initial count of the semaphore.
 * @param mutex_type        Flag indicating whether the semaphore is a mutex.
 * @return bool_t           TRUE: Operation success - FALSE: Operation fail.
 */
bool_t OSAL_SEMAPHORE_LoadStruct(osal_semaphore_t *semaphore_ptr, osal_semaphore_holder_t *holder_ptr, uint16_t max_count, uint16_t initial_count, bool_t mutex_type);

/**
 * @brief Create a semaphore.
 *
 * @param semaphore_ptr     Pointer to structure that hold the information about the semaphore.
 * @return bool_t           TRUE: Creation success - FALSE: Creation fail.
 */
bool_t OSAL_SEMAPHORE_Create(osal_semaphore_t *semaphore_ptr);

/**
 * @brief Give (release) the semaphore.
 *
 * @param semaphore_ptr     Pointer to structure that hold the information about the semaphore.
 * @return bool_t           TRUE: Operation success - FALSE: Operation fail.
 */
bool_t OSAL_SEMAPHORE_Give(osal_semaphore_t *semaphore_ptr);

/**
 * @brief Take (acquire) the semaphore with optional waiting time.
 *
 * @param semaphore_ptr     Pointer to structure that hold the information about the semaphore.
 * @param wait_time         Maximum time to wait for the semaphore.
 * @return bool_t           TRUE: Operation success - FALSE: Operation fail.
 */
bool_t OSAL_SEMAPHORE_Take(osal_semaphore_t *semaphore_ptr, osal_tick_t wait_time);

/**
 * @brief Give (release) the semaphore from an ISR.
 *
 * @param semaphore_ptr     Pointer to structure that hold the information about the semaphore.
 * @return bool_t           TRUE: Operation success - FALSE: Operation fail.
 */
bool_t OSAL_SEMAPHORE_GiveFromISR(osal_semaphore_t *semaphore_ptr);

/**
 * @brief Take (acquire) the semaphore from an ISR.
 *
 * @param semaphore_ptr     Pointer to structure that hold the information about the semaphore.
 * @return bool_t           TRUE: Operation success - FALSE: Operation fail.
 */
bool_t OSAL_SEMAPHORE_TakeFromISR(osal_semaphore_t *semaphore_ptr);

/**
 * @brief Generic Callback for giving a semaphore in a ISR
 *
 * @param context pointer to the Semaphore.
 */
void OSAL_SEMAPHORE_GiveCallback(void *context);

#ifdef  __cplusplus
}

#endif

#endif  /* _OSAL_SEMAPHORE_H */
