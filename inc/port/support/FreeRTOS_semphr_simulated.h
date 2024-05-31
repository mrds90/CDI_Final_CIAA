/**
 * @file FreeRTOS_semphr_simulated.h
 * @author Marcos Dominguez
 *
 * @brief Port for FreeRTOS Semaphore API (Simulated).
 *
 * This module provides a simulated implementation of FreeRTOS semaphore
 * functionality. It includes the creation of binary, mutex, and counting
 * semaphores, along with operations like giving and taking semaphores.
 *
 * @version 0.1
 * @date 2023-12-11
 */

#ifndef _PORT_SEMAPHORE_CUSTOM_H
#define _PORT_SEMAPHORE_CUSTOM_H

#ifdef  __cplusplus
extern "C" {
#endif

/*========= [DEPENDENCIES] =====================================================*/

/// \cond
#include "data_types.h"
/// \endcond
#include "FreeRTOS_simulated.h"

/*========= [PUBLIC MACRO AND CONSTANTS] =======================================*/

#define MAX_CONT_SEMPHR     0xFF

/*========= [PUBLIC DATA TYPE] =================================================*/

/**
 * @brief Structure representing a static semaphore and its buffer.
 */
typedef struct {
    uint8_t buffer[MAX_CONT_SEMPHR + 1]; /**< Buffer for the semaphore. */
    StaticQueue_t holder; /**< Holder structure for the semaphore. */
} StaticSemaphore_t;

/**
 * @brief Handle for a semaphore.
 */
typedef StaticSemaphore_t *SemaphoreHandle_t;

/*========= [PUBLIC FUNCTION DECLARATIONS] =====================================*/

/**
 * @brief Creates a binary semaphore.
 *
 * @param pxSemaphoreBuffer Pointer to a static semaphore buffer.
 * @return A handle to the created semaphore, or NULL if creation fails.
 */
SemaphoreHandle_t xSemaphoreCreateBinaryStatic(StaticSemaphore_t *pxSemaphoreBuffer);

/**
 * @brief Creates a mutex semaphore.
 *
 * @param pxSemaphoreBuffer Pointer to a static semaphore buffer.
 * @return A handle to the created semaphore, or NULL if creation fails.
 */
SemaphoreHandle_t xSemaphoreCreateMutexStatic(StaticSemaphore_t *pxSemaphoreBuffer);

/**
 * @brief Creates a counting semaphore.
 *
 * @param uxMaxCount The maximum count value of the semaphore.
 * @param uxInitialCount The initial count value of the semaphore.
 * @param pxSemaphoreBuffer Pointer to a static semaphore buffer.
 * @return A handle to the created semaphore, or NULL if creation fails.
 */
SemaphoreHandle_t xSemaphoreCreateCountingStatic(UBaseType_t uxMaxCount, UBaseType_t uxInitialCount, StaticSemaphore_t *pxSemaphoreBuffer);

/**
 * @brief Gives a semaphore.
 *
 * @param xSemaphore The handle of the semaphore.
 * @return pdTRUE if the semaphore is given successfully, pdFALSE otherwise.
 */
BaseType_t xSemaphoreGive(SemaphoreHandle_t xSemaphore);

/**
 * @brief Takes a semaphore.
 *
 * @param xSemaphore The handle of the semaphore.
 * @param wait_time The maximum time to wait for the semaphore.
 * @return pdTRUE if the semaphore is taken successfully, pdFALSE otherwise.
 */
BaseType_t xSemaphoreTake(SemaphoreHandle_t xSemaphore, TickType_t wait_time);

/**
 * @brief Gives a semaphore from an ISR.
 *
 * @param handler The handle of the semaphore.
 * @param HigherPriorityTaskWoken A pointer to a variable that will be set to pdTRUE if a task was woken.
 * @return pdTRUE if the semaphore is given successfully, pdFALSE otherwise.
 */
BaseType_t xSemaphoreGiveFromISR(SemaphoreHandle_t handler, BaseType_t *HigherPriorityTaskWoken);

/**
 * @brief Takes a semaphore from an ISR.
 *
 * @param handler The handle of the semaphore.
 * @param HigherPriorityTaskWoken A pointer to a variable that will be set to pdTRUE if a task was woken.
 * @return pdTRUE if the semaphore is taken successfully, pdFALSE otherwise.
 */
BaseType_t xSemaphoreTakeFromISR(SemaphoreHandle_t handler, BaseType_t *HigherPriorityTaskWoken);

#ifdef  __cplusplus
}

#endif

#endif  /* _PORT_SEMAPHORE_CUSTOM_H */
