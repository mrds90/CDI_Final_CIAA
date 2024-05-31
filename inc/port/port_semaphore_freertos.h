/**
 * @file port_semaphore_freertos.h
 * @author Marcos Dominguez
 *
 * @brief Port for FreeRTOS semaphore API.
 *
 * @version 0.1
 * @date 2023-12-11
 */

#ifndef _PORT_SEMAPHOR_EFREERTOS_H
#define _PORT_SEMAPHOR_EFREERTOS_H

#ifdef __cplusplus
extern "C" {
#endif

/*========= [DEPENDENCIES] =====================================================*/

#include "port_freertos.h"
#ifndef TEST
#include "semphr.h"
#else
#include "FreeRTOS_semphr_simulated.h"
#endif

/// \cond
#include "data_types.h"
/// \endcond

/*========= [PUBLIC MACRO AND CONSTANTS] =======================================*/

/**
 * @brief Create a binary semaphore.
 *
 * This function creates a binary semaphore with the provided holder.
 *
 * @param semaphore_holder The pointer to the semaphore holder.
 *
 * @return The handle of the created semaphore.
 */
#define PORT_SEMAPHORE_CreateBinary(semaphore_holder) xSemaphoreCreateBinaryStatic((osal_semaphore_holder_t *)semaphore_holder)

/**
 * @brief Create a mutex semaphore.
 *
 * This function creates a mutex semaphore with the provided holder.
 *
 * @param semaphore_holder The pointer to the semaphore holder.
 *
 * @return The handle of the created semaphore.
 */
#define PORT_SEMAPHORE_CreateMutex(semaphore_holder) xSemaphoreCreateMutexStatic((osal_semaphore_holder_t *)semaphore_holder)

/**
 * @brief Create a counting semaphore.
 *
 * This function creates a counting semaphore with the provided parameters and holder.
 *
 * @param max_count         The maximum count value.
 * @param initial_count     The initial count value.
 * @param semaphore_holder  The pointer to the semaphore holder.
 *
 * @return The handle of the created semaphore.
 */
#define PORT_SEMAPHORE_CreateCounting(max_count, initial_count, semaphore_holder) xSemaphoreCreateCountingStatic(max_count, initial_count, (osal_semaphore_holder_t *)semaphore_holder)

/*========= [PUBLIC DATA TYPE] =================================================*/

typedef SemaphoreHandle_t osal_semaphore_handle_t; /**< Type definition for the FreeRTOS semaphore handle. */

typedef StaticSemaphore_t osal_semaphore_holder_t; /**< Type definition for the static semaphore holder. */

/*========= [PUBLIC FUNCTION DECLARATIONS] =====================================*/


/**
 * @brief Give a semaphore.
 *
 * This function gives the semaphore.
 *
 * @param handler The handle of the semaphore.
 *
 * @return TRUE if the operation is successful, FALSE otherwise.
 */
bool_t PORT_SEMAPHORE_Give(osal_semaphore_handle_t handler);

/**
 * @brief Take a semaphore with a specified wait time.
 *
 * This function takes the semaphore with the specified wait time.
 *
 * @param handler   The handle of the semaphore.
 * @param wait_time The wait time in ticks.
 *
 * @return TRUE if the operation is successful, FALSE otherwise.
 */
bool_t PORT_SEMAPHORE_Take(osal_semaphore_handle_t handler, port_tick_t wait_time);

/**
 * @brief Give a semaphore from an ISR.
 *
 * This function gives the semaphore from an ISR.
 *
 * @param handler The handle of the semaphore.
 *
 * @return TRUE if the a task with highest priority has woken, FALSE otherwise.
 */
bool_t PORT_SEMAPHORE_GiveFromISR(osal_semaphore_handle_t handler);

/**
 * @brief Take a semaphore from an ISR.
 *
 * This function takes the semaphore from an ISR.
 *
 * @param handler The handle of the semaphore.
 *
 * @return TRUE if the a task with highest priority has woken, FALSE otherwise.
 */
bool_t PORT_SEMAPHORE_TakeFromISR(osal_semaphore_handle_t handler);

#ifdef __cplusplus
}

#endif

#endif /* _PORT_SEMAPHOR_EFREERTOS_H */
