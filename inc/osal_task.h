/**
 * @file osal_task.h
 * @author Marcos Dominguez
 *
 * @brief OS abstraction for task APIs.
 *
 * @version 0.1
 * @date 2023-11-24
 */

#ifndef _OSAL_TASK_H
#define _OSAL_TASK_H

#ifdef  __cplusplus
extern "C" {
#endif

/*========= [DEPENDENCIES] =====================================================*/

#if (OS_USED == OS_FREERTOS)
#include "port_task_freertos.h"
#endif

#include "osal_global.h"
#include "task_manager.h"
/// \cond
#include "data_types.h"
/// \endcond


/*========= [PUBLIC MACRO AND CONSTANTS] =======================================*/

/*========= [PUBLIC DATA TYPE] =================================================*/

/**
 * @brief Enumeration of task priorities.
 */
typedef enum {
    TASK_PRIORITY_IDLE,
    TASK_PRIORITY_VERY_LOW,
    TASK_PRIORITY_LOW,
    TASK_PRIORITY_NORMAL,
    TASK_PRIORITY_HIGH,
    TASK_PRIORITY_VERY_HIGH,
    TASK_PRIORITY_HIGHEST,

    TASK_PRIORITY_QTY,
} osal_task_priority_t;

/**
 * @brief Structure to hold information about a task.
 */
typedef struct {
    char name[OSAL_MAX_TASK_NAME_LEN];      /**< Task name. */
    osal_task_handler_t task_handler;       /**< Task handler. */
    osal_stack_holder_t *stack_ptr;         /**< Pointer to the memory region for the task stack. */
    osal_task_holder_t *task_hold_ptr;      /**< Pointer where task struct will be held. */
    uint16_t size;                          /**< Size of the stack. */
} osal_task_t;

/*========= [PUBLIC FUNCTION DECLARATIONS] =====================================*/

/**
 * @brief Load to the task struct with the information of the memory region where information will be statically storage.
 *
 * @param task_ptr          Task handler
 * @param stack_ptr         Pointer to the memory region for the task stack
 * @param hold_task_ptr     Pointer where task struct will is hold.
 * @param size              Size of the stack
 * @return bool_t           TRUE: Operation success - FALSE: Operation fail.
 */
bool_t OSAL_TASK_LoadStruct(osal_task_t *task_ptr, osal_stack_holder_t *stack_ptr, osal_task_holder_t *hold_task_ptr, uint16_t size_t);

/**
 * @brief Create task for be scheduled.
 *
 * @param task_ptr          Task handler.
 * @param function          Callback fuction to be called.
 * @param context           Argument for the task function.
 * @param priority          Priority of the task.
 * @return bool_t           TRUE: Creation success - FALSE: Creation fail.
 */
bool_t OSAL_TASK_Create(osal_task_t *task_ptr, OSAL_TASK_Callback_t function, void *context, osal_task_priority_t priority);

#ifdef  __cplusplus
}

#endif

#endif  /* _INC_TEMPLATE_H */
