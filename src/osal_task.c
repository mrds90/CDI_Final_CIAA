/**
 * @file osal_task.c
 * @author Marcos Dominguez
 *
 * @brief OS abstraction for task APIs.
 *
 * @version 0.1
 * @date 2023-11-24
 */

/*========= [DEPENDENCIES] =====================================================*/

#include "osal_task.h"

/*========= [PRIVATE MACROS AND CONSTANTS] =====================================*/

/*========= [PRIVATE DATA TYPES] ===============================================*/

/*========= [TASK DECLARATIONS] ================================================*/

/*========= [PRIVATE FUNCTION DECLARATIONS] ====================================*/

/*========= [INTERRUPT FUNCTION DECLARATIONS] ==================================*/

/*========= [LOCAL VARIABLES] ==================================================*/

/*========= [STATE FUNCTION POINTERS] ==========================================*/

/*========= [PUBLIC FUNCTION IMPLEMENTATION] ===================================*/

bool_t OSAL_TASK_LoadStruct(osal_task_t *task_ptr, osal_stack_holder_t *stack_ptr, osal_task_holder_t *hold_task_ptr, uint16_t size) {
    bool_t ret = FALSE;
    if (task_ptr != NULL) {
        if (stack_ptr != NULL) {
            if (hold_task_ptr != NULL) {
                if (size != 0) {
                    ret = TRUE;
                    task_ptr->stack_ptr = stack_ptr;
                    task_ptr->task_hold_ptr = hold_task_ptr;
                    task_ptr->size = size;
                }
            }
        }
    }
    return ret;
}

bool_t OSAL_TASK_Create(osal_task_t *task_ptr, OSAL_TASK_Callback_t function, void *context, osal_task_priority_t priority) {
    bool_t ret = FALSE;
    if ((task_ptr != NULL) && (function != NULL)) {
        if (priority >= TASK_PRIORITY_IDLE && priority < TASK_PRIORITY_QTY) {
            if (task_ptr->size > 0) {
                task_ptr->task_handler = PORT_TASK_CreateStaticTask(function,
                                                                    task_ptr->name,
                                                                    task_ptr->size,
                                                                    context,
                                                                    priority,
                                                                    task_ptr->stack_ptr,
                                                                    task_ptr->task_hold_ptr);
                if (task_ptr->task_handler != NULL) {
                    ret = TRUE;
                }
            }
        }
    }
    return ret;
}

/*========= [PRIVATE FUNCTION IMPLEMENTATION] ==================================*/

/*========= [INTERRUPT FUNCTION IMPLEMENTATION] ================================*/
