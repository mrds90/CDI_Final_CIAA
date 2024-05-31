/**
 * @file osal_semaphore.c
 * @author Marcos Dominguez
 *
 * @brief Os abstraction layer for semaphores
 *
 * @version 0.1
 * @date 2023-12-11
 */


/*========= [DEPENDENCIES] =====================================================*/

#include "osal_semaphore.h"

/*========= [PRIVATE MACROS AND CONSTANTS] =====================================*/

/*========= [PRIVATE DATA TYPES] ===============================================*/

/*========= [TASK DECLARATIONS] ================================================*/

/*========= [PRIVATE FUNCTION DECLARATIONS] ====================================*/

/*========= [INTERRUPT FUNCTION DECLARATIONS] ==================================*/

/*========= [LOCAL VARIABLES] ==================================================*/

/*========= [STATE FUNCTION POINTERS] ==========================================*/

/*========= [PUBLIC FUNCTION IMPLEMENTATION] ===================================*/

bool_t OSAL_SEMAPHORE_LoadStruct(osal_semaphore_t *semaphore_ptr, osal_semaphore_holder_t *holder_ptr, uint16_t max_count, uint16_t initial_count, bool_t mutex_type) {
    bool_t ret = FALSE;
    if ((semaphore_ptr != NULL) && (holder_ptr != NULL)) {
        if (mutex_type == TRUE) {
            max_count = 0;
            initial_count = 0;
        }
        semaphore_ptr->holder_ptr = holder_ptr;
        semaphore_ptr->initial_count = initial_count;
        semaphore_ptr->max_count = max_count;
        semaphore_ptr->mutex_type = mutex_type;
        ret = TRUE;
    }
    return ret;
}

bool_t OSAL_SEMAPHORE_Create(osal_semaphore_t *semaphore_ptr) {
    bool_t ret = FALSE;
    if ((semaphore_ptr != NULL) && (semaphore_ptr->holder_ptr != NULL)) {
        if (semaphore_ptr->mutex_type == TRUE) {
            semaphore_ptr->handler = PORT_SEMAPHORE_CreateMutex(semaphore_ptr->holder_ptr);
        }
        else if (semaphore_ptr->max_count > 1) {
            semaphore_ptr->handler = PORT_SEMAPHORE_CreateCounting(semaphore_ptr->max_count, semaphore_ptr->initial_count, semaphore_ptr->holder_ptr);
        }
        else {
            semaphore_ptr->handler = PORT_SEMAPHORE_CreateBinary(semaphore_ptr->holder_ptr);
        }
        if (semaphore_ptr->handler != NULL) {
            ret = TRUE;
        }
    }
    return ret;
}

bool_t OSAL_SEMAPHORE_Give(osal_semaphore_t *semaphore_ptr) {
    bool_t ret = FALSE;
    if (semaphore_ptr != NULL) {
        if (semaphore_ptr->handler != NULL) {
            ret = PORT_SEMAPHORE_Give(semaphore_ptr->handler);
        }
    }
    return ret;
}

bool_t OSAL_SEMAPHORE_Take(osal_semaphore_t *semaphore_ptr, osal_tick_t wait_time) {
    bool_t ret = FALSE;
    if (semaphore_ptr != NULL) {
        if (semaphore_ptr->handler != NULL) {
            ret = PORT_SEMAPHORE_Take(semaphore_ptr->handler, wait_time);
        }
    }
    return ret;
}

bool_t OSAL_SEMAPHORE_GiveFromISR(osal_semaphore_t *semaphore_ptr) {
    bool_t ret = FALSE;
    if (semaphore_ptr != NULL) {
        if (semaphore_ptr->handler != NULL) {
            ret = PORT_SEMAPHORE_GiveFromISR(semaphore_ptr->handler);
        }
    }
    return ret;
}

bool_t OSAL_SEMAPHORE_TakeFromISR(osal_semaphore_t *semaphore_ptr) {
    bool_t ret = FALSE;
    if (semaphore_ptr != NULL) {
        if (semaphore_ptr->handler != NULL) {
            ret = PORT_SEMAPHORE_TakeFromISR(semaphore_ptr->handler);
        }
    }
    return ret;
}

/*========= [PRIVATE FUNCTION IMPLEMENTATION] ==================================*/

/*========= [INTERRUPT FUNCTION IMPLEMENTATION] ================================*/

void OSAL_SEMAPHORE_GiveCallback(void *context) {
    osal_semaphore_t *semaphore =  (osal_semaphore_t *)context;
    OSAL_PORT_YIELD(PORT_SEMAPHORE_GiveFromISR(semaphore->handler));
}
