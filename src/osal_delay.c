/**
 * @file osal_delay.c
 * @author Marcos Dominguez
 *
 * @brief Manage not blocking delays
 *
 * @version 0.1
 * @date 2024-03-15
 */

/*========= [DEPENDENCIES] =====================================================*/

#include "osal_delay.h"

/*========= [PRIVATE MACROS AND CONSTANTS] =====================================*/

#define TICK_TYPE_MAX   ((1ULL << (sizeof(osal_tick_t) * 8)) - 1)

/*========= [PRIVATE DATA TYPES] ===============================================*/

/*========= [TASK DECLARATIONS] ================================================*/

/*========= [PRIVATE FUNCTION DECLARATIONS] ====================================*/

/*========= [CALLBACK FUNCTION DECLARATIONS] ===================================*/

/*========= [INTERRUPT FUNCTION DECLARATIONS] ==================================*/

/*========= [LOCAL VARIABLES] ==================================================*/

/*========= [STATE FUNCTION POINTERS] ==========================================*/

/*========= [PUBLIC FUNCTION IMPLEMENTATION] ===================================*/

void OSAL_DELAY_Reset(osal_delay_t *delay) {
    delay->time = 0;
    delay->initial_time = 0;
    delay->running = FALSE;
}

void OSAL_DELAY_Config(osal_delay_t *delay, osal_tick_t time) {
    delay->time = time;
    delay->running = FALSE;
}

bool_t OSAL_DELAY_Check(osal_delay_t *delay) {
    bool_t ready = FALSE;
    if (!delay->running) { // init the delay if wasn't asked before.
        delay->running = TRUE;
        delay->initial_time = OSAL_TASK_GetTickCount();
    }
    else {
        osal_tick_t time_count = OSAL_TASK_GetTickCount() - delay->initial_time;
        if (time_count < 0) {
            time_count = OSAL_TASK_GetTickCount() + (TICK_TYPE_MAX) -delay->initial_time;
        }
        if ((time_count) >= delay->time) {
            ready = TRUE;
            delay->running = FALSE;
        }
    }

    return ready;
}

/*========= [PRIVATE FUNCTION IMPLEMENTATION] ==================================*/

/*========= [CALLBACK FUNCTION IMPLEMENTATION] =================================*/

/*========= [INTERRUPT FUNCTION IMPLEMENTATION] ================================*/
