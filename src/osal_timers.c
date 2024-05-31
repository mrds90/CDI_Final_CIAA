/**
 * @file osal_timers.c
 * @author Marcos Dominguez
 *
 * @brief OS abstraction for timers APIs.
 *
 * @version 0.1
 * @date 2023-12-15
 */

/*========= [DEPENDENCIES] =====================================================*/

#include "osal_timers.h"

/*========= [PRIVATE MACROS AND CONSTANTS] =====================================*/

/*========= [PRIVATE DATA TYPES] ===============================================*/

/*========= [TASK DECLARATIONS] ================================================*/

/*========= [PRIVATE FUNCTION DECLARATIONS] ====================================*/

/*========= [INTERRUPT FUNCTION DECLARATIONS] ==================================*/

/*========= [LOCAL VARIABLES] ==================================================*/

/*========= [STATE FUNCTION POINTERS] ==========================================*/

/*========= [PUBLIC FUNCTION IMPLEMENTATION] ===================================*/

bool_t OSAL_TIMERS_LoadStruct(osal_timer_t *timer, char *name, osal_tick_t time, bool_t repeat, UtilsCallback_t Callback, void *context, osal_timer_holder_t *holder_ptr) {
    bool_t ret = FALSE;
    if ((timer != NULL) && (holder_ptr != NULL) && (Callback != NULL)) {
        timer->callback.callback = Callback;
        timer->callback.context = context;
        timer->holder_ptr = holder_ptr;
        timer->name = name;
        timer->repeat = repeat,
        timer->time = time;
        ret =  TRUE;
    }
    return ret;
}

bool_t OSAL_TIMERS_Create(osal_timer_t *timer, timer_index_t index) {
    bool_t ret = FALSE;
    if ((timer != NULL) && timer->callback.callback != NULL) {
        timer->handler = PORT_TIMER_Create(timer->name, timer->time, timer->repeat, &timer->callback, timer->holder_ptr, index);
        if (timer->handler != NULL) {
            ret = TRUE;
        }
    }
    return ret;
}

/*========= [PRIVATE FUNCTION IMPLEMENTATION] ==================================*/

/*========= [INTERRUPT FUNCTION IMPLEMENTATION] ================================*/
