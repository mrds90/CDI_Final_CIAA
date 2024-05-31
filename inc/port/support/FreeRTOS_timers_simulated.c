/**
 * @file FreeRTOS_timers_simulated.c
 * @author Marcos Dominguez
 *
 * @brief Port for FreeRTOS timers API.
 *
 * @version 0.1
 * @date 2023-12-15
 */

/*========= [DEPENDENCIES] =====================================================*/

#include "FreeRTOS_timers_simulated.h"

/*========= [PRIVATE MACROS AND CONSTANTS] =====================================*/

/*========= [PRIVATE DATA TYPES] ===============================================*/

/*========= [TASK DECLARATIONS] ================================================*/

/*========= [PRIVATE FUNCTION DECLARATIONS] ====================================*/

/*========= [INTERRUPT FUNCTION DECLARATIONS] ==================================*/

/*========= [LOCAL VARIABLES] ==================================================*/

/*========= [STATE FUNCTION POINTERS] ==========================================*/

/*========= [PUBLIC FUNCTION IMPLEMENTATION] ===================================*/

TimerHandle_t __attribute__((weak))  xTimerCreateStatic(const char *const pcTimerName, const TickType_t xTimerPeriodInTicks, const BaseType_t xAutoReload, void *const pvTimerID, TimerCallbackFunction_t pxCallbackFunction, StaticTimer_t *pxTimerBuffer) {
    return pxTimerBuffer;
}

BaseType_t __attribute__((weak)) xTimerReset(TimerHandle_t xTimer, TickType_t xTicksToWait) {
    return pdTRUE;
}

BaseType_t __attribute__((weak)) xTimerStart(TimerHandle_t xTimer, TickType_t xTicksToWait) {
    return pdTRUE;
}

BaseType_t __attribute__((weak)) xTimerStop(TimerHandle_t xTimer, TickType_t xTicksToWait) {
    return pdTRUE;
}

/*========= [PRIVATE FUNCTION IMPLEMENTATION] ==================================*/

/*========= [INTERRUPT FUNCTION IMPLEMENTATION] ================================*/
