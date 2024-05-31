/**
 * @file port_timers_freertos.c
 * @author Marcos Dominguez
 *
 * @brief Port for FreeRTOS timers API.
 *
 * @version 0.1
 * @date 2023-12-15
 */

/*========= [DEPENDENCIES] =====================================================*/

#if (OS_USED == OS_FREERTOS)
#include "port_timers_freertos.h"
#if (configSUPPORT_STATIC_ALLOCATION == 1)
#include "port_task_freertos.h"
#endif

/*========= [PRIVATE MACROS AND CONSTANTS] =====================================*/

/*========= [PRIVATE DATA TYPES] ===============================================*/

/**
 * @brief Structure to hold information about a FreeRTOS timer.
 *
 */
typedef struct {
    TimerHandle_t handler;       /**< FreeRTOS timer handle. */
    UtilsCallback_t callback;    /**< Callback function to be executed when the timer expires. */
    void *context;               /**< Context to be passed to the callback function. */
} port_timer_t;

/*========= [TASK DECLARATIONS] ================================================*/

/*========= [PRIVATE FUNCTION DECLARATIONS] ====================================*/

/*========= [INTERRUPT FUNCTION DECLARATIONS] ==================================*/

/**
 * @brief Callback function for FreeRTOS timers.
 *
 * This function is called when a FreeRTOS timer expires. It finds the corresponding timer in the timers_active array
 * and executes its callback function.
 *
 * @param xTimer The handle of the timer that expired.
 */
void TimerCallback(TimerHandle_t xTimer);

/*========= [LOCAL VARIABLES] ==================================================*/

port_timer_t timers_active[TIMER_INDEX_QTY] = {[0 ... (TIMER_INDEX_QTY - 1)] = {.callback = NULL}}; /**< Array to store information about active FreeRTOS timers. */

/*========= [STATE FUNCTION POINTERS] ==========================================*/

/*========= [PUBLIC FUNCTION IMPLEMENTATION] ===================================*/

port_timers_handler_t PORT_TIMER_Create(const char *name, port_tick_t time, bool_t repeat, PortSWTimerCallback_t *Callback, port_timers_holder_t *holder_ptr, timer_index_t index) {
    port_timers_handler_t handler = NULL;
    if (index < TIMER_INDEX_QTY) {
        timers_active[index].callback = Callback->callback;
        timers_active[index].context = Callback->context;
        timers_active[index].handler = xTimerCreateStatic(name, time, repeat & pdTRUE, NULL, TimerCallback, holder_ptr);
        handler = timers_active[index].handler;
    }
    return handler;
}

// #if (configSUPPORT_STATIC_ALLOCATION == 1)
// /**
//  * @brief Provide memory for the RTOS daemon/time task.
//  *
//  * If configUSE_STATIC_ALLOCATION is set to 1, this function provides memory for the RTOS daemon/time task.
//  *
//  * @param ppxTimerTaskTCBBuffer   The buffer to store the Timer task's TCB.
//  * @param ppxTimerTaskStackBuffer The buffer to store the Timer task's stack.
//  * @param pulTimerTaskStackSize   The size of the Timer task's stack.
//  */
// void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer,
//                                     StackType_t **ppxTimerTaskStackBuffer,
//                                     uint32_t *pulTimerTaskStackSize) {
//     /* If the buffers to be provided to the Timer task are declared inside this
//      * function, they must be declared static - otherwise, they will be allocated on
//      * the stack and will not exist after this function exits. */
//     static StaticTask_t xTimerTaskTCB;
//     static StackType_t uxTimerTaskStack[configTIMER_TASK_STACK_DEPTH];

//     /* Pass out a pointer to the StaticTask_t structure in which the Timer
//      * task's state will be stored. */
//     *ppxTimerTaskTCBBuffer = &xTimerTaskTCB;

//     /* Pass out the array that will be used as the Timer task's stack. */
//     *ppxTimerTaskStackBuffer = uxTimerTaskStack;

//     /* Pass out the size of the array pointed to by *ppxTimerTaskStackBuffer.
//      * Note that, as the array is necessarily of type StackType_t,
//      * configMINIMAL_STACK_SIZE is specified in words, not bytes. */
//     *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
// }

// #endif
/*========= [PRIVATE FUNCTION IMPLEMENTATION] ==================================*/

/*========= [INTERRUPT FUNCTION IMPLEMENTATION] ================================*/

void TimerCallback(port_timers_handler_t xTimer) {
    timer_index_t index = 0;
    while (index < TIMER_INDEX_QTY) {
        if (timers_active[index].handler == xTimer) {
            break;
        }
        else {
            index++;
        }
    }
    if (index < TIMER_INDEX_QTY) {
        if (timers_active[index].callback != NULL) {
            timers_active[index].callback(timers_active[index].context);
        }
    }
}

#endif
