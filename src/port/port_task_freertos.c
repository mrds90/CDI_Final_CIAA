/**
 * @file port_task_freertos.c
 * @author Marcos Dominguez
 *
 * @brief Port for FreeRTOS task API.
 *
 * @version 0.1
 * @date 2023-11-24
 */

/*========= [DEPENDENCIES] =====================================================*/

#include "port_task_freertos.h"

/*========= [PRIVATE MACROS AND CONSTANTS] =====================================*/

/*========= [PRIVATE DATA TYPES] ===============================================*/

/*========= [TASK DECLARATIONS] ================================================*/

/*========= [PRIVATE FUNCTION DECLARATIONS] ====================================*/

/*========= [INTERRUPT FUNCTION DECLARATIONS] ==================================*/

/*========= [LOCAL VARIABLES] ==================================================*/

/*========= [STATE FUNCTION POINTERS] ==========================================*/

/*========= [PUBLIC FUNCTION IMPLEMENTATION] ===================================*/

osal_task_handler_t PORT_TASK_CreateStaticTask(OSAL_TASK_Callback_t function, char *name, uint16_t size, void *context, uint8_t priority, osal_stack_holder_t *stack_ptr, osal_task_holder_t *task_hold_ptr) {
    return ((osal_task_handler_t) xTaskCreateStatic(function, name, size, context, (UBaseType_t)(tskIDLE_PRIORITY + priority), stack_ptr, task_hold_ptr));
}

// #if (configSUPPORT_STATIC_ALLOCATION == 1)

// /* configUSE_STATIC_ALLOCATION is set to 1, so the application must provide an
//  * implementation of vApplicationGetIdleTaskMemory() to provide the memory that is
//  * used by the Idle task. */
// void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
//                                    StackType_t **ppxIdleTaskStackBuffer,
//                                    uint32_t *pulIdleTaskStackSize) {
//     /* If the buffers to be provided to the Idle task are declared inside this
//      * function then they must be declared static - otherwise they will be allocated on
//      * the stack and so not exists after this function exits. */
//     static StaticTask_t xIdleTaskTCB;
//     static StackType_t uxIdleTaskStack[ configMINIMAL_STACK_SIZE ];

//     /* Pass out a pointer to the StaticTask_t structure in which the Idle
//      * task's state will be stored. */
//     *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;

//     /* Pass out the array that will be used as the Idle task's stack. */
//     *ppxIdleTaskStackBuffer = uxIdleTaskStack;

//     /* Pass out the size of the array pointed to by *ppxIdleTaskStackBuffer.
//      * Note that, as the array is necessarily of type StackType_t,
//      * configMINIMAL_STACK_SIZE is specified in words, not bytes. */
//     *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
// }

// #endif /* configSUPPORT_STATIC_ALLOCATION == 1 */

/*========= [PRIVATE FUNCTION IMPLEMENTATION] ==================================*/

/*========= [INTERRUPT FUNCTION IMPLEMENTATION] ================================*/
