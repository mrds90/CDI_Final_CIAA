/**
 * @file FreeRTOS_semphr_simulated.c
 * @author Marcos Dominguez
 *
 * @brief Port for FreeRTOS Semaphore API (Simulated).
 *
 * This module provides a simulated implementation of FreeRTOS semaphore
 * functionality. It includes the creation of binary, mutex, and counting
 * semaphores, along with operations like giving and taking semaphores.
 *
 * @version 0.1
 * @date 2023-11-24
 */

/*========= [DEPENDENCIES] =====================================================*/

#include "FreeRTOS_semphr_simulated.h"
#include "FreeRTOS_queue_simulated.h"

/*========= [PRIVATE MACROS AND CONSTANTS] =====================================*/

/*========= [PRIVATE DATA TYPES] ===============================================*/

/*========= [TASK DECLARATIONS] ================================================*/

/*========= [PRIVATE FUNCTION DECLARATIONS] ====================================*/

/*========= [INTERRUPT FUNCTION DECLARATIONS] ==================================*/

/*========= [LOCAL VARIABLES] ==================================================*/

/*========= [STATE FUNCTION POINTERS] ==========================================*/

/*========= [PUBLIC FUNCTION IMPLEMENTATION] ===================================*/

SemaphoreHandle_t __attribute__((weak)) xSemaphoreCreateBinaryStatic(StaticSemaphore_t *pxSemaphoreBuffer) {
    if (xQueueCreateStatic(1, sizeof(uint8_t), pxSemaphoreBuffer->buffer, &pxSemaphoreBuffer->holder) != NULL) {
        return (SemaphoreHandle_t)pxSemaphoreBuffer;
    }
    else {
        return NULL;
    }
}

SemaphoreHandle_t __attribute__((weak)) xSemaphoreCreateMutexStatic(StaticSemaphore_t *pxSemaphoreBuffer) {
    if (xQueueCreateStatic(1, sizeof(uint8_t), pxSemaphoreBuffer->buffer, &pxSemaphoreBuffer->holder) != NULL) {
        return (SemaphoreHandle_t)pxSemaphoreBuffer;
    }
    else {
        return NULL;
    }
}

SemaphoreHandle_t __attribute__((weak)) xSemaphoreCreateCountingStatic(UBaseType_t uxMaxCount, UBaseType_t uxInitialCount, StaticSemaphore_t *pxSemaphoreBuffer) {
    if (uxMaxCount > MAX_CONT_SEMPHR) {
        uxMaxCount = MAX_CONT_SEMPHR;
    }
    if (uxInitialCount > MAX_CONT_SEMPHR) {
        uxInitialCount = MAX_CONT_SEMPHR;
    }
    if (uxInitialCount > uxMaxCount) {
        uxInitialCount = uxMaxCount;
    }
    if (xQueueCreateStatic(uxMaxCount, sizeof(uint8_t), pxSemaphoreBuffer->buffer, &pxSemaphoreBuffer->holder) != NULL) {
        pxSemaphoreBuffer->holder.used_elements = uxMaxCount - uxInitialCount;
        return (SemaphoreHandle_t)pxSemaphoreBuffer;
    }
    else {
        return NULL;
    }
}

BaseType_t __attribute__((weak)) xSemaphoreGive(SemaphoreHandle_t xSemaphore) {
    if (xSemaphore != NULL) {
        uint8_t data = pdTRUE;
        xQueueSend(&xSemaphore->holder, &data, 0);
        return pdTRUE;
    }
    else {
        return pdFALSE;
    }
}

BaseType_t __attribute__((weak)) xSemaphoreTake(SemaphoreHandle_t xSemaphore, TickType_t wait_time) {
    if (xSemaphore != NULL) {
        uint8_t data;
        if (xQueueReceive(&xSemaphore->holder, &data, wait_time) && (data == pdTRUE)) {
            return pdTRUE;
        }
        else {
            return pdFALSE;
        }
    }
    else {
        return pdFALSE;
    }
}

BaseType_t __attribute__((weak)) xSemaphoreGiveFromISR(SemaphoreHandle_t handler, BaseType_t *HigherPriorityTaskWoken) {
    if (handler != NULL) {
        uint8_t data =  pdTRUE;
        xQueueSendFromISR(&handler->holder, &data, HigherPriorityTaskWoken);
        if (data == pdTRUE) {
            return pdTRUE;
        }
        else {
            return pdFALSE;
        }
    }
    else {
        return pdFALSE;
    }
}

BaseType_t __attribute__((weak)) xSemaphoreTakeFromISR(SemaphoreHandle_t handler, BaseType_t *HigherPriorityTaskWoken) {
    if (handler != NULL) {
        uint8_t data;
        xQueueReceiveFromISR(&handler->holder, &data, HigherPriorityTaskWoken);
        if (data == pdTRUE) {
            return pdTRUE;
        }
        else {
            return pdFALSE;
        }
    }
    else {
        return pdFALSE;
    }
}

/*========= [PRIVATE FUNCTION IMPLEMENTATION] ==================================*/

/*========= [INTERRUPT FUNCTION IMPLEMENTATION] ================================*/
