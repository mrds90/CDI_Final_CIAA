/**
 * @file osal_delay.h
 * @author Marcos Dominguez
 *
 * @brief Manage not blocking delays
 *
 * @version 0.1
 * @date 2024-03-15
 */


#ifndef _OSAL_DELAY_H
#define _OSAL_DELAY_H

#ifdef  __cplusplus
extern "C" {
#endif

/*========= [DEPENDENCIES] =====================================================*/

#include "osal_task.h"

/*========= [PUBLIC MACRO AND CONSTANTS] =======================================*/

/*========= [PUBLIC DATA TYPE] =================================================*/

typedef struct {
    osal_tick_t time;
    osal_tick_t initial_time;
    bool_t running;
} osal_delay_t;

/*========= [PUBLIC FUNCTION DECLARATIONS] =====================================*/

/**
 * @brief Config delay time and stops counting
 *
 * @param delay delay object
 * @param time  timeout
 */
void OSAL_DELAY_Config(osal_delay_t *delay, osal_tick_t time);

/**
 * @brief Reset delay object
 *
 * @param delay
 */
void OSAL_DELAY_Reset(osal_delay_t *delay);

/**
 * @brief Check if delay is ready
 *
 * @param delay
 * @return bool_t
 */
bool_t OSAL_DELAY_Check(osal_delay_t *delay);

#ifdef  __cplusplus
}

#endif

#endif  /* _OSAL_DELAY_H */
