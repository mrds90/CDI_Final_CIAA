
/**
 * @file timer_manager.h
 * @author Marcos Dominguez
 * @brief hold memory of OS.
 *
 *
 * @version 1.0
 * @date 2023-08-24
 *
 *
 */

#ifndef TIMER_MANAGER_H
#define TIMER_MANAGER_H

#ifdef __cplusplus
extern "C"
{
#endif

/*================ PUBLIC MACRO AND CONSTANTS ==========================================*/

/*================ PUBLIC DATA TYPE ====================================================*/

typedef enum {
    #ifdef TEST
    TIMER_INDEX_TEST_1,
    TIMER_INDEX_TEST_2,
    #else
    TIMER_GENERIC_1,
    #endif

    TIMER_INDEX_QTY
} timer_index_t;


/*================ PUBLIC FUNCTION DECLARATIONS ========================================*/

#ifdef __cplusplus
}
#endif

#endif /* UTILS_H */
