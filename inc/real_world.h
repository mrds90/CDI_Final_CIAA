/**
 * @file real_world.h
 * @author Marcos Dominguez
 *
 * @brief Simulate a physical plant
 *
 * @version 0.1
 * @date 2024-05-27
 */

#ifndef REAL_WORLD_H
#define REAL_WORLD_H

#ifdef  __cplusplus
extern "C" {
#endif

/*========= [DEPENDENCIES] =====================================================*/

#include "data_types.h"
#include "utils.h"

/*========= [PUBLIC MACRO AND CONSTANTS] =======================================*/

#define Q15_SCALE(x)  (int32_t)((x) * (1 << 15))

/*========= [PUBLIC DATA TYPE] =================================================*/

/*========= [PUBLIC FUNCTION DECLARATIONS] =====================================*/

void REAL_WORLD_Init(void);

void REAL_WORLD_Input(int32_t value);

int32_t REAL_WORLD_Output(void);

void REAL_WORLD_Reset(void);

#ifdef  __cplusplus
}

#endif

#endif  /* REAL_WORLD_H */
