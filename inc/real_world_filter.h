/**
 * @file real_world_filter.h
 * @author Marcos Dominguez
 *
 * @brief Module description
 *
 * @version X.Y
 * @date 2024-06-12
 */

#ifndef REAL_WORLD_FILTER_H
#define REAL_WORLD_FILTER_H

/*========= [DEPENDENCIES] =====================================================*/

#include <stdint.h>

/*========= [PUBLIC MACRO AND CONSTANTS] =======================================*/

/*========= [PUBLIC DATA TYPE] =================================================*/

/*========= [PUBLIC FUNCTION DECLARATIONS] =====================================*/

/**
 * @brief Resets the filter state.
 *
 * This function resets the internal state of the filter buffers.
 */
int32_t REAL_WORLD_FILTER_Reset(void);

/**
 * @brief Filters input signal using a digital filter.
 *
 * This function implements a digital filter to process the input signal. It takes
 * the input signal in Q15 format and applies a numerator-denominator filter with
 * coefficients provided as Q15 fixed-point values.
 *
 * @param input Input signal in Q15 format.
 * @return Filtered output signal in Q15 format.
 */
int32_t REAL_WORLD_FILTER_Filter(int32_t input);

#endif  /* REAL_WORLD_FILTER_H */
