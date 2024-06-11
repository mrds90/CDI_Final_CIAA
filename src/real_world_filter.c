/**
 * @file real_world_filter.c
 * @author Marcos Dominguez
 *
 * @brief Module description
 *
 * @version X.Y
 * @date 2024-06-11
 */

/*========= [DEPENDENCIES] =====================================================*/

#include "real_world_filter.h"
#include <string.h>

/*========= [PRIVATE MACROS AND CONSTANTS] =====================================*/

#ifndef MUL_ELEMENTS
#define MUL_ELEMENTS(x,y)  ((x) * (y))
#endif

#define F_TO_Q15(x)  (int32_t)((x) * (1 << 15))

#define NUM_SIZE 2
#define DEN_SIZE 3

/* Numerator coefficients in Q15 */
#define NUM0 F_TO_Q15(0.04976845243756167)
#define NUM1 F_TO_Q15(0.035050642374672925)

/* Denominator coefficients in Q15 */
#define DEN0 F_TO_Q15(1.0)
#define DEN1 F_TO_Q15(-1.2631799459800208)
#define DEN2 F_TO_Q15(0.34799904079225535)

/*========= [PRIVATE DATA TYPES] ===============================================*/

/*========= [TASK DECLARATIONS] ================================================*/

/*========= [PRIVATE FUNCTION DECLARATIONS] ====================================*/

/*========= [INTERRUPT FUNCTION DECLARATIONS] ==================================*/

/*========= [LOCAL VARIABLES] ==================================================*/

static int32_t input_buffer[NUM_SIZE] = {[0 ... (NUM_SIZE - 1)] = 0};
static int32_t output_buffer[DEN_SIZE - 1] = {[0 ... (DEN_SIZE - 2)] = 0};

/*========= [STATE FUNCTION POINTERS] ==========================================*/

/*========= [PUBLIC FUNCTION IMPLEMENTATION] ===================================*/

int32_t REAL_WORLD_FILTER_Reset() {
    memset(output_buffer, 0, sizeof(output_buffer));
    memset(input_buffer, 0, sizeof(input_buffer));
}

int32_t REAL_WORLD_FILTER_Filter(int32_t input) {
    /* Shift values in the input buffer */
    for (int i = NUM_SIZE - 1; i > 0; --i) {
        input_buffer[i] = input_buffer[i - 1];
    }
    input_buffer[0] = input;

    /* Calculate the numerator part */
    int32_t output = 0;
    output += MUL_ELEMENTS(NUM0, input_buffer[0]) >> 15;
    output += MUL_ELEMENTS(NUM1, input_buffer[1]) >> 15;

    /* Calculate the denominator part */
    output -= MUL_ELEMENTS(DEN1, output_buffer[0]) >> 15;
    output -= MUL_ELEMENTS(DEN2, output_buffer[1]) >> 15;

    /* Shift values in the output buffer */
    for (int i = DEN_SIZE - 2; i > 0; --i) {
        output_buffer[i] = output_buffer[i - 1];
    }
    output_buffer[0] = output;

    return output;
}

/*========= [PRIVATE FUNCTION IMPLEMENTATION] ==================================*/

/*========= [INTERRUPT FUNCTION IMPLEMENTATION] ================================*/
