/**
 * @file real_world_filter.c
 * @author Marcos Dominguez
 *
 * @brief Module description
 *
 * @version X.Y
 * @date 2024-06-12
 */

/*========= [DEPENDENCIES] =====================================================*/

#include "real_world_filter.h"
#include <string.h>


/*========= [PRIVATE MACROS AND CONSTANTS] =====================================*/

#ifndef MUL_ELEMENTS
#define MUL_ELEMENTS(x, y)  ((x) * (y))
#endif
#ifndef MUL_ELEMENTS_Q15
#define MUL_ELEMENTS_Q15(x, y)  (MUL_ELEMENTS(x, y) >> 15)
#endif
#ifndef MUL_SUM_ELEMENTS_Q15
#define MUL_SUM_ELEMENTS_Q15(x, y, z) ((z) + MUL_ELEMENTS_Q15((x), (y)))
#endif
#ifndef MUL_SUB_ELEMENTS_Q15
#define MUL_SUB_ELEMENTS_Q15(x, y, z) ((z) - MUL_ELEMENTS_Q15((x), (y)))
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
    memset(input_buffer, 0, sizeof(input_buffer));
    memset(output_buffer, 0, sizeof(output_buffer));
}

int32_t REAL_WORLD_FILTER_Filter(int32_t input) {
    /* Shift values in the input buffer */
    for (int i = NUM_SIZE - 1; i > 0; --i) {
        input_buffer[i] = input_buffer[i - 1];
    }
    input_buffer[0] = input;

    /* Calculate the numerator part */
    int32_t output = 0;
    output = MUL_SUM_ELEMENTS_Q15(NUM0, input_buffer[0], output);
    output = MUL_SUM_ELEMENTS_Q15(NUM1, input_buffer[1], output);

    /* Calculate the denominator part */
    output = MUL_SUB_ELEMENTS_Q15(DEN1, output_buffer[0], output);
    output = MUL_SUB_ELEMENTS_Q15(DEN2, output_buffer[1], output);

    /* Shift values in the output buffer */
    for (int i = DEN_SIZE - 2; i > 0; --i) {
        output_buffer[i] = output_buffer[i - 1];
    }
    output_buffer[0] = output;

    return output;
}

/*========= [PRIVATE FUNCTION IMPLEMENTATION] ==================================*/

/*========= [INTERRUPT FUNCTION IMPLEMENTATION] ================================*/
