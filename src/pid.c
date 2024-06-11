/**
 * @file pid.c
 * @author Marcos Dominguez
 *
 * @brief Module description
 *
 * @version X.Y
 * @date 2024-06-11
 */

/*========= [DEPENDENCIES] =====================================================*/

#include "pid.h"
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

#define NUM_SIZE 3
#define DEN_SIZE 3

/* Numerator coefficients in Q15 */
#define NUM0 F_TO_Q15(1.0)
#define NUM1 F_TO_Q15(-1.3556955132594553)
#define NUM2 F_TO_Q15(0.4263234504891082)

/* Denominator coefficients in Q15 */
#define DEN0 F_TO_Q15(1.0)
#define DEN1 F_TO_Q15(-1.2158768596305372)
#define DEN2 F_TO_Q15(0.28650479686019026)

/*========= [PRIVATE DATA TYPES] ===============================================*/

/*========= [TASK DECLARATIONS] ================================================*/

/*========= [PRIVATE FUNCTION DECLARATIONS] ====================================*/

/*========= [INTERRUPT FUNCTION DECLARATIONS] ==================================*/

/*========= [LOCAL VARIABLES] ==================================================*/

static int32_t input_buffer[NUM_SIZE] = {[0 ... (NUM_SIZE - 1)] = 0};
static int32_t output_buffer[DEN_SIZE - 1] = {[0 ... (DEN_SIZE - 2)] = 0};

/*========= [STATE FUNCTION POINTERS] ==========================================*/

/*========= [PUBLIC FUNCTION IMPLEMENTATION] ===================================*/

int32_t PID_Reset() {
    memset(output_buffer, 0, sizeof(output_buffer));
    memset(input_buffer, 0, sizeof(input_buffer));
}

int32_t PID_Filter(int32_t input) {
    /* Shift values in the input buffer */
    for (int i = NUM_SIZE - 1; i > 0; --i) {
        input_buffer[i] = input_buffer[i - 1];
    }
    input_buffer[0] = input;

    /* Calculate the numerator part */
    int32_t output = 0;
    output = MUL_SUM_ELEMENTS_Q15(NUM0, input_buffer[0], output);
    output = MUL_SUM_ELEMENTS_Q15(NUM1, input_buffer[1], output);
    output = MUL_SUM_ELEMENTS_Q15(NUM2, input_buffer[2], output);

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
