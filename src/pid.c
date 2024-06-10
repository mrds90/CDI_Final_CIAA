
/**
 * @file pid.c
 * @author Marcos Dominguez
 *
 * @brief Module description
 *
 * @version X.Y
 * @date 2024-06-10
 */

/*========= [DEPENDENCIES] =====================================================*/

#include "pid.h"
#include <string.h>

/*========= [PRIVATE MACROS AND CONSTANTS] =====================================*/

#define F_TO_Q15(x)  (int32_t)((x) * (1 << 15))

#define NUM_SIZE 3
#define DEN_SIZE 3

// Numerator coefficients in Q15
#define NUM0 F_TO_Q15(1.0)
#define NUM1 F_TO_Q15(-1.35569551)
#define NUM2 F_TO_Q15(0.42632345)

// Denominator coefficients in Q15
#define DEN0 F_TO_Q15(1.0)
#define DEN1 F_TO_Q15(-1.21587686)
#define DEN2 F_TO_Q15(0.2865048)

/*========= [PRIVATE DATA TYPES] ===============================================*/

/*========= [TASK DECLARATIONS] ================================================*/

/*========= [PRIVATE FUNCTION DECLARATIONS] ====================================*/

/*========= [INTERRUPT FUNCTION DECLARATIONS] ==================================*/

/*========= [LOCAL VARIABLES] ==================================================*/

static int32_t input_buffer[NUM_SIZE] = {[0 ... (NUM_SIZE - 1)] = 0};
static int32_t output_buffer[DEN_SIZE - 1] = {[0 ... (DEN_SIZE - 2)] = 0};

/*========= [STATE FUNCTION POINTERS] ==========================================*/

/*========= [PUBLIC FUNCTION IMPLEMENTATION] ===================================*/

/**
 * @brief Resets the filter state.
 *
 * This function resets the internal state of the filter buffers.
 */
int32_t PID_Reset() {
    memset(output_buffer, 0, sizeof(output_buffer));
    memset(input_buffer, 0, sizeof(input_buffer));
}

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
int32_t PID_Filter(int32_t input) {
    // Shift values in the input buffer
    for (int i = NUM_SIZE - 1; i > 0; --i) {
        input_buffer[i] = input_buffer[i - 1];
    }
    input_buffer[0] = input;

    // Calculate the numerator part
    int32_t output = 0;
    output += (NUM0 * input_buffer[0]) >> 15;
    output += (NUM1 * input_buffer[1]) >> 15;
    output += (NUM2 * input_buffer[2]) >> 15;


    // Calculate the denominator part
    output -= (DEN1 * output_buffer[0]) >> 15;
    output -= (DEN2 * output_buffer[1]) >> 15;


    // Shift values in the output buffer
    for (int i = DEN_SIZE - 2; i > 0; --i) {
        output_buffer[i] = output_buffer[i - 1];
    }
    output_buffer[0] = output;

    return output;
}

/*========= [PRIVATE FUNCTION IMPLEMENTATION] ==================================*/

/*========= [INTERRUPT FUNCTION IMPLEMENTATION] ================================*/
