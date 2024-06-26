/**
 * @file identificacion.c
 * @author 
 *
 * @brief 
 *
 * @version 0.1
 * @date 2024-05-27
 */

/*========= [DEPENDENCIES] =====================================================*/

#include "identificacion.h"
#include "interface.h"
#include <string.h>
#include "sapi.h"
#include "task_manager.h"

/*========= [PRIVATE MACROS AND CONSTANTS] =====================================*/

#define DATA_SIZE 400

#define MUL_ELEMENTS(a, b) ((a)*(b))

/*========= [PRIVATE DATA TYPES] ===============================================*/

/*========= [TASK DECLARATIONS] ================================================*/

/*========= [PRIVATE FUNCTION DECLARATIONS] ====================================*/

static void generate_prbs_signal(float *u, int size);

static void IdentificacionTask(void* not_used);

static void acquire_output_signal(float *u, float *y, int size);

static void InvertMatrix(float A[5][5], float A_inv[5][5]);

static void LeastSquares(float *u, float *y, int size, float *a, float *b);

static void IdentificacionTask(void* not_used);

static float q15_div(float a, float b);

/*========= [INTERRUPT FUNCTION DECLARATIONS] ==================================*/

/*========= [LOCAL VARIABLES] ==================================================*/

STATIC float u[DATA_SIZE] = {[0 ... (DATA_SIZE - 1)] = 0}; // Entrada

STATIC float y[DATA_SIZE] = {[0 ... (DATA_SIZE - 1)] = 0}; // Salida

/*========= [STATE FUNCTION POINTERS] ==========================================*/

/*========= [PUBLIC FUNCTION IMPLEMENTATION] ===================================*/

void IDENTIFICACION_Init(void) {
    static osal_task_t identificacion_task = {.name = "identificacion"};
    static osal_stack_holder_t identificacion_stack[STACK_SIZE_IDENTIFICACION];
    static osal_task_holder_t identificacion_holder;
    OSAL_TASK_LoadStruct(&identificacion_task, identificacion_stack, &identificacion_holder, STACK_SIZE_IDENTIFICACION);
    OSAL_TASK_Create(&identificacion_task, IdentificacionTask, NULL, TASK_PRIORITY_NORMAL);
}


/*========= [PRIVATE FUNCTION IMPLEMENTATION] ==================================*/

static void IdentificacionTask(void* not_used) {
    INTERFACE_Init();   
    OSAL_TASK_Delay(2000);
    float a[3], b[2];


    generate_prbs_signal(u, DATA_SIZE);
    acquire_output_signal(u, y, DATA_SIZE);
    LeastSquares(u, y, DATA_SIZE, a, b);
    
    static char str[150];
    sprintf(str, "Identified system parameters:\n");
    uartWriteString(UART_USB, str);
    sprintf(str, "DEN0 = %f\nDEN1 = %f\nDEN2 = %f\n", a[0], a[1], a[2]);
    uartWriteString(UART_USB, str);
    sprintf(str, "NUM0 = %f\nNUM1 = %f\n", b[0], b[1]);
    uartWriteString(UART_USB, str);

    OSAL_TASK_Delay(OSAL_MAX_DELAY);

}

// Función para dividir dos números Q15
float q15_div(float a, float b) {
    // Asegurarse de que no hay división por cero

    return (float)(a / b);
}

static void generate_prbs_signal(float *u, int size) {
    uint16_t lfsr = 0xACE1u; // Estado inicial no nulo
    uint16_t bit;

    for (int i = 0; i < size; i++) {
        // Generar el bit pseudo-aleatorio
        bit = ((lfsr >> 0) ^ (lfsr >> 2) ^ (lfsr >> 3) ^ (lfsr >> 5)) & 1;
        lfsr = (lfsr >> 1) | (bit << 15);

        // Mapear el valor del PRBS a +1 o -1
        u[i] = (lfsr & 1) ? 1 : 0;
    }
}

static void acquire_output_signal(float *u, float *y, int size) {
    STATIC osal_tick_t last_wake;
    last_wake = OSAL_TASK_GetTickCount();
    
    for (int i = 0; i < size; i++) {
        INTERFACE_DACWriteMv(u[i]*1000);
        y[i] = (float)(INTERFACE_ADCRead(1)) / 1000.0;
        OSAL_TASK_DelayUntil(&last_wake,OSAL_MS_TO_TICKS(5));

    }
}

// Función para invertir una matriz 5x5 (Gauss-Jordan)
void InvertMatrix(float A[5][5], float A_inv[5][5]) {
    int i, j, k;
    float ratio, a;

    // Inicializar A_inv como matriz identidad
    for (i = 0; i < 5; i++) {
        for (j = 0; j < 5; j++) {
            A_inv[i][j] = (i == j) ? 1.0 : 0.0;
        }
    }

    // Aplicar Gauss-Jordan
    for (i = 0; i < 5; i++) {
        a = A[i][i];
        for (j = 0; j < 5; j++) {
            A[i][j] = q15_div(A[i][j], a);
            A_inv[i][j] = q15_div(A_inv[i][j], a);
        }
        for (k = 0; k < 5; k++) {
            if (k != i) {
                ratio = A[k][i];
                for (j = 0; j < 5; j++) {
                    A[k][j] -= MUL_ELEMENTS(ratio, A[i][j]);
                    A_inv[k][j] -= MUL_ELEMENTS(ratio, A_inv[i][j]);
                }
            }
        }
    }
}

// Función para resolver el sistema de ecuaciones utilizando cuadrados mínimos
void LeastSquares(float *u, float *y, int size, float *a, float *b) {
    static float Phi[DATA_SIZE][5] = {
        [0 ... DATA_SIZE -1 ] = {
            [0 ... 4] = 0
        }
    }; // Matriz de diseño
    static float Y[DATA_SIZE] = {[0 ... DATA_SIZE -1 ] = 0};    // Vector de salida
    static float PhiTPhi[5][5];       // PhiT * Phi
    static float XtY[5];          // PhiT * Y
    static float invPhiTPhi[5][5];    // Inversa de PhiTPhi
    static float invPhiTPhiPhiT[5][DATA_SIZE] = {
        [0 ... 4] = {
            [0 ... DATA_SIZE -1 ] = 0
        }
    };

    // Llenar la matriz de diseño y el vector de salida
    for (int i = 2; i < (size); i++) {
        Phi[i-2][0] = y[i - 1];
        Phi[i-2][1] = y[i - 2];
        Phi[i-2][2] = u[i];
        Phi[i-2][3] = u[i-1];
        Phi[i-2][4] = u[i-2];
        Y[i - 2] = y[i];
    }

    // Calcular PhiTPhi
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 5; j++) {
            PhiTPhi[i][j] = 0;
            for (int k = 0; k < size; k++) {
                PhiTPhi[i][j] += MUL_ELEMENTS(Phi[k][i], Phi[k][j]);
            }
        }
    }

    InvertMatrix(PhiTPhi, invPhiTPhi);

    // Calcular invPhiTPhiPhiT
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < size; j++) {
            invPhiTPhiPhiT[i][j] = 0;
            for (int k = 0; k < 5; k++) {
                invPhiTPhiPhiT[i][j] += MUL_ELEMENTS(invPhiTPhi[i][k], Phi[j][k]);
            }
        }
    }

    // Calcular XtY
    for (int i = 0; i < 5; i++) {
        XtY[i] = 0;
        for (int j = 0; j < size; j++) {
            XtY[i] += MUL_ELEMENTS(invPhiTPhiPhiT[i][j], Y[j]);
        }
    }


    a[0] = 1;
    a[1] = -XtY[0];
    a[2] = -XtY[1];
    b[0] = XtY[3];
    b[1] = XtY[4];
}

/*========= [INTERRUPT FUNCTION IMPLEMENTATION] ================================*/


