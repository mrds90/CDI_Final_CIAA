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

#define ORDER 2
#define MAX_SAMPLES 200

#define DATA_SIZE 100

#define A_SIZE 3
#define B_SIZE 2

/*========= [PRIVATE DATA TYPES] ===============================================*/

/*========= [TASK DECLARATIONS] ================================================*/

/*========= [PRIVATE FUNCTION DECLARATIONS] ====================================*/

void generate_prbs_signal(double *u, int size);

void IdentificacionTask(void* not_used);

void acquire_output_signal(double *u, double *y, int size);

void invert_matrix(double A[5][5], double A_inv[5][5]);

void least_squares(double *u, double *y, int size, double *a, double *b);

void IdentificacionTask(void* not_used);

/*========= [INTERRUPT FUNCTION DECLARATIONS] ==================================*/

/*========= [LOCAL VARIABLES] ==================================================*/

STATIC double u[DATA_SIZE]; // Entrada
STATIC double y[DATA_SIZE]; // Salida

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

void IdentificacionTask(void* not_used) {
    INTERFACE_Init();   
    OSAL_TASK_Delay(2000);
    double a[3], b[2];


    generate_prbs_signal(u, DATA_SIZE);
    acquire_output_signal(u, y, DATA_SIZE);
    least_squares(u, y, DATA_SIZE, a, b);
    
    static char str[150];
    sprintf(str, "Identified system parameters:\n");
    uartWriteString(UART_USB, str);
    sprintf(str, "DEN0 = %f\nDEN1 = %f\nDEN2 = %f\n", a[0], a[1], a[2]);
    uartWriteString(UART_USB, str);
    sprintf(str, "NUM0 = %f\nNUM1 = %f\n", b[0], b[1]);
    uartWriteString(UART_USB, str);

    OSAL_TASK_Delay(OSAL_MAX_DELAY);

}

void generate_prbs_signal(double *u, int size) {
    uint16_t lfsr = 0xACE1u; // Estado inicial no nulo
    uint16_t bit;

    for (int i = 0; i < size; i++) {
        // Generar el bit pseudo-aleatorio
        bit = ((lfsr >> 0) ^ (lfsr >> 2) ^ (lfsr >> 3) ^ (lfsr >> 5)) & 1;
        lfsr = (lfsr >> 1) | (bit << 15);

        // Mapear el valor del PRBS a +1 o -1
        u[i] = (lfsr & 1) ? -1.0 : 1.0;
    }
}

void acquire_output_signal(double *u, double *y, int size) {
    STATIC osal_tick_t last_wake;
    last_wake = OSAL_TASK_GetTickCount();
    
    for (int i = 0; i < size; i++) {
        INTERFACE_DACWriteMv(u[i]*1000);
        y[i] = (float)(INTERFACE_ADCRead()) / 1000.0;
        OSAL_TASK_DelayUntil(&last_wake,OSAL_MS_TO_TICKS(5));

    }
}

// Función para invertir una matriz 5x5 (Gauss-Jordan)
void invert_matrix(double A[5][5], double A_inv[5][5]) {
    int i, j, k;
    double ratio, a;

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
            A[i][j] /= a;
            A_inv[i][j] /= a;
        }
        for (k = 0; k < 5; k++) {
            if (k != i) {
                ratio = A[k][i];
                for (j = 0; j < 5; j++) {
                    A[k][j] -= ratio * A[i][j];
                    A_inv[k][j] -= ratio * A_inv[i][j];
                }
            }
        }
    }
}

// Función para resolver el sistema de ecuaciones utilizando cuadrados mínimos
void least_squares(double *u, double *y, int size, double *a, double *b) {
    static double X[DATA_SIZE][5]; // Matriz de diseño
    static double Y[DATA_SIZE];    // Vector de salida
    static double Xt[5][DATA_SIZE]; // Transpuesta de X
    static double XtX[5][5];       // Xt * X
    static double XtY[5];          // Xt * Y
    static double invXtX[5][5];    // Inversa de XtX

    // Llenar la matriz de diseño y el vector de salida
    for (int i = 3; i < size; i++) {
        X[i][0] = y[i - 1];
        X[i][1] = y[i - 2];
        X[i][2] = y[i - 3];
        X[i][3] = u[i];
        X[i][4] = u[i - 1];
        Y[i] = y[i];
    }

    // Calcular Xt
    for (int i = 0; i < 5; i++) {
        for (int j = 3; j < size; j++) {
            Xt[i][j] = X[j][i];
        }
    }

    // Calcular XtX
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 5; j++) {
            XtX[i][j] = 0;
            for (int k = 3; k < size; k++) {
                XtX[i][j] += Xt[i][k] * X[k][j];
            }
        }
    }

    // Calcular XtY
    for (int i = 0; i < 5; i++) {
        XtY[i] = 0;
        for (int k = 3; k < size; k++) {
            XtY[i] += Xt[i][k] * Y[k];
        }
    }

    // Invertir XtX
    invert_matrix(XtX, invXtX);

    // Resolver para los coeficientes a y b
    static  double result[5];
    for (int i = 0; i < 5; i++) {
        result[i] = 0;
        for (int j = 0; j < 5; j++) {
            result[i] += invXtX[i][j] * XtY[j];
        }
    }

    a[0] = result[0];
    a[1] = result[1];
    a[2] = result[2];
    b[0] = result[3];
    b[1] = result[4];
}

/*========= [INTERRUPT FUNCTION IMPLEMENTATION] ================================*/


