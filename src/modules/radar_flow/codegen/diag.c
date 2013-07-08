/*
 * diag.c
 *
 * Code generation for function 'diag'
 *
 * C source code generated on: Fri Jul  5 14:52:28 2013
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "frontFlowKalmanFilter.h"
#include "wallEstimationFilter.h"
#include "wallEstimationFilter2.h"
#include "diag.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */

/* Function Definitions */
void diag(const real32_T v[10], real32_T d[100])
{
  int32_T j;
  memset(&d[0], 0, 100U * sizeof(real32_T));
  for (j = 0; j < 10; j++) {
    d[j + 10 * j] = v[j];
  }
}

/* End of code generation (diag.c) */
