/*
 * any.c
 *
 * Code generation for function 'any'
 *
 * C source code generated on: Fri Jul  5 14:52:28 2013
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "frontFlowKalmanFilter.h"
#include "wallEstimationFilter.h"
#include "wallEstimationFilter2.h"
#include "any.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */

/* Function Definitions */
boolean_T any(const real32_T x[32])
{
  boolean_T y;
  int32_T k;
  boolean_T exitg1;
  y = FALSE;
  k = 0;
  exitg1 = FALSE;
  while ((exitg1 == FALSE) && (k < 32)) {
    if (!(x[k] == 0.0F)) {
      y = TRUE;
      exitg1 = TRUE;
    } else {
      k++;
    }
  }

  return y;
}

boolean_T b_any(const boolean_T x[32])
{
  boolean_T y;
  int32_T k;
  boolean_T exitg1;
  y = FALSE;
  k = 0;
  exitg1 = FALSE;
  while ((exitg1 == FALSE) && (k < 32)) {
    if (!((int32_T)x[k] == 0)) {
      y = TRUE;
      exitg1 = TRUE;
    } else {
      k++;
    }
  }

  return y;
}

/* End of code generation (any.c) */
