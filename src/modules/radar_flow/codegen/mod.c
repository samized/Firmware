/*
 * mod.c
 *
 * Code generation for function 'mod'
 *
 * C source code generated on: Fri Jul  5 14:52:28 2013
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "frontFlowKalmanFilter.h"
#include "wallEstimationFilter.h"
#include "wallEstimationFilter2.h"
#include "mod.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */

/* Function Definitions */
real32_T b_mod(real32_T x, real32_T y)
{
  return x - (real32_T)floor(x / 32.0F) * 32.0F;
}

/* End of code generation (mod.c) */
