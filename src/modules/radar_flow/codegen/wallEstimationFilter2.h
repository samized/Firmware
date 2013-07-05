/*
 * wallEstimationFilter2.h
 *
 * Code generation for function 'wallEstimationFilter2'
 *
 * C source code generated on: Fri Jul  5 14:52:28 2013
 *
 */

#ifndef __WALLESTIMATIONFILTER2_H__
#define __WALLESTIMATIONFILTER2_H__
/* Include files */
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rt_nonfinite.h"

#include "rtwtypes.h"
#include "wallEstimationFilter2_types.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */
extern void wallEstimationFilter2(const real32_T wall_hist_left_k[40], const real32_T wall_hist_right_k[40], const real32_T flow_left[10], const real32_T weight_left[10], const real32_T flow_right[10], const real32_T weight_right[10], const real32_T speed[2], const real32_T position_update[2], real32_T attitude_update, const real32_T settings[5], const real32_T angles_sin[20], const real32_T unit_vectors[40], boolean_T use_position, real32_T radar[32], real32_T wall_hist_left[40], real32_T wall_hist_right[40]);
#endif
/* End of code generation (wallEstimationFilter2.h) */
