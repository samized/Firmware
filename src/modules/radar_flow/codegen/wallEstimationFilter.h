/*
 * wallEstimationFilter.h
 *
 * Code generation for function 'wallEstimationFilter'
 *
 * C source code generated on: Fri Jul  5 14:52:28 2013
 *
 */

#ifndef __WALLESTIMATIONFILTER_H__
#define __WALLESTIMATIONFILTER_H__
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
extern void b_eml_li_find(const boolean_T x[32], int32_T y_data[32], int32_T y_size[2]);
extern void eml_li_find(const boolean_T x[10], int32_T y_data[10], int32_T y_size[1]);
extern void wallEstimationFilter(const real32_T radar_filtered_k[32], const real32_T radar_weights_k[32], const real32_T flow_left[10], const real32_T weight_left[10], const real32_T flow_right[10], const real32_T weight_right[10], real32_T front_distance, const real32_T speed[2], const real32_T position_update[2], real32_T attitude_update, const real32_T settings[8], const real32_T angles_sin[20], const real32_T unit_vectors[40], boolean_T use_sonar, boolean_T use_position, real32_T radar[32], real32_T radar_filtered[32], real32_T radar_weights[32]);
#endif
/* End of code generation (wallEstimationFilter.h) */
