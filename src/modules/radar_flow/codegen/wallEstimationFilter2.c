/*
 * wallEstimationFilter2.c
 *
 * Code generation for function 'wallEstimationFilter2'
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
#include "mod.h"
#include "sum.h"
#include "power.h"
#include "mean.h"
#include "mldivide.h"
#include "diag.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */

/* Function Definitions */
void wallEstimationFilter2(const real32_T wall_hist_left_k[40], const real32_T
  wall_hist_right_k[40], const real32_T flow_left[10], const real32_T
  weight_left[10], const real32_T flow_right[10], const real32_T weight_right[10],
  const real32_T speed[2], const real32_T position_update[2], real32_T
  attitude_update, const real32_T settings[5], const real32_T angles_sin[20],
  const real32_T unit_vectors[40], boolean_T use_position, real32_T radar[32],
  real32_T wall_hist_left[40], real32_T wall_hist_right[40])
{
  int32_T i4;
  real32_T wall_est_left[2];
  real32_T wall_est_right[2];
  real32_T radar_distance_update_left[32];
  real32_T radar_distance_update_right[32];
  int32_T i;
  int32_T i5;
  int32_T i6;
  real32_T invalid_flow_filter[10];
  real32_T vectors[20];
  real32_T pitch_sum;
  real32_T y[10];
  real32_T A[20];
  real32_T W[100];
  real32_T b_W[20];
  real32_T wall[2];
  boolean_T hist_filter_left[10];
  boolean_T b_y[10];
  int32_T tmp_size[1];
  int32_T tmp_data[10];
  int32_T b_tmp_size[1];
  int32_T b_tmp_data[10];
  int32_T y_size[1];
  int32_T b_y_size[1];
  real32_T c_tmp_data[10];
  real32_T ss_tot;
  real32_T fv3[10];
  int32_T c_y_size[1];
  int32_T d_y_size[1];
  real32_T fv4[10];
  real32_T weight_sum;
  boolean_T exitg2;
  static const real32_T fv5[15] = { 0.196349546F, 0.392699093F, 0.589048624F,
    0.785398185F, 0.981747746F, 1.17809725F, 1.37444675F, 1.57079637F,
    1.76714587F, 1.96349537F, 2.15984488F, 2.3561945F, 2.55254412F, 2.7488935F,
    2.94524312F };

  boolean_T exitg1;
  boolean_T b_radar_distance_update_left[32];
  int32_T c_tmp_size[2];
  int32_T d_tmp_data[32];
  real32_T distance_filtering_left[32];
  real32_T distance_filtering_right[32];

  /* WALL_ESTIMATION Summary of this function goes here */
  /*    Detailed explanation goes here */
  /*  const settings */
  /*  only fix resolution is efficient */
  /*  calc from settings */
  /*  flow/speed thresholds */
  /* 0.05; */
  /* 0.05; */
  /* 2; */
  /*  window filter */
  /*  default values */
  for (i4 = 0; i4 < 40; i4++) {
    wall_hist_left[i4] = 0.0F;
    wall_hist_right[i4] = 0.0F;
  }

  for (i4 = 0; i4 < 32; i4++) {
    radar[i4] = 5.0F;
  }

  for (i4 = 0; i4 < 2; i4++) {
    wall_est_left[i4] = 0.0F;
    wall_est_right[i4] = 0.0F;
  }

  for (i4 = 0; i4 < 32; i4++) {
    radar_distance_update_left[i4] = 5.0F;
    radar_distance_update_right[i4] = 5.0F;
  }

  /*  wall estimation filter */
  /* wall_model =  */
  /*  --------------------------------------------------------------------- */
  /*  ATTITUDE / POSITION UPDATE CALCULATION (flow) */
  /*  --------------------------------------------------------------------- */
  /* if use_position */
  if (1.0F > settings[3] - 1.0F) {
    i = -1;
  } else {
    i = (int32_T)(settings[3] - 1.0F) - 1;
  }

  if (2.0F > settings[3]) {
    i4 = 0;
  } else {
    i4 = 1;
  }

  for (i5 = 0; i5 < 4; i5++) {
    for (i6 = 0; i6 <= i; i6++) {
      wall_hist_left[(i4 + i6) + 10 * i5] = wall_hist_left_k[i6 + 10 * i5];
    }
  }

  if (1.0F > settings[3] - 1.0F) {
    i = -1;
  } else {
    i = (int32_T)(settings[3] - 1.0F) - 1;
  }

  if (2.0F > settings[3]) {
    i4 = 0;
  } else {
    i4 = 1;
  }

  for (i5 = 0; i5 < 4; i5++) {
    for (i6 = 0; i6 <= i; i6++) {
      wall_hist_right[(i4 + i6) + 10 * i5] = wall_hist_right_k[i6 + 10 * i5];
    }
  }

  /*  --------------------------------------------------------------------- */
  /*  WALL UPDATE CALCULATION (flow) */
  /*  --------------------------------------------------------------------- */
  if ((real32_T)fabs(speed[0]) > settings[0]) {
    /*  LEFT ------------------------------------------------------------ */
    for (i = 0; i < 10; i++) {
      invalid_flow_filter[i] = 1.0F;
    }

    memset(&vectors[0], 0, 20U * sizeof(real32_T));
    for (i = 0; i < 10; i++) {
      pitch_sum = flow_left[i] / 1000.0F;
      if ((real32_T)fabs(pitch_sum) > settings[1]) {
        pitch_sum = speed[0] / pitch_sum * angles_sin[i];
        if (pitch_sum > 0.0F) {
          for (i4 = 0; i4 < 2; i4++) {
            vectors[i + 10 * i4] = unit_vectors[i + 20 * i4] * pitch_sum;
          }
        } else {
          invalid_flow_filter[i] = 0.0F;
        }
      } else {
        invalid_flow_filter[i] = 0.0F;
      }
    }

    /*  calc left wall with linear regresstion */
    if (sum(invalid_flow_filter) > settings[2]) {
      for (i4 = 0; i4 < 10; i4++) {
        y[i4] = vectors[10 + i4] * invalid_flow_filter[i4];
        A[i4] = vectors[i4] * invalid_flow_filter[i4];
        A[10 + i4] = invalid_flow_filter[i4];
      }

      diag(weight_left, W);
      for (i4 = 0; i4 < 10; i4++) {
        for (i5 = 0; i5 < 2; i5++) {
          b_W[i4 + 10 * i5] = 0.0F;
          for (i6 = 0; i6 < 10; i6++) {
            b_W[i4 + 10 * i5] += W[i4 + 10 * i6] * A[i6 + 10 * i5];
          }
        }

        invalid_flow_filter[i4] = 0.0F;
        for (i5 = 0; i5 < 10; i5++) {
          invalid_flow_filter[i4] += W[i4 + 10 * i5] * y[i5];
        }
      }

      mldivide(b_W, invalid_flow_filter, wall);
      if (wall[1] < 0.0F) {
        /* && wall(2) > -10 % left wall can only be in negative y range... else invalid data (max 10m) */
        for (i = 0; i < 10; i++) {
          hist_filter_left[i] = (y[i] != 0.0F);
          b_y[i] = (y[i] != 0.0F);
        }

        eml_li_find(hist_filter_left, tmp_data, tmp_size);
        eml_li_find(b_y, b_tmp_data, b_tmp_size);
        y_size[0] = tmp_size[0];
        i = tmp_size[0];
        for (i4 = 0; i4 < i; i4++) {
          invalid_flow_filter[i4] = y[tmp_data[i4] - 1];
        }

        pitch_sum = mean(invalid_flow_filter, y_size);
        b_y_size[0] = b_tmp_size[0];
        i = b_tmp_size[0];
        for (i4 = 0; i4 < i; i4++) {
          invalid_flow_filter[i4] = y[b_tmp_data[i4] - 1] - pitch_sum;
        }

        b_power(invalid_flow_filter, b_y_size, c_tmp_data, tmp_size);
        ss_tot = b_sum(c_tmp_data, tmp_size);

        /*  TODO verify that y>0 and not ~= 0... */
        wall_hist_left[0] = wall[0];
        wall_hist_left[10] = wall[1];
        for (i4 = 0; i4 < 10; i4++) {
          pitch_sum = 0.0F;
          for (i5 = 0; i5 < 2; i5++) {
            pitch_sum += A[i4 + 10 * i5] * wall[i5];
          }

          invalid_flow_filter[i4] = y[i4] - pitch_sum;
        }

        power(invalid_flow_filter, fv3);
        wall_hist_left[20] = 1.0F - sum(fv3) / ss_tot;
        wall_hist_left[30] = 1.0F;
      } else {
        wall_hist_left[30] = 0.0F;
      }
    } else {
      wall_hist_left[30] = 0.0F;
    }

    /*  RIGHT ----------------------------------------------------------- */
    for (i = 0; i < 10; i++) {
      invalid_flow_filter[i] = 1.0F;
    }

    memset(&vectors[0], 0, 20U * sizeof(real32_T));
    for (i = 0; i < 10; i++) {
      pitch_sum = flow_right[i] / 1000.0F;
      if ((real32_T)fabs(pitch_sum) > settings[1]) {
        pitch_sum = speed[0] / pitch_sum * angles_sin[10 + i];
        if (pitch_sum > 0.0F) {
          for (i4 = 0; i4 < 2; i4++) {
            vectors[i + 10 * i4] = unit_vectors[(i + 20 * i4) + 10] * pitch_sum;
          }
        } else {
          invalid_flow_filter[i] = 0.0F;
        }
      } else {
        invalid_flow_filter[i] = 0.0F;
      }
    }

    /*  calc right wall with linear regresstion */
    if (sum(invalid_flow_filter) > settings[2]) {
      for (i4 = 0; i4 < 10; i4++) {
        y[i4] = vectors[10 + i4] * invalid_flow_filter[i4];
        A[i4] = vectors[i4] * invalid_flow_filter[i4];
        A[10 + i4] = invalid_flow_filter[i4];
      }

      diag(weight_right, W);
      for (i4 = 0; i4 < 10; i4++) {
        for (i5 = 0; i5 < 2; i5++) {
          b_W[i4 + 10 * i5] = 0.0F;
          for (i6 = 0; i6 < 10; i6++) {
            b_W[i4 + 10 * i5] += W[i4 + 10 * i6] * A[i6 + 10 * i5];
          }
        }

        invalid_flow_filter[i4] = 0.0F;
        for (i5 = 0; i5 < 10; i5++) {
          invalid_flow_filter[i4] += W[i4 + 10 * i5] * y[i5];
        }
      }

      mldivide(b_W, invalid_flow_filter, wall);
      if (wall[1] > 0.0F) {
        /* && wall(2) < 10 % right wall can only be in positiv y range... else invalid data (max 10m) */
        for (i = 0; i < 10; i++) {
          hist_filter_left[i] = (y[i] != 0.0F);
          b_y[i] = (y[i] != 0.0F);
        }

        eml_li_find(hist_filter_left, tmp_data, tmp_size);
        eml_li_find(b_y, b_tmp_data, b_tmp_size);
        c_y_size[0] = tmp_size[0];
        i = tmp_size[0];
        for (i4 = 0; i4 < i; i4++) {
          invalid_flow_filter[i4] = y[tmp_data[i4] - 1];
        }

        pitch_sum = mean(invalid_flow_filter, c_y_size);
        d_y_size[0] = b_tmp_size[0];
        i = b_tmp_size[0];
        for (i4 = 0; i4 < i; i4++) {
          invalid_flow_filter[i4] = y[b_tmp_data[i4] - 1] - pitch_sum;
        }

        b_power(invalid_flow_filter, d_y_size, c_tmp_data, tmp_size);
        ss_tot = b_sum(c_tmp_data, tmp_size);
        wall_hist_right[0] = wall[0];
        wall_hist_right[10] = wall[1];
        for (i4 = 0; i4 < 10; i4++) {
          pitch_sum = 0.0F;
          for (i5 = 0; i5 < 2; i5++) {
            pitch_sum += A[i4 + 10 * i5] * wall[i5];
          }

          invalid_flow_filter[i4] = y[i4] - pitch_sum;
        }

        power(invalid_flow_filter, fv4);
        wall_hist_right[20] = 1.0F - sum(fv4) / ss_tot;
        wall_hist_right[30] = 1.0F;
      } else {
        wall_hist_right[30] = 0.0F;
      }
    } else {
      wall_hist_right[30] = 0.0F;
    }

    /*  filter HISTORY -------------------------------------------------- */
    /*  left */
    for (i4 = 0; i4 < 10; i4++) {
      hist_filter_left[i4] = (wall_hist_left[30 + i4] > 0.0F);
    }

    if (c_sum(hist_filter_left) > (real_T)settings[4]) {
      /*              wall_est_left(1) = sum(wall_hist_left(hist_filter_left,wall_hist_index_pitch)) / hist_count_left; */
      /*              wall_est_left(2) = sum(wall_hist_left(hist_filter_left,wall_hist_index_offset)) / hist_count_left; */
      pitch_sum = 0.0F;
      ss_tot = 0.0F;
      weight_sum = 0.0F;
      for (i = 0; i < (int32_T)settings[3]; i++) {
        if (hist_filter_left[(int32_T)(real32_T)(1 + i) - 1]) {
          pitch_sum += wall_hist_left[(int32_T)(1.0F + (real32_T)i) - 1] *
            wall_hist_left[(int32_T)(1.0F + (real32_T)i) + 19];
          ss_tot += wall_hist_left[(int32_T)(1.0F + (real32_T)i) + 9] *
            wall_hist_left[(int32_T)(1.0F + (real32_T)i) + 19];
          weight_sum += wall_hist_left[(int32_T)(1.0F + (real32_T)i) + 19];
        }
      }

      wall_est_left[0] = pitch_sum / weight_sum;
      wall_est_left[1] = ss_tot / weight_sum;
    }

    /*  right */
    for (i4 = 0; i4 < 10; i4++) {
      hist_filter_left[i4] = (wall_hist_right[30 + i4] > 0.0F);
    }

    if (c_sum(hist_filter_left) > (real_T)settings[4]) {
      /*              wall_est_right(1) = sum(wall_hist_right(hist_filter_right,wall_hist_index_pitch)) / hist_count_right; */
      /*              wall_est_right(2) = sum(wall_hist_right(hist_filter_right,wall_hist_index_offset)) / hist_count_right; */
      pitch_sum = 0.0F;
      ss_tot = 0.0F;
      weight_sum = 0.0F;
      for (i = 0; i < (int32_T)settings[3]; i++) {
        if (hist_filter_left[(int32_T)(real32_T)(1 + i) - 1]) {
          pitch_sum += wall_hist_right[(int32_T)(1.0F + (real32_T)i) - 1] *
            wall_hist_right[(int32_T)(1.0F + (real32_T)i) + 19];
          ss_tot += wall_hist_right[(int32_T)(1.0F + (real32_T)i) + 9] *
            wall_hist_right[(int32_T)(1.0F + (real32_T)i) + 19];
          weight_sum += wall_hist_right[(int32_T)(1.0F + (real32_T)i) + 19];
        }
      }

      wall_est_right[0] = pitch_sum / weight_sum;
      wall_est_right[1] = ss_tot / weight_sum;
    }

    /*  calc RADAR ------------------------------------------------------ */
    /*  left */
    if (wall_est_left[1] < 0.0F) {
      /*  only calc beta if there is a wall estimated */
      if (wall_est_left[0] > 0.0F) {
        ss_tot = (real32_T)atan(1.0F / wall_est_left[0]);
      } else if (wall_est_left[0] < 0.0F) {
        ss_tot = 3.14159274F - (real32_T)atan(1.0F / (real32_T)fabs
          (wall_est_left[0]));
      } else {
        ss_tot = 1.57079637F;
      }

      /*  calc distances left */
      radar_distance_update_left[8] = (real32_T)fabs(wall_est_left[1]);

      /*  c */
      i = 0;
      exitg2 = FALSE;
      while ((exitg2 == FALSE) && (i < 15)) {
        pitch_sum = (3.14159274F - fv5[i]) - ss_tot;
        weight_sum = (3.14159274F - fv5[i]) - (3.14159274F - ss_tot);

        /*  TODO change */
        if (pitch_sum > 0.0F) {
          radar_distance_update_left[9 + i] = (real32_T)fabs(wall_est_left[1]) /
            (real32_T)sin(pitch_sum) * (real32_T)sin(ss_tot);
        }

        if (weight_sum > 0.0F) {
          radar_distance_update_left[(int32_T)(b_mod(8.0F - (1.0F + (real32_T)i),
            32.0F) + 1.0F) - 1] = (real32_T)fabs(wall_est_left[1]) / (real32_T)
            sin(weight_sum) * (real32_T)sin(ss_tot);
        }

        if ((weight_sum <= 0.0F) && (pitch_sum <= 0.0F)) {
          exitg2 = TRUE;
        } else {
          i++;
        }
      }
    }

    /*  right */
    if (wall_est_right[1] > 0.0F) {
      if (wall_est_right[0] > 0.0F) {
        ss_tot = 3.14159274F - (real32_T)atan(1.0F / wall_est_right[0]);
      } else if (wall_est_right[0] < 0.0F) {
        ss_tot = (real32_T)atan(1.0F / (real32_T)fabs(wall_est_right[0]));
      } else {
        ss_tot = 1.57079637F;
      }

      /*  calc distances right */
      radar_distance_update_right[24] = wall_est_right[1];

      /*  c */
      i = 0;
      exitg1 = FALSE;
      while ((exitg1 == FALSE) && (i < 15)) {
        pitch_sum = (3.14159274F - fv5[i]) - ss_tot;
        weight_sum = (3.14159274F - fv5[i]) - (3.14159274F - ss_tot);
        if (pitch_sum > 0.0F) {
          radar_distance_update_right[23 - i] = wall_est_right[1] / (real32_T)
            sin(pitch_sum) * (real32_T)sin(ss_tot);
        }

        if (weight_sum > 0.0F) {
          radar_distance_update_right[(int32_T)(b_mod(24.0F + (1.0F + (real32_T)
            i), 32.0F) + 1.0F) - 1] = wall_est_right[1] / (real32_T)sin
            (weight_sum) * (real32_T)sin(ss_tot);
        }

        if ((weight_sum <= 0.0F) && (pitch_sum <= 0.0F)) {
          exitg1 = TRUE;
        } else {
          i++;
        }
      }
    }
  }

  /*  FILTERING ----------------------------------------------------------- */
  for (i4 = 0; i4 < 32; i4++) {
    b_radar_distance_update_left[i4] = (radar_distance_update_left[i4] < 0.1F);
  }

  b_eml_li_find(b_radar_distance_update_left, d_tmp_data, c_tmp_size);
  i = c_tmp_size[0] * c_tmp_size[1];
  for (i4 = 0; i4 < i; i4++) {
    radar_distance_update_left[d_tmp_data[i4] - 1] = 0.1F;
  }

  for (i4 = 0; i4 < 32; i4++) {
    b_radar_distance_update_left[i4] = (radar_distance_update_right[i4] < 0.1F);
  }

  b_eml_li_find(b_radar_distance_update_left, d_tmp_data, c_tmp_size);
  i = c_tmp_size[0] * c_tmp_size[1];
  for (i4 = 0; i4 < i; i4++) {
    radar_distance_update_right[d_tmp_data[i4] - 1] = 0.1F;
  }

  for (i4 = 0; i4 < 32; i4++) {
    b_radar_distance_update_left[i4] = (radar_distance_update_left[i4] > 5.0F);
  }

  b_eml_li_find(b_radar_distance_update_left, d_tmp_data, c_tmp_size);
  i = c_tmp_size[0] * c_tmp_size[1];
  for (i4 = 0; i4 < i; i4++) {
    radar_distance_update_left[d_tmp_data[i4] - 1] = 5.0F;
  }

  for (i4 = 0; i4 < 32; i4++) {
    b_radar_distance_update_left[i4] = (radar_distance_update_right[i4] > 5.0F);
  }

  b_eml_li_find(b_radar_distance_update_left, d_tmp_data, c_tmp_size);
  i = c_tmp_size[0] * c_tmp_size[1];
  for (i4 = 0; i4 < i; i4++) {
    radar_distance_update_right[d_tmp_data[i4] - 1] = 5.0F;
  }

  for (i4 = 0; i4 < 32; i4++) {
    distance_filtering_left[i4] = (real32_T)(radar_distance_update_left[i4] <
      5.0F);
    distance_filtering_right[i4] = (real32_T)(radar_distance_update_right[i4] <
      5.0F);
  }

  if (any(distance_filtering_left) || any(distance_filtering_right)) {
    /*  with updates */
    /*  calc radar values */
    for (i = 0; i < 32; i++) {
      /*  right and left available */
      if ((distance_filtering_left[i] != 0.0F) && (distance_filtering_right[i]
           != 0.0F)) {
        if ((1 + i < 17) || ((1 + i == 17) && (radar_distance_update_left[i] <
              radar_distance_update_right[i]))) {
          /*  believe left */
          radar[i] = radar_distance_update_left[i];
        } else {
          /*  believe right */
          radar[i] = radar_distance_update_right[i];
        }

        /*  only left available */
      } else if (distance_filtering_left[i] != 0.0F) {
        radar[i] = radar_distance_update_left[i];

        /*  only right available */
      } else {
        if (distance_filtering_right[i] != 0.0F) {
          radar[i] = radar_distance_update_right[i];
        }
      }
    }
  }
}

/* End of code generation (wallEstimationFilter2.c) */
