/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *   Author: @author Samuel Zihlmann <samuezih@ee.ethz.ch>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/*
 * @file radar_flow_params.c
 */

#include "radar_flow_params.h"

/* radar parameters */
PARAM_DEFINE_FLOAT(RF_BEEP_F, 0.0f);
PARAM_DEFINE_FLOAT(RF_KAL_K1, 0.0235f);
PARAM_DEFINE_FLOAT(RF_KAL_K2, 0.0140f);
PARAM_DEFINE_FLOAT(RF_FRONT_A, 0.1f);
PARAM_DEFINE_INT32(RF_SONAR, 0);
PARAM_DEFINE_INT32(RF_POS_UPDATE, 0);
PARAM_DEFINE_INT32(RF_LP_FILTER, 1);
PARAM_DEFINE_FLOAT(RF_S_V_THR, 0.05f); // speed_threshold
PARAM_DEFINE_FLOAT(RF_S_FLOW_THR, 0.05f); // flow_threshold
PARAM_DEFINE_FLOAT(RF_S_REG_THR, 2.0f);  // regr_threshold
PARAM_DEFINE_FLOAT(RF_S_A_DEF, 0.01f); // alpha_default
PARAM_DEFINE_FLOAT(RF_S_A_FLOWMAX, 0.3f);  // alpha_flow_max
PARAM_DEFINE_FLOAT(RF_S_A_FLOWMIN, 0.05f); // alpha_flow_min
PARAM_DEFINE_FLOAT(RF_S_A_WDOWN, 0.01f); // alpha_weight_down
PARAM_DEFINE_FLOAT(RF_S_A_WFOUT, 0.001f); // alpha_weight_fade_out
PARAM_DEFINE_FLOAT(RF_S_WIN_SIZE, 5.0f); // alpha_weight_fade_out
PARAM_DEFINE_FLOAT(RF_S_WIN_THR, 2.0f); // alpha_weight_fade_out
PARAM_DEFINE_INT32(RF_DEBUG, 0);


int parameters_init(struct radar_flow_param_handles *h)
{
	h->beep_front_sonar			=	param_find("RF_BEEP_F");
	h->kalman_k1	 			=	param_find("RF_KAL_K1");
	h->kalman_k2				=	param_find("RF_KAL_K2");
	h->front_lp_alpha			=	param_find("RF_FRONT_A");
	h->with_sonar				=	param_find("RF_SONAR");
	h->with_pos_update			=	param_find("RF_POS_UPDATE");
	h->with_lp_filter			=	param_find("RF_LP_FILTER");
	h->s_speed_threshold		=	param_find("RF_S_V_THR");
	h->s_flow_threshold			=	param_find("RF_S_FLOW_THR");
	h->s_regr_threshold			=	param_find("RF_S_REG_THR");
	h->s_alpha_default			=	param_find("RF_S_A_DEF");
	h->s_alpha_flow_max			=	param_find("RF_S_A_FLOWMAX");
	h->s_alpha_flow_min			=	param_find("RF_S_A_FLOWMIN");
	h->s_alpha_weight_down		=	param_find("RF_S_A_WDOWN");
	h->s_alpha_weight_fade_out	=	param_find("RF_S_A_WFOUT");
	h->s_window_size			=	param_find("RF_S_WIN_SIZE");
	h->s_window_threshold		=	param_find("RF_S_WIN_THR");
	h->debug					=	param_find("RF_DEBUG");

	return OK;
}

int parameters_update(const struct radar_flow_param_handles *h, struct radar_flow_params *p)
{
	param_get(h->beep_front_sonar, &(p->beep_front_sonar));
	param_get(h->kalman_k1, &(p->kalman_k1));
	param_get(h->kalman_k2, &(p->kalman_k2));
	param_get(h->front_lp_alpha, &(p->front_lp_alpha));
	param_get(h->with_sonar, &(p->with_sonar));
	param_get(h->with_pos_update, &(p->with_pos_update));
	param_get(h->with_lp_filter, &(p->with_lp_filter));
	param_get(h->s_speed_threshold, &(p->s_speed_threshold));
	param_get(h->s_flow_threshold, &(p->s_flow_threshold));
	param_get(h->s_regr_threshold, &(p->s_regr_threshold));
	param_get(h->s_alpha_default, &(p->s_alpha_default));
	param_get(h->s_alpha_flow_max, &(p->s_alpha_flow_max));
	param_get(h->s_alpha_flow_min, &(p->s_alpha_flow_min));
	param_get(h->s_alpha_weight_down, &(p->s_alpha_weight_down));
	param_get(h->s_alpha_weight_fade_out, &(p->s_alpha_weight_fade_out));
	param_get(h->s_window_size, &(p->s_window_size));
	param_get(h->s_window_threshold, &(p->s_window_threshold));
	param_get(h->debug, &(p->debug));

	p->filter_settings[0] = p->s_speed_threshold;
	p->filter_settings[1] = p->s_flow_threshold;
	p->filter_settings[2] = p->s_regr_threshold;
	p->filter_settings[3] = p->s_alpha_default;
	p->filter_settings[4] = p->s_alpha_flow_max;
	p->filter_settings[5] = p->s_alpha_flow_min;
	p->filter_settings[6] = p->s_alpha_weight_down;
	p->filter_settings[7] = p->s_alpha_weight_fade_out;

	p->filter_settings2[0] = p->s_speed_threshold;
	p->filter_settings2[1] = p->s_flow_threshold;
	p->filter_settings2[2] = p->s_regr_threshold;
	p->filter_settings2[3] = p->s_window_size;
	p->filter_settings2[4] = p->s_window_threshold;

	return OK;
}
