/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *   Author: Tobias Naegeli <naegelit@student.ethz.ch>
 *           Lorenz Meier <lm@inf.ethz.ch>
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
 * @file flow_navigation_params.c
 * 
 * Parameters for EKF filter
 */

#include "flow_navigation_params.h"

/* Extended Kalman Filter covariances */

/* controller parameters */
PARAM_DEFINE_FLOAT(FN_POS_SP_X, 0.0f);
PARAM_DEFINE_FLOAT(FN_POS_SP_Y, 0.0f);
PARAM_DEFINE_FLOAT(FN_BEEP_F, 0.0f);
PARAM_DEFINE_FLOAT(FN_BEEP_B, 0.0f);
PARAM_DEFINE_FLOAT(FN_KAL_K1, 0.0235f);
PARAM_DEFINE_FLOAT(FN_KAL_K2, 0.0140f);

int parameters_init(struct flow_navigation_param_handles *h)
{
	/* PID parameters */
	h->pos_sp_x	 			=	param_find("FN_POS_SP_X");
	h->pos_sp_y				=	param_find("FN_POS_SP_Y");
	h->beep_front_sonar		=	param_find("FN_BEEP_F");
	h->beep_bottom_sonar	=	param_find("FN_BEEP_B");
	h->kalman_k1	 		=	param_find("FN_KAL_K1");
	h->kalman_k2			=	param_find("FN_KAL_K2");

	return OK;
}

int parameters_update(const struct flow_navigation_param_handles *h, struct flow_navigation_params *p)
{
	param_get(h->pos_sp_x, &(p->pos_sp_x));
	param_get(h->pos_sp_y, &(p->pos_sp_y));
	param_get(h->beep_front_sonar, &(p->beep_front_sonar));
	param_get(h->beep_bottom_sonar, &(p->beep_bottom_sonar));
	param_get(h->kalman_k1, &(p->kalman_k1));
	param_get(h->kalman_k2, &(p->kalman_k2));

	return OK;
}