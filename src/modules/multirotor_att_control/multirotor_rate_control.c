/****************************************************************************
 *
 *   Copyright (C) 2012-2013 PX4 Development Team. All rights reserved.
 *   Author: Tobias Naegeli <naegelit@student.ethz.ch>
 *           Lorenz Meier <lm@inf.ethz.ch>
 *           Anton Babushkin <anton.babushkin@me.com>
 *           Julian Oes <joes@student.ethz.ch>
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

/**
 * @file multirotor_rate_control.c
 *
 * Implementation of rate controller for multirotors.
 *
 * @author Tobias Naegeli <naegelit@student.ethz.ch>
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author Anton Babushkin <anton.babushkin@me.com>
 * @author Julian Oes <joes@student.ethz.ch>
 */

#include "multirotor_rate_control.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <float.h>
#include <math.h>
#include <systemlib/pid/pid.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <drivers/drv_hrt.h>
#include "multirotor_att_control_params.h"

void multirotor_control_rates(const struct vehicle_rates_setpoint_s *rate_sp,
			      const float rates[], struct actuator_controls_s *actuators,
			      struct multirotor_att_control_params *params, bool params_updated)
{
	static uint64_t last_run = 0;
	const float deltaT = (hrt_absolute_time() - last_run) / 1000000.0f;
	static uint64_t last_input = 0;

	if (last_input != rate_sp->timestamp) {
		last_input = rate_sp->timestamp;
	}

	last_run = hrt_absolute_time();

	static int motor_skip_counter = 0;

	static PID_t pitch_rate_controller;
	static PID_t roll_rate_controller;

	static bool initialized = false;

	/* initialize the pid controllers when the function is called for the first time */
	if (initialized == false)
	{
		pid_init(&pitch_rate_controller, params->attrate_p, params->attrate_i, params->attrate_d,
				1.0f, 1.0f, PID_MODE_DERIVATIV_CALC_NO_SP, 0.003f);
		pid_init(&roll_rate_controller, params->attrate_p, params->attrate_i, params->attrate_d,
				1.0f, 1.0f, PID_MODE_DERIVATIV_CALC_NO_SP, 0.003f);

		initialized = true;
	}

	/* load new parameters if needed */
	if (params_updated)
	{
		/* update pid parameters */
		pid_set_parameters(&pitch_rate_controller, params->attrate_p, params->attrate_i, params->attrate_d, 1.0f, 1.0f);
		pid_set_parameters(&roll_rate_controller,  params->attrate_p, params->attrate_i, params->attrate_d, 1.0f, 1.0f);
	}

	/* reset integral if on ground */
	if (rate_sp->thrust < 0.01f) {
		pid_reset_integral(&pitch_rate_controller);
		pid_reset_integral(&roll_rate_controller);
	}

	/* control pitch (forward) output */
	float pitch_control = pid_calculate(&pitch_rate_controller, rate_sp->pitch ,
					    rates[1], 0.0f, deltaT);

	/* control roll (left/right) output */
	float roll_control = pid_calculate(&roll_rate_controller, rate_sp->roll ,
					   rates[0], 0.0f, deltaT);

	/* control yaw rate */ //XXX use library here
	float yaw_rate_control = params->yawrate_p * (rate_sp->yaw - rates[2]);

	/* increase resilience to faulty control inputs */
	if (!isfinite(yaw_rate_control)) {
		yaw_rate_control = 0.0f;
		warnx("rej. NaN ctrl yaw");
	}

	actuators->control[0] = roll_control;
	actuators->control[1] = pitch_control;
	actuators->control[2] = yaw_rate_control;
	actuators->control[3] = rate_sp->thrust;

	motor_skip_counter++;
}
