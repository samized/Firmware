/****************************************************************************
 *
 *   Copyright (C) 2008-2012 PX4 Development Team. All rights reserved.
 *   Author: Tobias Naegeli <naegelit@student.ethz.ch>
 *			 Thomas Gubler <thomasgubler@student.ethz.ch>
 *           Julian Oes <joes@student.ethz.ch>
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

/**
 * @file position_estimator_flow_main.c
 * Model-identification based position estimator for multirotors
 */

#include <nuttx/config.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <fcntl.h>
#include <float.h>
#include <nuttx/sched.h>
#include <sys/prctl.h>
#include <drivers/drv_hrt.h>
#include <termios.h>
#include <errno.h>
#include <limits.h>
#include <math.h>
#include <uORB/uORB.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_bodyframe_position.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/optical_flow.h>
#include <systemlib/perf_counter.h>
#include <poll.h>

#include "position_estimator_flow_params.h"

__EXPORT int position_estimator_flow_main(int argc, char *argv[]);
static bool thread_should_exit = false;		/**< Daemon exit flag */
static bool thread_running = false;		/**< Daemon status flag */
static int daemon_task;				/**< Handle of daemon task / thread */

int position_estimator_flow_thread_main(int argc, char *argv[]);
static void usage(const char *reason);

static void usage(const char *reason)
{
	if (reason)
		fprintf(stderr, "%s\n", reason);
	fprintf(stderr, "usage: daemon {start|stop|status} [-p <additional params>]\n\n");
	exit(1);
}

/**
 * The daemon app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to task_create().
 */
int position_estimator_flow_main(int argc, char *argv[])
{
	if (argc < 1)
		usage("missing command");

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			printf("flow position estimator already running\n");
			/* this is not an error */
			exit(0);
		}

		thread_should_exit = false;
		daemon_task = task_spawn("position_estimator_flow",
					 SCHED_RR,
					 SCHED_PRIORITY_MAX - 5,
					 4096,
					 position_estimator_flow_thread_main,
					 (argv) ? (const char **)&argv[2] : (const char **)NULL);
		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			printf("\tflow position estimator is running\n");
		} else {
			printf("\tflow position estimator not started\n");
		}
		exit(0);
	}

	usage("unrecognized command");
	exit(1);
}

int position_estimator_flow_thread_main(int argc, char *argv[])
{

	/* welcome user */
	thread_running = true;
	printf("[flow position estimator] starting\n");

	/* TODO should be a parameter */
	static const int8_t rotM_flow_sensor[3][3] =   {{  0, 1, 0 },
													{ -1, 0, 0 },
													{  0, 0, 1 }};
	const float const_earth_gravity = 9.81f;
	const float time_scale = pow(10,-6);
	static float speed[3] = {0.0f, 0.0f, 0.0f}; // x,y
	static float flow_speed[3] = {0.0f, 0.0f, 0.0f};
	static float global_speed[3] = {0.0f, 0.0f, 0.0f};
	static uint32_t counter = 0;
	static uint64_t time_last_flow = 0; // in ms
	static float dt = 0; // seconds
	static float sonar_last = 0;
	static bool sonar_valid = false;
	static float sonar_lp = 0.0f;

	/* barometer */
	static int baro_loop_cnt = 0;
	static int baro_loop_end = 100; /* 100Hz -> 1s */
	static float p0_Pa = 0.0f; /* to determin while start up */
	static float rho0 = 1.293f;
	static bool baro_initialized = false;
	static float barometer_height_est = 0.0f;

	/* subscribe to vehicle status, attitude, sensors and flow*/
	struct vehicle_status_s vstatus;
	memset(&vstatus, 0, sizeof(vstatus));
	struct vehicle_attitude_s att;
	memset(&att, 0, sizeof(att));
	struct vehicle_attitude_setpoint_s att_sp;
	memset(&att_sp, 0, sizeof(att_sp));
	struct optical_flow_s flow;
	memset(&flow, 0, sizeof(flow));
	struct sensor_combined_s sensor;
	memset(&sensor, 0, sizeof(sensor));

	/* subscribe to parameter changes */
	int parameter_update_sub = orb_subscribe(ORB_ID(parameter_update));

	/* subscribe to vehicle status */
	int vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));

	/* subscribe to attitude */
	int vehicle_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));

	/* subscribe to attitude setpoint */
	int vehicle_attitude_setpoint_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));

	/* subscribe to sensor combined */
	int sensor_combined_sub = orb_subscribe(ORB_ID(sensor_combined));

	/* subscribe to optical flow*/
	int optical_flow_sub = orb_subscribe(ORB_ID(optical_flow));

	/* publish global position messages */
	struct vehicle_local_position_s local_pos = {
		.x = 0.0f,
		.y = 0.0f,
		.z = 0.0f,
		.vx = 0.0f,
		.vy = 0.0f,
		.vz = 0.0f
	};
	struct vehicle_bodyframe_position_s bodyframe_pos = {
			.x = 0.0f,
			.y = 0.0f,
			.z = 0.0f,
			.vx = 0.0f,
			.vy = 0.0f,
			.vz = 0.0f
	};

	/* vehicle flying status parameters */
	bool vehicle_liftoff = false;
	bool vehicle_in_sonar_range = false;
	bool sensors_ready = false;

	/* parameters init*/
	struct position_estimator_flow_params params;
	struct position_estimator_flow_param_handles param_handles;
	parameters_init(&param_handles);
	parameters_update(&param_handles, &params);

	orb_advert_t local_pos_pub = orb_advertise(ORB_ID(vehicle_local_position), &local_pos);
	orb_advert_t bodyframe_pos_pub = orb_advertise(ORB_ID(vehicle_bodyframe_position), &bodyframe_pos);

	perf_counter_t mc_loop_perf = perf_alloc(PC_ELAPSED, "position_estimator_flow_runtime");
	perf_counter_t mc_interval_perf = perf_alloc(PC_INTERVAL, "position_estimator_flow_interval");
	perf_counter_t mc_err_perf = perf_alloc(PC_COUNT, "position_estimator_flow_err");

	while (!thread_should_exit) {

		if (sensors_ready) {
			/*This runs at the rate of the sensors */
			struct pollfd fds[3] = {
					{ .fd = optical_flow_sub, .events = POLLIN },
					{ .fd = sensor_combined_sub, .events = POLLIN },
					{ .fd = parameter_update_sub,   .events = POLLIN }
			};

			/* wait for a sensor update, check for exit condition every 500 ms */
			int ret = poll(fds, 3, 500);

			if (ret < 0) {
				/* poll error, count it in perf */
				perf_count(mc_err_perf);

			} else if (ret == 0) {
				/* no return value, ignore */
				printf("[flow position estimator] no bottom flow.\n");

			} else {

				/* parameter update available? */
				if (fds[2].revents & POLLIN){
					/* read from param to clear updated flag */
					struct parameter_update_s update;
					orb_copy(ORB_ID(parameter_update), parameter_update_sub, &update);

					parameters_update(&param_handles, &params);
					printf("[flow position estimator] parameters updated.\n");
				}

				/* only if flow data changed */
				if (fds[0].revents & POLLIN) {

					perf_begin(mc_loop_perf);

					orb_copy(ORB_ID(optical_flow), optical_flow_sub, &flow);
					/* got flow, updating attitude and status as well */
					orb_copy(ORB_ID(vehicle_attitude), vehicle_attitude_sub, &att);
					orb_copy(ORB_ID(vehicle_attitude_setpoint), vehicle_attitude_setpoint_sub, &att_sp);
					orb_copy(ORB_ID(vehicle_status), vehicle_status_sub, &vstatus);

					/* vehicle state estimation */
					float sonar = flow.ground_distance_m;

					// FIXME hardcoded...
					if (!vehicle_liftoff) {
						if (vstatus.flag_system_armed && att_sp.thrust > params.minimum_liftoff_thrust &&
								sonar > 0.3f && sonar < 1.0f && barometer_height_est > params.baro_liftoff_threshold) {
							/* vehicle on the way to a */
							vehicle_liftoff = true;
						}
					} else {
						if (!vstatus.flag_system_armed || (att_sp.thrust < params.minimum_liftoff_thrust &&
								sonar < 0.31f && barometer_height_est < 0.3f)) {
							vehicle_liftoff = false;
						}
					}

					/* calc dt between flow timestamps */
					/* ignore first flow msg */
					if(time_last_flow == 0)
					{
						time_last_flow = flow.timestamp;
						continue;
					}
					dt = (float)(flow.timestamp - time_last_flow) * time_scale ;
					time_last_flow = flow.timestamp;

					/* only make position update if vehicle is lift off */
					if (vehicle_liftoff || params.debug) {

						/*copy flow */
						flow_speed[0] = flow.flow_comp_x_m;
						flow_speed[1] = flow.flow_comp_y_m;
						flow_speed[2] = 0.0f;

						/* convert to bodyframe velocity */
						for(uint8_t i = 0; i < 3; i++) {
							float sum = 0.0f;
							for(uint8_t j = 0; j < 3; j++) {
								sum = sum + flow_speed[j] * rotM_flow_sensor[j][i];
							}
							speed[i] = sum;
						}

						/* update bodyframe position */
						bodyframe_pos.x = bodyframe_pos.x + speed[0] * dt;
						bodyframe_pos.y = bodyframe_pos.y + speed[1] * dt;
						bodyframe_pos.vx = speed[0];
						bodyframe_pos.vy = speed[1];

						// TODO add yaw rotation correction (distance to vehicle zero)

						/* convert to globalframe velocity -> not used for position control */
						for(uint8_t i = 0; i < 3; i++) {
							float sum = 0.0f;
							for(uint8_t j = 0; j < 3; j++) {
								sum = sum + speed[j] * att.R[i][j];
							}
							global_speed[i] = sum;
						}
						local_pos.x = local_pos.x + global_speed[0] * dt;
						local_pos.y = local_pos.y + global_speed[1] * dt;
						local_pos.vx = global_speed[0];
						local_pos.vy = global_speed[1];

					} else {

						/* set speed to zero and let position as it is */
						bodyframe_pos.vx = 0;
						bodyframe_pos.vy = 0;
						local_pos.vx = 0;
						local_pos.vy = 0;

					}

					/* filtering ground distance */
					if (!vehicle_liftoff || sonar > 2.0f) {
						sonar_valid = false;
					} else {
						sonar_valid = true;
					}

					if (!vehicle_liftoff || !vstatus.flag_system_armed) {
						/* not posible to fly */
						sonar_valid = false;
						bodyframe_pos.z = 0.0f;
						local_pos.z = 0.0f;
					}

					if (sonar_valid) {
						/* simple lowpass sonar filtering */

						/* if new value or with sonar update frequency */
						if (sonar != sonar_last || counter % 10 == 0) {
							sonar_lp = 0.05f * sonar + 0.95f * sonar_lp;
							sonar_last = sonar;
						}

						float height_diff = sonar - sonar_lp;

						/* if over 1/2m spike follow lowpass */
						if (height_diff < -params.sonar_lower_lp_threshold || height_diff > params.sonar_upper_lp_threshold)
						{
							bodyframe_pos.z = -sonar_lp;
							local_pos.z = -sonar_lp;

						} else
						{
							bodyframe_pos.z = -sonar;
							local_pos.z = -sonar;
						}

					}

					bodyframe_pos.timestamp = hrt_absolute_time();
					local_pos.timestamp = hrt_absolute_time();

					/* publish bodyframe position */
					if(isfinite(bodyframe_pos.x) && isfinite(bodyframe_pos.y) && isfinite(bodyframe_pos.z)
											&& isfinite(bodyframe_pos.vx) && isfinite(bodyframe_pos.vy))
					{
						orb_publish(ORB_ID(vehicle_bodyframe_position), bodyframe_pos_pub, &bodyframe_pos);
					}

					/* publish local position */
					if(isfinite(local_pos.x) && isfinite(local_pos.y) && isfinite(local_pos.z)
							&& isfinite(local_pos.vx) && isfinite(local_pos.vy))
					{
						orb_publish(ORB_ID(vehicle_local_position), local_pos_pub, &local_pos);
					}

					/* measure in what intervals the position estimator runs */
					perf_count(mc_interval_perf);
					perf_end(mc_loop_perf);

				}

				/* only if sensor data changed */
				if (fds[1].revents & POLLIN) {

					orb_copy(ORB_ID(sensor_combined), sensor_combined_sub, &sensor);
					orb_copy(ORB_ID(vehicle_status), vehicle_status_sub, &vstatus);

					if (!baro_initialized){
						// mean calculation over several measurements
						if(baro_loop_cnt < baro_loop_end) {
							p0_Pa += (sensor.baro_pres_mbar * 100);
							baro_loop_cnt++;
						}else{
							p0_Pa /= (float)(baro_loop_cnt);
							baro_initialized = true;
							printf("barometer initialized...\n");


							/* send log info */
//							char *baro_m_start = "barometer initialized with p0 = ";
//							char p0_char[15];
//							sprintf(p0_char, "%8.2f", p0_Pa/100);
//							char *baro_m_end = " mbar";
//							char str[80];
//							strcpy(str,baro_m_start);
//							strcat(str,p0_char);
//							strcat(str,baro_m_end);
//							mavlink_log_info(mavlink_fd, str);
						}
					}

					if(baro_initialized){
						float h_est = p0_Pa*log(p0_Pa/(sensor.baro_pres_mbar*100))/(rho0*const_earth_gravity);
						barometer_height_est = 0.005f * h_est + 0.995f * barometer_height_est;
					}
				}
			}

		} else {
			/* sensors not ready waiting for first attitude msg */

			/* polling */
			struct pollfd fds[1] = {
				{ .fd = vehicle_attitude_sub, .events = POLLIN },
			};

			/* wait for a flow msg, check for exit condition every 5 s */
			int ret = poll(fds, 1, 5000);

			if (ret < 0) {
				/* poll error, count it in perf */
				perf_count(mc_err_perf);

			} else if (ret == 0) {
				/* no return value, ignore */
				printf("[flow position estimator] no attitude received.\n");
			} else {

				if (fds[0].revents & POLLIN){
					sensors_ready = true;
					printf("[flow position estimator] initialized.\n");
				}
			}
		}

		counter++;
	}

	printf("[flow position estimator] exiting.\n");
	thread_running = false;

	//close(vehicle_attitude_sub);
	//close(vehicle_status_sub);
	close(optical_flow_sub);

	perf_print_counter(mc_loop_perf);
	perf_free(mc_loop_perf);

	fflush(stdout);
	return 0;
}


