/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
 *   Author: 	Lorenz Meier
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
 * @file att_pos_estimator_inav.cpp
 *
 * Original implementation for inertial navigation filter:
 *    Paul Riseborough
 *
 * More details and acknowledgements in the referenced library headers.
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 */

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <time.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_accel.h>
#include <arch/board/board.h>
#include <uORB/uORB.h>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_global_position_set_triplet.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/navigation_capabilities.h>
#include <uORB/topics/parameter_update.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <systemlib/pid/pid.h>
#include <geo/geo.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <mathlib/mathlib.h>

#include <external_lgpl/inav/inav.h>

/**
 * Estimator start / stop function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int att_pos_estimator_main(int argc, char *argv[]);

class AttitudePositionEstimatorInav
{
public:
	/**
	 * Constructor
	 */
	AttitudePositionEstimatorInav();

	/**
	 * Destructor, also kills the sensors task.
	 */
	~AttitudePositionEstimatorInav();

	/**
	 * Start the sensors task.
	 *
	 * @return		OK on success.
	 */
	int		start();

private:

	bool		_task_should_exit;		/**< if true, sensor task should exit */
	int		_control_task;			/**< task handle for sensor task */

	int		_global_pos_sub;
	int		_global_set_triplet_sub;
	int		_att_sub;			/**< vehicle attitude subscription */
	int		_attitude_sub;			/**< raw rc channels data subscription */
	int		_airspeed_sub;			/**< airspeed subscription */
	int		_control_mode_sub;		/**< vehicle status subscription */
	int 		_params_sub;			/**< notification of parameter updates */
	int 		_manual_control_sub;		/**< notification of manual control updates */
	int		_accel_sub;			/**< body frame accelerations */

	orb_advert_t	_attitude_sp_pub;		/**< attitude setpoint */
	orb_advert_t	_nav_capabilities_pub;		/**< navigation capabilities publication */

	struct vehicle_attitude_s			_att;				/**< vehicle attitude */
	struct airspeed_s				_airspeed;			/**< airspeed */
	struct vehicle_global_position_s		_global_pos;			/**< global vehicle position */
	struct accel_report				_accel_primary;			/**< body frame accelerations */
	struct mag_report				_mag_primary;			/**< body frame magnet vector */
	struct gyro_report				_gyro_primary;			/**< body frame angular rates */
	struct accel_report				_accel_secondary;		/**< body frame accelerations */
	struct mag_report				_mag_secondary;			/**< body frame magnet vector */
	struct gyro_report				_gyro_secondary;		/**< body frame angular rates */

	perf_counter_t	_loop_perf;			/**< loop performance counter */

	bool		_setpoint_valid;		/**< flag if the position control setpoint is valid */

	/** manual control states */

	math::Dcm _R_nb;				///< current attitude

	INav				_nav;

	struct {
		float l1_period;
	}		_parameters;			/**< local copies of interesting parameters */

	struct {

		param_t l1_period;
	}		_parameter_handles;		/**< handles for interesting parameters */


	/**
	 * Update our local parameter cache.
	 */
	int		parameters_update();

	/**
	 * Update control outputs
	 *
	 */
	void		control_update();

	/**
	 * Check for changes in vehicle status.
	 */
	void		vehicle_control_mode_poll();

	/**
	 * Check for airspeed updates.
	 */
	bool		vehicle_airspeed_poll();

	/**
	 * Check for position updates.
	 */
	void		vehicle_attitude_poll();

	/**
	 * Check for accel updates.
	 */
	void		vehicle_accel_poll();

	/**
	 * Check for mag updates.
	 */
	void		vehicle_mag_poll();

	/**
	 * Check for gyro updates.
	 */
	void		vehicle_gyro_poll();

	/**
	 * Control position.
	 */
	bool		control_position(const math::Vector2f &global_pos, const math::Vector2f &ground_speed,
					 const struct vehicle_global_position_set_triplet_s &global_triplet);

	/**
	 * Shim for calling task_main from task_create.
	 */
	static void	task_main_trampoline(int argc, char *argv[]);

	/**
	 * Main sensor collection task.
	 */
	void		task_main() __attribute__((noreturn));
};

namespace l1_control
{

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

AttitudePositionEstimatorInav	*g_control;
}

AttitudePositionEstimatorInav::AttitudePositionEstimatorInav() :

	_task_should_exit(false),
	_control_task(-1),

/* subscriptions */
	_global_pos_sub(-1),
	_global_set_triplet_sub(-1),
	_airspeed_sub(-1),
	_control_mode_sub(-1),
	_params_sub(-1),
	_manual_control_sub(-1),

/* publications */
	_att_pub(-1),

/* performance counters */
	_loop_perf(perf_alloc(PC_ELAPSED, "fw l1 control")),
/* states */
	_airspeed_valid(false),
	_global_pos_valid(false),
{

	_parameter_handles.l1_period = param_find("FW_L1_PERIOD");

	/* fetch initial parameter values */
	parameters_update();
}

AttitudePositionEstimatorInav::~AttitudePositionEstimatorInav()
{
	if (_control_task != -1) {

		/* task wakes up every 100ms or so at the longest */
		_task_should_exit = true;

		/* wait for a second for the task to quit at our request */
		unsigned i = 0;

		do {
			/* wait 20ms */
			usleep(20000);

			/* if we have given up, kill it */
			if (++i > 50) {
				task_delete(_control_task);
				break;
			}
		} while (_control_task != -1);
	}

	l1_control::g_control = nullptr;
}

int
AttitudePositionEstimatorInav::parameters_update()
{

	/* L1 control parameters */
	param_get(_parameter_handles.l1_period, &(_parameters.l1_period));

	_l1_control.set_l1_period(_parameters.l1_period);

	return OK;
}

bool
AttitudePositionEstimatorInav::vehicle_airspeed_poll()
{
	/* check if there is an airspeed update or if it timed out */
	bool airspeed_updated;
	orb_check(_airspeed_sub, &airspeed_updated);

	if (airspeed_updated) {
		orb_copy(ORB_ID(airspeed), _airspeed_sub, &_airspeed);
		_airspeed_valid = true;
		_airspeed_last_valid = hrt_absolute_time();

	} else {

		/* no airspeed updates for one second */
		if (_airspeed_valid && (hrt_absolute_time() - _airspeed_last_valid) > 1e6) {
			_airspeed_valid = false;
		}
	}

	/* update TECS state */
	_tecs.enable_airspeed(_airspeed_valid);

	return airspeed_updated;
}

void
AttitudePositionEstimatorInav::vehicle_accel_poll()
{
	/* check if there is a new position */
	bool accel_updated;
	orb_check(_accel_sub, &accel_updated);

	if (accel_updated) {
		orb_copy(ORB_ID(sensor_accel), _accel_sub, &_accel);
	}
}

void
AttitudePositionEstimatorInav::task_main_trampoline(int argc, char *argv[])
{
	l1_control::g_control->task_main();
}


bool
AttitudePositionEstimatorInav::control_position(const math::Vector2f &current_position, const math::Vector2f &ground_speed,
		const struct vehicle_global_position_set_triplet_s &global_triplet)
{
	
	return setpoint;
}

void
AttitudePositionEstimatorInav::task_main()
{

	/* inform about start */
	warnx("Initializing..");
	fflush(stdout);

	/*
	 * do subscriptions
	 */
	_gps_pos_sub =// orb_subscribe(ORB_ID(vehicle_global_position));
	_accel_sub = orb_subscribe(ORB_ID(sensor_accel));
	_gyro_sub = orb_subscribe(ORB_ID(sensor_gyro));
	_mag_sub = orb_subscribe(ORB_ID(sensor_mag));
	_airspeed_sub = orb_subscribe(ORB_ID(airspeed));
	_params_sub = orb_subscribe(ORB_ID(parameter_update));

	/* abort on a nonzero return value from the parameter init */
	if (parameters_update()) {
		/* parameter setup went wrong, abort */
		warnx("aborting startup due to errors.");
		_task_should_exit = true;
	}

	/* wakeup source(s) */
	struct pollfd fds[2];

	/* Setup of loop */
	fds[0].fd = _params_sub;
	fds[0].events = POLLIN;
	fds[1].fd = _global_pos_sub;
	fds[1].events = POLLIN;

	while (!_task_should_exit) {

		/* wait for up to 500ms for data */
		int pret = poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 100);

		/* timed out - periodic check for _task_should_exit, etc. */
		if (pret == 0)
			continue;

		/* this is undesirable but not much we can do - might want to flag unhappy status */
		if (pret < 0) {
			warn("poll error %d, %d", pret, errno);
			continue;
		}

		perf_begin(_loop_perf);

		/* only update parameters if they changed */
		if (fds[0].revents & POLLIN) {
			/* read from param to clear updated flag */
			struct parameter_update_s update;
			orb_copy(ORB_ID(parameter_update), _params_sub, &update);

			/* update parameters from storage */
			parameters_update();
		}

		/* only run estimator if gyros updated */
		if (fds[1].revents & POLLIN) {


			static uint64_t last_run = 0;
			float deltaT = (hrt_absolute_time() - last_run) / 1000000.0f;
			last_run = hrt_absolute_time();

			/* guard against too large deltaT's */
			if (deltaT > 1.0f)
				deltaT = 0.01f;

			/* load local copies */
			orb_copy(ORB_ID(vehicle_global_position), _global_pos_sub, &_global_pos);

			vehicle_accel_poll();
			vehicle_airspeed_poll();

			/* run one step */

		}

		perf_end(_loop_perf);
	}

	warnx("exiting.\n");

	_control_task = -1;
	_exit(0);
}

int
AttitudePositionEstimatorInav::start()
{
	ASSERT(_control_task == -1);

	/* start the task */
	_control_task = task_spawn_cmd("fw_pos_control_l1",
				       SCHED_DEFAULT,
				       SCHED_PRIORITY_MAX - 5,
				       4048,
				       (main_t)&AttitudePositionEstimatorInav::task_main_trampoline,
				       nullptr);

	if (_control_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}

int att_pos_estimator_main(int argc, char *argv[])
{
	if (argc < 1)
		errx(1, "usage: att_pos_estimator_inav {start|stop|status}");

	if (!strcmp(argv[1], "start")) {

		if (l1_control::g_control != nullptr)
			errx(1, "already running");

		l1_control::g_control = new AttitudePositionEstimatorInav;

		if (l1_control::g_control == nullptr)
			errx(1, "alloc failed");

		if (OK != l1_control::g_control->start()) {
			delete l1_control::g_control;
			l1_control::g_control = nullptr;
			err(1, "start failed");
		}

		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		if (l1_control::g_control == nullptr)
			errx(1, "not running");

		delete l1_control::g_control;
		l1_control::g_control = nullptr;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (l1_control::g_control) {
			errx(0, "running");

		} else {
			errx(1, "not running");
		}
	}

	warnx("unrecognized command");
	return 1;
}
