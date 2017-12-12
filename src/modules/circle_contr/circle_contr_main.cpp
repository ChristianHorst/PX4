/****************************************************************************
 *
 *   Copyright (c) 2013-2015 PX4 Development Team. All rights reserved.
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
 * @file circle_contr_main.cpp
 * Hippocampus path controller.
 *
 * Publication for the desired attitude tracking:
 * Daniel Mellinger and Vijay Kumar. Minimum Snap Trajectory Generation and Control for Quadrotors.
 * Int. Conf. on Robotics and Automation, Shanghai, China, May 2011.
 *
 * based on mc_att_control_main.cpp from
 * @author Lorenz Meier		<lorenz@px4.io>
 * @author Anton Babushkin	<anton.babushkin@me.com>
 * @author Sander Smeets	<sander@droneslab.com>
 *
 * adjusted by
 * @author Viktor Rausch
 *
 * simple app to control yaw and thrust
 */


#include <px4_config.h>
#include <px4_defines.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <drivers/drv_hrt.h>
#include <arch/board/board.h>
// uORB topics
#include <uORB/uORB.h>
#include <uORB/topics/actuator_controls.h>              // this topic gives the actuators control input
#include <uORB/topics/vehicle_attitude.h>               // orientation data
#include <uORB/topics/vehicle_local_position.h>         // position data
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/logging_hippocampus.h>            // logging message to have everything in one at the same time steps
// system libraries
#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <systemlib/circuit_breaker.h>
// internal libraries
#include <lib/mathlib/mathlib.h>
#include <lib/geo/geo.h>
#include <lib/tailsitter_recovery/tailsitter_recovery.h>


// Hippocampus path controller
extern "C" __EXPORT int circle_contr_main(int argc, char *argv[]);

// the class from which an instance will be initiated by starting this application
class HippocampusCircleControl
{
public:
	// Constructor
 HippocampusCircleControl(char *type_ctrl);


	// Destructor, also kills the main task
    ~HippocampusCircleControl();

	// Start the multicopter attitude control task, @return OK on success.
	int	start();

private:

	bool	_task_should_exit;		// if true, task_main() should exit
	int		_control_task;			// task handle

	// topic subscriptions

	int		_params_sub;			// parameter updates subscription

	// topic publications
	orb_advert_t	_actuators_0_pub;		    // attitude actuator controls publication
	orb_advert_t    _logging_hippocampus_pub;   // logging data publisher
	orb_id_t        _actuators_id;	            // pointer to correct actuator controls0 uORB metadata structure

	// topic structures, in this structures the data of the topics are stored
	struct actuator_controls_s			_actuators;			    // actuator controls
	struct logging_hippocampus_s        _logging_hippocampus;   // logging data

	// performance counters
	perf_counter_t	_loop_perf;
	perf_counter_t	_controller_latency_perf;


	// time
	float               t_ges;
	float               counter;

    // controller type
	char type_array[100];


	struct {

        param_t YAW;
        param_t THRUST;

	}		_params_handles;		// handles for to find parameters

	struct {

        float thrust;
		float yaw;
	}		_params;


    // circle controller.
        void		circle_control();

	// Update our local parameter cache.
	int			parameters_update();                // checks if parameters have changed and updates them

	// Check for parameter update and handle it.
	void		parameter_update_poll();            // receives parameters

	// Shim for calling task_main from task_create.
	static void	task_main_trampoline(int argc, char *argv[]);

	// Main attitude control task.
	void		task_main();
};


namespace circle_contr
{
HippocampusCircleControl	*g_control;
}

// constructor of class HippocampusCircleControl
HippocampusCircleControl::HippocampusCircleControl(char *type_ctrl) :

	// First part is about function which are called with the constructor
	_task_should_exit(false),
	_control_task(-1),

	// subscriptions
	_params_sub(-1),


	// publications
	_actuators_0_pub(nullptr),
	_logging_hippocampus_pub(nullptr),
	_actuators_id(nullptr),


	// performance counters
        _loop_perf(perf_alloc(PC_ELAPSED, "circle_contr")),
	_controller_latency_perf(perf_alloc_once(PC_ELAPSED, "ctrl_latency"))

// here starts the allocation of values to the variables
{
	// define publication settings

	memset(&_actuators, 0, sizeof(_actuators));
	memset(&_logging_hippocampus, 0, sizeof(_logging_hippocampus));


	// set parameters to zero
    _params.thrust = 0.0f;
	_params.yaw = 0.0f;


	t_ges = 0.0;
	counter = 0.0;

	// allocate controller type
	strcpy(&type_array[0], type_ctrl);

    if (!strcmp(type_array, "start")) {
        PX4_INFO("Start Circle controller!");


	}


	// allocate parameter handles

    _params_handles.THRUST	        = 	param_find("CC_THRUST");
    _params_handles.YAW	            = 	param_find("CC_YAW");

	// fetch initial parameter values
	parameters_update();
}

// destructor of class HippocampusCircleControl
HippocampusCircleControl::~HippocampusCircleControl()
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
				px4_task_delete(_control_task);
				break;
			}
		} while (_control_task != -1);
	}

        circle_contr::g_control = nullptr;
}

// updates parameters
int HippocampusCircleControl::parameters_update()
{
	float v;


    param_get(_params_handles.THRUST, &v);
    _params.thrust = v;
	param_get(_params_handles.YAW, &v);
	_params.yaw = v;

	return OK;
}

// check for parameter updates
void HippocampusCircleControl::parameter_update_poll()
{
	bool updated;

	/* Check if parameters have changed */
	orb_check(_params_sub, &updated);

	if (updated) {
		struct parameter_update_s param_update;
		orb_copy(ORB_ID(parameter_update), _params_sub, &param_update);
		parameters_update();


	}
}




void HippocampusCircleControl::circle_control()
{


	// give the inputs to the actuators
    _actuators.control[0] = 0;           // roll
    _actuators.control[1] = 0;           // pitch
    _actuators.control[2] = _params.yaw;           // yaw
    _actuators.control[3] = _params.thrust;           // thrust




}

// Just starts the task_main function
void HippocampusCircleControl::task_main_trampoline(int argc, char *argv[])
{
        circle_contr::g_control->task_main();
}

// this is the main_task function which does the control task
void HippocampusCircleControl::task_main()
{

    PX4_INFO(" Circle Controller has been started!");
	// subscribe to uORB topics

	_params_sub = orb_subscribe(ORB_ID(parameter_update));


	// initialize parameters cache
	parameters_update();

	while (!_task_should_exit) {

                // do path control
                        usleep(20000);
                        circle_control();
                    // check for parameter updates

                        // publish actuator timestamps


                        if (_actuators_0_pub != nullptr) {
                            orb_publish(_actuators_id, _actuators_0_pub, &_actuators);
                        //    perf_end(_controller_latency_perf);

                        } else if (_actuators_id) {
                            _actuators_0_pub = orb_advertise(_actuators_id, &_actuators);
                        }


                        _actuators_id = ORB_ID(actuator_controls_0);

                        parameter_update_poll();



	}


}




// start function
int HippocampusCircleControl::start()
{
	ASSERT(_control_task == -1);        // give a value -1

	// start the control task, performs any specific accounting, scheduler setup, etc.
        _control_task = px4_task_spawn_cmd("circle_contr",
					   SCHED_DEFAULT,
					   SCHED_PRIORITY_MAX - 5,
					   1500,
                       (px4_main_t)&HippocampusCircleControl::task_main_trampoline,
					   nullptr);

	if (_control_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}

// main function
int circle_contr_main(int argc, char *argv[])
{
	if (argc < 2) {
                warnx("usage: circle_contr {start|stop|status}");
		return 1;
	}

	// if command is start, then first control if class exists already, if not, allocate new one
    if (!strcmp(argv[1], "start") ) {

                if (circle_contr::g_control != nullptr) {
			warnx("already running");
			return 1;
		}

        // allocate new class HippocampusCircleControl
                circle_contr::g_control = new HippocampusCircleControl(argv[1]);

		// check if class has been allocated, if not, give back failure
                if (circle_contr::g_control == nullptr) {
			warnx("alloc failed");
			return 1;
		}

        // if function start() can not be called, delete instance of HippocampusCircleControl and allocate null pointer
                if (OK != circle_contr::g_control->start()) {
                        delete circle_contr::g_control;
                        circle_contr::g_control = nullptr;
			warnx("start failed");
			return 1;
		}

		return 0;
	}

	// if command is start, check if class exists, if not can not stop anything
	if (!strcmp(argv[1], "stop")) {
                if (circle_contr::g_control == nullptr) {
			warnx("not running");
			return 1;
		}

		// if class exists, delete it and allocate null pointer
                delete circle_contr::g_control;
                circle_contr::g_control = nullptr;
		return 0;
	}

	// if command is status and class exists, give back running, else give back not running
	if (!strcmp(argv[1], "status")) {
                if (circle_contr::g_control) {
			warnx("running");
			return 0;

		} else {
			warnx("not running");
			return 1;
		}
	}

	warnx("unrecognized command");
	return 1;
}
