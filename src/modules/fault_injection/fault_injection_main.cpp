/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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

#include "fault_injection.h"

#include <px4_getopt.h>
#include <px4_log.h>
#include <px4_posix.h>
#include <unistd.h>

#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/fault_injection.h>
#include <uORB/topics/debug_value.h>


int FaultInjection::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Section that describes the provided module functionality.

This is a template for a module running as a task in the background with start/stop/status functionality.

### Implementation
Section describing the high-level implementation of this module.

### Examples
CLI usage example:
$ module start -f -p 42

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("module", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_FLAG('f', "Optional example flag", true);
	PRINT_MODULE_USAGE_PARAM_INT('p', 0, 0, 1000, "Optional example parameter", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int FaultInjection::print_status()
{
	PX4_INFO("Running");
	// TODO: print additional runtime information about the state of the module

	return 0;
}

int FaultInjection::custom_command(int argc, char *argv[])
{
	/*
	if (!is_running()) {
		print_usage("not running");
		return 1;
	}

	// additional custom commands can be handled like this:
	if (!strcmp(argv[0], "do-something")) {
		get_instance()->do_something();
		return 0;
	}
	 */

	return print_usage("unknown command");
}


int FaultInjection::task_spawn(int argc, char *argv[])
{
    _task_id = px4_task_spawn_cmd("fault_injection",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_DEFAULT,
                      2000,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

FaultInjection *FaultInjection::instantiate(int argc, char *argv[])
{
	int example_param = 0;
	bool example_flag = false;
	bool error_flag = false;

	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	// parse CLI arguments
	while ((ch = px4_getopt(argc, argv, "p:f", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'p':
			example_param = (int)strtol(myoptarg, nullptr, 10);
			break;

		case 'f':
			example_flag = true;
			break;

		case '?':
			error_flag = true;
			break;

		default:
			PX4_WARN("unrecognized flag");
			error_flag = true;
			break;
		}
	}

	if (error_flag) {
		return nullptr;
	}

    FaultInjection *instance = new FaultInjection(example_param, example_flag);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

FaultInjection::FaultInjection(int example_param, bool example_flag)
	: ModuleParams(nullptr)
{
}

void FaultInjection::run()
{
    /* subscribe to debug_value topic */

    int debug_sub_fd = orb_subscribe(ORB_ID(debug_value));

    /* wakeup source: gyro data from sensor selected by the sensor app */
    px4_pollfd_struct_t poll_fds = {};
    poll_fds.events = POLLIN;

    /* advertise fault_injection topic */
    struct fault_injection_s fi;
    memset(&fi, 0, sizeof(fi));
    orb_advert_t fi_pub = orb_advertise(ORB_ID(fault_injection), &fi);

    int error_counter = 0;

	while (!should_exit()) {

        /* wait for sensor update of 1 file descriptor for 1000 ms (1 second) */
        poll_fds.fd = debug_sub_fd;

        /* wait for up to 100ms for data */
        int poll_ret = px4_poll(&poll_fds, 1, 1000);

        /* handle the poll result */
        if (poll_ret == 0) {
            /* this means none of our providers is giving us data */
            //PX4_INFO("Got no data within a second, running...");
            continue;

        } else if (poll_ret < 0) {
            /* this is seriously bad - should be an emergency */
            if (error_counter < 10 || error_counter % 50 == 0) {
                /* use a counter to prevent flooding (and slowing us down) */
                PX4_ERR("ERROR return value from poll(): %d", poll_ret);
            }

            error_counter++;

        } else {

            /* obtained data for the first file descriptor */
            struct debug_value_s debug_topic;
            /* copy sensors raw data into local buffer */
            orb_copy(ORB_ID(debug_value), debug_sub_fd, &debug_topic);
            /*PX4_INFO("Accelerometer:\t%8.4f\t%8.4f\t%8.4f",
                 (double)raw.accelerometer_m_s2[0],
                 (double)raw.accelerometer_m_s2[1],
                 (double)raw.accelerometer_m_s2[2]);*/

            /* set att and publish this information for other apps
             the following does not have any meaning, it's just an example
            */
            fi.fault_name = debug_topic.ind;
            fi.fault_value = debug_topic.value;

            PX4_INFO("fault recv:%d %f",fi.fault_name, (double)fi.fault_value);

            orb_publish(ORB_ID(fault_injection), fi_pub, &fi);
        }

    }

}


void FaultInjection::parameters_update(int parameter_update_sub, bool force)
{
	bool updated;
	struct parameter_update_s param_upd;

	orb_check(parameter_update_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(parameter_update), parameter_update_sub, &param_upd);
	}

	if (force || updated) {
		updateParams();
	}
}


int fault_injection_main(int argc, char *argv[])
{
    return FaultInjection::main(argc, argv);
}
