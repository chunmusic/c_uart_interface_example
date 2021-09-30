/****************************************************************************
 *
 *   Copyright (c) 2014 MAVlink Development Team. All rights reserved.
 *   Author: Trent Lukaczyk, <aerialhedgehog@gmail.com>
 *           Jaycee Lock,    <jaycee.lock@gmail.com>
 *           Lorenz Meier,   <lm@inf.ethz.ch>
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
 * @file mavlink_control.cpp
 *
 * @brief An example offboard control process via mavlink
 *
 * This process connects an external MAVLink UART device to send an receive data
 *
 * @author Trent Lukaczyk, <aerialhedgehog@gmail.com>
 * @author Jaycee Lock,    <jaycee.lock@gmail.com>
 * @author Lorenz Meier,   <lm@inf.ethz.ch>
 *
 */



// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#include "mavlink_control.h"


// ------------------------------------------------------------------------------
//   TOP
// ------------------------------------------------------------------------------
int
top (int argc, char **argv)
{

	bool autotakeoff = false;

	Generic_Port *port;

	port = new Serial_Port("/dev/ttyACM8", 921600);

	Autopilot_Interface autopilot_interface(port);


	port_quit         = port;
	autopilot_interface_quit = &autopilot_interface;
	signal(SIGINT,quit_handler);

	/*
	 * Start the port and autopilot_interface
	 * This is where the port is opened, and read and write threads are started.
	 */
	port->start();
	autopilot_interface.start();


	// --------------------------------------------------------------------------
	//   RUN COMMANDS
	// --------------------------------------------------------------------------

	/*
	 * Now we can implement the algorithm we want on top of the autopilot interface
	 */
	commands(autopilot_interface, autotakeoff);


	// --------------------------------------------------------------------------
	//   THREAD and PORT SHUTDOWN
	// --------------------------------------------------------------------------

	/*
	 * Now that we are done we can stop the threads and close the port
	 */
	autopilot_interface.stop();
	port->stop();

	delete port;

	// --------------------------------------------------------------------------
	//   DONE
	// --------------------------------------------------------------------------

	// woot!
	return 0;

}


// ------------------------------------------------------------------------------
//   COMMANDS
// ------------------------------------------------------------------------------

void
commands(Autopilot_Interface &api, bool autotakeoff)
{

	// --------------------------------------------------------------------------
	//   GET A MESSAGE
	// --------------------------------------------------------------------------
	printf("READ SOME MESSAGES \n");

	// copy current messages
	// Mavlink_Messages messages = api.current_messages;

    while (true)
    {
        // local position in ned frame

		Mavlink_Messages messages = api.current_messages;

		mavlink_uav1_thrust_t uav1_thrust = messages.uav1_thrust;
		mavlink_uav2_thrust_t uav2_thrust = messages.uav2_thrust;
		mavlink_uav3_thrust_t uav3_thrust = messages.uav3_thrust;
		mavlink_uav4_thrust_t uav4_thrust = messages.uav4_thrust;

		mavlink_uav_command_t uav_command = messages.uav_command;

		printf("nav_state: %d \n", uav_command.nav_state);
		printf("arming_state: %d \n", uav_command.arming_state);

		printf("armed: %d \n", uav_command.armed);
		printf("prearmed: %d \n", uav_command.prearmed);

		printf("1_acc[0]: %f \n", uav1_thrust.actuator_control[0]);
		printf("1_acc[1]: %f \n", uav1_thrust.actuator_control[1]);
		printf("1_acc[2]: %f \n", uav1_thrust.actuator_control[2]);
		printf("1_acc[3]: %f \n", uav1_thrust.actuator_control[3]);
		
		printf("2_acc[0]: %f \n", uav2_thrust.actuator_control[0]);
		printf("2_acc[1]: %f \n", uav2_thrust.actuator_control[1]);
		printf("2_acc[2]: %f \n", uav2_thrust.actuator_control[2]);
		printf("2_acc[3]: %f \n", uav2_thrust.actuator_control[3]);
		
		printf("3_acc[0]: %f \n", uav3_thrust.actuator_control[0]);
		printf("3_acc[1]: %f \n", uav3_thrust.actuator_control[1]);
		printf("3_acc[2]: %f \n", uav3_thrust.actuator_control[2]);
		printf("3_acc[3]: %f \n", uav3_thrust.actuator_control[3]);
		
		printf("4_acc[0]: %f \n", uav4_thrust.actuator_control[0]);
		printf("4_acc[1]: %f \n", uav4_thrust.actuator_control[1]);
		printf("4_acc[2]: %f \n", uav4_thrust.actuator_control[2]);
		printf("4_acc[3]: %f \n", uav4_thrust.actuator_control[3]);

        printf("\n");
    }

	// --------------------------------------------------------------------------
	//   END OF COMMANDS
	// --------------------------------------------------------------------------

	return;

}


// ------------------------------------------------------------------------------
//   Parse Command Line
// ------------------------------------------------------------------------------
// throws EXIT_FAILURE if could not open the port
void
parse_commandline(int argc, char **argv, char *&uart_name, int &baudrate,
		bool &use_udp, char *&udp_ip, int &udp_port, bool &autotakeoff)
{

	// string for command line usage
	const char *commandline_usage = "usage: mavlink_control [-d <devicename> -b <baudrate>] [-u <udp_ip> -p <udp_port>] [-a ]";

	// Read input arguments
	for (int i = 1; i < argc; i++) { // argv[0] is "mavlink"

		// Help
		if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
			printf("%s\n",commandline_usage);
			throw EXIT_FAILURE;
		}

		// UART device ID
		if (strcmp(argv[i], "-d") == 0 || strcmp(argv[i], "--device") == 0) {
			if (argc > i + 1) {
				i++;
				uart_name = argv[i];
			} else {
				printf("%s\n",commandline_usage);
				throw EXIT_FAILURE;
			}
		}

		// Baud rate
		if (strcmp(argv[i], "-b") == 0 || strcmp(argv[i], "--baud") == 0) {
			if (argc > i + 1) {
				i++;
				baudrate = atoi(argv[i]);
			} else {
				printf("%s\n",commandline_usage);
				throw EXIT_FAILURE;
			}
		}

		// UDP ip
		if (strcmp(argv[i], "-u") == 0 || strcmp(argv[i], "--udp_ip") == 0) {
			if (argc > i + 1) {
				i++;
				udp_ip = argv[i];
				use_udp = true;
			} else {
				printf("%s\n",commandline_usage);
				throw EXIT_FAILURE;
			}
		}

		// UDP port
		if (strcmp(argv[i], "-p") == 0 || strcmp(argv[i], "--port") == 0) {
			if (argc > i + 1) {
				i++;
				udp_port = atoi(argv[i]);
			} else {
				printf("%s\n",commandline_usage);
				throw EXIT_FAILURE;
			}
		}

		// Autotakeoff
		if (strcmp(argv[i], "-a") == 0 || strcmp(argv[i], "--autotakeoff") == 0) {
			autotakeoff = true;
		}

	}
	// end: for each input argument

	// Done!
	return;
}


// ------------------------------------------------------------------------------
//   Quit Signal Handler
// ------------------------------------------------------------------------------
// this function is called when you press Ctrl-C
void
quit_handler( int sig )
{
	printf("\n");
	printf("TERMINATING AT USER REQUEST\n");
	printf("\n");

	// autopilot interface
	try {
		autopilot_interface_quit->handle_quit(sig);
	}
	catch (int error){}

	// port
	try {
		port_quit->stop();
	}
	catch (int error){}

	// end program here
	exit(0);

}


// ------------------------------------------------------------------------------
//   Main
// ------------------------------------------------------------------------------
int
main(int argc, char **argv)
{
	// This program uses throw, wrap one big try/catch here
	try
	{
		int result = top(argc,argv);
		return result;
	}

	catch ( int error )
	{
		fprintf(stderr,"mavlink_control threw exception %i \n" , error);
		return error;
	}

}


