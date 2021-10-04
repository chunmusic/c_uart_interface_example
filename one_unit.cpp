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

#include "one_unit.h"


// ------------------------------------------------------------------------------
//   TOP
// ------------------------------------------------------------------------------
int
top (int argc, char **argv)
{

	Generic_Port *port_uav1, *port_uav3;

	port_uav1 = new Serial_Port("/dev/ttyACM0", 921600);
	// port_uav3 = new Serial_Port("/dev/ttyACM7", 921600);
	// port_uav4 = new Serial_Port("/dev/ttyACM2", 921600);
	// port_uav5 = new Serial_Port("/dev/ttyACM3", 921600);
	// port_uav6 = new Serial_Port("/dev/ttyACM3", 921600);

	Autopilot_Interface autopilot_interface_uav1(port_uav1);
	// Autopilot_Interface autopilot_interface_uav3(port_uav3);
	// Autopilot_Interface autopilot_interface_uav4(port_uav4);
	// Autopilot_Interface autopilot_interface_uav5(port_uav5);	
	// Autopilot_Interface autopilot_interface_uav6(port_uav6);

	port_quit_uav1         = port_uav1;
	// port_quit_uav3         = port_uav3;
	// port_quit_uav4         = port_uav4;
	// port_quit_uav5         = port_uav5;
	// port_quit_uav6         = port_uav6;

	autopilot_interface_quit_uav1 = &autopilot_interface_uav1;
	// autopilot_interface_quit_uav3 = &autopilot_interface_uav3;
	// autopilot_interface_quit_uav4 = &autopilot_interface_uav4;
	// autopilot_interface_quit_uav5 = &autopilot_interface_uav5;
	// autopilot_interface_quit_uav6 = &autopilot_interface_uav6;

	signal(SIGINT,quit_handler);

	port_uav1->start();
	// port_uav3->start();
	// port_uav4->start();
	// port_uav5->start();
	// port_uav6->start();

	autopilot_interface_uav1.start();
	// autopilot_interface_uav3.start();
	// autopilot_interface_uav4.start();
	// autopilot_interface_uav5.start();
	// autopilot_interface_uav6.start();

	commands(autopilot_interface_uav1);

	autopilot_interface_uav1.stop();
	// autopilot_interface_uav3.stop();
	// autopilot_interface_uav4.stop();
	// autopilot_interface_uav5.stop();
	// autopilot_interface_uav6.stop();

	port_uav1->stop();
	// port_uav3->stop();
	// port_uav4->stop();
	// port_uav5->stop();
	// port_uav6->stop();

	delete port_uav1;
	// delete port_uav3;
	// delete port_uav4;
	// delete port_uav5;
	// delete port_uav6;

	return 0;

}

void print_status(mavlink_uav_command_t uav_command, mavlink_uav1_thrust_t uav1_thrust, mavlink_uav2_thrust_t uav2_thrust,
				  mavlink_uav3_thrust_t uav3_thrust, mavlink_uav4_thrust_t uav4_thrust)
{

	// printf("nav_state: %d \n", uav_command.nav_state);
	// printf("arming_state: %d \n", uav_command.arming_state);

	// printf("armed: %d \n", uav_command.armed);
	// printf("prearmed: %d \n", uav_command.prearmed);

	for (int i; i<4; i++){
		printf("1_acc[0]: %f \n", uav1_thrust.actuator_control[i]);
	}
	// for (int i; i<4; i++){
	// 	printf("2_acc[0]: %f \n", uav2_thrust.actuator_control[1]);
	// }
	// for (int i; i<4; i++){
	// 	printf("3_acc[0]: %f \n", uav3_thrust.actuator_control[2]);
	// }
	// for (int i; i<4; i++){
	// 	printf("4_acc[0]: %f \n", uav4_thrust.actuator_control[3]);
	// }

	printf("\n");

}

// ------------------------------------------------------------------------------
//   COMMANDS
// ------------------------------------------------------------------------------

void
commands(Autopilot_Interface &uav1_api)
{

	printf("READ SOME MESSAGES \n");

    while (true)
    {

		Mavlink_Messages uav1_msg = uav1_api.current_messages;

		mavlink_uav_command_t uav_command = uav1_msg.uav_command;

		mavlink_uav1_thrust_t uav1_thrust = uav1_msg.uav1_thrust;
		mavlink_uav2_thrust_t uav2_thrust = uav1_msg.uav2_thrust;
		mavlink_uav3_thrust_t uav3_thrust = uav1_msg.uav3_thrust;
		mavlink_uav4_thrust_t uav4_thrust = uav1_msg.uav4_thrust;

		for (int i=0; i < 4; i++){
			printf("actuator_%d: %f", i,uav1_thrust.actuator_control[i]);
			printf("\n");
		}

		// print_status(uav_command,uav1_thrust,uav2_thrust,uav3_thrust,uav4_thrust);

		// // TODO create send message
		// uav3_api.send_uav_command(uav_command.nav_state, uav_command.arming_state, uav_command.armed, uav_command.prearmed, uav_command.ready_to_arm, 
		// 		 				  uav_command.lockdown, uav_command.manual_lockdown, uav_command.force_failsafe, uav_command.in_esc_calibration_mode, uav_command.soft_stop);

    }

	return;

}

void
quit_handler( int sig )
{
	printf("\n");
	printf("TERMINATING AT USER REQUEST\n");
	printf("\n");

	try {
		autopilot_interface_quit_uav1->handle_quit(sig);
		// autopilot_interface_quit_uav3->handle_quit(sig);
		// autopilot_interface_quit_uav4->handle_quit(sig);
		// autopilot_interface_quit_uav5->handle_quit(sig);
		// autopilot_interface_quit_uav6->handle_quit(sig);
	}
	catch (int error){}

	// port
	try {
		port_quit_uav1->stop();
		// port_quit_uav3->stop();
		// port_quit_uav4->stop();
		// port_quit_uav5->stop();
		// port_quit_uav6->stop();
	}
	catch (int error){}

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


