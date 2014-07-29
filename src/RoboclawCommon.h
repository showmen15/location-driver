/*
 * RoboclawCommon.h
 *
 *  Created on: 02-12-2012
 *      Author: michal
 */

#ifndef ROBOCLAWCOMMON_H_
#define ROBOCLAWCOMMON_H_

#include <exception>
#include <linux/types.h>
#include <string>

struct MotorsSpeedStruct {

	int frontLeftSpeed;
	int frontRightSpeed;
	int rearLeftSpeed;
	int rearRightSpeed;

};

struct RoboclawConfiguration {

	std::string uart_port;
	unsigned int uart_speed;

	std::string reset_gpio_path;
	__u32 reset_delay;

	std::string led1_gpio_path;
	std::string led2_gpio_path;

	__u8 front_rc_address;
	__u8 rear_rc_address;

	__u32 motors_max_qpps;
	__u32 motors_p_const;
	__u32 motors_i_const;
	__u32 motors_d_const;

	__u32 pulses_per_revolution;
	__u32 wheel_radius;
	
	__u32 battery_monitor_interval;
	__u32 error_monitor_interval;
	__u32 temperature_monitor_interval;

	__u16 temperature_critical;
	__u16 temperature_drop;

	__u32 critical_read_repeats;

};

class RoboclawSerialException: public std::exception {

};


#endif /* ROBOCLAWCOMMON_H_ */
