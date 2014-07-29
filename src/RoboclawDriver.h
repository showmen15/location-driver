/*
 * StargazerDriver.h
 *
 *  Created on: 30-11-2012
 *      Author: michal
 */

#ifndef STARGAZERDRIVER_H_
#define STARGAZERDRIVER_H_

#include <boost/interprocess/sync/interprocess_condition.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>
#include <log4cxx/logger.h>

#include "RoboclawCommon.h"

#define UART_SPEED B38400
#define ROBOCLAW_PORT "/dev/ttyO3"

class RoboclawDriver {

public:
	RoboclawDriver(RoboclawConfiguration *configuration);
	virtual ~RoboclawDriver();

	void initializeDriver();
	void sendEncoderSettings();
	void readCurrentSpeed(MotorsSpeedStruct *mss) throw(RoboclawSerialException);
	void sendMotorsEncoderCommand(MotorsSpeedStruct *mss) throw(RoboclawSerialException);
	void readMainBatteryVoltage(__u16 *voltage) throw(RoboclawSerialException);
	void readErrorStatus(__u8 *frontErrorStatus, __u8 *rearErrorStatus) throw(RoboclawSerialException);
	void readTemperature(__u16 *frontTemperature, __u16 *rearTemperature) throw(RoboclawSerialException);
	void stopMotors() throw(RoboclawSerialException);
	void reset();
	void setLed1(bool state);
	void setLed2(bool state);

	boost::interprocess::interprocess_condition driverIsNotReady;
	boost::interprocess::interprocess_mutex serialPortMutex;

	bool driverReady;

private:

	static log4cxx::LoggerPtr _logger;

	int _fd;
	int _gpioFd;
	int _led1GpioFd;
	int _led2GpioFd;

	RoboclawConfiguration *_configuration;



};

#endif /* STARGAZERDRIVER_H_ */
