/*
 * StargazerController.h
 *
 *
 *  Created on: 30-11-2012
 *      Author: michal
 */

#ifndef ROBOCLAWCONTROLLER_H_
#define ROBOCLAWCONTROLLER_H_

#include <log4cxx/logger.h>
#include <boost/thread.hpp>
#include <boost/ref.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>

#include "AmberScheduler.h"
#include "AmberPipes.h"
#include "RoboclawDriver.h"
#include "drivermsg.pb.h"
#include "roboclaw.pb.h"
#include "RoboclawLib.h"
#include "location.pb.h"


class RoboclawController: public MessageHandler {
public:
	RoboclawController(int pipeInFd, int pipeOutFd, const char *confFilename);
	virtual ~RoboclawController();

	void handleDataMsg(amber::DriverHdr *driverHdr, amber::DriverMsg *driverMsg);
	void handleClientDiedMsg(int clientID);
	void operator()();

private:
	//RoboclawDriver *_roboclawDriver;
	AmberPipes *_amberPipes;

	bool _roboclawDisabled;
	bool _overheated;

	//RoboclawConfiguration *_configuration;
//	boost::thread *_batteryMonitorThread;
	//boost::thread *_errorMonitorThread;
//	boost::thread *_temperatureMonitorThread;

	static log4cxx::LoggerPtr _logger;

	amber::DriverMsg *buildCurrentSpeedMsg();
	void sendCurrentSpeedMsg(int receiver, int ackNum);
	void handleCurrentSpeedRequest(int sender, int synNum);
	void handleMotorsEncoderCommand(amber::roboclaw_proto::MotorsSpeed *motorsCommand);
//	void parseConfigurationFile(const char *filename);
//	void resetAndWait();

//	void batteryMonitor();
	//void errorMonitor();
	//void temperatureMonitor();

	//std::string getErorDescription(__u8 errorStatus);
	//int toQpps(int in);
	//int toMmps(int in);

};


#endif /* ROBOCLAWCONTROLLER_H_ */
