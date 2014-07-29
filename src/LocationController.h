/*
 * LocationController.h
 *
 *  Created on: 29-07-2014
 *      Author: szsz
 */

#ifndef LocationController_H_
#define LocationController_H_

#include <log4cxx/logger.h>
#include <boost/thread.hpp>
#include <boost/ref.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>

#include "AmberScheduler.h"
#include "AmberPipes.h"
#include "drivermsg.pb.h"
#include "location.pb.h"


class LocationController: public MessageHandler {
public:
	LocationController(int pipeInFd, int pipeOutFd, const char *confFilename);
	virtual ~LocationController();

	void handleDataMsg(amber::DriverHdr *driverHdr, amber::DriverMsg *driverMsg);
	void handleClientDiedMsg(int clientID);
	void operator()();

private:
	AmberPipes *_amberPipes;

	bool _roboclawDisabled;
	bool _overheated;

	//RoboclawConfiguration *_configuration;
	//	boost::thread *_batteryMonitorThread;
	//boost::thread *_errorMonitorThread;
	//	boost::thread *_temperatureMonitorThread;

	static log4cxx::LoggerPtr _logger;

	amber::DriverMsg *buildCurrentLocationMsg();
	void sendCurrentLocationMsg(int receiver, int ackNum);
	void handleCurrentLocationRequest(int sender, int synNum);
//	void parseConfigurationFile(const char *filename);

};


#endif /* LocationController_H_ */
