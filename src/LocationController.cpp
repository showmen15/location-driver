/*
 * LocationController.cpp
 *
 *  Created on: 29-07-2014
 *      Author: szsz
 */

#include <log4cxx/propertyconfigurator.h>

#include "LocationController.h"

#include <boost/program_options.hpp>
#include <string>
#include <cmath>

using namespace std;
using namespace boost;
using namespace boost::interprocess;
using namespace boost::program_options;
using namespace log4cxx;
using namespace amber;

LoggerPtr LocationController::_logger(Logger::getLogger("Roboclaw.Controller"));

LocationController::LocationController(int pipeInFd, int pipeOutFd, const char *confFilename)
{
   // parseConfigurationFile(confFilename);
    
	_amberPipes = new AmberPipes(this, pipeInFd, pipeOutFd);

 /*   if (_configuration->battery_monitor_interval > 0)
  * {
        _batteryMonitorThread = new boost::thread(boost::bind(&LocationController::batteryMonitor, this));
    }
*/
}

LocationController::~LocationController() {
    LOG4CXX_INFO(_logger, "Stopping controller.");

    delete _amberPipes;
}

void LocationController::handleDataMsg(amber::DriverHdr *driverHdr, amber::DriverMsg *driverMsg)
{
    if (_logger->isDebugEnabled()) {
        LOG4CXX_DEBUG(_logger, "Message came");
    }

    int clientId = driverHdr->clientids_size() > 0 ? driverHdr->clientids(0) : 0;

    // DataRequest
    if (driverMsg->HasExtension(location_proto::get_location)) 
	{
        if (!driverMsg->has_synnum()) {
            LOG4CXX_WARN(_logger, "Got CurrentLocationRequest, but syn num not set. Ignoring.");
            return;
        }

        if (driverMsg->GetExtension(location_proto::get_location))
        {
            handleCurrentLocationRequest(clientId, driverMsg->synnum());
        }
    }
}

void LocationController::handleClientDiedMsg(int clientID) {
    LOG4CXX_INFO(_logger, "Client " << clientID << " died");

}

void LocationController::operator()() {
    _amberPipes->operator ()();
}

amber::DriverMsg *LocationController::buildCurrentLocationMsg()
{
LOG4CXX_INFO(_logger, "build current location msg");
 
 	amber::DriverMsg *message = new amber::DriverMsg();
    message->set_type(amber::DriverMsg_MsgType_DATA);

    location_proto::Location *currentLocation = message->MutableExtension(location_proto::currentLocation);

    currentLocation->set_x(12345);
    currentLocation->set_y(65432);

    return message;
}

void LocationController::sendCurrentLocationMsg(int receiver, int ackNum)
{
    if (_logger->isDebugEnabled()) {
        LOG4CXX_DEBUG(_logger, "Sending currentLocationRequest message");
    }

    amber::DriverMsg *currentLocationMsg = buildCurrentLocationMsg();
    currentLocationMsg->set_acknum(ackNum);
    amber::DriverHdr *header = new amber::DriverHdr();
    header->add_clientids(receiver);

    _amberPipes->writeMsgToPipe(header, currentLocationMsg);

    delete currentLocationMsg;
    delete header;
}

void LocationController::handleCurrentLocationRequest(int sender, int synNum)
{
    if (_logger->isDebugEnabled()) {
        LOG4CXX_DEBUG(_logger, "Handling currentLocationRequest message");
    }

    sendCurrentLocationMsg(sender, synNum);
}

/*
void LocationController::parseConfigurationFile(const char *filename) {
    LOG4CXX_INFO(_logger, "Parsing configuration file: " << filename);

    _configuration = new RoboclawConfiguration();

    unsigned int front_rc_address;
    unsigned int rear_rc_address;

    options_description desc("Roboclaw options");
    desc.add_options()
            ("roboclaw.uart_port", value<string>(&_configuration->uart_port)->default_value("/dev/ttyO3"))
            ("roboclaw.uart_speed", value<unsigned int>(&_configuration->uart_speed)->default_value(38400))
            ("roboclaw.reset_gpio_path", value<string>(&_configuration->reset_gpio_path)->default_value("/sys/class/gpio/gpio136/value"))
            ("roboclaw.reset_delay", value<unsigned int>(&_configuration->reset_delay)->default_value(260))
            ("roboclaw.led1_gpio_path", value<string>(&_configuration->led1_gpio_path)->default_value("/sys/class/gpio/gpio139/value"))
            ("roboclaw.led2_gpio_path", value<string>(&_configuration->led2_gpio_path)->default_value("/sys/class/gpio/gpio137/value"))
            ("roboclaw.front_rc_address", value<unsigned int>(&front_rc_address)->default_value(128))
            ("roboclaw.rear_rc_address", value<unsigned int>(&rear_rc_address)->default_value(129))
            ("roboclaw.motors_max_qpps", value<unsigned int>(&_configuration->motors_max_qpps)->default_value(13800))
            ("roboclaw.motors_p_const", value<unsigned int>(&_configuration->motors_p_const)->default_value(65536))
            ("roboclaw.motors_i_const", value<unsigned int>(&_configuration->motors_i_const)->default_value(32768))
            ("roboclaw.motors_d_const", value<unsigned int>(&_configuration->motors_d_const)->default_value(16384))
            ("roboclaw.pulses_per_revolution", value<unsigned int>(&_configuration->pulses_per_revolution)->default_value(1865))
            ("roboclaw.wheel_radius", value<unsigned int>(&_configuration->wheel_radius)->default_value(60))
            ("roboclaw.battery_monitor_interval", value<unsigned int>(&_configuration->battery_monitor_interval)->default_value(0))
            ("roboclaw.error_monitor_interval", value<unsigned int>(&_configuration->error_monitor_interval)->default_value(0))
            ("roboclaw.temperature_monitor_interval", value<unsigned int>(&_configuration->temperature_monitor_interval)->default_value(0))
            ("roboclaw.temperature_critical", value<__u16>(&_configuration->temperature_critical)->default_value(70))
            ("roboclaw.temperature_drop", value<__u16>(&_configuration->temperature_drop)->default_value(60))
            ("roboclaw.critical_read_repeats", value<unsigned int>(&_configuration->critical_read_repeats)->default_value(0));


    variables_map vm;

    try {
        store(parse_config_file<char>(filename, desc), vm);
        notify(vm);

        _configuration->front_rc_address = (__u8) front_rc_address;
        _configuration->rear_rc_address = (__u8) rear_rc_address;

    } catch (std::exception& e) {
        LOG4CXX_ERROR(_logger, "Error in parsing configuration file: " << e.what());
        exit(1);
    }

}
*/

int main(int argc, char *argv[]) {

   if (argc < 3) {
       return 1;
    }

    const char *confFile =  argv[1];
    const char *logConfFile = argv[2];

    PropertyConfigurator::configure(logConfFile);

    // STDIN_FD = 0, STDOUT_FD = 1
    // pipe_in_fd = 0, pipe_out_fd = 1
    LoggerPtr logger(Logger::getLogger("main"));

    LOG4CXX_INFO(logger, "-------------");
    LOG4CXX_INFO(logger, "Creating controller, config_file: " << argv[1] << ", log_config_file: " << argv[2]);

    LOG4CXX_INFO(logger, "jestem w main 3 ");

    LocationController controller(0, 1, confFile);
    controller();
}
