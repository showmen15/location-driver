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
   // _roboclawDisabled = false;
 //   _overheated = false;

    //_roboclawDriver = new RoboclawDriver(_configuration);
    
	_amberPipes = new AmberPipes(this, pipeInFd, pipeOutFd);

//    _roboclawDriver->initializeDriver();

 /*   if (_configuration->battery_monitor_interval > 0) {
        _batteryMonitorThread = new boost::thread(boost::bind(&LocationController::batteryMonitor, this));
    }

    if (_configuration->error_monitor_interval > 0) {
        _errorMonitorThread = new boost::thread(boost::bind(&LocationController::errorMonitor, this));
    }

    if (_configuration->temperature_monitor_interval > 0) {
        _temperatureMonitorThread = new boost::thread(boost::bind(&LocationController::temperatureMonitor, this));
    }

    _roboclawDriver->setLed1(true);
    _roboclawDriver->setLed2(false);
*/
}

LocationController::~LocationController() {
    LOG4CXX_INFO(_logger, "Stopping controller.");

   // _roboclawDriver->setLed1(false);

   // delete _roboclawDriver;
    delete _amberPipes;
}

void LocationController::handleDataMsg(amber::DriverHdr *driverHdr, amber::DriverMsg *driverMsg) //to jest to
{
    if (_logger->isDebugEnabled()) {
        LOG4CXX_DEBUG(_logger, "Message came");
    }

    //_roboclawDriver->setLed1(true);

    // TODO: hack for now
    int clientId = driverHdr->clientids_size() > 0 ? driverHdr->clientids(0) : 0;

    // DataRequest
    if (driverMsg->HasExtension(location_proto::get_location)) 
	{

        /*if (!driverMsg->has_synnum()) {
            LOG4CXX_WARN(_logger, "Got CurrentSpeedRequest, but syn num not set. Ignoring.");
            return;
        }*/

        if (driverMsg->GetExtension(location_proto::get_location))
        {
            handleCurrentSpeedRequest(clientId, driverMsg->synnum());
        }

    } 
	
	//else if (driverMsg->HasExtension(roboclaw_proto::motorsCommand)) {
    //    handleMotorsEncoderCommand(driverMsg->MutableExtension(roboclaw_proto::motorsCommand));
   // }

   // _roboclawDriver->setLed1(false);
}

void LocationController::handleClientDiedMsg(int clientID) {
    LOG4CXX_INFO(_logger, "Client " << clientID << " died");

   // _roboclawDriver->stopMotors();
}

void LocationController::operator()() {
    _amberPipes->operator ()();
}

amber::DriverMsg *LocationController::buildCurrentSpeedMsg()  //zrobic za moment
{
LOG4CXX_INFO(_logger, "build current speed msg -> jestem w tu ");
 

 amber::DriverMsg *message = new amber::DriverMsg();
    message->set_type(amber::DriverMsg_MsgType_DATA);

    location_proto::Location *currentLocation = message->MutableExtension(location_proto::currentLocation);

    currentLocation->set_x(12345);
    currentLocation->set_y(65432);

    return message;
}

void LocationController::sendCurrentSpeedMsg(int receiver, int ackNum) //zrobic pozniej
{
    if (_logger->isDebugEnabled()) {
        LOG4CXX_DEBUG(_logger, "Sending currentSpeedRequest message");
    }

    amber::DriverMsg *currentSpeedMsg = buildCurrentSpeedMsg();
    currentSpeedMsg->set_acknum(ackNum);
    amber::DriverHdr *header = new amber::DriverHdr();
    header->add_clientids(receiver);

    _amberPipes->writeMsgToPipe(header, currentSpeedMsg);

    delete currentSpeedMsg;
    delete header;
}

void LocationController::handleCurrentSpeedRequest(int sender, int synNum) //ok
{
    if (_logger->isDebugEnabled()) {
        LOG4CXX_DEBUG(_logger, "Handling currentSpeedRequest message");
    }

    sendCurrentSpeedMsg(sender, synNum);
}

/*void LocationController::handleMotorsEncoderCommand(roboclaw_proto::MotorsSpeed *motorsCommand)
{
    if (_logger->isDebugEnabled()) {
        LOG4CXX_DEBUG(_logger, "Handling motorsEncoderCommand message");
    }

    MotorsSpeedStruct mc;

    mc.frontLeftSpeed = toQpps(motorsCommand->frontleftspeed());
    mc.frontRightSpeed = toQpps(motorsCommand->frontrightspeed());
    mc.rearLeftSpeed = toQpps(motorsCommand->rearleftspeed());
    mc.rearRightSpeed = toQpps(motorsCommand->rearrightspeed());

    if (!_roboclawDisabled) {

        try {
            _roboclawDriver->sendMotorsEncoderCommand(&mc);
        } catch (RoboclawSerialException& e) {
            // do nothing
        }
    }
}

int LocationController::toQpps(int in) {
    double rps = in / (double) (_configuration->wheel_radius * M_PI * 2);
    int out = (int) (rps * _configuration->pulses_per_revolution);

    if (_logger->isDebugEnabled()) {
        LOG4CXX_DEBUG(_logger, "toOpps: " << in << ", " << out);
    }

    return out;
}

int LocationController::toMmps(int in) {
    int out = (int) (in * (int) _configuration->wheel_radius * M_PI * 2 / (double) _configuration->pulses_per_revolution);

    if (_logger->isDebugEnabled()) {
        LOG4CXX_DEBUG(_logger, "toMmps: " << in << ", " << out);
    }

    return out;
}

void LocationController::batteryMonitor() {
    LOG4CXX_INFO(_logger, "Battery monitor thread started, interval: " << _configuration->battery_monitor_interval << "ms");

    __u16 voltage;

    while (1) {
        boost::this_thread::sleep(boost::posix_time::milliseconds(_configuration->battery_monitor_interval));

        if (!_roboclawDisabled) {

            try {
                _roboclawDriver->readMainBatteryVoltage(&voltage);
                LOG4CXX_INFO(_logger, "Main battery voltage level: " << voltage / 10.0 << "V");
            } catch (RoboclawSerialException& e) {
                // do nothing
            }
        }
    }
}

void LocationController::errorMonitor() {
    LOG4CXX_INFO(_logger, "Hardware error monitor started, interval: " << _configuration->error_monitor_interval << "ms");

    __u8 frontErrorStatus, rearErrorStatus, frontErrorStatusTmp, rearErrorStatusTmp;
    bool same_errors;

    while (1) {
        boost::this_thread::sleep(boost::posix_time::milliseconds(_configuration->error_monitor_interval));

        try {
            _roboclawDriver->readErrorStatus(&frontErrorStatus, &rearErrorStatus);

            if (frontErrorStatus != RC_ERROR_NORMAL || rearErrorStatus != RC_ERROR_NORMAL) {

                frontErrorStatusTmp = frontErrorStatus;
                rearErrorStatusTmp = rearErrorStatus;

                // check again in case of read errors
                same_errors = true;

                for (unsigned int i = 0; i < _configuration->critical_read_repeats; i++) {
                    _roboclawDriver->readErrorStatus(&frontErrorStatus, &rearErrorStatus);

                    if (frontErrorStatus != frontErrorStatusTmp || rearErrorStatus != rearErrorStatusTmp) {
                        same_errors = false;
                        break;
                    }
                }

                // if errors still the same
                if (same_errors) {
                    if (frontErrorStatus != RC_ERROR_NORMAL) {
                        LOG4CXX_WARN(_logger, "Front Roboclaw error: " << getErorDescription(frontErrorStatus));
                    }

                    if (rearErrorStatus != RC_ERROR_NORMAL) {
                        LOG4CXX_WARN(_logger, "Rear Roboclaw error: " << getErorDescription(rearErrorStatus));
                    }

                    if (frontErrorStatus == RC_ERROR_M1_OVERCURRENT || frontErrorStatus == RC_ERROR_M2_OVERCURRENT ||
                            rearErrorStatus == RC_ERROR_M1_OVERCURRENT || rearErrorStatus == RC_ERROR_M2_OVERCURRENT) {

                        resetAndWait();
                    } else if (frontErrorStatus == RC_ERROR_MAIN_BATTERY_LOW || rearErrorStatus == RC_ERROR_MAIN_BATTERY_LOW) {
                        _roboclawDriver->setLed2(true);

                        _roboclawDisabled = true;
                    }
                }
            }



        } catch (RoboclawSerialException& e) {
            // do nothing
        }
    }

}

void LocationController::temperatureMonitor() {
    LOG4CXX_INFO(_logger, "Temperature monitor thread started, interval: " << _configuration->temperature_monitor_interval << "ms");

    __u16 frontTemperature, rearTemperature;

    while (1) {
        boost::this_thread::sleep(boost::posix_time::milliseconds(_configuration->temperature_monitor_interval));

        if (!_roboclawDisabled) {
            _roboclawDriver->readTemperature(&frontTemperature, &rearTemperature);

            LOG4CXX_INFO(_logger, "Front temperature: " << frontTemperature / 10.0 << "C, " <<
                    "rear temperature: " << rearTemperature / 10.0 << "C");

            if (_overheated) {
                // if temperature droped down below drop level
                if (frontTemperature < _configuration->temperature_drop && rearTemperature < _configuration->temperature_drop) {
                    _overheated = false;

                    // check again in case of read errors
                    for (unsigned int i = 0; i < _configuration->critical_read_repeats; i++) {
                        _roboclawDriver->readTemperature(&frontTemperature, &rearTemperature);

                        if (frontTemperature > _configuration->temperature_drop || rearTemperature > _configuration->temperature_drop) {
                            _overheated = true;
                            break;
                        }
                    }

                    // if still cool and ok
                    if (!_overheated) {
                        LOG4CXX_INFO(_logger, "Roboclaw cooled down, reseting");
                        resetAndWait();
                    }
                }

            } else {
                // if temperature above critical
                if (frontTemperature > _configuration->temperature_critical || rearTemperature > _configuration->temperature_critical) {

                    _overheated = true;

                    // check again in case of read errors
                    for (unsigned int i = 0; i < _configuration->critical_read_repeats; i++) {
                        _roboclawDriver->readTemperature(&frontTemperature, &rearTemperature);

                        if (frontTemperature < _configuration->temperature_critical && rearTemperature < _configuration->temperature_critical) {
                            _overheated = false;
                            break;
                        }
                    }

                    // if still _overheated
                    if (_overheated) {
                        _roboclawDriver->stopMotors();

                        LOG4CXX_WARN(_logger, "Roboclaw _overheated, waiting for cool down to " << _configuration->temperature_drop / 10.0 << "C");
                    }
                }
            }

        }
    }

}

void LocationController::resetAndWait() {
    LOG4CXX_INFO(_logger, "Reseting Roboclaws and waiting " << _configuration->reset_delay << "ms");

    _roboclawDisabled = true;

    _roboclawDriver->reset();
    boost::this_thread::sleep(boost::posix_time::milliseconds(_configuration->reset_delay));

    _roboclawDriver->sendEncoderSettings();

    _roboclawDisabled = false;
}

string LocationController::getErorDescription(__u8 errorStatus) {

    string description;

    switch (errorStatus) {
        case RC_ERROR_NORMAL:
            description = "no error";
            break;

        case RC_ERROR_M1_OVERCURRENT:
            description = "m1 overcurrent";
            break;

        case RC_ERROR_M2_OVERCURRENT:
            description = "m2 overcurrent";
            break;

        case RC_ERROR_ESTOP:
            description = "emergency stop";
            break;

        case RC_ERROR_TEMPERATURE:
            description = "temperature high";
            break;

        case RC_ERROR_MAIN_BATTERY_HIGH:
            description = " battery high";
            break;

        case RC_ERROR_MAIN_BATTERY_LOW:
            description = "main battery low";
            break;

        case RC_ERROR_LOGIC_BATTERY_HIGH:
            description = "logic battery high";
            break;

        case RC_ERROR_LOGIC_BATTERY_LOW:
            description = "logic battery low";
            break;

        default:
            description = "";
    }

    return description;
}

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

//printf("zakonczylem program");
}
