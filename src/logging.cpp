#include "move4d_ros_lib/logging.hpp"
#include <ros/console.h>
#include <stdexcept>

namespace move4d {
namespace logm3d {
    ros::console::levels::Level OutputHandlerROS::getLevel(LogLevel lvl){
        switch(lvl){
        case LogLevel::TRACE:
            return ros::console::levels::Debug;
        case LogLevel::DEBUG:
            return ros::console::levels::Debug;
        case LogLevel::INFO:
            return ros::console::levels::Info;
        case LogLevel::WARN:
            return ros::console::levels::Warn;
        case LogLevel::ERROR:
            return ros::console::levels::Error;
        case LogLevel::FATAL:
            return ros::console::levels::Fatal;
        default:
            throw std::out_of_range("OutputHandlerROS: required log level do not exist");
        }
    }
    void OutputHandlerROS::log(LogLevel lvl, LoggerPtr logger, const std::string& message)
    {
        std::string fullName=ROSCONSOLE_DEFAULT_NAME "." +logger->getName();
        ROS_LOG(getLevel(lvl),fullName.c_str(),"%s %s",logger->getName().c_str(),message.c_str());
    }
    bool OutputHandlerROS::isEnabledFor(LogLevel lvl, LoggerPtr logger)
    {
        if(lvl==LogLevel::TRACE) return false; //no trace level in ROS
        ROSCONSOLE_DEFINE_LOCATION(true,getLevel(lvl),logger->getName());
        return __rosconsole_define_location__enabled;
    }
}
}


