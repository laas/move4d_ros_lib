#ifndef MOVE4D_ROS_LIB_LOGGIN_HPP
#define MOVE4D_ROS_LIB_LOGGIN_HPP

#include <move4d/Logging/Logger.h>
#include <ros/console_backend.h>

namespace move4d {
namespace logm3d {
class OutputHandlerROS : public OutputHandler
{
    static ros::console::levels::Level getLevel(LogLevel lvl);
    virtual bool isEnabledFor(LogLevel lvl, LoggerPtr logger) override;
    virtual void log(LogLevel lvl, LoggerPtr logger, const std::string& message) override;

};
}
}

#endif //MOVE4D_ROS_LIB_LOGGIN_HPP
