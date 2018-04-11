#ifndef CONVERSIONS_H
#define CONVERSIONS_H

#include <geometry_msgs/PoseStamped.h>
#include <move4d/API/ConfigSpace/RobotState.hpp>

namespace move4d{
namespace helpers{

geometry_msgs::PoseStamped stateToPoseStamp(const RobotState &state);


}
}

#endif // CONVERSIONS_H
