#include <move4d_ros_lib/conversions.h>
#include <move4d_ros_lib/scenemanager.h>

#undef QT_LIBRARY
#include <move4d/utils/Geometry.h>

namespace move4d{
namespace helpers{

geometry_msgs::PoseStamped stateToPoseStamp(const RobotState &state)
{
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = SceneManager::instance()->getOriginTfFrame();

    pose.pose.position.x = state.at(6+0);
    pose.pose.position.y = state.at(6+1);
    pose.pose.position.z = state.at(6+2);


    Eigen::Quaterniond q= const_cast<RobotState &>(state).getQuaternion();
    pose.pose.orientation.w=q.w();
    pose.pose.orientation.x=q.x();
    pose.pose.orientation.y=q.y();
    pose.pose.orientation.z=q.z();

    return pose;
}

}
}
