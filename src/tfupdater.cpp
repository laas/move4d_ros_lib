#include "move4d_ros_lib/tfupdater.hpp"

#include "move4d_ros_lib/scenemanager.h"

#include <move4d/API/ConfigSpace/RobotState.hpp>
#include <move4d/API/project.hpp>

#include <tf2_ros/transform_listener.h>

namespace move4d {

TfUpdater::TfUpdater(SceneManager *scMgr, ros::NodeHandle &nh, const std::string &robot_name, const std::string &base_frame_name, const std::map<std::string, std::string> &frame_joint_map):
    _scMgr(scMgr),
    _nh(nh),
    _robot_name(robot_name),
    _baseFrameName(base_frame_name),
    _frameJointMap(frame_joint_map)
{
    if(!_scMgr->project()->getActiveScene()->getRobotByName(_robot_name)){
        throw std::invalid_argument("move4d::JointStateUpdater: no robot named "+robot_name);
    }
}

void TfUpdater::update(const geometry_msgs::Pose &base_pose, const std::map<std::string, geometry_msgs::Pose> &poses)
{
    _scMgr->updateHuman(_robot_name,base_pose,poses);

}

void TfUpdater::update()
{
    std::map<std::string,geometry_msgs::Pose> poses;

    for(auto jnt : _frameJointMap){
        poses[jnt.second]=getPoseFromTf(jnt.first);
    }

    update(getPoseFromTf(_baseFrameName),poses);
}

geometry_msgs::Pose TfUpdater::getPoseFromTf(const std::string &name)
{
    geometry_msgs::TransformStamped transform;
    geometry_msgs::Pose pose;

    transform = _scMgr->getTfBuffer().lookupTransform(_scMgr->getOriginTfFrame(),name,ros::Time(0));
    pose.orientation= transform.transform.rotation;
    pose.position.x = transform.transform.translation.x;
    pose.position.y = transform.transform.translation.y;
    pose.position.z = transform.transform.translation.z;
    return pose;
}

} // namespace move4d
