#include "move4d_ros_lib/jointstateupdater.hpp"

#include "move4d_ros_lib/scenemanager.h"

#include <tf/tf.h>

#include <move4d/API/ConfigSpace/RobotState.hpp>
#include <move4d/API/project.hpp>

namespace move4d {

JointStateUpdater::JointStateUpdater(SceneManager *scMgr, ros::NodeHandle &nh,
                                     const std::string &robot_name, const std::string &topic_name, const std::string &frame_name):
    _scMgr(scMgr),
    _nh(nh),
    _robot_name(robot_name),
    _robot_frame(frame_name),
    _topic(topic_name)
{
    if(!scMgr->project()->getActiveScene()->getRobotByName(_robot_name)){
        throw std::invalid_argument("move4d::JointStateUpdater: no robot named "+robot_name);
    }
    _lastJointState = sensor_msgs::JointStatePtr(new sensor_msgs::JointState());
    _subscribe();
}

//void JointStateUpdater::update(const sensor_msgs::JointState &joint_state, const geometry_msgs::Pose &pose, const geometry_msgs::Twist &twist, const geometry_msgs::Accel &accel)
//{
//    _scMgr->updateRobot(_robot_name,pose,joint_state.position);
//}

void JointStateUpdater::update(const sensor_msgs::JointState &joint_state, const geometry_msgs::Pose &pose)
{
    _scMgr->setDofNameOrdered(_robot_name,joint_state.name);
    bool ok=_scMgr->updateRobot(_robot_name,pose,joint_state.position);
    if(!ok) ROS_WARN("failed to update robot %s",_robot_name.c_str());
}

void JointStateUpdater::updateFromTopic()
{
    geometry_msgs::TransformStamped transform;
    transform = _scMgr->getTfBuffer().lookupTransform(_scMgr->getOriginTfFrame(),_robot_frame,ros::Time(0));

    geometry_msgs::Pose pose;

    pose.orientation= transform.transform.rotation;
    pose.position.x = transform.transform.translation.x;
    pose.position.y = transform.transform.translation.y;
    pose.position.z = transform.transform.translation.z;

    _lock.lock();
    update(*_lastJointState,pose);
    _lock.unlock();
}

void JointStateUpdater::autoUpdate(bool autoUpdate)
{
    _updateOnReceive=autoUpdate;
}

void JointStateUpdater::_jointStateCallback(sensor_msgs::JointStateConstPtr msg)
{
    _lock.lock();
    _lastJointState = msg;
    _lock.unlock();
    if(_updateOnReceive){
        updateFromTopic();
    }
}

void JointStateUpdater::_subscribe()
{
    _subscriber = _nh.subscribe(_topic,1,&JointStateUpdater::_jointStateCallback,this);
}

} // namespace move4d
