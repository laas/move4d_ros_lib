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
}

//void JointStateUpdater::update(const sensor_msgs::JointState &joint_state, const geometry_msgs::Pose &pose, const geometry_msgs::Twist &twist, const geometry_msgs::Accel &accel)
//{
//    _scMgr->updateRobot(_robot_name,pose,joint_state.position);
//}

void JointStateUpdater::update(const sensor_msgs::JointState &joint_state, const geometry_msgs::Pose &pose)
{
    _scMgr->updateRobot(_robot_name,pose,joint_state.position);
}

void JointStateUpdater::updateFromTopic()
{
    tf::StampedTransform transform;
    _listener.lookupTransform(_robot_frame,_scMgr->getOriginTfFrame(),ros::Time(0),transform);

    geometry_msgs::Transform msg;
    tf::transformTFToMsg(transform,msg);
    geometry_msgs::Pose pose;

    pose.orientation= msg.rotation;
    pose.position.x = msg.translation.x;
    pose.position.y = msg.translation.y;
    pose.position.z = msg.translation.z;

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
