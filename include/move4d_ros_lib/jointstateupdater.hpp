#ifndef JOINTSTATEUPDATER_HPP
#define JOINTSTATEUPDATER_HPP

#include <ros/node_handle.h>
#include <move4d/API/forward_declarations.hpp>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Accel.h>
#include <tf2_ros/transform_listener.h>
#include <mutex>

namespace move4d {
class SceneManager;

class JointStateUpdater
{
public:
    JointStateUpdater(SceneManager *scMgr, ros::NodeHandle &nh, const std::string &robot_name, const std::string &topic_name, const std::string &frame_name);

    /**
     * @brief update
     * @param joint_state
     * @param pose
     * @param twist unused
     * @param accel unused
     *
     * @todo only position is used (not the derivatices)
     */
    //void update(const sensor_msgs::JointState &joint_state, const geometry_msgs::Pose &pose, const geometry_msgs::Twist &twist, const geometry_msgs::Accel &accel);
    void update(const sensor_msgs::JointState &joint_state, const geometry_msgs::Pose &pose);

    void updateFromTopic();

    void autoUpdate(bool autoUpdate);

protected:
    void _jointStateCallback(sensor_msgs::JointStateConstPtr);
    void _subscribe();


private:
    SceneManager *_scMgr;
    ros::NodeHandle _nh;
    std::string _robot_name;
    std::string _robot_frame;
    std::string _topic;
    ros::Subscriber _subscriber;
    bool _updateOnReceive=false;

    sensor_msgs::JointStateConstPtr _lastJointState;

    std::mutex _lock;


};

} // namespace move4d

#endif // JOINTSTATEUPDATER_HPP
