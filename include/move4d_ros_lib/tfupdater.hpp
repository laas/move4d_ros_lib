#ifndef TFUPDATER_HPP
#define TFUPDATER_HPP

#include <map>
#include <string>

#include <ros/node_handle.h>
#include <geometry_msgs/Pose.h>
#include <tf2_ros/transform_listener.h>

#include <move4d/API/forward_declarations.hpp>


namespace move4d {
class SceneManager;

class TfUpdater
{
public:
    TfUpdater(SceneManager *scMgr, ros::NodeHandle &nh, const std::string &robot_name, const std::string &base_frame_name, const std::map<std::string,std::string> &frame_joint_map);

    void update(const geometry_msgs::Pose &base_pose, const std::map<std::string, geometry_msgs::Pose> &poses);

    void update();


protected:
    geometry_msgs::Pose getPoseFromTf(const std::string &name);
private:
    SceneManager *_scMgr;
    ros::NodeHandle _nh;

    std::string _robot_name;
    std::string _baseFrameName;
    std::map<std::string,std::string> _frameJointMap;

};

} // namespace move4d

#endif // TFUPDATER_HPP
