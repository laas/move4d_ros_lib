#ifndef MOVE3D_ROBOTSTFPUBLISHER_H
#define MOVE3D_ROBOTSTFPUBLISHER_H

#include <vector>
#include <string>
#include <tf/transform_broadcaster.h>

namespace ros{
class NodeHandle;
}
namespace move4d {
class Robot;

class SceneManager;

/**
 * @brief The RobotsTfPublisher class publishes tf for robots in the scene uppon request
 *
 * The tf published are named <move4d_robot_name>/<move4d_link_name>
 */
class RobotsTfPublisher
{
public:
    /**
     * @brief RobotsTfPublisher constructor
     * @param scMgr the scene manager from which to get robot positions
     * @param nh the node handle used to publish tf
     */
    RobotsTfPublisher(SceneManager *scMgr, ros::NodeHandle *nh);
    ~RobotsTfPublisher();

    /**
     * @brief publish tf for given robots
     * @param names list of robots names
     * @param date date used in the tf header stamp
     * @return
     *
     * Uses the *current* world state inside move4d scene. date parameter is used only
     * for stamping the transforms sent to the broadcaster
     */
    bool publishTf(const std::vector<std::string > &names, const ros::Time &date);
    /**
     * @brief publish tf for a single move4d robot object
     * @param r pointer to the move4d robot
     * @param date date used in the tf header stamp
     * @return
     *
     * Uses the *current* world state inside move4d scene. date parameter is used only
     * for stamping the transforms sent to the broadcaster
     */
    bool publishTf(Robot *r, const ros::Time &date);

private:
    SceneManager *_scMgr;
    ros::NodeHandle *_nh;
    tf::TransformBroadcaster *_tf_br;
};

} // namespace move4d

#endif // MOVE3D_ROBOTSTFPUBLISHER_H
