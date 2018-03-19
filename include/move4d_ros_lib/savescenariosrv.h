#ifndef SAVESCENARIOSRV_H
#define SAVESCENARIOSRV_H
#include <string>
#include <ros/ros.h>
#include <ros/node_handle.h>

#include <move4d_ros_lib/SaveScenario.h>

namespace move4d
{
class SceneManager;

/**
 * @brief The SaveScenarioSrv class is an helper to easily create a service that saves a move4d scenario.
 * When called, the service creates a .sce file representing the state of the world.
 * @see SceneManager::saveScenario(string)
 * The service is a move4d_ros_lib/SaveScenario
 */
class SaveScenarioSrv
{
public:
    /**
     * @brief SaveScenarioSrv constructor
     * @param sceneMgr get the move4d scene from there
     * @param node node handle for advertising the service
     */
    SaveScenarioSrv(SceneManager *sceneMgr,ros::NodeHandle *node);
    ~SaveScenarioSrv();

    /**
     * @brief advertise the service and start responding to requests
     * @param name name of the service
     * @return
     */
    bool advertise(const std::string &name);

private:
    /// the function executed on service call: calls SceneManager::saveScenario(string)
    bool save(move4d_ros_lib::SaveScenarioRequest &req,move4d_ros_lib::SaveScenarioResponse &res);

    SceneManager *_sceneMgr;
    ros::NodeHandle *_node;
    ros::ServiceServer *_srv;
};
}

#endif // SAVESCENARIOSRV_H
