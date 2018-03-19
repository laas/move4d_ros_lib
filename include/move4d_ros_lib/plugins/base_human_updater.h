#ifndef BASEHUMANUPDATER_H
#define BASEHUMANUPDATER_H

#include <boost/shared_ptr.hpp>
#include <Eigen/Geometry>

#include <vector>
#include <string>
#include <map>

namespace move4d
{
class Robot;
class RobotState;

class BaseHumanUpdater;
/** @brief The HumanSettings struct stores data for each human.
 */
struct HumanSettings{
    ///default constructor
    HumanSettings():policy(""){}
    ///constructor
    HumanSettings(const std::map<std::string, float> &settings, const std::string &policy, const std::string &move4d_name)
    {
        this->policy=policy;
        data=settings;
        this->move4d_name=move4d_name;
    }
    /// handler to access to the data in the data map
    bool getData(std::string key,float &v) const{
        std::map<std::string,float>::const_iterator it=data.find(key);
        if(it!=data.end()){
            v=it->second;
            return true;
        }
        return false;
    }
    ///name of the plugin to use
    std::string policy;
    /// pointer to the updater object (plugin)
    boost::shared_ptr<BaseHumanUpdater> updater;
    /// specific data passed to the updater
    std::map<std::string,float> data;
    /// name of the human in move4d (i.e. in the p3d file)
    std::string move4d_name;

};
typedef boost::shared_ptr<HumanSettings> HumanSettingsPtr;

/**
 * Base class for implementing plugin human updater.
 *
 * BaseHumanUpdater provides the interface for plugin classes that handle the translation of some specific human description to move4d
 *
 * @see http://wiki.ros.org/pluginlib
 */
class BaseHumanUpdater
{
public:
    virtual ~BaseHumanUpdater(){}

    /**
     * @brief update the robot with the given elements
     * @param h the Robot object representing the human in move4d
     * @param base the base position
     * @param joints the joint positions
     * @param settings settings for that human
     * @return
     */
    virtual bool update(Robot *h,const Eigen::Affine3d &base,const std::map<std::string,Eigen::Affine3d> &joints,const HumanSettings &settings)=0;
    /**
     * @brief compute the configuration without updating the human in move4d
     * @param[in] h Robot object representing the human in move4d
     * @param[in] base base position
     * @param[in] joints joint positions
     * @param[in] settings settings for that human
     * @param[out] q the computed configuration
     * @return
     */
    virtual bool computeConf(Robot *h,const Eigen::Affine3d &base,const std::map<std::string,Eigen::Affine3d> &joints,const HumanSettings &settings, RobotState &q)=0;

protected:
    BaseHumanUpdater(){}

};



}

#endif
