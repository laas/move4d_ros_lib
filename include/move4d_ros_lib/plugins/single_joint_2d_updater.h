#ifndef MOVE3D_SINGLEJOINT2DUPDATER_H
#define MOVE3D_SINGLEJOINT2DUPDATER_H

#include "move4d_ros_lib/plugins/base_human_updater.h"

namespace move4d {

/**
 * @brief The SingleJoint2dUpdater class is a plugin class for the HumanMgr. It takes a 2D+rotation position and a constant height.
 *
 * It reads only the base position in the input and takes a "height" parameter from the settings. The human base joint (e.g. pelvis)
 * is placed in the position "base" translated of (0,0,height).
 */
class SingleJoint2dUpdater : public BaseHumanUpdater
{
public:
    SingleJoint2dUpdater();
    virtual ~SingleJoint2dUpdater();

    virtual bool update(move4d::Robot *h, const Eigen::Affine3d &base, const std::map<std::string, Eigen::Affine3d> &joints, const HumanSettings &settings);
    virtual bool computeConf(move4d::Robot *h, const Eigen::Affine3d &base, const std::map<std::string, Eigen::Affine3d> &joints, const HumanSettings &settings, move4d::RobotState &q);
};

} // namespace move4d

#endif // MOVE3D_SINGLEJOINT2DUPDATER_H
