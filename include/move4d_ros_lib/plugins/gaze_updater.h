#ifndef MOVE4D_GAZEUPDATER_H
#define MOVE4D_GAZEUPDATER_H

#include "move4d_ros_lib/plugins/base_human_updater.h"

namespace move4d {

/**
 * @brief The GazeUpdater class is a plugin class for the HumanMgr. It takes a 3D postion of the gaze frame.
 */
class GazeUpdater : public BaseHumanUpdater
{
public:
    GazeUpdater();
    virtual ~GazeUpdater();

    virtual bool update(move4d::Robot *h, const Eigen::Affine3d &base, const std::map<std::string, Eigen::Affine3d> &joints, const HumanSettings &settings);
    virtual bool computeConf(move4d::Robot *h, const Eigen::Affine3d &base, const std::map<std::string, Eigen::Affine3d> &joints, const HumanSettings &settings, move4d::RobotState &q);
};

} // namespace move4d

#endif // MOVE4D_GAZEUPDATER_H
