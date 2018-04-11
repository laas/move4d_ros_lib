#include "move4d_ros_lib/plugins/gaze_updater.h"

#include <move4d/API/Device/robot.hpp>
#include <move4d/API/Device/joint.hpp>
#include <move4d/API/ConfigSpace/RobotState.hpp>
#undef QT_LIBRARY
#include <move4d/utils/Geometry.h>

#include <pluginlib/class_list_macros.h>

#include <ros/ros.h>


PLUGINLIB_EXPORT_CLASS(move4d::GazeUpdater, move4d::BaseHumanUpdater)

namespace move4d {

GazeUpdater::GazeUpdater():BaseHumanUpdater()
{

}

GazeUpdater::~GazeUpdater()
{

}

bool GazeUpdater::update(Robot *h, const Eigen::Affine3d &base, const std::map<std::string, Eigen::Affine3d> &joints, const HumanSettings &settings)
{
    ROS_ASSERT(h);
    RobotState q(h);
    bool ok=computeConf(h,base,joints,settings,q);
    if(ok)
        h->setAndUpdate(q);
    return ok;
}

bool GazeUpdater::computeConf(Robot *h, const Eigen::Affine3d &base, const std::map<std::string, Eigen::Affine3d> &joints, const HumanSettings &settings, RobotState &q)
{
    bool ok=true;
    Joint *gaze_joint = nullptr;
    if(h->getHriAgent())
        gaze_joint = h->getHriAgent()->perspective;
    if(!gaze_joint){
        ROS_FATAL("No perspective joint set for robot %s",h->getName().c_str());
        return false;
    }
    //base frame is the projection of the gaze frame to the plane Z=0.
    //TODO we don't update the orientation
    for(uint i=0;i<2;i++){
        q[i+6]=base.translation()[i];
    }
    q[2+6]=0.; //z=0
    //rotation
    //compute the projection of the camera Z vector onto the env (X,Y) plane
    Eigen::Vector3d z_gaze = base.linear() * Eigen::Vector3d::UnitZ();
    Eigen::Vector3d z_proj = z_gaze;
    z_gaze.z()=0.;
    z_proj.normalize();
    //that vector is // to the X axis of the base frame
    //compute the rotation between current X axis of the conf base and desired one
    Eigen::Vector3d x=Eigen::Vector3d::UnitX();
    Eigen::Vector3d z=Eigen::Vector3d::UnitZ();
    double angle = std::atan2((x.cross(z_proj)).dot(z),x.dot(z_proj));
    q[5+6]=angle;
    Eigen::Affine3d base_pos,rel_gaze_pos;
    //the position of the move4d base joint (3d pos + theta)
    base_pos=Eigen::Translation3d(base.translation().x(),base.translation().y(),0.)
            * q.getQuaternion();
    //pose of the gaze in the base frame
    rel_gaze_pos = m3dGeometry::changeFrame(base,m3dGeometry::absoluteFramePos(),base_pos);
    Eigen::Vector3d pos = rel_gaze_pos.translation();
    Eigen::Vector3d rotations = rel_gaze_pos.rotation().eulerAngles(0,1,2);
    uint ix = gaze_joint->getIndexOfFirstDof()-6;
    ROS_DEBUG("first dof of perspective joint is %i",ix);
    for (uint i=0;i<3;i++){
        ROS_ASSERT(ix+i<q.getNbDof());
        ROS_ASSERT(ix+i+3<q.getNbDof());
        q[ix+i]=pos[i];
        q[ix+i+3]=rotations[i];
    }
    
    return ok;
}

} // namespace move4d

