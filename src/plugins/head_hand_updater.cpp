#undef QT_LIBRARY
#include "move4d_ros_lib/plugins/head_hand_updater.h"

#include <ros/ros.h>

#include <move4d/API/Device/robot.hpp>
#include <move4d/API/Device/joint.hpp>
#include <move4d/utils/Geometry.h>
#include <move4d/API/ConfigSpace/RobotState.hpp>

#include <pluginlib/class_list_macros.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

PLUGINLIB_EXPORT_CLASS(move4d::HeadHandUpdater, move4d::BaseHumanUpdater)

namespace move4d {

typedef std::map<std::string, Eigen::Affine3d>::const_iterator AffineMapConstIterator;

HeadHandUpdater::HeadHandUpdater()
{
    _tf_br = new tf::TransformBroadcaster();
}

HeadHandUpdater::~HeadHandUpdater()
{
    delete _tf_br;
}

bool HeadHandUpdater::update(Robot *h, const Eigen::Affine3d &base, const std::map<std::string, Eigen::Affine3d> &joints, const HumanSettings &settings)
{
    RobotState q(h);
    bool ok=computeConf(h,base,joints,settings,q);
    h->setAndUpdate(q);
    return ok;
}

bool HeadHandUpdater::computeConf(Robot *h, const Eigen::Affine3d &base, const std::map<std::string, Eigen::Affine3d> &joints, const HumanSettings &settings, RobotState &q)
{
    using namespace Eigen;
    bool ok=true;
    q = *h->getCurrentPos();
    std::string torsojoint_name("base_torso"), headjoint_name("torso_head"), handjoint_name("right_hand_joint");
    std::string input_head_name("head"),input_torso_name("torso"),input_hand_name("rightHand");

    //base
    ok=updateFreeFlyer(q,6,base);

    //tf for debug
    tf::Transform tr;
    tr.setFromOpenGLMatrix(base.data());
    _tf_br->sendTransform(tf::StampedTransform(tr,ros::Time::now(),"move4d_origin","/move4d_dbg/base"));
    for (auto  it=joints.begin();it!=joints.end();it++){
        tf::Transform tr;
        tr.setFromOpenGLMatrix(it->second.data());
        _tf_br->sendTransform(tf::StampedTransform(tr,ros::Time::now(),"move4d_origin","/move4d_dbg/"+it->first));
    }


    //joints

    uint jnt_id;
    AffineMapConstIterator affine_it;
    Affine3d rel_pos_torso, abs_pos_head,abs_pos_torso;
    if(ok){
        //torso
        //torso follows base
        //torso origin is the same as the head without the x and y rotations
        if(getBothJoints(h,joints,input_head_name,torsojoint_name,affine_it,jnt_id)){
            Joint *joint=h->getJoint(jnt_id);
            assert(joint->getName() == torsojoint_name);
            abs_pos_head=affine_it->second;
            abs_pos_torso=affine_it->second;
            Eigen::Vector3d all_rot = abs_pos_torso.rotation().eulerAngles(0,1,2);
            abs_pos_torso =Eigen::AngleAxisd(all_rot[2],Eigen::Vector3d::UnitZ());
            abs_pos_torso.pretranslate(affine_it->second.translation());

            rel_pos_torso = m3dGeometry::changeFrame(abs_pos_torso,m3dGeometry::absoluteFramePos(),base);
            updateFreeFlyer(q,joint->getIndexOfFirstDof(),rel_pos_torso);
            tf::Transform tr;
            tr.setFromOpenGLMatrix(rel_pos_torso.data());
            _tf_br->sendTransform(tf::StampedTransform(tr,ros::Time::now(),"/move4d_dbg/base","/move4d_dbg/"+joint->getName()));
            //reset Rx and Ry
            q[joint->getIndexOfFirstDof()+3]=0;
            q[joint->getIndexOfFirstDof()+4]=0;
        }else{
            ok=false;
        }
    }

    if(ok){
        //head
        //follows torso
        if(getBothJoints(h,joints,input_head_name,headjoint_name,affine_it,jnt_id)){
            Joint *joint=h->getJoint(jnt_id);
            assert(joint->getName() == headjoint_name);
            Affine3d rel_pos_head = m3dGeometry::changeFrame(affine_it->second,m3dGeometry::absoluteFramePos(),abs_pos_torso);
            updateFreeFlyer(q,joint->getIndexOfFirstDof(),rel_pos_head);
            tf::Transform tr;
            tr.setFromOpenGLMatrix(rel_pos_head.data());
            _tf_br->sendTransform(tf::StampedTransform(tr,ros::Time::now(),"/move4d_dbg/"+torsojoint_name,"/move4d_dbg/"+joint->getName()));
            assert(rel_pos_head.translation().norm() <0.00001);
        }else{
            ok=false;
        }
    }

    if(ok){
        //hand
        //follows base
        if(getBothJoints(h,joints,input_hand_name,handjoint_name,affine_it,jnt_id)){
            Joint *joint=h->getJoint(jnt_id);
            assert(joint->getName() == handjoint_name);
            Affine3d rel_pos_hand = m3dGeometry::changeFrame(affine_it->second,m3dGeometry::absoluteFramePos(),base);
            updateFreeFlyer(q,joint->getIndexOfFirstDof(),rel_pos_hand);
        }else{
            ok=false;
        }
    }
    return ok;
}

bool HeadHandUpdater::getBothJoints(Robot *h, const std::map<std::string, Eigen::Affine3d> &joints,
                                   const std::string &input_name, const std::string &m3d_name,
                                   AffineMapConstIterator &joint, uint &m3d_joint_index)
{
    bool ok(true);
    AffineMapConstIterator it=joints.find(input_name);
    if(it==joints.end()){
        ok=false;
        ROS_WARN("Human model %s is updated without specifying its %s joint position",h->getName().c_str(),input_name.c_str());
    }
    Joint *jnt;
    if(ok){
        jnt=h->getJoint(m3d_name);
        if(!jnt){
            ok=false;
            ROS_WARN("Human model %s has no joint named %s in move4d",h->getName().c_str(),m3d_name.c_str());
        }
    }
    if(ok){
        joint = it;
        m3d_joint_index = jnt->getId();
    }
    return ok;
}

bool HeadHandUpdater::updateFreeFlyer(RobotState &q, uint first_dof_index, const Eigen::Affine3d &pos)
{
    for(uint i=0;i<3;i++){
        q[i+first_dof_index]=pos.translation()[i];
    }
    Eigen::Vector3d rotations = pos.rotation().eulerAngles(0,1,2);
    for(uint i=0;i<3;i++){
        q[i+3+first_dof_index]=rotations[i];
    }
    // tf::Transform tr;
    // tf::Quaternion tfq(rotations[0],rotations[1],rotations[2]);
    // tr.setRotation(tfq);
    // tr.setOrigin(tf::Vector3(pos.translation()[0],pos.translation()[1],pos.translation()[2]));
    // uint dummy;
    // tf_br.sendTransform(tf::StampedTransform(tr,ros::Time::now(),"move4d_origin","/move4d_dbg/"+q.getRobot()->getJointAt(first_dof_index,dummy)->getName()));
}

} // namespace move4d

