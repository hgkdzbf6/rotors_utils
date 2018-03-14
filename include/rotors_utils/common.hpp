
#ifndef ROTORS_UTILS_COMMON_HPP__
#define ROTORS_UTILS_COMMON_HPP__

#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "sophus/se3.h"
#include "sophus/so3.h"

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
using namespace std;

static void tfPoseStamped2Rt(const geometry_msgs::PoseStamped& pose,
    Sophus::SO3& R,Eigen::Vector3d& t
)
{
    tfPose2RT(pose.pose,R,t);
}

static void tfPoseStamped2SE3(const geometry_msgs::PoseStamped& pose,
    Sophus::SE3& T 
){
    tfPose2SE3(pose.pose,T);
}

static void tfPose2SE3(const geometry_msgs::Pose& pose, Sophus::SE3& T)
{
    Sophus::SO3& R;
    Eigen::Vector3d& t;
    tfPose2SE3(pose,R,t);
    T=Sophus::SE3(R,t);
}

static void tfPose2Rt(const geometry_msgs::Pose& pose, Sophus::SO3& R,
    Eigen::Vector3d& t)
{
    t(0)=pose.position.x;
    t(1)=pose.position.y;
    t(2)=pose.position.z;
    Eigen::Quaterniond q(
        pose.orientation.w,
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z
    );
    R=q.matrix();
}

#endif