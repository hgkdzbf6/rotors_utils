
#ifndef ROTORS_UTILS_COMMON_HPP__
#define ROTORS_UTILS_COMMON_HPP__

#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <sophus/so3.h>
#include <sophus/se3.h>
#include <sophus/scso3.h>
#include <sophus/sim3.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
using namespace std;

#define LEADER_INDEX 0
#define FOLLOWER_INDEX 1
static const string BASE_NS="hummingbird";
static const string LEADER_NS= BASE_NS + to_string(LEADER_INDEX);
static const string FOLLOWER_NS=BASE_NS + to_string(FOLLOWER_INDEX);

void geoPose2Rt(const geometry_msgs::Pose& pose, Sophus::SO3& R,  Eigen::Vector3d& t);
void geoPose2SE3(const geometry_msgs::Pose& pose, Sophus::SE3& T);
void geoPoseStamped2Rt(const geometry_msgs::PoseStamped& pose,  Sophus::SO3& R,Eigen::Vector3d& t);
void geoPoseStamped2SE3(const geometry_msgs::PoseStamped& pose,  Sophus::SE3& T);
void SE32geoPose(const Sophus::SE3& T, geometry_msgs::Pose& pose);

void geoPose2Sim3(const geometry_msgs::Pose& pose, Sophus::Sim3& S);
void geoPose2Sim3(const geometry_msgs::Pose& pose, double s, Sophus::Sim3& S);
void geoPoseStamped2Sim3(const geometry_msgs::PoseStamped& pose, Sophus::Sim3& S);
void Sim32geoPose(const Sophus::Sim3& S, geometry_msgs::Pose& pose);
void Sim32geoPoseStamped(const Sophus::Sim3& S,geometry_msgs::PoseStamped& pose);

void SE32Sim3(const Sophus::SE3& T, const double s, Sophus::Sim3& S);
#endif