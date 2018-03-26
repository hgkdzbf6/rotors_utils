#include "rotors_utils/common.hpp"


using namespace Sophus;
void geoPose2Rt(const geometry_msgs::Pose& pose, Sophus::SO3& R,
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
  R=Sophus::SO3(q.matrix());
}

void geoPose2SE3(const geometry_msgs::Pose& pose, Sophus::SE3& T)
{
  Sophus::SO3 R;
  Eigen::Vector3d t;
  geoPose2Rt(pose,R,t);
  T=Sophus::SE3(R,t);
}

void geoPoseStamped2Rt(const geometry_msgs::PoseStamped& pose,
  Sophus::SO3& R,Eigen::Vector3d& t)
{
  geoPose2Rt(pose.pose,R,t);
}

void geoPoseStamped2SE3(const geometry_msgs::PoseStamped& pose,
  Sophus::SE3& T){
  geoPose2SE3(pose.pose,T);
}

void SE32geoPose(const Sophus::SE3& T, geometry_msgs::Pose& pose){
  Eigen::Quaterniond q;
  Eigen::Vector3d t;
  q = T.unit_quaternion();
  pose.orientation.w = q.w();
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  t = T.translation();
  pose.position.x=t(0);
  pose.position.y=t(1);
  pose.position.z=t(2);
}

void geoPose2Sim3(const geometry_msgs::Pose& pose, Sophus::Sim3& S){
  geoPose2Sim3(pose,1,S);
}

void geoPose2Sim3(const geometry_msgs::Pose& pose,const double s, Sophus::Sim3& S){
  Sophus::SE3 T;
  geoPose2SE3(pose,T);
  S=Sophus::Sim3::from_SE3(T);
  S.scso3().setScale(s);
}

void geoPoseStamped2Sim3(const geometry_msgs::PoseStamped& pose, Sophus::Sim3& S){
  geoPose2Sim3(pose.pose,S);
}

void Sim32geoPose(const Sophus::Sim3& S, geometry_msgs::Pose& pose, double& s){
  s=S.scale();
  Sophus::SE3 T;
  T=S.to_SE3();
  SE32geoPose(T,pose);
}

void Sim32geoPoseStamped(const Sophus::Sim3& S,geometry_msgs::PoseStamped& pose, double& s){
  Sim32geoPose(S,pose.pose,s);
}

void SE32Sim3(const Sophus::SE3& T, const double s, Sophus::Sim3& S){
  S=Sophus::Sim3(s,T.rotation_matrix(),T.translation());
}