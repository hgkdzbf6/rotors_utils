
#ifndef ROTORS_UTILS_COMMON_HPP__
#define ROTORS_UTILS_COMMON_HPP__

#include <iostream>                                                              
#include <string>
#include <sstream>
#include <vector>

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

template<typename T> inline void GetRosParameter(const ros::NodeHandle& nh,
                                                 const std::string& key,
                                                 const T& default_value,
                                                 T* value) {
  ROS_ASSERT(value != nullptr);
  bool have_parameter = nh.getParam(key, *value);
  if (!have_parameter) {
    ROS_WARN_STREAM("[rosparam]: could not find parameter " << nh.getNamespace()
                    << "/" << key << ", setting to default: " << default_value);
    *value = default_value;
  }
}

using namespace std;
inline void Parse(vector<int> &arr, const string &str, char delim=' ') {
  vector<string> tokens;
  stringstream ss(str); //convert string to stream
  string item;
  while(getline(ss, item, delim)) {
    tokens.push_back(item); //add token to vector
  }
  for(auto it=tokens.begin();it<tokens.end();it++)
  {
    arr.push_back(atoi(it->c_str()));
  }
}

// 这里leader虽然只有一个,但是还是设成一个数组吧
inline void readLeaders(const vector<int> &arr, const int size, const int index,
 vector<int> & leaders){
  int i=size*index;
  for(;i<size*(index+1);i++){
    if(arr[i]==1)
      leaders.push_back(i%size);
  }
}

inline void readFollowers(const vector<int> &arr,const int size, const int index,
 vector<int> & followers){
   int i=index;
   for(;i<size*size;i=i+size){
     if(arr[i]==1){
       followers.push_back(i/size);
     }
   }
}

#endif