#ifndef ROTORS_UTILS_TRAJECTORY_GENERATION_H_
#define ROTORS_UTILS_TRAJECTORY_GENERATION_H_

#include <ros/ros.h>
#include <tf/tf.h>
#include <ros/timer.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include "rotors_utils/common.hpp"

// 这个类的作用是生成轨迹,输入是目标位置和当前位置,以及时间戳
// 输出是下一个时刻的目标位置
// 存在的意义是给其他算法留出接口,然后
class TrajectoryGeneration{
private:
  ros::NodeHandle nh_;
  ros::Timer timer_;
  int my_id_;
  int follower_id_;
	bool is_self_control_;

  ros::Publisher follower_pub_;
  ros::Subscriber T_L2_F2_sub_;
  ros::Subscriber T_LS_L2_sub_;
  ros::Subscriber T_FS_F2_sub_;
  ros::Subscriber T_L2_dF2_sub_;

  std::string T_L2_F2_str_;
  std::string T_LS_L2_str_;
  std::string T_FS_F2_str_;
  std::string T_L2_dF2_str_;
  std::string T_FW_dF2_str_;
  
  geometry_msgs::PoseStamped pose_;
  
  Sophus::SE3 T_LS_LW_;
  Sophus::SE3 T_FS_FW_;
  Sophus::SE3 T_LW_FW_; 
  Sophus::SE3 T_LS_L2_;
  Sophus::SE3 T_L2_F2_;
  Sophus::SE3 T_FS_F2_;
  Sophus::SE3 T_FW_dF2_;
  Sophus::SE3 T_L2_dF2_;

  Sophus::Sim3 S_LS_LW_;
  Sophus::Sim3 S_FS_FW_;
  Sophus::Sim3 S_LW_FW_;
  Sophus::Sim3 S_LS_L2_;
  Sophus::Sim3 S_L2_F2_;
  Sophus::Sim3 S_FS_F2_;
  Sophus::Sim3 S_FW_dF2_;
  Sophus::Sim3 S_L2_dF2_;

  void TimerCallback(const ros::TimerEvent & e);
  void RelativePositionCallback(const geometry_msgs::PoseStampedConstPtr& msg);

void T_L2_F2Callback(const geometry_msgs::PoseStampedConstPtr& msg);
void T_LS_L2Callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);
void T_FS_F2Callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);
void T_L2_dF2Callback(const geometry_msgs::PoseStampedConstPtr& msg);
public:
  TrajectoryGeneration();
};

#endif // ROTORS_UTILS_TRAJECTORY_GENERATION_H_
