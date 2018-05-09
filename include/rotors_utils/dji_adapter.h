#ifndef ROTORS_UTILS_DJI_ADAPTER_H_
#define ROTORS_UTILS_DJI_ADAPTER_H_

// 功能: 把传感器的信息打包成nav_msgs/Odometry消息

#include <ros/ros.h>
#include <ros/timer.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float32.h>
#include <ros/timer.h>
class DjiAdapter{
public:
  DjiAdapter();
  virtual ~DjiAdapter();
private:

  ros::NodeHandle nh_;
  ros::Subscriber pos_sub_;
  ros::Subscriber atti_sub_;
  ros::Subscriber imu_sub_;
  ros::Subscriber rate_sub_;
  ros::Subscriber vel_sub_;
  ros::Subscriber height_sub_;

  ros::Publisher odo_pub_;
  ros::Publisher cmd_pub_;  

  float height_;
  geometry_msgs::QuaternionStamped atti_;
  geometry_msgs::Vector3Stamped rate_;
  geometry_msgs::Vector3Stamped vel_;
  geometry_msgs::PoseWithCovarianceStamped pos_;
  sensor_msgs::Imu imu_;

  nav_msgs::Odometry odometry_;
  ros::Timer timer_;

  // svo 
  void PositionCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr & msg);
  void AttitudeCallback(const geometry_msgs::QuaternionStampedConstPtr & msg);
  void ImuCallback(const sensor_msgs::ImuConstPtr & msg);
  void RateCallback(const geometry_msgs::Vector3StampedConstPtr & msg);
  void VelocityCallback(const geometry_msgs::Vector3StampedConstPtr & msg);
  void HeightCallback(const std_msgs::Float32ConstPtr & msg);
  void TimerCallback(const ros::TimerEvent & e);
};

#endif