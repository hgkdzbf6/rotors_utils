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
#include <sensor_msgs/NavSatFix.h>
#include <std_srvs/Trigger.h>

// dji sdk 所需
#include "dji_sdk/dji_sdk.h"
#include <dji_sdk/DroneTaskControl.h>
#include <dji_sdk/SDKControlAuthority.h>
#include <dji_sdk/QueryDroneVersion.h>
#include <dji_sdk/SetLocalPosRef.h>

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
  ros::Subscriber lee_sub_;

  ros::Publisher odo_pub_;
  ros::Publisher attitude_pub_control_;

  ros::ServiceClient control_authority_service_client_;
  ros::ServiceClient drone_task_service_client_;
  ros::ServiceClient query_version_service_client_;
  ros::ServiceClient set_local_pos_reference_;

  ros::ServiceServer take_off_service_server_;

  sensor_msgs::NavSatFix current_gps_;
  geometry_msgs::Quaternion current_atti_;
  geometry_msgs::Point current_local_pos_;
  geometry_msgs::TwistStamped twist_;

  sensor_msgs::Joy cmd_;

  bool obtain_control_result_;
  // 起飞的结果
  bool takeoff_result_;
  // 是否接收到要起飞的信号
  bool take_off_received_;
  uint8_t flight_status_; 
  uint8_t display_mode_;  
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
  void TwistCallback(const geometry_msgs::TwistStampedConstPtr& msg);
  bool TakeoffCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  bool setLocalPosition();
  bool isM100();
  bool takeoffLand(int task);
  bool M100monitoredTakeoff();
  bool monitoredTakeoff();
  bool obtainControl();
};

#endif