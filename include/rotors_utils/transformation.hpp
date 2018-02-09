
#ifndef ROTORS_UTILS_TRANSFORMATION_H_
#define ROTORS_UTILS_TRANSFORMATION_H_


// #define USE_APRIL_TAG 
#define USE_SVO 

#include <geometry_msgs/Vector3Stamped.h>
#include <ros/ros.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Imu.h>
#include <tf/tf.h>
#include <ros/timer.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/FluidPressure.h>
#include <hector_uav_msgs/Altimeter.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_srvs/Trigger.h>

#ifdef USE_SVO
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#endif



static const hector_uav_msgs::Altimeter::_qnh_type STANDARD_PRESSURE = 1013.25;

class Transformation{
private:
  ros::NodeHandle nh_;

  ros::Publisher mag_pub_;
  ros::Subscriber mag_sub_;

  ros::Subscriber filtered_imu_sub_;

  ros::Publisher baro_pub_;
  ros::Subscriber pressure_sub_;

  ros::Publisher odometry_pub_;
  ros::Subscriber odometry_sub_;
  ros::Subscriber estimate_pose_sub_;
  ros::Subscriber velocity_sub_;
  ros::Subscriber rate_sub_;

  ros::Publisher image_pub_;
  ros::Subscriber image_sub_;

  ros::Publisher svo_pub_;
  ros::Subscriber svo_sub_;

  ros::Timer timer_;

  std::string mag_pub_frame_;
  std::string mag_sub_frame_;
  std::string filtered_imu_sub_frame_;
  std::string pressure_sub_frame_;
  std::string baro_pub_frame_;
  std::string odometry_pub_frame_;
  std::string odometry_sub_frame_;
  std::string estimate_pose_frame_;
  std::string image_sub_frame_;
  std::string image_pub_frame_;
  std::string svo_sub_frame_;
  std::string svo_pub_frame_;

  double ref_north_;
  double ref_east_;
  double ref_down_;
  
  geometry_msgs::TwistStamped base_twist_;
  geometry_msgs::TwistStamped twist_;
  hector_uav_msgs::Altimeter baro_;
  geometry_msgs::PoseStamped pose_;
  nav_msgs::Odometry odometry_;
  sensor_msgs::FluidPressure pressure_;
  sensor_msgs::Imu imu_;
  sensor_msgs::MagneticField mag_;

  geometry_msgs::Vector3Stamped velocity_,rate_;
  void SvoCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr & msg);
  void ImageTwistCallback(const geometry_msgs::TwistStampedConstPtr & msg);
  void TimerCallback(const ros::TimerEvent & e);
  void VelocityCallback(const geometry_msgs::Vector3StampedConstPtr & msg);
  void RateCallback(const geometry_msgs::Vector3StampedConstPtr & msg);
  void PressureCallback(const sensor_msgs::FluidPressureConstPtr & msg);
  void ImuCallback(const sensor_msgs::ImuConstPtr & msg);
  void MagCallback(const sensor_msgs::MagneticFieldConstPtr & msg);
  void EstimatePoseCallback(const geometry_msgs::PoseStampedConstPtr & msg);
  void OdometryCallback(const nav_msgs::OdometryConstPtr & msg);

  static inline hector_uav_msgs::Altimeter::_altitude_type altitudeFromPressure(
    hector_uav_msgs::Altimeter::_pressure_type pressure, 
    hector_uav_msgs::Altimeter::_qnh_type qnh = STANDARD_PRESSURE) {
    return 288.15 / 0.0065 * (1.0 - pow(pressure / qnh, 1.0/5.255));
  }

  static inline hector_uav_msgs::Altimeter::_pressure_type pressureFromAltitude(
    hector_uav_msgs::Altimeter::_altitude_type altitude, 
    hector_uav_msgs::Altimeter::_qnh_type qnh = STANDARD_PRESSURE) {
    return qnh * pow(1.0 - (0.0065 * altitude) / 288.15, 5.255);
  }
public:
  Transformation();
};

#endif // ROTORS_UTILS_TRANSFORMATION_H_
