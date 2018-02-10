#ifndef ROTORS_UTILS_TRAJECTORY_GENERATION_H_
#define ROTORS_UTILS_TRAJECTORY_GENERATION_H_

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
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>

// 这个类的作用是生成轨迹,输入是目标位置和当前位置,以及时间戳
// 输出是下一个时刻的目标位置
// 存在的意义是给其他算法留出接口,然后
class TrajectoryGeneration{
private:
  ros::NodeHandle nh_;

  ros::Timer timer_;
  ros::Subscriber follower_sub_;
  ros::Publisher follower_pub_;

  geometry_msgs::PoseStamped ps_;
  void TimerCallback(const ros::TimerEvent & e);
  void RelativePositionCallback(const geometry_msgs::PoseStampedConstPtr& msg);

public:
  TrajectoryGeneration();
};

#endif // ROTORS_UTILS_TRAJECTORY_GENERATION_H_
