#ifndef BUFFER_H_
#define BUFFER_H_

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <ros/timer.h>
#include <math.h>
class Buffer
{
public:

  Buffer(double max_speed, double dt = 0.1, int mode = 0);
  Buffer();
  virtual ~Buffer();

private:
  // 模式: 0:x,y模式
  // 1: x,y,z 模式
  int mode_;
  // 最大速度,单位m/s
  double max_speed_;
  // 时间间隔
  double dt_;
  // 前一个时刻的pose
  geometry_msgs::PoseStamped pre_pose_;
  // 当前时刻的pose
  geometry_msgs::PoseStamped pose_;
  // 输出的pose
  geometry_msgs::PoseStamped output_;
  // 开始时候
  ros::NodeHandle nh_;
  // 订阅,原来没有平滑过的
  ros::Subscriber command_sub_;
  // 发布,平滑过后的
  ros::Publisher filtered_pub_;
  // 定时器
  ros::Timer timer_;
  // 运行,timer的回调函数
  void TimerCallback(const ros::TimerEvent & e);
  void CommandCallback(const geometry_msgs::PoseStampedConstPtr & msg);
};

#endif // BUFFER_H_