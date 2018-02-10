#include "rotors_utils/trajectory_generation.hpp"

TrajectoryGeneration::TrajectoryGeneration(){
  ros::NodeHandle pnh("~");
  follower_sub_=nh_.subscribe("target_position",5,&TrajectoryGeneration::RelativePositionCallback,this);
  follower_pub_=nh_.advertise<geometry_msgs::PoseStamped>("relative_position0",5);
  timer_=nh_.createTimer(ros::Duration(0.1),&TrajectoryGeneration::TimerCallback,this);
}
void TrajectoryGeneration::RelativePositionCallback(const geometry_msgs::PoseStampedConstPtr& msg){
  ps_=*msg;
}
void TrajectoryGeneration::TimerCallback(const ros::TimerEvent & e){
  
}
int main(int argc, char** argv) {
  ros::init(argc, argv, "rotors_joy_interface");
  TrajectoryGeneration trajectory_generation;
  ros::spin();
  return 0;
}
