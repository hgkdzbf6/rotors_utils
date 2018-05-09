#include "rotors_utils/dji_adapter.h"

DjiAdapter::DjiAdapter(){
  ros::NodeHandle pnh("~");
  pos_sub_=nh_.subscribe("dji_sdk/local_position",10,&DjiAdapter::PositionCallback,this);
  vel_sub_=nh_.subscribe("dji_sdk/velocity",10,&DjiAdapter::VelocityCallback,this);
  imu_sub_=nh_.subscribe("dji_sdk/imu",10,&DjiAdapter::ImuCallback,this);
  atti_sub_=nh_.subscribe("dji_sdk/attitude",10,&DjiAdapter::AttitudeCallback,this);
  rate_sub_=nh_.subscribe("dji_sdk/angular_velocity_fused",10,&DjiAdapter::RateCallback,this);
  height_sub_=nh_.subscribe("dji_sdk/height_above_takeoff",10,&DjiAdapter::HeightCallback,this);

  odo_pub_=nh_.advertise<nav_msgs::Odometry>("dji_odometry",10);
  // cmd_pub_=nh_.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_rollpitch_yawrate_zposition",10);
  timer_=nh_.createTimer(ros::Duration(0.01),&DjiAdapter::TimerCallback,this);
}

DjiAdapter::~DjiAdapter(){

}
void DjiAdapter::TimerCallback(const ros::TimerEvent & e){
  odometry_.header.stamp=ros::Time::now();
  odometry_.header.frame_id="";
  odometry_.child_frame_id="";
  odometry_.pose.pose.orientation=atti_.quaternion;
  odometry_.twist.twist.linear=vel_.vector;
  odometry_.twist.twist.angular=imu_.angular_velocity;
  odometry_.pose.pose.position.x=pos_.pose.pose.position.x;
  odometry_.pose.pose.position.y=pos_.pose.pose.position.y;
  odometry_.pose.pose.position.z=height_;
  odo_pub_.publish(odometry_);
}
void DjiAdapter::PositionCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr & msg){
  pos_=*msg;
}
void DjiAdapter::AttitudeCallback(const geometry_msgs::QuaternionStampedConstPtr & msg){
  atti_=*msg;
}
void DjiAdapter::ImuCallback(const sensor_msgs::ImuConstPtr & msg){
  imu_=*msg;
}
void DjiAdapter::RateCallback(const geometry_msgs::Vector3StampedConstPtr & msg){
  rate_=*msg;
}
void DjiAdapter::VelocityCallback(const geometry_msgs::Vector3StampedConstPtr & msg){
  vel_=*msg;
}
void DjiAdapter::HeightCallback(const std_msgs::Float32ConstPtr & msg){
  height_=msg->data;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "dji_adapter");
  DjiAdapter djiAdapter;
  ros::spin();
  return 0;
}
