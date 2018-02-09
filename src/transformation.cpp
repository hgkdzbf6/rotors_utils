
#include "rotors_utils/transformation.hpp"

Transformation::Transformation():base_twist_(){
  ros::NodeHandle pnh("~");
  // magnetometer -> vector3  
  pnh.param<std::string>("mag_sub_frame",mag_sub_frame_,"mag_sub_frame");
  pnh.param<std::string>("mag_pub_frame",mag_pub_frame_,"mag_pub_frame");
  
  // for estimator. the estimator will publish pose while some one 
  pnh.param<std::string>("filtered_imu_sub_frame",filtered_imu_sub_frame_,"filtered_imu");

  pnh.param<std::string>("pressure_sub_frame",pressure_sub_frame_,"air_pressure");
  pnh.param<std::string>("baro_pub_frame",baro_pub_frame_,"altimeter");

  pnh.param<std::string>("odometry_pub_frame",odometry_pub_frame_,"odometry_pub_frame");
  pnh.param<std::string>("odometry_sub_frame",odometry_sub_frame_,"odometry_sensor1/odometry");
  pnh.param<std::string>("estimate_pose_frame",estimate_pose_frame_,"estimate_pose");

  pnh.param<std::string>("image_sub_frame",image_sub_frame_,"image_sub_frame");
  pnh.param<std::string>("image_pub_frame",image_pub_frame_,"image_pub_frame");

  pnh.param<std::string>("svo_sub_frame",svo_sub_frame_,"svo/pose");
  pnh.param<std::string>("svo_pub_frame",svo_pub_frame_,"svo/fusion_pose");

  pnh.param<double>("ref_north",ref_north_,0.0000332802);
  pnh.param<double>("ref_east",ref_east_,-0.0000034168);
  pnh.param<double>("ref_down",ref_down_,0.0000353465);
  mag_sub_=nh_.subscribe(mag_sub_frame_,10,&Transformation::MagCallback,this);
  mag_pub_=nh_.advertise<geometry_msgs::Vector3Stamped>(mag_pub_frame_,10);

  filtered_imu_sub_=nh_.subscribe(filtered_imu_sub_frame_,10,&Transformation::ImuCallback,this);

  pressure_sub_=nh_.subscribe(pressure_sub_frame_,10,&Transformation::PressureCallback,this);
  baro_pub_=nh_.advertise<hector_uav_msgs::Altimeter>(baro_pub_frame_,10);

  odometry_sub_=nh_.subscribe(odometry_sub_frame_,10,&Transformation::OdometryCallback,this);
  odometry_pub_=nh_.advertise<nav_msgs::Odometry>(odometry_pub_frame_,10);
  estimate_pose_sub_=nh_.subscribe(estimate_pose_frame_,10,&Transformation::EstimatePoseCallback,this);
  velocity_sub_=nh_.subscribe("estimate_velocity",10,&Transformation::VelocityCallback,this);
  rate_sub_=nh_.subscribe("estimate_rate",10,&Transformation::RateCallback,this);

  image_pub_=nh_.advertise<geometry_msgs::TwistStamped>(image_pub_frame_,10);
  image_sub_=nh_.subscribe(image_sub_frame_,10,&Transformation::ImageTwistCallback,this);

  svo_pub_=nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(svo_pub_frame_,10);
  svo_sub_=nh_.subscribe(svo_sub_frame_,10,&Transformation::SvoCallback,this);
  
  timer_=nh_.createTimer(ros::Duration(0.01),&Transformation::TimerCallback,this);
}

void Transformation::ImageTwistCallback(const geometry_msgs::TwistStampedConstPtr & msg){
  twist_=*msg;
}

void Transformation::TimerCallback(const ros::TimerEvent & e){
  // nav_msgs::Odometry odometry;
  geometry_msgs::TwistStamped twist;
  double test;
  test=fabs(base_twist_.twist.linear.x-twist_.twist.linear.x);
  if(test>0.5){
    base_twist_.twist.linear.x=twist_.twist.linear.x;
    base_twist_.twist.linear.y=twist_.twist.linear.y;
    base_twist_.twist.angular.x=twist_.twist.angular.x;
    base_twist_.twist.angular.y=twist_.twist.angular.y;
  }
  twist.header.stamp=ros::Time::now();
  twist.header.frame_id=twist_.header.frame_id;
  twist.twist.linear.x=twist_.twist.linear.x-base_twist_.twist.linear.x;
  twist.twist.linear.y=twist_.twist.linear.y-base_twist_.twist.linear.y;
  twist.twist.angular.x=twist_.twist.angular.x-base_twist_.twist.angular.x;
  twist.twist.angular.y=twist_.twist.angular.y-base_twist_.twist.angular.y;
  twist.twist.angular.x/=15;
  twist.twist.angular.y/=15;
  image_pub_.publish(twist);

  odometry_.header=pose_.header;
  odometry_.pose.pose.position=pose_.pose.position;
  odometry_.pose.pose.orientation=pose_.pose.orientation;
  odometry_.twist.twist.linear=velocity_.vector;
  odometry_.twist.twist.angular=rate_.vector;
  odometry_pub_.publish(odometry_);
}

void Transformation::SvoCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr & msg){

  geometry_msgs::PoseWithCovarianceStamped pose;
  pose.header=msg->header;
  pose.pose.pose=msg->pose.pose;
  tf::Quaternion quat;
  tf::quaternionMsgToTF(pose.pose.pose.orientation, quat);
  double yaw,pitch,roll;
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  double x,y;
  // double cosYaw=cos(yaw),sinYaw=sin(yaw);
  double cosYaw=cos(yaw),sinYaw=sin(yaw);
  // x=pose.pose.pose.position.y;
  // y=pose.pose.pose.position.x;
  y =   pose.pose.pose.position.x * cosYaw + pose.pose.pose.position.y * sinYaw;
  x = - pose.pose.pose.position.x * sinYaw + pose.pose.pose.position.y * cosYaw;
  x = x * 3 / odometry_.pose.pose.position.z * 2.5;
  y = y * 3 / odometry_.pose.pose.position.z * 2.5;
  pose.pose.pose.position.x=x;
  pose.pose.pose.position.y=y;
  pose.pose.covariance[0]=msg->pose.covariance[0];
  pose.pose.covariance[1]=msg->pose.covariance[1];
  pose.pose.covariance[2]=msg->pose.covariance[2];
  pose.pose.covariance[3]=msg->pose.covariance[3];
  // pose.pose.pose.position.z+=odometry_.pose.pose.position.z;
  // ROS_INFO("I plus %lf",odometry_.pose.pose.position.z);
  svo_pub_.publish(pose);
}

void Transformation::EstimatePoseCallback(const geometry_msgs::PoseStampedConstPtr & msg){
  pose_=*msg;
}

void Transformation::RateCallback(const geometry_msgs::Vector3StampedConstPtr & msg){
  rate_=*msg;
}

void Transformation::VelocityCallback(const geometry_msgs::Vector3StampedConstPtr  & msg){
  velocity_=*msg;
}

void Transformation::OdometryCallback(const nav_msgs::OdometryConstPtr & msg){

}

void Transformation::PressureCallback(const sensor_msgs::FluidPressureConstPtr & msg){
  pressure_=*msg;
  baro_.header=pressure_.header;
  baro_.pressure=pressure_.fluid_pressure/100;
  baro_.altitude=altitudeFromPressure(baro_.pressure,STANDARD_PRESSURE);
  baro_.qnh=STANDARD_PRESSURE;
  baro_pub_.publish(baro_);
}

void Transformation::ImuCallback(const sensor_msgs::ImuConstPtr & msg){
  imu_=*msg;
}

void Transformation::MagCallback(const sensor_msgs::MagneticFieldConstPtr & msg){
  geometry_msgs::Vector3Stamped v;
  // tf::Vector3 axis(1,0,0);
  // tf::Vector3 vv(ref_north_,ref_east_,ref_down_);
  tf::Vector3 vv2(msg->magnetic_field.x,msg->magnetic_field.y,msg->magnetic_field.z);
  // vv=vv.normalize();
  vv2=vv2.normalize();
  // double angle1=vv.angle(axis);
  // double angle2=vv2.angle(axis);
  // tf::Quaternion q1(
  //   vv.x()*sin(angle1/2),
  //   vv.y()*sin(angle1/2),
  //   vv.z()*sin(angle1/2),
  //   cos(angle1/2)
  // ),q2(
  //   vv2.x()*sin(angle2/2),
  //   vv2.y()*sin(angle2/2),
  //   vv2.z()*sin(angle2/2),
  //   cos(angle2/2)
  // );
  // tf::Quaternion q;
  // q=q1.inverse()*q2;
  // double roll,pitch,yaw;
  // tf::Matrix3x3 m(q);
  // m.getRPY(roll,pitch,yaw);

  v.header.frame_id=msg->header.frame_id;
  v.header.stamp=ros::Time::now();
  // v.vector.x=roll;
  // v.vector.y=pitch;
  // v.vector.z=yaw;  
  v.vector.x=vv2.x();
  v.vector.y=vv2.y();
  v.vector.z=vv2.z();
  mag_pub_.publish(v);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "rotors_joy_interface");
  Transformation transformation;
  ros::spin();
  return 0;
}
