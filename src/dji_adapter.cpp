#include "rotors_utils/dji_adapter.h"

DjiAdapter::DjiAdapter():flight_status_(255),display_mode_(255){
  ros::NodeHandle pnh("~");
  // 订阅所需主题
  pos_sub_=nh_.subscribe("dji_sdk/local_position",10,&DjiAdapter::PositionCallback,this);
  vel_sub_=nh_.subscribe("dji_sdk/velocity",10,&DjiAdapter::VelocityCallback,this);
  imu_sub_=nh_.subscribe("dji_sdk/imu",10,&DjiAdapter::ImuCallback,this);
  atti_sub_=nh_.subscribe("dji_sdk/attitude",10,&DjiAdapter::AttitudeCallback,this);
  rate_sub_=nh_.subscribe("dji_sdk/angular_velocity_fused",10,&DjiAdapter::RateCallback,this);
  height_sub_=nh_.subscribe("dji_sdk/height_above_takeoff",10,&DjiAdapter::HeightCallback,this);
  // 广播高度,给状态估计器用.
  height_pub_ = nh_.advertise<geometry_msgs::PointStamped>("pressure_height",10);
  // 真实的里程计,这个虽然不用做控制,但是作为参考看看也是可以的
  odo_pub_=nh_.advertise<nav_msgs::Odometry>("dji_odometry",10);
  // 广播控制信号,roll pitch yaw thrust控制

  // 还要订阅从lee position controller里面出来的roll pitch yaw thrust
  lee_sub_=nh_.subscribe("cmd_twist",3,&DjiAdapter::TwistCallback,this);
  attitude_pub_control_ = nh_.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_rollpitch_yawrate_zposition",10);
  // 其他服务
  control_authority_service_client_ = nh_.serviceClient<dji_sdk::SDKControlAuthority>("dji_sdk/sdk_control_authority");
  drone_task_service_client_ = nh_.serviceClient<dji_sdk::DroneTaskControl>("dji_sdk/drone_task_control");
  query_version_service_client_ = nh_.serviceClient<dji_sdk::QueryDroneVersion>("dji_sdk/query_drone_version");
  set_local_pos_reference_ = nh_.serviceClient<dji_sdk::SetLocalPosRef>("dji_sdk/set_local_pos_ref");
  
  // 起飞服务,接受到这个服务之后才起飞
  take_off_service_server_ = nh_.advertiseService("dji_takeoff",&DjiAdapter::TakeoffCallback,this);

  cmd_.axes.push_back(0);
  cmd_.axes.push_back(0);
  cmd_.axes.push_back(0);
  cmd_.axes.push_back(0);
  // cmd_pub_=nh_.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_rollpitch_yawrate_zposition",10);
  timer_=nh_.createTimer(ros::Duration(0.1),&DjiAdapter::TimerCallback,this);
  // 获取控制权
  // 失败的原因是,dji_sdk还没有初始化完成.等5秒把.
  ros::Duration(15).sleep(); 
  obtain_control_result_ = obtainControl();
  if (!setLocalPosition()) // We need this for height
  {
    ROS_ERROR("GPS health insufficient - No local frame reference for height. Exiting.");
    return;
  }
}

DjiAdapter::~DjiAdapter(){

}

void DjiAdapter::TwistCallback(const geometry_msgs::TwistStampedConstPtr& msg){
  twist_.header.stamp = ros::Time::now();
  twist_.header.frame_id = "cmd";
  twist_.twist = msg->twist;
}

bool DjiAdapter::obtainControl()
{
  dji_sdk::SDKControlAuthority authority;
  authority.request.control_enable=1;
  control_authority_service_client_.call(authority);

  if(!authority.response.result)
  {
    ROS_ERROR("obtain control failed!");
    return false;
  }
  ROS_INFO("obtain control succeed.");
  return true;
}

bool DjiAdapter::TakeoffCallback(std_srvs::Trigger::Request &req,
		std_srvs::Trigger::Response &res)
{	
  take_off_received_=true;
	res.message="received take off success";
	res.success=true;
	return true;
}

void DjiAdapter::TimerCallback(const ros::TimerEvent & e){
  odometry_.header.stamp=ros::Time::now();
  odometry_.header.frame_id="";
  odometry_.child_frame_id="";
  odometry_.pose.pose.orientation=atti_.quaternion;
  odometry_.twist.twist.linear=vel_.vector;
  odometry_.twist.twist.angular=imu_.angular_velocity;
  odometry_.pose.pose.position.x=pos_.point.x;
  odometry_.pose.pose.position.y=pos_.point.y;
  odometry_.pose.pose.position.z=height_;
  odo_pub_.publish(odometry_);

  // roll
  cmd_.axes[0] = twist_.twist.linear.x;
  // pitch
  cmd_.axes[1] = twist_.twist.linear.y;
  // height/thrust 
  cmd_.axes[2] = twist_.twist.linear.z;
  // yawrate
  cmd_.axes[3] = twist_.twist.angular.z;
  attitude_pub_control_.publish(cmd_);
  height_point_.point.x = 0;
  height_point_.point.y = 0;
  height_point_.point.z = height_;
  height_pub_.publish(height_point_);

}

void DjiAdapter::PositionCallback(const geometry_msgs::PointStampedConstPtr & msg){
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

// 设定了本地位置,才能够得到高度
bool DjiAdapter::setLocalPosition()
{
  dji_sdk::SetLocalPosRef localPosReferenceSetter;
  set_local_pos_reference_.call(localPosReferenceSetter);
  return localPosReferenceSetter.response.result;
}

bool DjiAdapter::isM100()
{
  dji_sdk::QueryDroneVersion query;
  query_version_service_client_.call(query);

  if(query.response.version == DJISDK::DroneFirmwareVersion::M100_31)
  {
    return true;
  }

  return false;
}

bool DjiAdapter::takeoffLand(int task)
{
  dji_sdk::DroneTaskControl droneTaskControl;

  droneTaskControl.request.task = task;

  drone_task_service_client_.call(droneTaskControl);

  if(!droneTaskControl.response.result)
  {
    ROS_ERROR_STREAM("takeoff_land fail. and error id is "<< droneTaskControl.response.result);
    return false;
  }

  return true;
}

/*!
 * This function demos how to use M100 flight_status
 * to monitor the take off process with some error
 * handling. Note M100 flight status is different
 * from A3/N3 flight status.
 */
bool DjiAdapter::M100monitoredTakeoff()
{
  ros::Time start_time = ros::Time::now();

  float home_altitude = current_gps_.altitude;
  if(!takeoffLand(dji_sdk::DroneTaskControl::Request::TASK_TAKEOFF))
  {
    return false;
  }

  ros::Duration(0.01).sleep();
  ros::spinOnce();

  // Step 1: If M100 is not in the air after 10 seconds, fail.
  while (ros::Time::now() - start_time < ros::Duration(10))
  {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if(flight_status_ != DJISDK::M100FlightStatus::M100_STATUS_IN_AIR ||
      current_gps_.altitude - home_altitude < 1.0)
  {
    ROS_ERROR("Takeoff failed.");
    return false;
  }
  else
  {
    start_time = ros::Time::now();
    ROS_INFO("Successful takeoff!");
    ros::spinOnce();
  }

  return true;
}


/*!
 * This function demos how to use the flight_status
 * and the more detailed display_mode (only for A3/N3)
 * to monitor the take off process with some error
 * handling. Note M100 flight status is different
 * from A3/N3 flight status.
 */
bool DjiAdapter::monitoredTakeoff()
{
  ros::Time start_time = ros::Time::now();

  if(!takeoffLand(dji_sdk::DroneTaskControl::Request::TASK_TAKEOFF)) {
    return false;
  }

  ros::Duration(0.01).sleep();
  ros::spinOnce();

  // Step 1.1: Spin the motor
  while (flight_status_ != DJISDK::FlightStatus::STATUS_ON_GROUND &&
         display_mode_ != DJISDK::DisplayMode::MODE_ENGINE_START &&
         ros::Time::now() - start_time < ros::Duration(5)) {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if(ros::Time::now() - start_time > ros::Duration(5)) {
    ROS_ERROR("Takeoff failed. Motors are not spinnning.");
    return false;
  }
  else {
    start_time = ros::Time::now();
    ROS_INFO("Motor Spinning ...");
    ros::spinOnce();
  }


  // Step 1.2: Get in to the air
  while (flight_status_ != DJISDK::FlightStatus::STATUS_IN_AIR &&
          (display_mode_ != DJISDK::DisplayMode::MODE_ASSISTED_TAKEOFF 
          || display_mode_ != DJISDK::DisplayMode::MODE_AUTO_TAKEOFF) &&
          ros::Time::now() - start_time < ros::Duration(20)) {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if(ros::Time::now() - start_time > ros::Duration(20)) {
    ROS_ERROR("Takeoff failed. Aircraft is still on the ground, but the motors are spinning.");
    return false;
  }
  else {
    start_time = ros::Time::now();
    ROS_INFO("Ascending...");
    ros::spinOnce();
  }

  // Final check: Finished takeoff
  while ( (display_mode_ == DJISDK::DisplayMode::MODE_ASSISTED_TAKEOFF || 
    display_mode_ == DJISDK::DisplayMode::MODE_AUTO_TAKEOFF) &&
          ros::Time::now() - start_time < ros::Duration(20)) {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if ( display_mode_ != DJISDK::DisplayMode::MODE_P_GPS ||
   display_mode_ != DJISDK::DisplayMode::MODE_ATTITUDE)
  {
    ROS_INFO("Successful takeoff!");
    start_time = ros::Time::now();
  }
  else
  {
    ROS_ERROR("Takeoff finished, but the aircraft is in an unexpected mode. Please connect DJI GO.");
    return false;
  }

  return true;
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "dji_adapter");
  DjiAdapter djiAdapter;
  ros::spin();
  return 0;
}
