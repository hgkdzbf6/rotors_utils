/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#include "rotors_utils/joy_pose.h"

#include <mav_msgs/default_topics.h>
#include <geometry_msgs/PoseStamped.h>
JoyPose::~JoyPose(){
  
}

JoyPose::JoyPose():state_(UAV_STATE_ON_GROUND),joy_action_time_(0),fly_by_joy_(true),yaw_(0){
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  pose_.pose.position.x = 0;
  pose_.pose.position.y  = 0;
  pose_.pose.position.z = 0;
  pose_.pose.orientation.x = 0;
  pose_.pose.orientation.y  = 0;
  pose_.pose.orientation.z = 0;
  pose_.pose.orientation.w = 1;
  pnh.param<bool >("is_real",is_real_,false);

  pnh.param<int >("my_id",my_id_,0);
  pnh.param<int >("leader_id",leader_id_,0);
  pnh.param<bool >("is_follower",is_follower_,false);
  pnh.param<double >("take_off_height", take_off_height_, 2);

  pnh.param<int>("x_axis", axes_.x.axis, 5);
  pnh.param<int>("y_axis", axes_.y.axis, 4);
  pnh.param<int>("z_axis", axes_.z.axis, 2);
  pnh.param<int>("yaw_axis", axes_.yaw.axis, 1);

  pnh.param<double>("yaw_velocity_max", axes_.yaw.factor, 90.0);

  pnh.param<int>("slow_button", buttons_.slow.button, 4);
  // 从机开始按照相对位置飞行
  pnh.param<int>("go_button", buttons_.go.button, 3);
  // 开始接收图像,然后获取到了相对位置
  pnh.param<int>("receive_image_button", buttons_.receive_image.button, 2);
  pnh.param<int>("interrupt_button", buttons_.interrupt.button, 3);
  // 起飞
  pnh.param<int>("take_off_button", buttons_.takeoff.button, 1);
  
  pnh.param<double>("slow_factor", slow_factor_, 0.2);

  pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped> ("command_pose", 10);
  timer_=nh_.createTimer(ros::Duration(0.1), &JoyPose::TimerCallback,this);
  namespace_ = nh_.getNamespace();

  // TODO factor out
  nh_.param<std::string>("base_link_frame", base_link_frame_, "base_link");
  nh_.param<std::string>("world_frame", world_frame_, "world");
  nh_.param<std::string>("base_stabilized_frame", base_stabilized_frame_, "base_stabilized");

  pnh.param<int >("mode", mode_.mode, 0);
  
  taking_off_client_ = nh_.serviceClient<std_srvs::Trigger>("taking_off");
  dji_takeff_client_ = nh_.serviceClient<std_srvs::Trigger>("dji_takeoff");
  
  pnh.param<std::string>("receive_image", receive_image_str_, "receive_image01");
  receive_image_client_ = nh_.serviceClient<std_srvs::Trigger>(receive_image_str_);

  pnh.param<std::string>("follower_pose", follower_pose_str_, "follower_pose");
  follower_pose_client_=nh_.serviceClient<rotors_comm::SuccessiveControl>(follower_pose_str_);

  pnh.param<std::string>("target_pose",target_pose_str_,"/hummingbird0/target_pose");
  target_pose_sub_ = nh_.subscribe(target_pose_str_,10,&JoyPose::TargetPoseCallback,this);
  
  joy_sub_ = nh_.subscribe("joy", 10, &JoyPose::JoyCallback, this);

  if(is_real_){
    dji_rc_sub_ = nh_.subscribe("dji_sdk/rc",10, &JoyPose::DjiCallback,this);
    dji_status_sub_ = nh_.subscribe("dji_sdk/flight_status",10,&JoyPose::DjiStatusCallback,this);
  }else{
    dji_status_ =1;
  }
  // 为啥能用这种方法判断呢?
  // 因为根节点的id和leader id才一样
  // 其他的,就算是leader节点,但是都是隶属于他们之上的leader节点的
  // 其实还是和协议设计有关系
  is_leader_= (my_id_ == leader_id_)? true: false;
}

void JoyPose::TargetPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg){
  target_pose_=*msg;
}

bool JoyPose::JoyAction(){
  double x,y,z,yaw;
  x = GetAxis(axes_.x);
  y = GetAxis(axes_.y);
  z = GetAxis(axes_.z);
  yaw = GetAxis(axes_.yaw);

  if(!is_real_){
    if ( x<-0.5 && y>0.5 &&
      yaw<-0.5 && z<=0.5){
        joy_action_time_++;
    }else{
      joy_action_time_ = 0;
    }
  }else{    
    if ( x==-1 && y==-1 &&
      yaw==90 && z==-1){
        joy_action_time_++;
    }else{
      // ROS_INFO_STREAM(x<<" "<<y<<" "<<z<<" "<<yaw<<" ");
      joy_action_time_ = 0;
    }
  }
  if (joy_action_time_ > 5){
    ROS_INFO("Joy action!");
    return true;
  }
  return false;
}

void JoyPose::TimerCallback(const ros::TimerEvent& e){
  static bool fly_by_joy_switch=false;
  ros::Time now=ros::Time::now();
  double dt=0.0;
  if (!pose_.header.stamp.isZero()) {
	  dt = std::max(0.01, std::min(1.0, (now - pose_.header.stamp).toSec()));
  }else{
  }

  // 解锁
  if (state_ == UAV_STATE_ON_GROUND && dji_status_ == 1){
    std_srvs::Trigger dji_takeoff_srv;
    state_ = UAV_STATE_ARMED;
    joy_action_time_=0;
    if(is_real_){    
      if(dji_takeff_client_.call(dji_takeoff_srv)){
        ROS_INFO("dji n3 ready to take off ");
      }else{
        ROS_ERROR("Failed to call dji n3 to take off ");
      }
    }
  }

  // 起飞
  if (state_==UAV_STATE_ARMED && (GetButton(buttons_.takeoff) || GetAxis(axes_.z)==1 )){
    state_=UAV_STATE_TAKING_OFF;
    std_srvs::Trigger srv;
    pose_.pose.position.z= take_off_height_ ;
    if(taking_off_client_.call(srv)){
      ROS_INFO("takeoff message assumed index %d: %s",my_id_,srv.response.message.c_str());
    }else{
      ROS_ERROR("Failed to call service %d takeoff",my_id_);
    }    
  }else{
    pose_.pose.position.z += GetAxis(axes_.z) * dt;
  }

  // 接收图像
  // 这里数量要对等
  // if(is_follower_){
  if (GetButton(buttons_.receive_image)){
    std_srvs::Trigger receive_image_srv;
    if(receive_image_client_.call(receive_image_srv)){
      ROS_INFO("receive image message assumed index %d: %s",my_id_,receive_image_srv.response.message.c_str());
    }else{
      ROS_ERROR("Failed to call service %d receive_image, %s",my_id_, receive_image_str_.c_str());
    }
  }
  // }
  // 从机飞行套路
  // if(is_follower_){
    // 按下左边的按键
  if (GetButton(buttons_.go)){
    ROS_INFO("wtf");
    mode_.state= CONTROL_SWITCHED;
    rotors_comm::SuccessiveControl follower_pose_srv;
    follower_pose_srv.request.pose=pose_;
    if(follower_pose_client_.call(follower_pose_srv)){
      ROS_INFO("follower message %d : %s",my_id_,follower_pose_srv.response.message.c_str());
      if(fly_by_joy_switch)fly_by_joy_=false;
      fly_by_joy_switch=true;
    }else{
      ROS_ERROR("Failed to call service %d go",my_id_);
    }    
  }
  // }
  pose_.header.stamp = now;
  pose_.header.frame_id = world_frame_;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw_);
  pose_.pose.orientation = tf2::toMsg(q);
  // 首先都被手柄控制
  if(mode_.state == CONTROL_SWITCHED){
    pose_.pose.position.x = target_pose_.pose.position.x;
    pose_.pose.position.y = target_pose_.pose.position.y;
    pose_.pose.position.z = take_off_height_; 
  } else if(fly_by_joy_) {
    pose_.pose.position.x += (cos(yaw_) * GetAxis(axes_.x) - sin(yaw_) * GetAxis(axes_.y)) * dt / 5;
    pose_.pose.position.y += (cos(yaw_) * GetAxis(axes_.y) + sin(yaw_) * GetAxis(axes_.x)) * dt / 5;
    yaw_ += GetAxis(axes_.yaw) * M_PI/180.0 * dt /3;
  } else if(is_follower_) {
    // 这边注释掉，follower就停掉了。
    pose_.pose.position.x = target_pose_.pose.position.x;
    pose_.pose.position.y = target_pose_.pose.position.y;
    pose_.pose.position.z = take_off_height_; 
    // ROS_INFO_STREAM(std::endl<<"  target x:"<<pose_.pose.position.x
    // <<"  target y:"<< pose_.pose.position.y<<std::endl);
  }
  // 否则就不更新了
  // 从pose中获得x^L2_d,f
  pose_pub_.publish(pose_);
}

void JoyPose::JoyCallback(const sensor_msgs::JoyConstPtr& joy) {
  current_joy_ = *joy;
}

void JoyPose::DjiCallback(const sensor_msgs::JoyConstPtr& joy) {
  dji_joy_ = *joy;
}
void JoyPose::DjiStatusCallback(const std_msgs::UInt8ConstPtr& msg) {
  dji_status_ = msg->data;
}

double JoyPose::GetAxis(const Axis &axis){
  sensor_msgs::Joy joy;
  if(!is_real_){
    joy = current_joy_;
  } else{
    joy = dji_joy_;
  }
  if (axis.axis == 0 || ( static_cast<uint>(std::abs(axis.axis)) > joy.axes.size()) )
  {
//    ROS_ERROR_STREAM("Axis " << axis.axis << " out of range, joy has " << current_joy_.axes.size() << " axes");
    return 0;
  }
  double output = std::abs(axis.axis) / axis.axis * joy.axes[std::abs(axis.axis) - 1] * axis.factor + axis.offset;
// TODO keep or remove deadzone? may not be needed
// if (std::abs(output) < axis.max_ * 0.2)
// {
//   output = 0.0;
// }
  return output;
}

bool JoyPose::GetButton( const Button &button)
{
  if (button.button <= 0 || static_cast<uint>(button.button) > current_joy_.buttons.size())
  {
    return false;
  }
  bool a=current_joy_.buttons[button.button - 1] > 0;
  // if(a==true)ROS_INFO("button %d pressed", button.button);
  return a;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "rotors_joy_pose_interface");
  JoyPose joy_pose;
  ros::spin();
  return 0;
}

