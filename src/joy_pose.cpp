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

JoyPose::JoyPose():yaw_(0) {
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  pose_.pose.position.x = 0;
  pose_.pose.position.y  = 0;
  pose_.pose.position.z = 0;
  pose_.pose.orientation.x = 0;
  pose_.pose.orientation.y  = 0;
  pose_.pose.orientation.z = 0;
  pose_.pose.orientation.w = 1;

  pnh.param<double >("take_off_height", take_off_height_, 2);

  pnh.param<int>("x_axis", axes_.x.axis, 5);
  pnh.param<int>("y_axis", axes_.y.axis, 4);
  pnh.param<int>("z_axis", axes_.z.axis, 2);
  pnh.param<int>("thrust_axis", axes_.thrust.axis, -3);
  pnh.param<int>("yaw_axis", axes_.yaw.axis, 1);

  pnh.param<double>("yaw_velocity_max", axes_.yaw.factor, 90.0);

  pnh.param<int>("slow_button", buttons_.slow.button, 4);
  pnh.param<int>("go_button", buttons_.go.button, 1);
  pnh.param<int>("stop_button", buttons_.stop.button, 2);
  pnh.param<int>("interrupt_button", buttons_.interrupt.button, 3);
  pnh.param<int>("take_off_button", buttons_.takeoff.button, 1);
  
  pnh.param<double>("slow_factor", slow_factor_, 0.2);

  pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped> ("command_pose", 10);
  timer_=nh_.createTimer(ros::Duration(0.1), &JoyPose::TimerCallback,this);
  namespace_ = nh_.getNamespace();

  // TODO factor out
  nh_.param<std::string>("base_link_frame", base_link_frame_, "base_link");
  nh_.param<std::string>("world_frame", world_frame_, "world");
  nh_.param<std::string>("base_stabilized_frame", base_stabilized_frame_, "base_stabilized");

  taking_off_client_ = nh_.serviceClient<std_srvs::Trigger>("taking_off");

  joy_sub_ = nh_.subscribe("joy", 10, &JoyPose::JoyCallback, this);
}

void JoyPose::TimerCallback(const ros::TimerEvent& e){
  ros::Time now=ros::Time::now();
  double dt=0.0;
  if (!pose_.header.stamp.isZero()) {
	  dt = std::max(0.01, std::min(1.0, (now - pose_.header.stamp).toSec()));
  }else{
  }

  if (GetButton(buttons_.takeoff)){
    std_srvs::Trigger srv;
    pose_.pose.position.z= take_off_height_ ;
    if(taking_off_client_.call(srv)){
      ROS_INFO("message: %s",srv.response.message.c_str());
    }else{
      ROS_ERROR("Failed to call service add_two_ints");
    }
//    ROS_INFO("taking off button pressed");
  }else{
    pose_.pose.position.z += GetAxis(axes_.z) * dt;
  }

  pose_.header.stamp = now;
  pose_.header.frame_id = world_frame_;
  pose_.pose.position.x += (cos(yaw_) * GetAxis(axes_.x) - sin(yaw_) * GetAxis(axes_.y)) * dt;
  pose_.pose.position.y += (cos(yaw_) * GetAxis(axes_.y) + sin(yaw_) * GetAxis(axes_.x)) * dt;
  yaw_ += GetAxis(axes_.yaw) * M_PI/180.0 * dt /3;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw_);
  pose_.pose.orientation = tf2::toMsg(q);
  pose_pub_.publish(pose_);
}


void JoyPose::JoyCallback(const sensor_msgs::JoyConstPtr& joy) {
  current_joy_ = *joy;
}

double JoyPose::GetAxis(const Axis &axis){
  if (axis.axis == 0 || std::abs(axis.axis) > current_joy_.axes.size())
  {
//    ROS_ERROR_STREAM("Axis " << axis.axis << " out of range, joy has " << current_joy_.axes.size() << " axes");
    return 0;
  }
  double output = std::abs(axis.axis) / axis.axis * current_joy_.axes[std::abs(axis.axis) - 1] * axis.factor + axis.offset;
// TODO keep or remove deadzone? may not be needed
// if (std::abs(output) < axis.max_ * 0.2)
// {
//   output = 0.0;
// }
  return output;
}

bool JoyPose::GetButton( const Button &button)
{
  if (button.button <= 0 || button.button > current_joy_.buttons.size())
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
