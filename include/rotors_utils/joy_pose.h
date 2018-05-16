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


#ifndef ROTORS_JOY_INTERFACE_JOY_POSE_H_
#define ROTORS_JOY_INTERFACE_JOY_POSE_H_

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/UInt8.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "rotors_comm/SuccessiveControl.h"
#include "rotors_utils/common.hpp"

struct Axis
{
  Axis()
      : axis(0), factor(1.0), offset(0.0)
  {}

  int axis;
  double factor;
  double offset;
};

struct Button
{
  Button()
    : button(0)
  {}

  int button;
};


struct Axes {
  Axis x;
  Axis y;
  Axis z;
  Axis yaw;
} ;

struct Buttons {
  Button slow;
  Button go;
  Button receive_image;
  Button interrupt;
  Button takeoff;
} ;

// struct Max {
//   double v_xy;
//   double roll;
//   double pitch;
//   double rate_yaw;
// };
enum ControlState{
  // 一开始肯定是手柄控制
  CONTROL_BEGIN,
  // 控制使用新的控制模式来控制
  CONTROL_SWITCHED,
  // 控制结束
  CONTROL_END,
};

typedef struct ControlMode_{
  // 控制模式的序号
  int mode;
  // 控制模式的状态
  ControlState state;
}ControlMode;

enum UavState{
  UAV_STATE_ON_GROUND,
  UAV_STATE_ARMED, 
  UAV_STATE_TAKING_OFF,
  UAV_STATE_FLYING,
};

/**
* 功能: 产生command_pose
*/
class JoyPose {
  typedef sensor_msgs::Joy::_buttons_type ButtonType;

 private:
  UavState state_;
  ros::NodeHandle nh_;
  ros::Publisher pose_pub_;
  ros::Subscriber joy_sub_; 
  ros::Subscriber target_pose_sub_;
  ros::Subscriber dji_rc_sub_;
  ros::Subscriber dji_status_sub_;

  ros::ServiceClient taking_off_client_;
  ros::ServiceClient dji_takeff_client_;
  ros::ServiceClient receive_image_client_;
  ros::ServiceClient follower_pose_client_;
  
  std::string namespace_;
  std::string receive_image_str_;
  std::string follower_pose_str_;
  
  ControlMode mode_;
  double slow_factor_;
  Axes axes_;
  Buttons buttons_;
  bool is_leader_;
  bool is_follower_;
  bool fly_by_joy_;
  int my_id_;
  int leader_id_;
  unsigned char dji_status_;
  // 是否真是的大疆无人机飞控
  bool is_real_;
  // 摇杆姿势计数
  int joy_action_time_;

  // mav_msgs::RollPitchYawrateThrust control_msg_;
  geometry_msgs::PoseStamped pose_;
  geometry_msgs::PoseStamped target_pose_;
  sensor_msgs::Joy current_joy_;
  sensor_msgs::Joy dji_joy_;

  double take_off_height_;
  ros::Timer timer_;

  std::string base_link_frame_;
  std::string world_frame_;
  std::string base_stabilized_frame_;
  std::string target_pose_str_;
  double yaw_;
  // Max max_;

  // double current_yaw_vel_;
  // double v_yaw_step_;

  // bool is_fixed_wing_;
  void TimerCallback(const ros::TimerEvent& e);
  void JoyCallback(const sensor_msgs::JoyConstPtr& msg);
  void TargetPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg);
  // void SetButton(const );
  // void SetAxis();
  bool GetButton(const Button & button);
  bool JoyAction();
  void DjiStatusCallback(const std_msgs::UInt8ConstPtr& msg);
  double GetAxis(const Axis &axis);
  void DjiCallback(const sensor_msgs::JoyConstPtr& joy);
 public:
  JoyPose();
  virtual ~JoyPose();
};

#endif // ROTORS_JOY_INTERFACE_JOY_POSE_H_
