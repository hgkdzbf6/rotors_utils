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
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

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
  Axis thrust;
  Axis yaw;
} ;

struct Buttons {
  Button slow;
  Button go;
  Button stop;
  Button interrupt;
  Button takeoff;
} ;

// struct Max {
//   double v_xy;
//   double roll;
//   double pitch;
//   double rate_yaw;
//   double thrust;
// };

class JoyPose {
  typedef sensor_msgs::Joy::_buttons_type ButtonType;

 private:
  ros::NodeHandle nh_;
  ros::Publisher pose_pub_;
  ros::Subscriber joy_sub_;

  ros::ServiceClient taking_off_client_;
  
  std::string namespace_;
  double slow_factor_;
  Axes axes_;
  Buttons buttons_;

  // mav_msgs::RollPitchYawrateThrust control_msg_;
  geometry_msgs::PoseStamped pose_;
  sensor_msgs::Joy current_joy_;

  double take_off_height_;
  ros::Timer timer_;

  std::string base_link_frame_;
  std::string world_frame_;
  std::string base_stabilized_frame_;
  double yaw_;
  // Max max_;

  // double current_yaw_vel_;
  // double v_yaw_step_;

  // bool is_fixed_wing_;
  void TimerCallback(const ros::TimerEvent& e);
  void JoyCallback(const sensor_msgs::JoyConstPtr& msg);
  // void SetButton(const );
  // void SetAxis();
  bool GetButton(const Button & button);
  double GetAxis(const Axis &axis);
 public:
  JoyPose();
};

#endif // ROTORS_JOY_INTERFACE_JOY_POSE_H_
