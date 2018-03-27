#include "rotors_utils/follower_pose.hpp"

FollowerPose::FollowerPose():receive_first_msg_(false),follower_pose_start_(false)
{
  ros::NodeHandle pnh("~");
  pnh.param<std::string>("relative_pose",relative_str_,"relative_pose01");
  pnh.param<bool>("is_leader",is_leader_,true);

  server_= nh_.advertiseService("follower_pose", &FollowerPose::callback,this);  
 
  relative_sub_=nh_.subscribe(relative_str_,5,&FollowerPose::relativeCallback,this);
  timer_=nh_.createTimer(ros::Duration(0.1),&FollowerPose::TimerCallback,this);
  command_pub_=nh_.advertise<geometry_msgs::PoseStamped>("leader_desired_pose", 10);
}

FollowerPose::~FollowerPose()
{

}

void FollowerPose::TimerCallback(const ros::TimerEvent& e){
  if(is_leader_)return ;
  if(follower_pose_start_){
    if(receive_first_msg_){
      float dt = e.current_real.toSec() - e.last_real.toSec();
      if(dt<0.1) dt=0.1;
      if(dt>0.2) dt=0.2;
      if(circle_.is_initialized_){
        if(new_data_approach_){
          // 这里产生x^L2_d,F
          circle_.update(dt);
          // new_data_approach_=false;
        }
        // 在需要发布的时候更新时间
        command_pub_.publish(circle_.pose_);
      }
    }
  }
}

void FollowerPose::relativeCallback(const geometry_msgs::PoseStampedConstPtr& msg){
  if(is_leader_)return ;
  if(follower_pose_start_){
    // 接收到服务, 手柄控制停掉.
    // 其实不用等于Pose_base的,因为是两套轨迹系统
    // 但是时间戳什么的还是有用的,还是保留吧
    pose_.header.frame_id=pose_base_.header.frame_id;
    pose_.pose.position.x=msg->pose.position.x;
    pose_.pose.position.y=msg->pose.position.y;
    pose_.pose.position.z=msg->pose.position.z;
    new_data_approach_=true;
    // ROS_INFO_STREAM("x: "<<pose_.pose.position.x <<"   y: "<< pose_.pose.position.y <<"   z: "<< pose_.pose.position.z);
    if(!receive_first_msg_){
      ROS_INFO_ONCE("relative position callback received");
      receive_first_msg_=true;
      circle_=Circle(pose_);
      // ROS_INFO_STREAM("x: "<<pose_.pose.position.x <<"   y: "<< pose_.pose.position.y<< "circle.radius: "<<circle_.radius_);
    }
  }
}

bool FollowerPose::callback(rotors_comm::SuccessiveControl::Request& request, 
  rotors_comm::SuccessiveControl::Response& response)
{  
  // 接收到服务之后, follower_pose_start这个true了
  this->follower_pose_start_=true;
  pose_base_=request.pose;
  response.message="get follower pose start";
	response.success=true;
  return true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "follower_pose");
  FollowerPose follower_pose;
  ros::spin();
  return 0;
}
