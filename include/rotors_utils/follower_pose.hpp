
#ifndef FOLLOWER_POSE_H_
#define FOLLOWER_POSE_H_

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include "rotors_comm/SuccessiveControl.h"
#include <sensor_msgs/Joy.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#ifndef MY_PI
#	define MY_PI 3.14159265358979323846
#endif

// 绕中心点旋转的结构体
struct Circle{
	// 开始的位置
	geometry_msgs::PoseStamped pose_base_;
	geometry_msgs::PoseStamped pose_;
	bool is_initialized_;

	bool is_clockwise_;
	// 半径的平方,米
	float radius2_;
	// 半径,米
	float radius_;
	// 初始角度
	float theta0_;
	// 角度
	float theta_;

	// 周期
	float T_;
	// 已经流逝的时间
	float t_;
	Circle():is_initialized_(false){}
	Circle(geometry_msgs::PoseStamped pose, float T=100):T_(T),t_(0),theta_(0),is_initialized_(true){
		radius2_=pose.pose.position.x*pose.pose.position.x+pose.pose.position.y*pose.pose.position.y;
		radius_=sqrt(radius2_);
		theta0_=atan2(pose.pose.position.y,pose.pose.position.x);
		pose_base_=pose;
		pose_=pose;
	}

	geometry_msgs::PoseStamped update(float dt){
		if(dt<0.1)dt=0.1;
		if(dt>0.2)dt=0.2;
		this->t_+=dt;
		// 断言
		assert(t_>=0);
		while(t_>T_){
			t_-=T_;
		}
		assert(t_>=0);
		theta_=t_/T_*2*MY_PI;
		pose_.pose.position.x=cos(theta_+theta0_)*radius_;
		pose_.pose.position.y=sin(theta_+theta0_)*radius_;
		pose_.pose.position.z=pose_base_.pose.position.z;
		// 在需要发布的时候更新时间
		pose_.header.stamp=ros::Time::now();
		return pose_;
	}
};

struct KeepStruct{
	// 开始的位置
	geometry_msgs::PoseStamped base_pose;
};

class FollowerPose
{
public:
	FollowerPose();
	virtual ~FollowerPose();

private:
	ros::NodeHandle nh_;
	ros::Publisher command_pub_;

	ros::Subscriber relative_sub_;
	bool is_leader_;
	bool receive_first_msg_;
	bool follower_pose_start_;

	bool new_data_approach_;
	ros::ServiceServer server_;

	Circle circle_;
	geometry_msgs::PoseStamped pose_;
	geometry_msgs::PoseStamped pose_base_;
	std::string relative_str_;
	ros::Timer timer_;

	void TimerCallback(const ros::TimerEvent& e);
	void relativeCallback(const geometry_msgs::PoseStampedConstPtr& msg);

	bool callback(rotors_comm::SuccessiveControl::Request& request,
		rotors_comm::SuccessiveControl::Response& response);
};

#endif // FOLLOWER_POSE_H_