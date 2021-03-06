
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

	bool is_clockwise_;
	// 半径的平方,米
	double radius2_;
	// 半径,米
	double radius_;

	// 已经流逝的时间
	double t_;
	// 角度
	double theta_;
	// 初始角度
	double theta0_;
	bool is_initialized_;
	// 周期
	double T_;
	Circle():is_initialized_(false){}
	explicit Circle(geometry_msgs::PoseStamped pose, double T=100, double px=0.0,double py=0.0)
		:t_(0),theta_(0),is_initialized_(true),T_(T){
		radius2_=pose.pose.position.x*pose.pose.position.x+pose.pose.position.y*pose.pose.position.y;
		radius_=sqrt(radius2_);
		theta0_=atan2(pose.pose.position.y,pose.pose.position.x);
		pose_base_=pose;
		pose_base_.pose.position.x=px;
		pose_base_.pose.position.y=py;
		pose_=pose;
	}

	explicit Circle(double radius, double T=100, double px=0.0, double py=0.0)
		:t_(0),theta_(0),is_initialized_(true),T_(T),radius_(radius),
		radius2_(radius*radius),theta0_(0){
		pose_.header.stamp=ros::Time::now();
		pose_base_=pose_;
		pose_base_.pose.position.x=px;
		pose_base_.pose.position.y=py;
	}

	geometry_msgs::PoseStamped update(double dt){
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
		pose_.pose.position.x=cos(theta_+theta0_)*radius_+pose_base_.pose.position.x;
		pose_.pose.position.y=sin(theta_+theta0_)*radius_+pose_base_.pose.position.y;
		// pose_.pose.position.x=cos(theta_)*radius_;
		// pose_.pose.position.y=sin(theta_)*radius_;
		pose_.pose.position.z=pose_base_.pose.position.z;
		// 在需要发布的时候更新时间
		pose_.header.stamp=ros::Time::now();
		pose_.pose.orientation.x=0;
		pose_.pose.orientation.y=0;
		pose_.pose.orientation.z=0;
		pose_.pose.orientation.w=1;
		return pose_;
	} 
	
	void reset_center(double px, double py){
		pose_base_.pose.position.x=px;
		pose_base_.pose.position.y=py;
	}
};

// 保持原来位置的结构体
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
	int my_id_;
	int other_id_;
	ros::NodeHandle nh_;
	ros::Publisher command_pub_;
	ros::Publisher trajectory_base_pub_;

	ros::Subscriber relative_sub_;
	bool receive_first_msg_;
	bool follower_pose_start_;

	bool new_data_approach_;
	bool is_leader_;
	ros::ServiceServer server_;

	Circle circle_;
	geometry_msgs::PoseStamped pose_;
	geometry_msgs::PoseStamped pose_base_;
	std::string relative_str_;
	std::string follower_pose_str_;
	std::string leader_desired_pose_str_;
	std::string trajectory_base_str_;
	ros::Timer timer_;

	void TimerCallback(const ros::TimerEvent& e);
	void relativeCallback(const geometry_msgs::PoseStampedConstPtr& msg);

	bool callback(rotors_comm::SuccessiveControl::Request& request,
		rotors_comm::SuccessiveControl::Response& response);
};

#endif // FOLLOWER_POSE_H_