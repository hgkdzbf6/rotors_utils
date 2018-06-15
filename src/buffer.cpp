// 写这个类的目的是,轨迹生成的前一个时刻和后一个时刻的偏差太大了,
// 这个类的输入是目标点的坐标,然后会储存前一个时刻的坐标
// 暂时不考虑旋转

#include "rotors_utils/buffer.hpp"

Buffer::Buffer(double max_speed, double dt,int mode):
  max_speed_(max_speed), dt_(dt),mode_(mode){
	  ros::NodeHandle pnh("~");
    pre_pose_.pose.orientation.w=1;
    pre_pose_.header.stamp=ros::Time::now();
    pose_.pose.orientation.w=1;
    pose_.header.stamp=ros::Time::now();
    output_.pose.orientation.w=1;
    output_.header.stamp=ros::Time::now();

    // 读一些参数 
    timer_=nh_.createTimer(ros::Duration(dt_),&Buffer::TimerCallback,this);
    command_sub_=nh_.subscribe("command_pose",5,&Buffer::CommandCallback,this);
    filtered_pub_=nh_.advertise< geometry_msgs::PoseStamped >("filtered_pose", 5);
}

Buffer::Buffer(){
  Buffer(0.5,0.1,0);
  
}

Buffer::~Buffer(){
  
}

void Buffer::TimerCallback(const ros::TimerEvent & e){
  double dx;
  double dy;
  double dz;
  double ds2;
  double dd2;
  double scale;
  dx = pose_.pose.position.x - pre_pose_.pose.position.x;
  dy = pose_.pose.position.y - pre_pose_.pose.position.y;
  dz = pose_.pose.position.z - pre_pose_.pose.position.z;
  ds2 = (max_speed_ * dt_) * (max_speed_ * dt_);
  if(mode_==0){
    // x,y模式
    dd2 = dx * dx + dy * dy;
  }else if(mode_==1){
    // x,y,z模式
    dd2 = dx * dx + dy * dy + dz * dz;
  }
  // 这个距离ds2在之内
  if(dd2 < ds2){
    output_.pose.position = pose_.pose.position;
  }else{
    // ds2 < dd2, 目标的移动距离大于限幅值
    scale = sqrt(dd2 / ds2);
    if(mode_ == 0){
      output_.pose.position.x = pre_pose_.pose.position.x + dx / scale; 
      output_.pose.position.y = pre_pose_.pose.position.y + dy / scale; 
    }else if(mode_ == 1){
      output_.pose.position.x = pre_pose_.pose.position.x + dx / scale; 
      output_.pose.position.y = pre_pose_.pose.position.y + dy / scale;
      output_.pose.position.z = pre_pose_.pose.position.z + dz / scale; 
    }
  }
  output_.pose.orientation = pose_.pose.orientation;
  filtered_pub_.publish(output_);
  pre_pose_= output_;
}

void Buffer::CommandCallback(const geometry_msgs::PoseStampedConstPtr & msg){
  pose_ = *msg;
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "buffer");
  Buffer buffer(1.5,0.01,0);
  ros::spin();
  return 0;
}
