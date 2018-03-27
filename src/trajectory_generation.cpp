#include "rotors_utils/trajectory_generation.hpp"

// 这个是中心的计算节点
TrajectoryGeneration::TrajectoryGeneration(){
  ros::NodeHandle pnh("~");
  // follower_sub_=nh_.subscribe("target_pose",5,&TrajectoryGeneration::RelativePositionCallback,this);
  //需要知道leader id和follower id
  pnh.param<int>("my_id",my_id_,0);
  pnh.param<int>("follower_id",follower_id_,1);
  // 这边的参数是瞎写的，以launch为准
  pnh.param<std::string>("T_L2_F2",T_L2_F2_str_,"relative_pose01");
  pnh.param<std::string>("T_LS_L2",T_LS_L2_str_,"svo/fusion_pose");
  pnh.param<std::string>("T_FS_F2",T_FS_F2_str_,"/hummingbird1/svo/fusion_pose");
  pnh.param<std::string>("T_L2_dF2",T_L2_dF2_str_,"/hummingbird1/leader_desired_pose");
  pnh.param<std::string>("T_FW_dF2",T_FW_dF2_str_,"target_pose");
  // 两个SVO坐标系下面的相对位姿, 这个是节点计算出来的
  // 第一位T是SE3，S是Sim3，
  // 第二位是到哪个坐标系
  // 第三位是从哪个坐标系
  // 符号最好统一下吧
  T_L2_F2_sub_=nh_.subscribe(T_L2_F2_str_,5,&TrajectoryGeneration::T_L2_F2Callback,this);
  // leader在leader SVO坐标系下面的坐标
  T_LS_L2_sub_=nh_.subscribe(T_LS_L2_str_,5,&TrajectoryGeneration::T_LS_L2Callback,this);
  // follower在follower SVO坐标系下面的坐标
  T_FS_F2_sub_=nh_.subscribe(T_FS_F2_str_,5,&TrajectoryGeneration::T_FS_F2Callback,this);
  // 在leader坐标系下，follower的期望位置
  // TODO: 这块逻辑应该改改，应该是leader给的位置。日后再改吧。
  T_L2_dF2_sub_=nh_.subscribe(T_L2_dF2_str_,5,&TrajectoryGeneration::T_L2_dF2Callback,this);
  // 最后输出的位置。应该给控制器的。
  follower_pub_=nh_.advertise<geometry_msgs::PoseStamped>(T_FW_dF2_str_,5);
  timer_=nh_.createTimer(ros::Duration(0.1),&TrajectoryGeneration::TimerCallback,this);


  Eigen::Matrix3d R=Eigen::AngleAxisd(M_PI,Eigen::Vector3d(1,0,0)).toRotationMatrix();
  Eigen::Vector3d t(0,0,0);
  // ROS_INFO_STREAM("R:"<<std::endl<<R<<std::endl);
  T_LS_LW_=Sophus::SE3(R,t);
  T_FS_FW_=Sophus::SE3(R,t);
  SE32Sim3(T_LS_LW_,1,S_LS_LW_);
  SE32Sim3(T_FS_FW_,1,S_FS_FW_);

}

void TrajectoryGeneration::T_L2_F2Callback(const geometry_msgs::PoseStampedConstPtr& msg){
  // 只执行一次,为了获取初始值
  static bool T_L_switch=false;
  if(!T_L_switch){
    geoPoseStamped2SE3(*msg,T_LW_FW_);
    SE32Sim3(T_LW_FW_,1,S_LW_FW_);
  }
  T_L_switch=true;
  // 其他时候获取L2_F2
  geoPoseStamped2SE3(*msg,T_L2_F2_);
  SE32Sim3(T_L2_F2_,1,S_L2_F2_);
}

void TrajectoryGeneration::T_LS_L2Callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg){
  geoPose2SE3(msg->pose.pose,T_LS_L2_);
  // T_LS_L2_ *= x_rot_se3 ;
  SE32Sim3(T_LS_L2_,1,S_LS_L2_);
}

void TrajectoryGeneration::T_FS_F2Callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg){
  geoPose2SE3(msg->pose.pose,T_FS_F2_);
  // T_LS_L2_ *= x_rot_se3 ;
  SE32Sim3(T_FS_F2_,1,S_FS_F2_);
}

void TrajectoryGeneration::T_L2_dF2Callback(const geometry_msgs::PoseStampedConstPtr& msg){
  geoPose2SE3(msg->pose,T_L2_dF2_);
  SE32Sim3(T_L2_dF2_,1,S_L2_dF2_);
  // ROS_INFO_STREAM(S_L2_dF_);
}

void TrajectoryGeneration::TimerCallback(const ros::TimerEvent & e){
  // 核心算法就这么一行。需不需要重构呢？变成Sim3？
  // 这边广播出来，没啥问题。
  // 应该是上面的定时器的问题。
  // S_F_dF_ = S_L_F_ * S_L2_L_ * S_L2_dF_.inverse();  
  // S_LW_FW_ = S_LS_LW_.inverse() * S_LS_L2_ * S_L2_F2_ * S_FS_F2_.inverse() * S_FS_FW_;
  S_FW_dF2_ = S_LW_FW_.inverse() * S_LS_LW_.inverse() * S_LS_L2_ * S_L2_dF2_;
  // S_L_F_ = T_L_F 
  // S_F_dF_ = S_L_F_.inverse() * S_L2_dF_;
  T_FW_dF2_=S_FW_dF2_.to_SE3();
  SE32geoPose(T_FW_dF2_,pose_.pose);
  pose_.header.stamp=ros::Time::now();
  // ROS_INFO_STREAM("S_L2_F2_:"<<std::endl<<S_L2_F2_<<std::endl);
  // ROS_INFO_STREAM("S_LS_L2_:"<<std::endl<<S_LS_L2_<<std::endl);
  // ROS_INFO_STREAM("S_FS_F2_:"<<std::endl<<S_FS_F2_<<std::endl);
  // ROS_INFO_STREAM("S_L2_dF2_:"<<std::endl<<S_L2_dF2_<<std::endl);

  // ROS_INFO_STREAM("S_LW_FW_:"<<std::endl<<S_LW_FW_<<std::endl);
  // ROS_INFO_STREAM("S_LS_LW_.inverse():"<<std::endl<<S_LS_LW_.inverse()<<std::endl);
  // ROS_INFO_STREAM("S_LS_L2_:"<<std::endl<<S_LS_L2_<<std::endl);
  // ROS_INFO_STREAM("S_L2_dF2_:"<<std::endl<<S_L2_dF2_<<std::endl);
  // ROS_INFO_STREAM("S_FW_dF2_:"<<std::endl<<S_FW_dF2_<<std::endl);
  follower_pub_.publish(pose_);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "rotors_joy_interface");
  TrajectoryGeneration trajectory_generation;
  ros::spin();
  return 0;
}
