#include <ros/ros.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <sensor_msgs/Joy.h>
 
#define KEYCODE_R 0x43 
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71
// 功能: 模拟joy输出
// 需要订阅键盘输入
// 发布joy信息
class JoySim
{
public:
	JoySim();
	virtual ~JoySim();
  double speed_;
  int mode_;
  void stopRobot();
  void keyLoop();
	ros::NodeHandle nh_;
	ros::Publisher pub_;
	sensor_msgs::Joy joy_;
	sensor_msgs::Joy empty_joy_;
	struct termios cooked_, raw_;
};