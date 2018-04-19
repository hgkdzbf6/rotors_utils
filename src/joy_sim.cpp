#include "rotors_utils/joy_sim.h"

JoySim::JoySim():speed_(0.3),mode_(0)
{
  pub_ = nh_.advertise<sensor_msgs::Joy>("joy", 1);
  empty_joy_.axes={0,0,0,0,0,0,0,0};
  empty_joy_.buttons={0,0,0,0,0,0,0,0,0,0,0};
}
JoySim::~JoySim(){}

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
  (void)sig;
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}

void JoySim::keyLoop()
{
  char c;
  const int dirty_num=3;
  int dirty=dirty_num;
  // get the console in raw mode                                                              
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file                         
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use arrow keys to move the turtle.");


  for(;;)
  {
    // get the next event from the keyboard  
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }

    ROS_DEBUG("value: 0x%02X\n", c);
  
    joy_ = empty_joy_;
    switch(c)
    {
      case 'A':
      case 'a':
        joy_.axes[0]=speed_;
        dirty = dirty_num;
        break;
      case 'D':
      case 'd':
        joy_.axes[0]=-speed_;
        dirty = dirty_num;
        break;
      case 'W':
      case 'w':
        joy_.axes[1]=speed_;
        dirty = dirty_num;
        break;
      case 'S':
      case 's':
        joy_.axes[1]=-speed_;
        dirty = dirty_num;
        break;
      case 'J':
      case 'j':
        joy_.axes[3]=speed_;
        dirty = dirty_num;
        break;
      case 'L':
      case 'l':
        joy_.axes[3]=-speed_;
        dirty = dirty_num;
        break;
      case 'I':
      case 'i':
        joy_.axes[4]=speed_;
        dirty = dirty_num;
        break;
      case 'K':
      case 'k':
        joy_.axes[4]=-speed_;
        dirty = dirty_num;
        break;
      case '1':
        joy_.buttons[0]=1;
        dirty = dirty_num;
        break;
      case '2':
        joy_.buttons[1]=1;
        dirty = dirty_num;
        break;
      case '3':
        joy_.buttons[2]=1;
        dirty = dirty_num;
        break;
      case '4':
        joy_.buttons[3]=1;
        dirty = dirty_num;
        break;
      case ' ':
        joy_.buttons[3]=0;
        dirty = dirty_num;
        break;
    }
    joy_.header.stamp = ros::Time::now();
    if(dirty > 0)
    {
      pub_.publish(joy_);    
      dirty--;
    }
  }


  return;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "joy_sim");
  JoySim joy_sim;
  signal(SIGINT,quit);
  joy_sim.keyLoop();
  return(0);
}


