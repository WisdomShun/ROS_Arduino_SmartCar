#include <ros/ros.h>
#include <signal.h>
#include <geometry_msgs/Twist.h>
#include <termios.h>
#include "teleop.h"

#define KEYCODE_R 0x43
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_S 0x73 //停止操作

Teleop::Teleop():
        linear_(0),
        angular_(0),
        l_scale_(2.0),
        a_scale_(2.0){
    nh_.param("scale_angular", a_scale_, a_scale_);//发布角速度参数
    nh_.param("scale_linear", l_scale_, l_scale_);//发布线速度参数
    //指定发布消息到smartCar/cmd_vel这个Topic上
    twist_pub_ = nh_.advertise<geometry_msgs::Twist>("smartcar/cmd_vel", 1);

}

int kfd = 0;
struct termios cooked, raw;

void quit(int sig){//捕获ctrl + c中断信号，重写默认操作
    (void)sig;
    tcsetattr(kfd, TCSANOW, &cooked);
    ros::shutdown();
    exit(0);
}

void Teleop::keyLoop(){
    char c;
    bool dirty=false;


    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);

    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);

    puts("Reading from keyboard");
    puts("---------------------------");
    puts("Use arrow keys to move the smartCar.");


    for(;;)
    {
        // get the next event from the keyboard
        if(read(kfd, &c, 1) < 0)
        {
            perror("read():");
            exit(-1);
        }

        linear_=angular_=0;
        ROS_DEBUG("value: 0x%02X\n", c);

        switch(c)
        {
            case KEYCODE_L:
                ROS_DEBUG("LEFT");
                angular_ = 1.0;
                dirty = true;
                break;
            case KEYCODE_R:
                ROS_DEBUG("RIGHT");
                angular_ = -1.0;
                dirty = true;
                break;
            case KEYCODE_U:
                ROS_DEBUG("UP");
                linear_ = 1.0;
                dirty = true;
                break;
            case KEYCODE_D:
                ROS_DEBUG("DOWN");
                linear_ = -1.0;
                dirty = true;
                break;
            case KEYCODE_S:
                ROS_DEBUG("STOP");
                linear_ = -2.0;
                dirty = true;
                break;
        }


        geometry_msgs::Twist twist;
        twist.angular.z = a_scale_*angular_;
        twist.linear.x = l_scale_*linear_;
        if(dirty == true)
        {
            twist_pub_.publish(twist);
            dirty = false;
        }
    }


    return;
}


int main(int argc, char** argv){
    ros::init(argc, argv, "teleop_smartCar");
    Teleop teleop_smartCar;

    signal(SIGINT,quit);//转发ctrl + c信号

    teleop_smartCar.keyLoop();

    return(0);
}
