/*
 * Created by shun on 19-4-13.
 * 订阅/smartcar/cmd_vel节点，并接受消息并处理相应的消息
 * 同时定义发布消息节点，实时发布传感器消息到节点/smartcar/sensors中
 * 同时定义service接受消息测量前左右的障碍物距离
 * 小车分为两个模式，手动模式和自动模式，手动模式就是通过s，上下左右定义小车的移动
 * 自动模式就是自己开，不遇到障碍物就不停的那种，遇到障碍物就停下来往没障碍物的那边开
 * 定义两个service，一个是检测前面和左右障碍物距离的API，一个是用来开关主动模式和自动模式的API
*/

#include <ros.h>
#include <avr/pgmspace.h>

#include "SmartCar.h"
#include <geometry_msgs/Twist.h>

//#include <std_msgs/Float32.h>   //发布障碍物距离信息
#include "SmartCarControl/front.h"      //发布左右红外传感器测得的障碍物存在与否
#include "SmartCarControl/distance.h"
#include "SmartCarControl/switchmode.h"

/*
 * Arduino要求定义全局变量段
 */
ros::NodeHandle nh;

SmartCar mycar;
char buf[16];
//std_msgs::Float32 distance;
SmartCarControl::front front;

//ros::Publisher chatter1("smartcar/front_distance",&distance);
ros::Publisher chatter2("smartcar/front_obstacle", &front);

bool isAuto = false;//默认为手动模式，防止一开机就乱跑

void switchAutoService(const SmartCarControl::switchmode::Request &req, SmartCarControl::switchmode::Response &res) {
    memset(buf,0,sizeof(buf));
    if (req.antoon){
        if (isAuto){
            strcpy_P(buf,PSTR("AlreadyAtAuto"));
            res.echo = buf;
        } else if (!isAuto){
            isAuto = true;
            strcpy_P(buf,PSTR("ChangeToAuto"));
            res.echo = buf;
        }
    } else if (!req.antoon) {
        if (!isAuto){
            strcpy_P(buf,PSTR("AlreadyAtManual"));
            res.echo = buf;
        } else if (isAuto){
            isAuto = false;
            strcpy_P(buf,PSTR("ChangeToManual"));
            //停车
            mycar.stop();
            res.echo = buf;
        }
    }
//    if (req.antoon) {
//        isAuto = true;
//    } else {
//        isAuto = false;
//    }

}

void distanceService(const SmartCarControl::distance::Request &req, SmartCarControl::distance::Response &res) {
    res.left = mycar.distanceDetection(LEFT);
    res.right = mycar.distanceDetection(RIGHT);
    res.front = mycar.distanceDetection(FORWARD);
}

void callback(const geometry_msgs::Twist &msg) {
    //ROS_DEBUG("linear : [x = %f] \tangular : [z = %f]", msg.linear.x, msg.angular.z);
    if (abs(msg.angular.z - 2.0) < 0.000001) {
        //ROS_INFO("turning left");
        mycar.turnLeft();
    } else if (abs(msg.angular.z + 2.0) < 0.000001) {
        //ROS_INFO("turning right");
        mycar.turnRight();
    } else if (abs(msg.linear.x - 2.0) < 0.000001) {
        //ROS_INFO("forwarding");

        mycar.forward();
    } else if (abs(msg.linear.x + 2.0) < 0.000001) {
        //ROS_INFO("backwarding");
        mycar.back();
    } else if (abs(msg.linear.x + 4.0) < 0.000001) {
        //ROS_INFO("STOP");
        mycar.stop();
    }
}

void updateStatus() {
    char left = digitalRead(SENSOR_LEFT);
    char right = digitalRead(SENSOR_RIGHT);
    switch (left) {
        case HIGH:
            front.left = false;
            break;
        case LOW:
            front.left = true;
            break;
        default:
            break;
    }
    switch (right) {
        case HIGH:
            front.right = false;
            break;
        case LOW:
            front.right = true;
            break;
        default:
            break;
    }
    //distance.data = mycar.distance();
}

ros::ServiceServer<SmartCarControl::distance::Request, SmartCarControl::distance::Response> servicedistance(
        "getDistance", &distanceService);
ros::ServiceServer<SmartCarControl::switchmode::Request, SmartCarControl::switchmode::Response> serviceswitch(
        "switchToAuto", &switchAutoService);
ros::Subscriber<geometry_msgs::Twist> sub("smartcar/cmd_vel", &callback);


void setup() {
    //初始化调正超声波传感器舵机，防止头看歪
    mycar.init();
    nh.initNode();
    nh.advertiseService(servicedistance);
    nh.advertiseService(serviceswitch);
    //nh.advertise(chatter1);
    nh.advertise(chatter2);
    nh.subscribe(sub);
}

void adjust() {
    float l = mycar.distanceDetection(LEFT);
    float r = mycar.distanceDetection(RIGHT);

#ifndef DAY
    bool ir_l = front.left;
    bool ir_r = front.right;

    if ((l < 20 && r < 20) || (ir_l)&&(ir_r) ) {
        mycar.back();//这里的后退操作没有指定具体的后退到的条件，会导致重复进入死角
    } else {
        if (l > r || ir_r) {
            mycar.spinLeft();
        } else if (r > l || ir_l){
            mycar.spinRight();
        }
    }
#else
    if (l < 20 && r < 20) {
        mycar.back();//这里的后退操作没有指定具体的后退到的条件，会导致重复进入死角
    } else {
        if (l > r) {
            mycar.spinLeft();
        } else {
            mycar.spinRight();
        }
    }
#endif


}

void selfRun() {

#ifndef DAY
    if (front.right && front.left) {
        mycar.stop();
        adjust();
    } else if (front.right) {
        mycar.turnLeft();
    } else if (front.left) {
        mycar.turnRight();
    } else {
        mycar.forward();
    }
#endif
    if (mycar.distanceDetection(FORWARD) < 25) {
        mycar.stop();
        adjust();
    } else {
        mycar.forward();
    }
}

void loop() {
    updateStatus();
    //chatter1.publish(&distance);
    chatter2.publish(&front);
    nh.spinOnce();
    switch (isAuto) {
        case true:
            selfRun();
            break;
        case false:
            break;
    }
    delay(5);
}