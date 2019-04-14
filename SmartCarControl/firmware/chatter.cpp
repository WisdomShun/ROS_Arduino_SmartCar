/*
 * Created by shun on 19-4-13.
 * 订阅/smartcar/cmd_vel节点，并接受消息并处理相应的消息
 * 同时定义发布消息节点，实时发布传感器消息到节点/smartcar/sensors中
 * 同时定义service和action用以支持smartcar的高级操作
*/

#include <ros.h>
#include "SmartCar.h"
#include <geometry_msgs/Twist.h>

#include <std_msgs/Float32.h>   //发布障碍物距离信息
#include "SmartCarControl/front.h"      //发布左右红外传感器测得的障碍物存在与否

/*
 * Arduino要求定义全局变量段
 */
ros::NodeHandle nh;
SmartCar mycar;

std_msgs::Float32 distance;
SmartCarControl::front front;

//int SensorLeftStatus;           //左红外传感器状态
//int SensorRightStatus;          //右红外传感器状态

ros::Publisher chatter1("smartcar/distance_to_obstacle",&distance);
ros::Publisher chatter2("smartcar/front_obstacle",&front);

void callback(const geometry_msgs::Twist &msg){
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
    }else {
        //ROS_ERROR("ERROR Twist Information!");
    }
}

void updateStatus(){
    switch (digitalRead(SENSOR_LEFT)){
        case HIGH:
            front.left = true;
            break;
        case LOW:
            front.left = false;
            break;
        default:
            break;
    }
    switch (digitalRead(SENSOR_RIGHT)){
        case HIGH:
            front.right = true;
            break;
        case LOW:
            front.right = false;
            break;
        default:
            break;
    }
    distance.data = mycar.distance();
}

//https://www.ncnynl.com/archives/201610/921.html tf坐标系

ros::Subscriber<geometry_msgs::Twist> sub("smartcar/cmd_vel", &callback);

void setup() {
    mycar.init();
    nh.initNode();
    nh.advertise(chatter1);
    nh.advertise(chatter2);
    nh.subscribe(sub);
}

void loop() {
    updateStatus();
    chatter1.publish(&distance);
    chatter2.publish(&front);
    nh.spinOnce();
    delay(1);
}