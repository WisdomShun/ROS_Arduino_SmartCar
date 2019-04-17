//
// Created by shun on 19-4-13.
//

#ifndef SRC_SMARTCONTROLMETHODS_H
#define SRC_SMARTCARCONTROL_H

#include <Arduino.h>

#define LEFT -1
#define FORWARD 0
#define RIGHT 1

#define LEFT_MOTOR_BACK 9       //左电机后退(IN1)
#define LEFT_MOTOR_FORWARD 5    //左电机前进(IN2)
#define RIGHT_MOTOR_FORWARD 6   //右电机前进(IN3)
#define RIGHT_MOTOR_BACK 10     //右电机后退(IN4)

#define SENSOR_RIGHT_FINDER A2  //右循迹红外传感器(P3.2 OUT1)
#define SENSOR_LEFT_FINDER A3   //左循迹红外传感器(P3.3 OUT2)
#define SENSOR_LEFT A4          //左红外传感器(P3.4 OUT3)
#define SENSOR_RIGHT A5         //右红外传感器(P3.5 OUT4)

#define ECHO A0                 //超声接收
#define TRIG A1                 //超声发送

#define SERVOPIN 2              //舵机

class SmartCar {
private:
    void servoPulse(int myangle);
public:
    void forward();
    void back();
    void stop();
    void spinLeft();
    void spinRight();
    void turnLeft();
    void turnRight();
    void adjustHead();
    float distance();
    float distanceDetection(char direction);
    void init();
};


#endif //SRC_SMARTCONTROLMETHODS_H
