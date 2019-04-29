//
// Created by shun on 19-4-13.
//

#include "SmartCar.h"

void SmartCar::forward() {
    digitalWrite(RIGHT_MOTOR_FORWARD, HIGH);
    digitalWrite(RIGHT_MOTOR_BACK, LOW);
    digitalWrite(LEFT_MOTOR_FORWARD, HIGH);
    digitalWrite(LEFT_MOTOR_BACK, LOW);//设置驱动方式为前进
    analogWrite(LEFT_MOTOR_FORWARD, 209);//设置左轮的速率
    analogWrite(RIGHT_MOTOR_FORWARD, 206);//设置右轮的速率
    analogWrite(LEFT_MOTOR_BACK, 0);
    analogWrite(RIGHT_MOTOR_BACK, 0);
}
void SmartCar::back() {
    digitalWrite(RIGHT_MOTOR_FORWARD, LOW);
    digitalWrite(RIGHT_MOTOR_BACK, HIGH);
    digitalWrite(LEFT_MOTOR_FORWARD, LOW);
    digitalWrite(LEFT_MOTOR_BACK, HIGH);//设置驱动方式为前进
    analogWrite(LEFT_MOTOR_FORWARD, 0);//设置左轮的速率
    analogWrite(RIGHT_MOTOR_FORWARD, 0);//设置右轮的速率
    analogWrite(LEFT_MOTOR_BACK, 202);
    analogWrite(RIGHT_MOTOR_BACK, 200);
    delay(100);
}

void SmartCar::stop() {
    digitalWrite(RIGHT_MOTOR_FORWARD, LOW);
    digitalWrite(RIGHT_MOTOR_BACK, LOW);
    digitalWrite(LEFT_MOTOR_FORWARD, LOW);
    digitalWrite(LEFT_MOTOR_BACK, LOW);
}

void SmartCar::spinLeft() {
    digitalWrite(RIGHT_MOTOR_FORWARD, HIGH);    // 右电机前进
    digitalWrite(RIGHT_MOTOR_BACK, LOW);
    analogWrite(RIGHT_MOTOR_FORWARD, 200);
    analogWrite(RIGHT_MOTOR_BACK, 0);//PWM比例0~255调速
    digitalWrite(LEFT_MOTOR_FORWARD, LOW);   //左轮不动
    digitalWrite(LEFT_MOTOR_BACK, HIGH);
    analogWrite(LEFT_MOTOR_FORWARD, 0);
    analogWrite(LEFT_MOTOR_BACK, 200);
    delay(80);
}

void SmartCar::spinRight() {
    digitalWrite(RIGHT_MOTOR_FORWARD, LOW);    // 右电机前进
    digitalWrite(RIGHT_MOTOR_BACK, HIGH);
    analogWrite(RIGHT_MOTOR_FORWARD, 0);
    analogWrite(RIGHT_MOTOR_BACK, 200);//PWM比例0~255调速
    digitalWrite(LEFT_MOTOR_FORWARD, HIGH);   //左轮不动
    digitalWrite(LEFT_MOTOR_BACK, LOW);
    analogWrite(LEFT_MOTOR_FORWARD, 202);
    analogWrite(LEFT_MOTOR_BACK, 0);
    delay(80);
}

void SmartCar::turnLeft() {
    digitalWrite(RIGHT_MOTOR_FORWARD, HIGH);    // 右电机前进
    digitalWrite(RIGHT_MOTOR_BACK, LOW);
    analogWrite(RIGHT_MOTOR_FORWARD, 180);
    analogWrite(RIGHT_MOTOR_BACK, 0);//PWM比例0~255调速
    digitalWrite(LEFT_MOTOR_FORWARD, LOW);   //左轮不动
    digitalWrite(LEFT_MOTOR_BACK, LOW);
    analogWrite(LEFT_MOTOR_FORWARD, 0);
    analogWrite(LEFT_MOTOR_BACK, 0);
    delay(400);
}

void SmartCar::turnRight() {
    digitalWrite(RIGHT_MOTOR_FORWARD, LOW);   //右电机不动
    digitalWrite(RIGHT_MOTOR_BACK, LOW);
    analogWrite(RIGHT_MOTOR_FORWARD, 0);
    analogWrite(RIGHT_MOTOR_BACK, 0);//PWM比例0~255调速
    digitalWrite(LEFT_MOTOR_FORWARD, HIGH);//左电机前进
    digitalWrite(LEFT_MOTOR_BACK, LOW);
    analogWrite(LEFT_MOTOR_FORWARD, 180);
    analogWrite(LEFT_MOTOR_BACK, 0);
    delay(400);
}

float SmartCar::distance() {   // 量出前方距离,注意SR04 VCC电源工作电压为5V
    digitalWrite(TRIG, LOW);   // 给触发脚低电平2μs
    delayMicroseconds(2);
    digitalWrite(TRIG, HIGH);  // 给触发脚高电平10μs，这里至少是10μs
    delayMicroseconds(10);
    digitalWrite(TRIG, LOW);    // 持续给触发脚低电
    float Fdistance = pulseIn(ECHO, HIGH);  // 读取高电平时间(单位：微秒)
    Fdistance = Fdistance / 58;       //为什么除以58等于厘米，  Y米=（X秒*344）/2
    // X秒=（ 2*Y米）/344 ==》X秒=0.0058*Y米 ==》厘米=微秒/58
    //Serial.print("Distance:");      //输出距离（单位：厘米）
    //Serial.println(Fdistance);         //显示距离
    //Distance = Fdistance;
    return Fdistance;
}

void SmartCar::servoPulse(int myangle) {/*定义一个脉冲函数，用来模拟方式产生PWM值*/
    int pulsewidth = (myangle * 11) + 500;//将角度转化为500-2480 的脉宽值
    digitalWrite(SERVOPIN, HIGH);//将舵机接口电平置高
    delayMicroseconds(pulsewidth);//延时脉宽值的微秒数
    digitalWrite(SERVOPIN, LOW);//将舵机接口电平置低
    delay(20 - pulsewidth / 1000);//延时周期内剩余时间
}

void SmartCar::adjustHead(){
    for (int i = 0; i <= 10; i++) //产生PWM个数，等效延时以保证能转到响应角度
    {
        servoPulse(90);//模拟产生PWM
    }
}

float SmartCar::distanceDetection(char direction) {
    switch (direction) {
        case LEFT:
            for (int i = 0; i <= 15; i++) //产生PWM个数，等效延时以保证能转到响应角度
            {
                servoPulse(175);//模拟产生PWM
            }
            return distance();
        case RIGHT:
            for (int i = 0; i <= 21; i++) //产生PWM个数，等效延时以保证能转到响应角度
            {
                servoPulse(5);//模拟产生PWM
            }
            return distance();
        case FORWARD:
            for (int i = 0; i <= 9; i++) //产生PWM个数，等效延时以保证能转到响应角度
            {
                servoPulse(90);//模拟产生PWM
            }
            return distance();
        default:
            break;

    }
    return 0.0;
}

void SmartCar::init() {
    //初始化左右电机为输出模式
    pinMode(LEFT_MOTOR_BACK, OUTPUT);
    pinMode(LEFT_MOTOR_FORWARD, OUTPUT);
    pinMode(RIGHT_MOTOR_BACK, OUTPUT);
    pinMode(RIGHT_MOTOR_FORWARD, OUTPUT);

    //初始化左右(循迹)红外传感器为输入模式
    pinMode(SENSOR_RIGHT_FINDER, INPUT);
    pinMode(SENSOR_RIGHT, INPUT);
    pinMode(SENSOR_LEFT_FINDER, INPUT);
    pinMode(SENSOR_LEFT, INPUT);

    //初始化超声波引脚
    pinMode(ECHO, INPUT);
    pinMode(TRIG, OUTPUT);

    //初始化舵机引脚为输出模式
    pinMode(SERVOPIN, OUTPUT);
}
