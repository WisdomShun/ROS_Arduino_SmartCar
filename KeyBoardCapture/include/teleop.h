//
// Created by shun on 19-4-12.
//

#ifndef SRC_TELEOP_H
#define SRC_TELEOP_H

class Teleop{
public:
    Teleop();
    void keyLoop();

private:
    ros::NodeHandle nh_;
    double linear_, angular_, l_scale_, a_scale_;
    ros::Publisher twist_pub_;
};

#endif //SRC_TELEOP_H
