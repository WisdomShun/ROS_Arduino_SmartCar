//
// Created by shun on 19-4-13.
//

#ifndef SRC_SMARTCATMASTER_H
#define SRC_SMARTCATMASTER_H

#include <geometry_msgs/Twist.h>

class Listener {
public:
    virtual void callback(const geometry_msgs::Twist::ConstPtr &msg);
};

#endif //SRC_SMARTCATMASTER_H
