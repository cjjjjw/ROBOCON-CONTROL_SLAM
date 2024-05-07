#ifndef __MAIN_CONTROL_H_
#define __MAIN_CONTROL_H_

#include "r2control/serial_connect.h"
#include "r2control/local_handle.h"
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#define pi 3.1415926535f


struct Euler
{
    double pitch;
    double yaw;
    double roll;
};




class main_control
{
private:
    /* data */
public:
    main_control(/* args */);
    Euler Orien_to_PRC,*PRC_data=&Orien_to_PRC;
    void tf_quaternion2RPY(const geometry_msgs::Quaternion& msg);
    // void main_control::current_vel_2d();
    ~main_control();
};

main_control::main_control(/* args */)
{
}

main_control::~main_control()
{
}





#endif