#include "ros/ros.h"
#include <iostream>
#include <string>
#include <thread>
#include <unistd.h>
#include "r2control/main_control.h"


using namespace std;

//实例化全局串口发送对象
R2serHandler local_ser;
main_control _main_control;
//初始化串口结构体
ser_setting ser0;

//四元数转换欧拉角
void main_control::tf_quaternion2RPY(const geometry_msgs::Quaternion& orientation)
{
    tf::Quaternion quat;
    tf::quaternionMsgToTF(orientation ,quat);
    tf::Matrix3x3(quat).getRPY(this->PRC_data->roll, this->PRC_data->pitch, this->PRC_data->yaw); 
}

//通过里程计来计算
// void main_control::current_vel_2d()
// {

// }


void velCallBcak(const geometry_msgs::Twist& msg)
{
    unsigned char *goal_buf;
    usleep(50510);
    //由于/cmd_vel订阅的消息类型是Twist是Euler的形式

    ROS_INFO("the cmd_vel(v_x,v_y,omega) is %f,%f,%f",msg.linear.x,msg.linear.y,(msg.angular.z));
    goal_buf=local_ser.writeSpeedGoal(msg.linear.x,msg.linear.y,msg.angular.z);
    // for (int i=0;i<21;i++)
    // ROS_INFO("%d",goal_buf[i]);
}


void odomCallBcak(const nav_msgs::Odometry::ConstPtr &msg)
{
    unsigned char *now_buf;
    usleep(50510);
    //为了方便，直接订阅odom的current_pose
    _main_control.tf_quaternion2RPY(msg->pose.pose.orientation);
   
    ROS_INFO("the odom(v_x,v_y,omega) is %f,%f,%f",msg->pose.pose.position.x,msg->pose.pose.position.y,_main_control.PRC_data->yaw);
    now_buf=local_ser.writeSpeedNow(msg->pose.pose.position.x,msg->pose.pose.position.y,_main_control.PRC_data->yaw);
    for (int i=0;i<21;i++)
    ROS_INFO("%d",now_buf[i]);
}



int main(int argc,char **argv)
{
    try
    {
        serialInit(&ser0);//串口初始化
    }
    catch(const std::exception& e)
    {
        ROS_ERROR("UNSUCCESSFULLY START");
        return false;
    }
    //激活ROS
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "FUCKING");
    ros::NodeHandle nh;
    ros::Rate loop_rate(100);//定义定时器循环时间长度
    //通过设置多线程来做到异步收发
    // ros::Subscriber sub_vel=nh.subscribe("/cmd_vel",10,velCallBcak);
    ros::Subscriber sub_odom=nh.subscribe("/Odometry",10,odomCallBcak);
    ros::spin();
    
    return 0;
}