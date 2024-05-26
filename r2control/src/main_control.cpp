#include "ros/ros.h"
#include <iostream>
#include <string>
#include <thread>
#include <unistd.h>
#include "r2control/main_control.h"
#include "r2control/dynamic_tf_handle.h"


using namespace std;

//实例化全局串口发送对象
R2serHandler local_ser;
DyncTFhandle dync_handle_tf;
//初始化串口结构体
ser_setting ser0;



void velCallBcak(const geometry_msgs::Twist& msg)
{
    unsigned char *goal_buf;
    usleep(10510);
    //由于/cmd_vel订阅的消息类型是Twist是Euler的形式
    ROS_INFO("the cmd_vel(v_x,v_y,omega) is %f,%f,%f",msg.linear.x,msg.linear.y,(msg.angular.z));
    goal_buf=local_ser.writeSpeedGoal(msg.linear.x,msg.linear.y,msg.angular.z);
    // for (int i=0;i<21;i++)
    // ROS_INFO("%d",goal_buf[i]);
    // writeData(goal_buf,0x01);
}


void odomCallBcak(const nav_msgs::Odometry::ConstPtr &msg)
{
    unsigned char *speed_now_buf,*goal_now_buf;
    Euler *PRY_data=new Euler;
    DyncTFhandle::transformData *carbody_to_world=new DyncTFhandle::transformData{"body","map",Eigen::Vector3d(0,-0.370,0),Eigen::Vector3d(0,0,0),Euler{0,0,-0.7853981}};     //给carbody_to_world结构体赋值,Eigen::Vector3d(0,0,0),Eigen::Vector3d(0,0,0),Euler{0,0,-0.785398
    usleep(10510);
    //动态的速度关系
    // ROS_INFO("the odom(v_x,v_y,omega) is %f,%f,%f",msg->twist.twist.linear.x,msg->twist.twist.linear.y,msg->twist.twist.angular.z);
    // speed_now_buf=local_ser.writeSpeedNow(msg->twist.twist.linear.x,msg->twist.twist.linear.y,msg->twist.twist.angular.z);
    //动态的相对坐标
    dync_handle_tf.tf_quaternion2RPY(msg->pose.pose.orientation,PRY_data);
    Eigen::Vector3d p1(msg->pose.pose.position.x,msg->pose.pose.position.y,msg->pose.pose.position.z);
    Eigen::Vector3d p2=dync_handle_tf.force_transform_pose(p1,carbody_to_world);//转换到地图坐标系
    p2=dync_handle_tf.force_transform_origin(p2,carbody_to_world);//转换到地图原点
    goal_now_buf=local_ser.writePosNow(p2[0],p2[1],PRY_data->yaw);
    // ROS_INFO("x1=%.3f,y1=%.3f,z1=%.3f",p1[0],p1[1],p1[2]);
    ROS_INFO("x2=%.3f,y2=%.3f,z2=%.3f",p2[0],p2[1],PRY_data->yaw+0.7853981);
    delete PRY_data;
    // for (int i=0;i<24;i++)
    // ROS_INFO("%d",speed_now_buf[i]);
    // writeData(speed_now_buf,0x01);
    // writeData(goal_now_buf,0x01);
}



int main(int argc,char **argv)
{
    // try
    //  {
    //      serialInit(&ser0);//串口初始化
    //  }
    //  catch(const std::exception& e)
    //  {
    //      ROS_ERROR("UNSUCCESSFULLY START");
    //      return false;
    // }
    //激活ROS
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "FUCKING");
    ros::NodeHandle nh; 
    ros::Rate loop_rate(1);//定义定时器循环时间长度
    //发布雷达odom与车体odom的tf关系
    DyncTFhandle::transformData *car_to_cam=new DyncTFhandle::transformData{"car_body","body",Eigen::Vector3d(0,-0.370,0),Eigen::Vector3d(0,0,0),Euler{0,0,0}}; 
    thread receive_thread(&DyncTFhandle::static_pub,&dync_handle_tf,(car_to_cam));
    receive_thread.detach();
   
    // ros::Subscriber sub_vel=nh.subscribe("/cmd_vel",100,velCallBcak);
    ros::Subscriber sub_odom=nh.subscribe("/Odometry",100,odomCallBcak);
    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::waitForShutdown();
    
    return 0;
}