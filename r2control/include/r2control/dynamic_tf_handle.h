#ifndef __TF_DYNC_HANDLE_
#define __TF_DYNC_HANDLE_
#define Pi 3.1415926535f

#include"tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include"tf2_ros/transform_listener.h"
#include"tf2_ros/buffer.h"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include"tf2/LinearMath/Quaternion.h"

#include <iostream>
#include <vector>

#include "tf/tf.h"

#include <algorithm>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>


struct Euler
{
    double roll=0;
    double pitch=0;
    double yaw=0;
};

struct Quaternion
{
    double x=0;
    double y=0;
    double z=0;
    double w=0;
};


class DyncTFhandle
{
public:
    struct transformData
    {
        //设立子坐标和设立父坐标
        std::string child_frame;
        std::string parent_frame;
        //设立6个，计算欧拉数的TF转换
        Eigen::Vector3d pose_XYZ_Child;//1.position.x;2.position.y;3.position.z;
        Eigen::Vector3d pose_XYZ_Parent;//1.position.x;2.position.y;3.position.z;
        Euler pose_RPY_Child;
        Euler pose_RPY_Parent;
    };
private:
    /* data */
    // transformData *TF_data;
public:
    DyncTFhandle();
    void static_pub(transformData *static_PoseRelation);
    void dync_pub(transformData *dync_PoseRelation);
    void tf_quaternion2RPY(const geometry_msgs::Quaternion& orientation,Euler*outOrien);
    void tf_quaternion2XYZW(const Euler& orientation,Quaternion *outOrien);
    Eigen::Matrix<double, 3, 1> force_transform_pose(Eigen::Vector3d &p1,transformData *Pose_link);
    Eigen::Matrix<double, 3, 1> force_transform_origin(Eigen::Vector3d &origin_data,transformData *Pose_link);
    ~DyncTFhandle();
};



DyncTFhandle::DyncTFhandle()
{
}

DyncTFhandle::~DyncTFhandle()
{

}



#endif