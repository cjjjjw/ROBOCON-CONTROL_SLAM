#include"ros/ros.h"
#include"geometry_msgs/PointStamped.h"
#include "r2control/dynamic_tf_handle.h"


 //四元数 -->> 欧拉角
void DyncTFhandle::tf_quaternion2RPY(const geometry_msgs::Quaternion& orientation,Euler*outOrien)
{
    tf::Quaternion quat;
    tf::quaternionMsgToTF(orientation ,quat);
    tf::Matrix3x3(quat).getRPY(outOrien->roll, outOrien->pitch,outOrien->yaw); 
}

 //欧拉角 -->>四元数 
void DyncTFhandle::tf_quaternion2XYZW(const Euler& orientation,Quaternion *outOrien)
{
    tf::Quaternion qtn;
    qtn.setRPY(orientation.roll,orientation.pitch,orientation.yaw);// 设置偏航值，俯仰值，翻滚值，单位是弧度
    outOrien->w=qtn.getW();
    outOrien->x=qtn.getX();
    outOrien->y=qtn.getY();
    outOrien->z=qtn.getZ();
}

//静态坐标转换
void DyncTFhandle::static_pub(transformData *static_PoseRelation)
{
    while (1)
    {
        // ROS_INFO("11111");
        tf2_ros::StaticTransformBroadcaster pub;
        geometry_msgs::TransformStamped TF_Origin;// 创建坐标系信息
        TF_Origin.header.stamp=ros::Time::now();
        TF_Origin.header.frame_id=static_PoseRelation->parent_frame;
        TF_Origin.child_frame_id=static_PoseRelation->child_frame;
        TF_Origin.transform.translation.x=static_PoseRelation->pose_XYZ_Child[0];
        TF_Origin.transform.translation.y=static_PoseRelation->pose_XYZ_Child[1];
        TF_Origin.transform.translation.z=static_PoseRelation->pose_XYZ_Child[2];

        Quaternion *Orien_to_XYZW=new Quaternion;
        this->tf_quaternion2XYZW(static_PoseRelation->pose_RPY_Child,Orien_to_XYZW);
        TF_Origin.transform.rotation.x = Orien_to_XYZW->x;
        TF_Origin.transform.rotation.y = Orien_to_XYZW->y;
        TF_Origin.transform.rotation.z = Orien_to_XYZW->z;
        TF_Origin.transform.rotation.w =Orien_to_XYZW->w;
        //发布数据
        delete Orien_to_XYZW;
        pub.sendTransform(TF_Origin);
    }
}

//动态坐标转换
void DyncTFhandle::dync_pub(transformData *dync_PoseRelation)
{
    ROS_INFO("2222");
    tf2_ros::TransformBroadcaster pub;
    geometry_msgs::TransformStamped TF_Origin;// 创建坐标系信息
    TF_Origin.header.stamp=ros::Time::now();
    TF_Origin.header.frame_id=dync_PoseRelation->parent_frame;
    TF_Origin.child_frame_id=dync_PoseRelation->child_frame;
    TF_Origin.transform.translation.x=dync_PoseRelation->pose_XYZ_Child[0];
    TF_Origin.transform.translation.y=dync_PoseRelation->pose_XYZ_Child[1];
    TF_Origin.transform.translation.z=dync_PoseRelation->pose_XYZ_Child[2];

    Quaternion *Orien_to_XYZW=new Quaternion;
    this->tf_quaternion2XYZW(dync_PoseRelation->pose_RPY_Child,Orien_to_XYZW);
    TF_Origin.transform.rotation.x = Orien_to_XYZW->x;
    TF_Origin.transform.rotation.y = Orien_to_XYZW->y;
    TF_Origin.transform.rotation.z = Orien_to_XYZW->z;
    TF_Origin.transform.rotation.w =Orien_to_XYZW->w;
    //发布数据
    delete Orien_to_XYZW;
    pub.sendTransform(TF_Origin);
}


//为了将激光里程计转移到底盘上为底盘提供坐标与速度，由于通过监听的速度太慢，因此进行强制转换
Eigen::Matrix<double, 3, 1> DyncTFhandle::force_transform_pose(Eigen::Vector3d &p1,transformData *Pose_link)
{
    //动态分配结构体
    Quaternion *Orien1=new Quaternion;
    Quaternion *Orien2=new Quaternion;
    //将输入的两个位姿的欧拉角，转为四元素，将子坐标系上的坐标转换到父坐标系上
    this->tf_quaternion2XYZW(Pose_link->pose_RPY_Child,Orien1);
    this->tf_quaternion2XYZW(Pose_link->pose_RPY_Parent,Orien2);
    
    Eigen::Quaterniond Q1(Orien1->w,Orien1->x,Orien1->y,Orien1->z),Q2(Orien2->w,Orien2->x,Orien2->y,Orien2->z);//设置基于世界坐标系下的旋转四元数矩阵
    Q1.normalize(),Q2.normalize();//归一化
    Eigen::Vector3d T1=Pose_link->pose_XYZ_Child,T2=Pose_link->pose_XYZ_Parent;//设置基于世界坐标系下的平移矩阵
    //利用四元素变换
    // 求坐标系1在坐标系2下的坐标
    Eigen::Vector3d p2=Q2 * Q1.inverse() * (p1 - T1) + T2;  // 公式
    //清空被分配的内存
    delete Orien1;delete Orien2;
    return p2.transpose();
}


//计算不同之间的坐标系的原点之间的关系，将坐标系下转换起始原点到另一个坐标系的起始原点，可用于速度与位移
//origin_data(posx,posy,poz,pitch,roll,yaw) or (speedx,speedy,speedz,pitch,roll,yaw)
Eigen::Matrix<double, 3, 1> DyncTFhandle::force_transform_origin(Eigen::Vector3d &origin_data,transformData *Pose_link)
{
    //计算出两个坐标系开始的模长，相减求模长
    const double static_R=(Pose_link->pose_XYZ_Child-Pose_link->pose_XYZ_Parent).norm();
    //通过极坐标计算出，不同坐标轴下的模长
    double origin_R=origin_data.norm();
    double origin_xeta=atan(origin_data[1]/origin_data[0]);
    double origin_fai=origin_data[0]>=0  ?   (acos(origin_data[2]/origin_R))    :   (Pi+acos(origin_data[2]/origin_R));
    //通过当前坐标值，减去不同坐标轴下的模长，得到当前的坐标系下的坐标
    Eigen::Vector3d p2= Eigen::Vector3d(
        origin_data[0]-static_R*sin(origin_fai)*cos(origin_xeta),
        origin_data[1]-static_R*sin(origin_fai)*sin(origin_xeta),   
        origin_data[2]-static_R*cos(origin_fai)
    );
    return p2.transpose();
}


// void DyncTFhandle::static_listener()
// {
//     //创建一个buffer缓存
//     tf2_ros::Buffer buffer;
//     //再创建监听对象(监听对象可以将订阅的数据存入buffer)
//     tf2_ros::TransformListener listener(buffer);
//     //
//     geometry_msgs::PoseStamped pose_child;
//     try
//     {
//         //获得子父两个坐标系的转换关系
//         geometry_msgs::TransformStamped child_to_parent = buffer.lookupTransform(this->TF_data->child_frame,this->TF_data->parent_frame,ros::Time(0.0)); 
//         pose_child.pose.position.x=child_to_parent.transform.translation.x;
//         pose_child.pose.position.y=child_to_parent.transform.translation.y;
//         ROS_INFO("point_x:%f,point_y:%f",pose_child.pose.position.x,pose_child.pose.position.y);
//     }
//     catch(const std::exception& e)
//     {
//         ROS_INFO("异常消息:%s",e.what());
//     }
//     // ros::spin();
// }