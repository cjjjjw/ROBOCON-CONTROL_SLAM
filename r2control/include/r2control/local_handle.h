#ifndef __LOCAL_HANDLE_H_
#define __LOCAL_HANDLE_H_
#include "r2control/serial_connect.h"

//这是一个数据帧头的结构体
struct Header
    {
        const unsigned char ender[2] = {0x0d, 0x0a};//设置检验数据尾
        const unsigned char header_now[2] = {0x55, 0xaa};//设置检验数据头
        const unsigned char header_goal[2] = {0xaa, 0x55};//设置检验数据头
        unsigned char ctrlFlag=0x01;//控制位数据
    };

class R2serHandler
{
private:  
    unsigned char *postionChange(int data,int len);//ASCII的不定长转化
    unsigned char *ASCIITranform(double data,int len);//将位置数字信息转换为ACSII的字符串信息
public:
    R2serHandler();//构造函数
    Header header_code,*header=&header_code;//构造结构体指针
    unsigned char *writeSpeedNow(double pos_x_now,double pos_y_now,double omega);
    unsigned char *writeSpeedGoal(double pos_x_goal,double pos_y_goal,double omega);
    ~R2serHandler();
};


R2serHandler::R2serHandler()
{
}

R2serHandler::~R2serHandler()
{
}

#endif