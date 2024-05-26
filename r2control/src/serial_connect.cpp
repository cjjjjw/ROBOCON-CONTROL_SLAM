#include "r2control/serial_connect.h"
#include <string.h>
#include <iostream>
#include <math.h>
#include <unistd.h>


using namespace std;//设定工作空间的名称

//串口相关对象
//创建串口实例化对象



bool serialInit(ser_setting *ser)
{
    if(Ros_to_Stm32.isOpen())
    {
    ROS_INFO("serial port has already opened");
        return true;
    }
    else 
    {
        Ros_to_Stm32.setPort(ser->port);//选择要开启的串口号
        Ros_to_Stm32.setBaudrate(ser->Baudrate);//设置波特率
        serial::Timeout _time =serial::Timeout::simpleTimeout(ser->Time_out);//超时等待
        Ros_to_Stm32.setTimeout(_time);
        Ros_to_Stm32.open();
        if(Ros_to_Stm32.isOpen())
        {
        ROS_INFO("serial port is opening");
            return true;
        }else{
        ROS_INFO("serial port has down");
            return false;
        }
    }
}



unsigned char getCrc8(unsigned char *ptr, unsigned short len)
{
    unsigned char crc;
    unsigned char i;
    crc = 0;
    while(len--)
    {
        crc ^= *ptr++;
        for(i = 0; i < 8; i++)
        {
            if(crc&0x01)
                crc=(crc>>1)^0x8C;
            else 
                crc >>= 1;
        }
    }
    return crc;
}
//传入数据，以及控制位
void writeData(unsigned char *buf,bool ctrl_flag)
{
    if (ctrl_flag)
    {
        Ros_to_Stm32.write(buf,24);
    }else
    {
        ROS_INFO("Unsuccsessfully send");
    }
    
}

unsigned char receiveData(void)
{
    //接收前确定接收的长度
    int length=0;
    unsigned char buf_re[150]={};
    size_t Receive_N =Ros_to_Stm32.available();//获取缓冲区内的字节数
    if (Receive_N !=0)
    {
        length=Ros_to_Stm32.read(buf_re,Receive_N);
    }
    return *buf_re;
}