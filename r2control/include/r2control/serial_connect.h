#ifndef __SERIAL_CONNECT_H_
#define __SERIAL_CONNECT_H_

#include <serial/serial.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <string>

using namespace std;

const string ser_port="/dev/ttyUSB0";
const size_t baudrate=115200;
const int time_out=200;

struct ser_setting
{
    string port=ser_port;
    size_t Baudrate=baudrate;
    int Time_out=time_out;
};


using namespace std;
serial::Serial Ros_to_Stm32;
bool serialInit(ser_setting *ser);//串口初始化
unsigned char getCrc8(unsigned char *ptr, unsigned short len);//串口命令检验和
void writeData(unsigned char *buf,bool ctrl_flag);
unsigned char receiveData(void);
// bool writeData(bool flag);//数据发送


#endif