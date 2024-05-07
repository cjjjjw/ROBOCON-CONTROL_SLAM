#include "r2control/local_handle.h"


/*
 * function：R2serHandler::postionChange
 * brief:ASCII的不定长转化
 * args:
 *   - data: 数字信息，如速度，位置
 *   - len: 转化为ASCII的长度
 * return: data_vel的指针
 */
 unsigned char *R2serHandler::postionChange(int data,int len)  
 {
    int pre_subtract=0;
    int now_subtract=0;
    unsigned char *data_pos=(unsigned char*)malloc(len);
    for (int i = 0; i <len; i++)
    {
        now_subtract=(data-pre_subtract)/(int)pow(10,len-1-i);//小于数值长度一位作相除
        data_pos[i]=(unsigned char)(now_subtract);
        pre_subtract+=now_subtract*(int)pow(10,len-1-i);//小于数值长度一位作相乘回去
    }
    return data_pos;
 }


/*
 * function：R2serHandler::ASCIITranform
 * brief:将位置数字信息转换为ACSII的字符串信息
 * args:
 *   - data: 数字信息，如速度，位置
 *   - len: 转化为ASCII的长度，包括第一位的数据位
 * return: data_vel的指针
 */
unsigned char *R2serHandler::ASCIITranform(double data,int len)
{
    unsigned char *data_vel=(unsigned char*)malloc(len);//增加一个数据位
    int pos_xc;
    int nums_length=len-1;//减去第一位数据位，让其方便计算
    //直接用ascii码代替数字
    //正反判断
    if (data<0){
        data_vel[0]=0x2d;
    }else{
        data_vel[0]=0x2b;
    }
    
    //数值转int处理,为了避免丢下一位的精度，作四舍五入
    pos_xc=(int)(round(data*pow(10,nums_length-1)));//数值长度为nums_length，当数值为1时，10000；改为-2，数值为1时，01000，往后推一位
    pos_xc=abs(pos_xc);
    //数值转ascii的处理
    unsigned char *postion=R2serHandler::postionChange(pos_xc,nums_length);
    for (int i=1;i<len;i++){
        //指针位置需要向前置一，i存在符号位
        data_vel[i]=*(postion+i-1);
    }
   //返回数组指针
   free (postion);
   return data_vel;
}


unsigned char *R2serHandler::writeSpeedNow(double pos_x_now,double pos_y_now,double omega)
{

    static unsigned char buf_NOW[21] = {};
    // unsigned char *buf_NOW=(unsigned char*)malloc(22);
    //初始化数组
    memset(buf_NOW,0x00,21);
    int length;
    //五位数据，第一位是符号位
    unsigned char *postion_data_now_x =R2serHandler::ASCIITranform(pos_x_now,5);
    unsigned char *postion_data_now_y=R2serHandler::ASCIITranform(pos_y_now,5);
    // unsigned char *postion_data_now_yaw=R2serHandler::ASCIITranform(yaw_now,3);
    unsigned char *omeag_now=R2serHandler::ASCIITranform(omega,5);

    for(int i = 0; i < 2; i++)
        buf_NOW[i] = this->header->header_now[i];             //buf[0]  buf[1]
    // 设置机器人前进旋转
    
    for(int i = 0; i < 5; i++)
    {
        buf_NOW[i + 2] = *(postion_data_now_x+i);    //buf[2] buf[3] buf[4] buf[5] buf[6] x_postion
        buf_NOW[i + 7] = *(postion_data_now_y+i);  //buf[7] buf[8] buf[9] buf[10] buf[11]y_postion
        buf_NOW[i + 12] = *(omeag_now+i);//buf[12] buf[13] buf[14] buf[15] buf[16] omega
    }
    //释放创建的空间
    free(postion_data_now_x);
    free(postion_data_now_y);

    buf_NOW[17] = this->header->ctrlFlag;       //buf[17]
    
    // 设置校验值、消息尾
    length=18;  //length=buf[18]

    buf_NOW[length] = getCrc8(buf_NOW, length);//buf[18]
    buf_NOW[length + 1] = this->header->ender[0];     //buf[19]
    buf_NOW[length + 2] = this->header->ender[1];     //buf[20]
    
    return buf_NOW;
}



unsigned char *R2serHandler::writeSpeedGoal(double pos_x_goal,double pos_y_goal,double omega)
{
    
    // unsigned char *buf_Goal=(unsigned char*)malloc(22);
    static unsigned char buf_Goal[21] = {} ;
    //初始化数组
    memset(buf_Goal,0x00,21);
    unsigned char *postion_data_goal_x;
    unsigned char *postion_data_goal_y;
    // unsigned char *postion_data_goal_yaw;
    unsigned char *omega_goal;

    int length;
    // 设置消息头
    for(int i = 0; i < 2; i++)
        buf_Goal[i] =this->header->header_goal[i];             //buf[0]  buf[1]
    
    // 设置机器人前进旋转
    //五位数据，第一位是符号位
    postion_data_goal_x=R2serHandler::ASCIITranform(pos_x_goal,5);
    postion_data_goal_y=R2serHandler::ASCIITranform(pos_y_goal,5);
    // postion_data_goal_yaw=R2serHandler::ASCIITranform(yaw_goal,3);
    omega_goal=R2serHandler::ASCIITranform(omega,5);



    for(int i = 0; i < 5; i++)
    {
        buf_Goal[i + 2] = *(postion_data_goal_x+i);    //buf[2] buf[3] buf[4] buf[5] buf[6] x_postion
        buf_Goal[i + 7] = *(postion_data_goal_y+i);  //buf[7] buf[8] buf[9] buf[10] buf[11]y_postion
        // buf_Goal[i + 10] = *(postion_data_goal_yaw+i);//buf[10] buf[11] buf[12] buf[13]yaw_z
        buf_Goal[i + 12] = *(omega_goal+i);//buf[12] buf[13] buf[14] buf[15] buf[16] omega

    }
    free(postion_data_goal_x);
    free(postion_data_goal_y);
    // free(postion_data_goal_yaw);
    free(omega_goal);

    // 预留控制指令
    buf_Goal[17] = this->header->ctrlFlag;       //buf[17]
    
    // 设置校验值、消息尾
    length=18;
    buf_Goal[length] = getCrc8(buf_Goal, length);//buf[18]
    buf_Goal[length + 1] = this->header->ender[0];     //buf[19]
    buf_Goal[length + 2] = this->header->ender[1];     //buf[20]
    
    return buf_Goal;
}