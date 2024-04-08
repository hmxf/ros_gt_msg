#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <pthread.h>
#include "controlcan.h"

#include <ctime>
#include <cstdlib>
#include "unistd.h"

#include <iostream>

#include "ros/ros.h"
#include <sstream>
#include <geometry_msgs/Twist.h>

#include <string>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include <ctime>
#include <stdint.h>

#include "ros_gt_msg/Lift_control.h"  //自定义消息类型
#include "ros_gt_msg/Lift_state.h"  //自定义消息类型
#include "ros_gt_msg/gt_motion.h"  //自定义消息类型
#include "ros_gt_msg/gt_control.h"//自定义消息类型

#include "ros_gt_msg.h"

#define PI 3.141592653

VCI_BOARD_INFO pInfo;//用来获取设备信息。
using namespace std;


/**************************************底盘控制*******************************************/
void SendSpeedToGt(u8 mode,s16 x,int y,u8 stop)
{
	//需要发送的帧，结构体设置
	VCI_CAN_OBJ send[1];
	send[0].ID=0x00000001;//根据底盘can协议，发送时候can的ID为0x01
	send[0].SendType=0;
	send[0].RemoteFlag=0;
	send[0].ExternFlag=0;
	send[0].DataLen=8;
        
	if(mode<=2)
	{
		//mode =1 轮速  mode=2 线速度和角速度
		send[0].Data[0] = 0x01;
		send[0].Data[1] = mode;
		send[0].Data[2] =  x     & 0xFF;
		send[0].Data[3] = (x>>8) & 0xFF;
		send[0].Data[4] =  y     & 0xFF;
		send[0].Data[5] = (y>>8) & 0xFF;
		send[0].Data[6] =  stop  & 0xFF;
		send[0].Data[7] =  0;
	}
	else if(mode==4)
	{
		send[0].Data[0] = 0x01;
		send[0].Data[1] = mode;
		send[0].Data[2] = 1;
		send[0].Data[3] = 0;
		send[0].Data[4] = 0;
		send[0].Data[5] = 0;
		send[0].Data[6] = 0;
		send[0].Data[7] = 0;


	}



	//写入数据
	if(VCI_Transmit(VCI_USBCAN2, 0, 0, send, 1) == 1)
	{
		printf("TX data successful!\n");
	}                  
}

void cmd_velCallback(const ros_gt_msg::gt_control::ConstPtr &gt_control_msg)//速度控制回调
{
	SendSpeedToGt(gt_control_msg->mode,gt_control_msg->x,gt_control_msg->y,gt_control_msg->stop);
}
/**********************************END***********************************************/

/************************************升降台*********************************************/
static void SendLift(u8 mode,s16 x,u8 clear_flag)
{
	VCI_CAN_OBJ lift_send;

	lift_send.ID=0x00000001;//根据底盘can协议，发送时候can的ID为0x01
	lift_send.SendType=0;
	lift_send.RemoteFlag=0;
	lift_send.ExternFlag=0;
	lift_send.DataLen=8;

	lift_send.Data[0] = 0x01;

	lift_send.Data[1] = 0x03;
	lift_send.Data[2] = mode;
	lift_send.Data[3] =  x     & 0xFF;
	lift_send.Data[4] = (x>>8) & 0xFF;
	lift_send.Data[5] =  clear_flag;
	lift_send.Data[6] =  0;
	lift_send.Data[7] =  0;

	//写入数据
	if(VCI_Transmit(VCI_USBCAN2, 0, 0, &lift_send, 1) == 1)
	{
		printf("TX data successful!\n");
	}   

}
static void cmd_LiftCallback(const ros_gt_msg::Lift_control::ConstPtr &lift_control_msg)//速度控制回调
{
	SendLift(lift_control_msg->mode,lift_control_msg->data,lift_control_msg->clear_flag);
}


/************************************END*********************************************/
int main(int argc, char **argv)
{
	int i=0,re_size = 0;
	/*

	   ros_gt_msg::gt_drive_motor gt_drive_motor_msg;
	   ros_gt_msg::gt_steering_motor gt_steering_motor_msg;
	   ros_gt_msg::gt_system  gt_system_msg;
	   */
	ros_gt_msg::gt_control   gt_control_msg;
	ros_gt_msg::gt_motion    gt_motion_msg;
	ros_gt_msg::Lift_state   lift_msg;
	ros_gt_msg::Lift_control lift_control_msg;
	printf(">>this is hello !\r\n");//指示程序已运行
	int num = VCI_OpenDevice(VCI_USBCAN2,0,0);
	if(num == 0 || num == 1)//打开设备
	{
		printf(">>open deivce success!\n");//打开设备成功
	}else
	{
		printf(">>open deivce error %d!\n",num);
		exit(1);
	}
	if(VCI_ReadBoardInfo(VCI_USBCAN2,0,&pInfo)==1)//读取设备序列号、版本等信息。
	{
		printf(">>Get VCI_ReadBoardInfo success!\n");
	}else
	{
		printf(">>Get VCI_ReadBoardInfo error!\n");
		exit(1);
	}

	//初始化参数，严格参数二次开发函数库说明书。
	VCI_INIT_CONFIG config;
	config.AccCode=0;
	config.AccMask=0xFFFFFFFF;//FFFFFFFF全部接收
	config.Filter=2;//接收所有帧  2-只接受标准帧  3-只接受扩展帧
	config.Timing0=0x00;/*波特率500 Kbps  0x00  0x1C*/
	config.Timing1=0x1C;
	config.Mode=0;//正常模式

	if(VCI_InitCAN(VCI_USBCAN2,0,0,&config)!=1)
	{
		printf(">>Init CAN1 error\n");
		VCI_CloseDevice(VCI_USBCAN2,0);
	}

	if(VCI_StartCAN(VCI_USBCAN2,0,0)!=1)
	{
		printf(">>Start CAN1 error\n");
		VCI_CloseDevice(VCI_USBCAN2,0);
	}

	//需要读取的帧，结构体设置
	VCI_CAN_OBJ rev[2500];
	rev[0].SendType=0;
	rev[0].RemoteFlag=0;
	rev[0].ExternFlag=0;
	rev[0].DataLen=8;

	ros::init(argc, argv, "publish_gt_msg");
	ros::NodeHandle n;
	ros::Subscriber gt_control_sub = n.subscribe<ros_gt_msg::gt_control>("/GT_Control", 100, cmd_velCallback);//速度回调
	ros::Subscriber lift_control_sub = n.subscribe<ros_gt_msg::Lift_control>("/Lift_Control", 100, cmd_LiftCallback);//升降台速度回调

	ros::Publisher gt_motion_pub      = n.advertise<ros_gt_msg::gt_motion>("/GT_Motion", 100);//运动状态消息发布
	ros::Publisher lift_motion_pub      = n.advertise<ros_gt_msg::Lift_state>("/Lift_Motion", 100);//运动状态消息发布

	ros::Rate loop_rate(100);

	int16_t temp_Vz=0;
	while (ros::ok())
	{
		//读取数据
		re_size = VCI_Receive(VCI_USBCAN2, 0, 0,rev, 2500, 0);
		if(re_size > 0)
		{
			for(int i=0;i< re_size;i++)
			{
				//运动状态反馈
				if((rev[i].ID == 0x50)&&(rev[i].Data[0] == 02) && (rev[i].Data[1] == 01))//接收速度反馈
				{
					gt_motion_msg.Vx = rev[i].Data[2]|(rev[i].Data[3]<<8);
					temp_Vz = rev[i].Data[4]|(rev[i].Data[5]<<8);
					gt_motion_msg.Vz = (float)temp_Vz/1000.0f;
					gt_motion_msg.power = rev[i].Data[6]|(rev[i].Data[7]<<8);
				}
				else if((rev[i].ID == 0x50)&&(rev[i].Data[0] == 02) && (rev[i].Data[1] == 02))//接收底盘状态
				{
					gt_motion_msg.robot_data = rev[i].Data[2];
					gt_motion_msg.robot_contr_mode = rev[i].Data[3];
					gt_motion_msg.temp_l = rev[i].Data[4]|(rev[i].Data[5]<<8);
				}
				else if(rev[i].ID == 0x55)//驱动器状态反馈
				{
					gt_motion_msg.driver_err_L = rev[i].Data[0]|(rev[i].Data[1]<<8);
					gt_motion_msg.driver_err_R = rev[i].Data[2]|(rev[i].Data[3]<<8);
					gt_motion_msg.Current_L = rev[i].Data[4]|(rev[i].Data[5]<<8);
					gt_motion_msg.Current_R = rev[i].Data[6]|(rev[i].Data[7]<<8);
				}
				else if(rev[i].ID == 0x56)//驱动电机状态反馈
				{
					gt_motion_msg.driver_init_L = rev[i].Data[0]|(rev[i].Data[1]<<8);
					gt_motion_msg.driver_init_R = rev[i].Data[2]|(rev[i].Data[3]<<8);
					gt_motion_msg.Current_max_L = rev[i].Data[4]|(rev[i].Data[5]<<8);
					gt_motion_msg.Current_max_R = rev[i].Data[6]|(rev[i].Data[7]<<8);
				}
				else if((rev[i].ID == 0x50)&&(rev[i].Data[0] == 02) && (rev[i].Data[1] == 03))//接收升降台状态
				{
					lift_msg.ctr_mode = rev[i].Data[2];
					lift_msg.speed_rpm = rev[i].Data[3]|(rev[i].Data[4]<<8);
					lift_msg.hight = rev[i].Data[5]|(rev[i].Data[6]<<8);
					lift_msg.lift_state = rev[i].Data[7];
				}


			}
			gt_motion_pub.publish(gt_motion_msg);
			lift_motion_pub.publish(lift_msg);

		}        
		ros::spinOnce();
		loop_rate.sleep();
	}
	VCI_CloseDevice(VCI_USBCAN2,0);//关闭设备。
	return 0;
}
