//实现了点到点移动
//先旋转到车头对准目标点，然后直线移动到目标点，然后调整姿态角
//实现了输入目标点，小车寻及，但是不能调整到达最终点的姿态角
//开环控制，精度非常低

#include <ros/ros.h>
#include "/home/ustb814/car/devel/include/ultra_serial_port/Ultrasound.h"
#include "/home/ustb814/car/src/cancontrol/include/cancontrol/controlcan.h"

#include <iostream>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <pthread.h>
#include <ctime>
#include <cstdlib>
#include "unistd.h"
#include <visualization_msgs/Marker.h>
#include <vector> 
#include <geometry_msgs/Point.h>

#include <math.h>

#include "cancontrol/vandw.h"

using namespace std;

typedef unsigned short uint16;
typedef short int int16;
typedef unsigned char u8;

bool pathInfo = false;

vector<geometry_msgs::Point> folowPoints; //跟踪的点
geometry_msgs::Point folowPoint;

//订阅超声波消息
void personInfoCallback(const ultra_serial_port::Ultrasound::ConstPtr& Ultrasound_msg)
{
    extern int ultra_dis;
    // 将接收到的消息打印出来
    //ROS_INFO("Ultrasound_info: dis=%d", Ultrasound_msg->dis);
    ultra_dis = Ultrasound_msg->dis;
}
//订阅最新路径消息
void pathInfoCallback(const geometry_msgs::Point::ConstPtr& msg)
{
		extern int kzdgs;
		extern int ifstart;
		folowPoint.x = 0.05*(msg->x);
		folowPoint.y = 0.05*(msg->y);
		folowPoints.push_back(folowPoint);
		if((folowPoint.x==4.75)&&(folowPoint.y==4.75))
			{ifstart=1;pathInfo=true;}
		kzdgs++;
		printf("x-->%f,y-->%f \n", folowPoint.x, folowPoint.y); 
}

int xorcheck(int *pdata, int len); //异或校验取值

int16 v_w_get(int data4, int data5); //读取速度值

clock_t startTime,endTime;//计时
void *receive_func(void* param);//接收线程。
VCI_BOARD_INFO pInfo;//用来获取设备信息。
int count=0;//数据列表中，用来存储列表序号。
VCI_BOARD_INFO pInfo1 [50];
int num=0;

int* v_control(int v); 
int* w_control(int w);
int *V;int *W;//2021.4.5 14:47 用于给小车线速度、角速度
int return_v=0; int return_w=0;//2021.4.5 16:57 小车返回的线、角速度
int return_dis=0; float return_theta=0; //2021.4.6 17:26 小车返回的距离和角度

int ultra_dis=0; //超声波探测的距离

//*****************************************
//move_control 运动控制
//给定目标点，小车两点间直线控制，姿态格式[x,y,theta]
//w_flag,v_flag-标志位，标志位0-不动，1-动
float InitPos[2]={0.1,0.1}; //2*0.05
float Goal[2]={4.75,4.75}; //95*0.05
float theta;
float theta_error;
float theta_goal;
int s_dis; float s_time;
float theta_actual;
int w_flag=0; int v_flag=0; //标志位0,速度为0,标志位1,速度不为0
static float theta_sum_close=0; //闭环，小车从开始运行到程序结束的theta角累加
static float theta_sum_open=0; //开环
static int dis_sum_close=0;  //角度float型，rad，距离int型，mm
static int dis_sum_open=0; //开环

int order_control=0; //控制小车顺序
//*****************************************2021.4.6 16:53
int ifstart=0; //0-小车不动，1-小车路径跟踪(判断是否接收完路径点)
int kzdgs=0; //控制点个数

Ackermann_Car_Feedback car_back[1];//反馈消息   a ke man

int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "cancontrol");

    // 创建节点句柄
    ros::NodeHandle nc;

    // 创建一个Subscriber，订阅名为/person_info的topic，注册回调函数personInfoCallback
    ros::Subscriber person_info_sub = nc.subscribe("/Ultrasound_info", 10, personInfoCallback);

    //创建一个发布者，发布小车的路径信息
	ros::Publisher vandw_info_pub = nc.advertise<cancontrol::vandw>("/vandw_info",1);

	// 创建一个Subscriber，订阅名为/person_info的topic，注册回调函数personInfoCallback
    ros::Subscriber final_path_sub = nc.subscribe("/cancontrol_points", 10, pathInfoCallback);

    // 循环等待回调函数
    //ros::spinOnce();

/*
while(ros::ok())
{

	ros::spin();
}
*/


	printf(">>this is hello !!!!!!!!!!!!!!!!!!\r\n");//指示程序已运行

	num=VCI_FindUsbDevice2(pInfo1);

	printf(">>USBCAN DEVICE NUM:");printf("%d", num);printf(" PCS");printf("\n");

		for(int i=0;i<num;i++)
		{
		printf("Device:");printf("%d", i);printf("\n");
        printf(">>Get VCI_ReadBoardInfo success!\n");
		printf(">>Serial_Num:%c", pInfo1[i].str_Serial_Num[0]);
		printf("%c", pInfo1[i].str_Serial_Num[1]);
		printf("%c", pInfo1[i].str_Serial_Num[2]);
		printf("%c", pInfo1[i].str_Serial_Num[3]);
		printf("%c", pInfo1[i].str_Serial_Num[4]);
		printf("%c", pInfo1[i].str_Serial_Num[5]);
		printf("%c", pInfo1[i].str_Serial_Num[6]);
		printf("%c", pInfo1[i].str_Serial_Num[7]);
		printf("%c", pInfo1[i].str_Serial_Num[8]);
		printf("%c", pInfo1[i].str_Serial_Num[9]);
		printf("%c", pInfo1[i].str_Serial_Num[10]);
		printf("%c", pInfo1[i].str_Serial_Num[11]);
		printf("%c", pInfo1[i].str_Serial_Num[12]);
		printf("%c", pInfo1[i].str_Serial_Num[13]);
		printf("%c", pInfo1[i].str_Serial_Num[14]);
		printf("%c", pInfo1[i].str_Serial_Num[15]);
		printf("%c", pInfo1[i].str_Serial_Num[16]);
		printf("%c", pInfo1[i].str_Serial_Num[17]);
		printf("%c", pInfo1[i].str_Serial_Num[18]);
		printf("%c", pInfo1[i].str_Serial_Num[19]);printf("\n");

		printf(">>hw_Type:%c", pInfo1[i].str_hw_Type[0]);
		printf("%c", pInfo1[i].str_hw_Type[1]);
		printf("%c", pInfo1[i].str_hw_Type[2]);
		printf("%c", pInfo1[i].str_hw_Type[3]);
		printf("%c", pInfo1[i].str_hw_Type[4]);
		printf("%c", pInfo1[i].str_hw_Type[5]);
		printf("%c", pInfo1[i].str_hw_Type[6]);
		printf("%c", pInfo1[i].str_hw_Type[7]);
		printf("%c", pInfo1[i].str_hw_Type[8]);
		printf("%c", pInfo1[i].str_hw_Type[9]);printf("\n");	

		printf(">>Firmware Version:V");
		printf("%x", (pInfo1[i].fw_Version&0xF00)>>8);
		printf(".");
		printf("%x", (pInfo1[i].fw_Version&0xF0)>>4);
		printf("%x", pInfo1[i].fw_Version&0xF);
		printf("\n");
	}
	printf(">>\n");
	printf(">>\n");
	printf(">>\n");
	if(VCI_OpenDevice(VCI_USBCAN2,0,0)==1)//打开设备
	{
		printf(">>open deivce success!\n");//打开设备成功
	}else
	{
		printf(">>open deivce error!\n");
		exit(1);
	}
	if(VCI_ReadBoardInfo(VCI_USBCAN2,0,&pInfo)==1)//读取设备序列号、版本等信息。
	{
        printf(">>Get VCI_ReadBoardInfo success!\n");
		//printf(" %08X", pInfo.hw_Version);printf("\n");
		//printf(" %08X", pInfo.fw_Version);printf("\n");
		//printf(" %08X", pInfo.dr_Version);printf("\n");
		//printf(" %08X", pInfo.in_Version);printf("\n");
		//printf(" %08X", pInfo.irq_Num);printf("\n");
		//printf(" %08X", pInfo.can_Num);printf("\n");
		printf(">>Serial_Num:%c", pInfo.str_Serial_Num[0]);
		printf("%c", pInfo.str_Serial_Num[1]);
		printf("%c", pInfo.str_Serial_Num[2]);
		printf("%c", pInfo.str_Serial_Num[3]);
		printf("%c", pInfo.str_Serial_Num[4]);
		printf("%c", pInfo.str_Serial_Num[5]);
		printf("%c", pInfo.str_Serial_Num[6]);
		printf("%c", pInfo.str_Serial_Num[7]);
		printf("%c", pInfo.str_Serial_Num[8]);
		printf("%c", pInfo.str_Serial_Num[9]);
		printf("%c", pInfo.str_Serial_Num[10]);
		printf("%c", pInfo.str_Serial_Num[11]);
		printf("%c", pInfo.str_Serial_Num[12]);
		printf("%c", pInfo.str_Serial_Num[13]);
		printf("%c", pInfo.str_Serial_Num[14]);
		printf("%c", pInfo.str_Serial_Num[15]);
		printf("%c", pInfo.str_Serial_Num[16]);
		printf("%c", pInfo.str_Serial_Num[17]);
		printf("%c", pInfo.str_Serial_Num[18]);
		printf("%c", pInfo.str_Serial_Num[19]);printf("\n");

		printf(">>hw_Type:%c", pInfo.str_hw_Type[0]);
		printf("%c", pInfo.str_hw_Type[1]);
		printf("%c", pInfo.str_hw_Type[2]);
		printf("%c", pInfo.str_hw_Type[3]);
		printf("%c", pInfo.str_hw_Type[4]);
		printf("%c", pInfo.str_hw_Type[5]);
		printf("%c", pInfo.str_hw_Type[6]);
		printf("%c", pInfo.str_hw_Type[7]);
		printf("%c", pInfo.str_hw_Type[8]);
		printf("%c", pInfo.str_hw_Type[9]);printf("\n");

		printf(">>Firmware Version:V");
		printf("%x", (pInfo.fw_Version&0xF00)>>8);
		printf(".");
		printf("%x", (pInfo.fw_Version&0xF0)>>4);
		printf("%x", pInfo.fw_Version&0xF);
		printf("\n");	
	}else
	{
		printf(">>Get VCI_ReadBoardInfo error!\n");
		exit(1);
	}

	//初始化参数，严格参数二次开发函数库说明书。
	VCI_INIT_CONFIG config;
	config.AccCode=0;       //------------------------------
	config.AccMask=0xFFFFFFFF;
	config.Filter=1;//接收所有帧
	config.Timing0=0x00;/*波特率500 Kbps  0x00  0x1C*/
	config.Timing1=0x1C;
	config.Mode=0;//正常模式		
	
	if(VCI_InitCAN(VCI_USBCAN2,0,0,&config)!=1)
	{
		printf(">>Init CAN1 error\n");
		VCI_CloseDevice(VCI_USBCAN2,0);
	}
	else
	{
		printf(">>Init can1 success\n" );
	}	
	if(VCI_StartCAN(VCI_USBCAN2,0,0)!=1)
	{
		printf(">>Start CAN1 error\n");
		VCI_CloseDevice(VCI_USBCAN2,0);

	}
	else
	{
		printf(">>Start can1 success\n" );
	}	
	if(VCI_InitCAN(VCI_USBCAN2,0,1,&config)!=1)
	{
		printf(">>Init can2 error\n");
		VCI_CloseDevice(VCI_USBCAN2,0);

	}
	else
	{
		printf(">>Init can2 success\n" );
	}
	if(VCI_StartCAN(VCI_USBCAN2,0,1)!=1)
	{
		printf(">>Start can2 error\n");
		VCI_CloseDevice(VCI_USBCAN2,0);
	}
	else
	{
		printf(">>Start can2 success\n" );
	}
	
	VCI_CAN_OBJ send[2];
	send[0].ID=0x601;//0x601-数据，0x701-心跳
	send[0].SendType=0;
	send[0].RemoteFlag=0;
	send[0].ExternFlag=0;
	send[0].DataLen=8;

	int i=0;
	int Datasource[8][8];
	int Jumpsource[8]={0x05,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
////////////////////////////////////////////////////////
// 	//初始状态
//	theta=atan2((double)(Goal[1]-InitPos[1]),(double)(Goal[0]-InitPos[0]));
//	theta_error = theta; //rad,先自己旋转
//	//末状态调整姿态
//	theta_goal = -theta_error;
//	//直线距离
//	s_dis=sqrt((float)((Goal[0]-InitPos[0])*(Goal[0]-InitPos[0])+(Goal[1]-InitPos[1])*(Goal[1]-InitPos[1])));
//	s_dis=s_dis*1000; //单位mm
////////////////////////////////////////////////////////
	int vx=0;
	int vy=0;
	static int sx=0;
	static int sy=0;

	int m_run0=1;
	pthread_t threadid;
	int ret;
	ret=pthread_create(&threadid,NULL,receive_func,&m_run0);

	startTime = clock();//计时开始

	int times = 500;//times = 500 = 1S
	int rownum=0;

	//-------------------------------------------------------
	//主要控制代码的一些变量
	float point_before[2];
	float point_after[2]; //小车从point_before移动到point_after
	int if_finish=1; //判断两点之间的移动是否完成
	int djcxj=0; //第几次寻级
	int allend=0; //全部寻及结束为1
	int delayms=0; //yanshi 1s
	//-------------------------------------------------------

	int loop_num = 0;

while(pathInfo == false)
{
	ros::spinOnce();
}

	while(ros::ok())
	{

		//printf("Move to main while! \n" );
		//等待回调函数
		ros::spinOnce();
	
		vx = return_v * cos(theta_sum_close);
		vy = return_v * sin(theta_sum_close);

		sx = sx + vx * 0.01 * 0.001;
		sy = sy + vy * 0.01 * 0.001; 

		cancontrol::vandw vandw_msg;
        vandw_msg.v = return_v;
        vandw_msg.w = return_w; 
        vandw_msg.dis_x = sx;
        vandw_msg.dis_y = sy;
        vandw_info_pub.publish(vandw_msg);
        //ROS_INFO("Publish Person Info: %d, %d \n dis_x:%d dis_y:%d \n", vandw_msg.v, vandw_msg.w, vandw_msg.dis_x, vandw_msg.dis_y);


//*********************************************************************************************
//*********************************************************************************************
//主要控制代码***********************************************************************************
if(ifstart==0 || ultra_dis < 50 || allend==1 || delayms!=0) //未接收到路径点 或 超声波笔账 或 全部寻及结束 或 需要延时
{
	V = v_control(0);
	W = w_control(0);
	for(int j=0; j<8; j++)
	{
		Datasource[0][j]=*(V+j);
		Datasource[1][j]=*(W+j);
	}
	if(delayms!=0)
		delayms--;
}
else
{
	if( if_finish==1 )
	{
		point_before[0] = folowPoints[djcxj].x;
		point_before[1] = folowPoints[djcxj].y;
		point_after[0] = folowPoints[djcxj+1].x;
		point_after[1] = folowPoints[djcxj+1].y;
		//初始状态
		theta=atan2((double)(point_after[1]-point_before[1]),(double)(point_after[0]-point_before[0]));
		theta_error = theta - theta_sum_open; //rad,先自己旋转
		//末状态调整姿态
		if((point_after[0]==Goal[0])&&(point_after[1]==Goal[1]))
		theta_goal = -theta_sum_open;
		//直线距离
		s_dis=sqrt((float)((point_before[0]-point_after[0])*(point_before[0]-point_after[0])+(point_before[1]-point_after[1])*(point_before[1]-point_after[1])));
		s_dis=s_dis*1000; //单位mm

		if_finish=0;
	}
	




	if(order_control == 0) //step1:程序刚开始
	{
		if(fabs(theta - theta_sum_open) > 0.1)
		{
			if(theta_error > 0)
			{
				V = v_control(0);
				W = w_control(500);
				for(int j=0; j<8; j++)
				{
					Datasource[0][j]=*(V+j);
					Datasource[1][j]=*(W+j);
				}
				theta_sum_open += 500*0.001*0.01; //rad
				printf("I am here!-----1   \n");
			}
			else if(theta_error <= 0)
			{
				V = v_control(0);
				W = w_control(-500);
				for(int j=0; j<8; j++)
				{
					Datasource[0][j]=*(V+j);
					Datasource[1][j]=*(W+j);
				}
				theta_sum_open += (-500)*0.001*0.01; //rad
				printf("I am here!-----2   \n");
			}		
		}
		else //if(fabs(theta_error - theta_sum_close) <= 0.1)
		{
			V = v_control(0);
			W = w_control(0);
			for(int j=0; j<8; j++)
			{
				Datasource[0][j]=*(V+j);
				Datasource[1][j]=*(W+j);
			}
			order_control =1;
			printf("Step1 success! \n \n \n \n \n \n");
		}
	}

	if(order_control == 1) //step2:走直线
	{
		if((fabs(s_dis - dis_sum_open) > 100) && (dis_sum_open < 10000))
		{
			V = v_control(1000);
			W = w_control(0);
			for(int j=0; j<8; j++)
			{
				Datasource[0][j]=*(V+j);
				Datasource[1][j]=*(W+j);
			}
			dis_sum_open += 1000*0.01;
			//printf("s_dis:%f, dis_sum_close:%f \n",s_dis,dis_sum_close );
			printf("I here! \n");
		}
		else //if((fabs(s_dis - dis_sum_close) <= 0.5) || (dis_sum_close > s_dis))
		{
				V = v_control(0);
				W = w_control(0);
				for(int j=0; j<8; j++)
				{
					Datasource[0][j]=*(V+j);
					Datasource[1][j]=*(W+j);
				}
			order_control = 0;
			dis_sum_open= 0;
			if_finish=1;
			djcxj++;
			if((djcxj+1)==kzdgs)
				allend=1;
			delayms=200;
		}
	}

/*
	if(order_control == 2) //step3:调整
	{
		if(fabs(0 - theta_sum_close) > 0.1)
		{
			if(0 - theta_sum_close > 0)
			{
				V = v_control(0);
				W = w_control(500);
				for(int j=0; j<8; j++)
				{
					Datasource[0][j]=*(V+j);
					Datasource[1][j]=*(W+j);
				}
				printf("I am here!-----4   \n");
			}
			else
			{
				V = v_control(0);
				W = w_control(-500); //程序运行到这里，正向旋转++++++++++++++++++++++++++++++++++++++++++++++++
				for(int j=0; j<8; j++)
				{
					Datasource[0][j]=*(V+j);
					Datasource[1][j]=*(W+j);
				}
				printf("I am here!-----5   \n");
			}	
		}
		else //if(fabs(Goal[2] - theta_sum_close) <= 0.1)
		{
			V = v_control(0);
			W = w_control(0);
			for(int j=0; j<8; j++)
			{
				Datasource[0][j]=*(V+j);
				Datasource[1][j]=*(W+j);
			}	
			order_control =3;
			//dis_sum_close=0; theta_sum_close=0;
		}
	}

	*/
}



/*
if(order_control == 3) //step4：小车停止
{
	V = v_control(0);
	W = w_control(0);
	for(int j=0; j<8; j++)
	{
		Datasource[0][j]=*(V+j);
		Datasource[1][j]=*(W+j);
	}
}
*/
//*********************************************************************************************
//*********************************************************************************************
//*********************************************************************************************
//*********************************************************************************************


	//测试***************************
	
	//printf("theta_sum_close=%f, dis_sum_close=%d \n",theta_sum_close,dis_sum_close);
	// dis_sum_close 单位 mm， theta_sum_close 单位 rad
		//can1TX**			
		//if(rownum == sizeof(Datasource)/sizeof(Datasource[0]) ) rownum = 0;////复位计数Datasource 行
		//if(send[0].ID == 0x18C4D6D0) send[0].ID = 0x18C4D1D0;//复位计数ID
		//if(send[0].ID != 0x18C4D1D0) send[0].ID = 0x18C4D1D0;//复位计数ID
		if(rownum>1) rownum=0;
		if(loop_num==20)
		{
			send[0].ID = 0x701;
			loop_num=0;
		} 
		else
		{
		     send[0].ID = 0x601;
		}//复位计数ID,2021.04.03 9:30
		
		if(times >= 5)
		{
			for(i = 0; i < send[0].DataLen; i++)//给数据
			{
				if(send[0].ID == 0x701)
				{
					send[0].Data[i] =Jumpsource[i];
				}
				else
				{
					send[0].Data[i] = Datasource[rownum][i];
				}
				
			}
		}
		else//最后给空挡、刹车
		{

			for(i = 0; i < send[0].DataLen; i++)//给数据
			{
				//Datasource[i]=Datasourcestop[i];//-------2021.4.1 16:36
				send[0].Data[i] = Datasource[0][i];
			}
		}

		rownum++;

		if(VCI_Transmit(VCI_USBCAN2, 0, 0, send, 1) == 1)
		{
			//////////////////////////////////////////////////////////
			//printf(">>Transmit success! \n");
			//printf("CAN1 TX ID:0x%08X \n",send[0].ID);//2021.4.1 20:24
			////////////////////////////////////////////////////////////

			for(int q = 0; q < send[0].DataLen; q++)//给数据
			{
				//////////////////////////////////////////////////
				//printf(" DATA is : %x \n",send[0].Data[q]);
				//////////////////////////////////////////////////
			}


		}
		else
		{
			printf(">>Transmit error!\n");
			break;
		}
		usleep(10000);//延时10ms。
		loop_num++;
		

	}

	usleep(10000000/5);//延时单位us，这里设置 10 000 000=10s    10s后关闭接收线程，并退出主程序。
	m_run0=0;//线程关闭指令。
	pthread_join(threadid,NULL);//等待线程关闭。
	                         ;//延时100ms。
	VCI_ResetCAN(VCI_USBCAN2, 0, 0);//复位CAN1通道。
	usleep(100000);//延时100ms。
	VCI_ResetCAN(VCI_USBCAN2, 0, 1);//复位CAN2通道。
	usleep(100000);//延时100ms。
	VCI_CloseDevice(VCI_USBCAN2,0);//关闭设备。
	//除收发函数外，其它的函数调用前后，最好加个毫秒级的延时，即不影响程序的运行，又可以让USBCAN设备有充分的时间处理指令。
	//goto ext;
}

void *receive_func(void* param)  //接收线程。
{
	int reclen=0;
	VCI_CAN_OBJ rec[3000];//接收缓存，设为3000为佳。
	int i,j;

	//将接收到的速度真存在数组中,数组值8，设置20防溢出报错
	int receive_data[20];
	//********************2021.4.5 16:45
	
	int *run=(int*)param;//线程启动，退出控制。
    	int ind=0;//can1的通道号

	
	while((*run)&0x0f)
	{
		if((reclen=VCI_Receive(VCI_USBCAN2,0,ind,rec,3000,100))>0)//调用接收函数，如果有数据，进行数据处理显示。
		{
			for(j=0;j<reclen;j++)
			{
				/////////////////////////////////////////////////////////////
				//printf("Index:%04d  ",count);count++;//序号递增
				//printf("CAN%d RX ID:0x%08X", ind+1, rec[j].ID);//ID
				//if(rec[j].ExternFlag==0) printf(" Standard ");//帧格式：标准帧
				//if(rec[j].ExternFlag==1) printf(" Extend   ");//帧格式：扩展帧
				//if(rec[j].RemoteFlag==0) printf(" Data   ");//帧类型：数据帧
				//if(rec[j].RemoteFlag==1) printf(" Remote ");//帧类型：远程帧
				//printf("DLC:0x%02X",rec[j].DataLen);//帧长度
				//printf(" data:0x");	//数据
				//////////////////////////////////////////////////////////////


				for(i = 0; i < rec[j].DataLen; i++)
				{
					////////////////////////////////////////////////
					//printf(" %02X", rec[j].Data[i]);
					///////////////////////////////////////////////
					receive_data[i]=rec[j].Data[i];
				}

				//*****************************解析数据，v,w
				if(receive_data[0]==0x43 || receive_data[0]==0x47 || receive_data[0]==0x4B || receive_data[0]==0x4F)
				{
					if(receive_data[1]==0x01 && receive_data[2]==0x21 && receive_data[3]==0x00)//线速度
					{
						//return_v=((receive_data[5]<<8)|receive_data[4]);
						return_v = v_w_get(receive_data[4], receive_data[5]);
						//printf("\n v = %d \n", return_v);
						return_dis = return_v * 0.01 ; //return_v 单位mm/s return_dis 单位mm
						dis_sum_close = dis_sum_close + return_dis;
					}
					if(receive_data[1]==0x02 && receive_data[2]==0x21 && receive_data[3]==0x00)//角速度
					{
						//return_w=((receive_data[5]<<8)|receive_data[4]);
						return_w = v_w_get(receive_data[4], receive_data[5]);
						//return_w = v_w_get(0x0C, 0xfe);
						//printf("\n w = %d \n", return_w);
						return_theta = return_w * 0.01 * 0.001; //return_w 单位mmrad/s return_theta 单位rad
						theta_sum_close = theta_sum_close + return_theta;
					}
				}
				//*****************************2021.4.5
				/////////////////////////////////////////////////////////
				//printf(" TimeStamp:0x%08X",rec[j].TimeStamp);//时间标识。
				/////////////////////////////////////////////////////////
				//printf("\n");

			}
			
		}

	}
	//printf("run thread exit\n");//退出接收线程	
	pthread_exit(0);
}

int xorcheck(int *pdata, int len)
{
	int xori = *pdata;
	while(--len) 
		{
			pdata++;
			xori = xori ^ (*pdata);
		}
	return xori;
}

//************************************************
//给小车速度 2021.4.5 15:09
int* v_control(int v)
{
	static int vv[8];
	int b,c;
	vv[0]=0x23;vv[1]=0x08;vv[2]=0x20;vv[3]=0x00;
	if(v>=0 && v<=2000)
	{
		c=v>>8; b=v&0xff;
		vv[4]=b; vv[5]=c;
		vv[6]=0x00;vv[7]=0x00;
	}
	else if(v<0 && v>=-2000)
	{
		int16 val;
		val=(int16)v;
		uint16 tmp;
		tmp = *((uint16*)&val);
		vv[4]=(u8)tmp;
		vv[5]=(u8)(tmp >> 8);
		vv[6]=0xff;vv[7]=0xff;
	}
	else
	{
		vv[4]=0; vv[5]=0;
	}
	return vv;
}
int* w_control(int w)
{
	
	static int ww[8];
	int b,c;
	ww[0]=0x23;ww[1]=0x09;ww[2]=0x20;ww[3]=0x00;
	if(w>=0 && w<=1500)
	{
		c=w>>8; b=w&0xff;
		ww[4]=b; ww[5]=c;
		ww[6]=0x00;ww[7]=0x00;
	}
	else if(w<0 && w>=-1500)
	{
		int16 val;
		val=(int16)w;
		uint16 tmp;
		tmp = *((uint16*)&val);
		ww[4]=(u8)tmp;
		ww[5]=(u8)(tmp >> 8);
		ww[6]=0xff;ww[7]=0xff;
	}
	else
	{
		ww[4]=0; ww[5]=0;
	}
	return ww;
}
//************************************************
//***********************************************
//接收到的数据代码解析
int16 v_w_get(int data4, int data5)
{
	int16 val;
	uint16 tmp;
	tmp = ((uint16)data5 << 8) | ((uint16)data4);
	val = *((int16*)&tmp);
	return val;
}