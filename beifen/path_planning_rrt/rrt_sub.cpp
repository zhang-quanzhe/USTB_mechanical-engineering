//*************************************************************
//这个节点是订阅rrt规划好的路径点，将旧的路径点删掉一部分，发布新的路径点
//（定心转向-直线运动-定心转向）小车沿直线运动，不转弯，避免安全事故，更适合于物流工业等场合
//优化思想：如果一个点与前面的点之间没有障碍物，则删除这个点
//*************************************************************

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <path_planning/rrt.h>
#include <path_planning/obstacles.h>
#include <iostream>
#include <cmath>
#include <math.h>
#include <stdlib.h>
#include <unistd.h>
#include <vector>
#include <time.h>

using namespace rrt;

void initializeMarkers(visualization_msgs::Marker &newPath)
{
    //init headers
	newPath.header.frame_id    = "path_planner";
	newPath.header.stamp       = ros::Time::now();
	newPath.ns                 = "path_planner";
	newPath.action             = visualization_msgs::Marker::ADD;
	newPath.pose.orientation.w = 1.0;

    //setting id for each marker
    newPath.id    = 5;
	
	//defining types
	newPath.type  = visualization_msgs::Marker::LINE_STRIP;
	
	//setting scale
	newPath.scale.x = 0.2;
	
    //assigning colors
	newPath.color.r = 0.0f;
	newPath.color.g = 1.0f;
	newPath.color.b = 0.0f;

	newPath.color.a = 1.0f;
}


vector<geometry_msgs::Point> oldPoint;  //原始点
vector<geometry_msgs::Point> newPoint;  //新点 newPoint[i].x
vector<geometry_msgs::Point> obstList;  //障碍物
geometry_msgs::Point subpoint;
geometry_msgs::Point lastpoint;
geometry_msgs::Point obstpoint;
visualization_msgs::Marker AnewPath; //显示新的路径
//geometry_msgs::Point AnewPoint;

int sub_num=0;
int sub_num2=0;

// 接收到订阅的消息后，会进入消息回调函数
void rrtInfoCallback(const geometry_msgs::Point::ConstPtr& msg)
{
    subpoint.x= msg->x;
    subpoint.y= msg->y;
    subpoint.z= 0; 
    oldPoint.push_back(subpoint);
        
    // 将接收到的消息打印出来
    ROS_INFO("Subcribe Info+++++%d: x--%f, y--%f \n", sub_num,msg->x, msg->y);
    sub_num++;
}
void obstInfoCallback(const geometry_msgs::Point::ConstPtr& msgg)
{
    obstpoint.x= msgg->x;
    obstpoint.y= msgg->y;
    obstpoint.z= msgg->z;
    obstList.push_back(obstpoint);
    
    printf("oooooooooooooooooh!!!!! --%d\n ",sub_num2);
    sub_num2++;
}

int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "rrt_sub");

    // 创建节点句柄
    ros::NodeHandle n;

    // 创建一个Subscriber，订阅名为/person_info的topic，注册回调函数personInfoCallback
    ros::Subscriber rrt_info_sub = n.subscribe("/planning_rrt_points", 10, rrtInfoCallback);
    // 创建一个Subscriber，订阅名为/person_info的topic，注册回调函数personInfoCallback
    ros::Subscriber obst_info_sub = n.subscribe("/obstacle_points", 10, obstInfoCallback);
	//====================================================================================================
    ros::Publisher newpoints_publisher = n.advertise<visualization_msgs::Marker>("/planning_new_points", 1);
    //====================================================================================================
    ros::Publisher cancontrol_newpoints = n.advertise<geometry_msgs::Point>("/cancontrol_points", 1);
    //====================================================================================================

	initializeMarkers(AnewPath);
    int flag=1; //标志位，标志是否接收完成

    while(ros::ok() && flag)
    {
        if((subpoint.x==95)&&(subpoint.y==95))
        {
            flag=0;
        }
        // 循环等待回调函数
        ros::spinOnce();
    }
    for(int i=0;i<15;i++)
    {
        printf("x:---%f, y:---%f \n", obstList[i].x, obstList[i].y);
    }
 


    newPoint.push_back(oldPoint[0]);
//**************************************************************封装
    //vector< vector<geometry_msgs::Point> >  obstList = getObstacles_sub();
    double P0P1[2], P0A[2], P0B[2], AP0[2], AP1[2], AB[2]; //存向量的数组
    int newpoint_num=1; //存放新点个数(函数之前存在一个起始点)
    int flag1=0; int flag2=0;
    int ifcross=0;
    //lastpoint 为 P0
    //P1 为遍历的点
    lastpoint.x = oldPoint[0].x;
    lastpoint.y = oldPoint[0].y;
    for(int i=1;i<sub_num;i++) //从第二个点开始
    {
        //当前点：oldPoint[i]
        ifcross=0; 
        //判断线段是否与障碍物相交
        //叉乘 a(x1,y1) x b(x2,y2) = (x1y2-x2y1)
        //相交 (P0P1 X P0A)*(P0P1 X P0B)<0 && (AB X AP0)*(AB X AP1)<0
        //           flag1                            flag2
        //P0(oldPoint[i-1].x,oldPoint[i-1].y)
        P0P1[0] = oldPoint[i].x - lastpoint.x; //x
        P0P1[1] = oldPoint[i].y - lastpoint.y; //y
        for(int j=0; j<11; j=j+5) 
        {
            for(int no=0; no<4; no++)
            {
            P0A[0] = obstList[j+no].x - lastpoint.x;
            P0A[1] = obstList[j+no].y - lastpoint.y;
            P0B[0] = obstList[j+1+no].x - lastpoint.x;
            P0B[1] = obstList[j+1+no].y - lastpoint.y;
            AP0[0] = lastpoint.x - obstList[j+no].x;
            AP0[1] = lastpoint.y - obstList[j+no].y;
            AP1[0] = oldPoint[i].x - obstList[j+no].x;
            AP1[1] = oldPoint[i].y - obstList[j+no].y;
            AB[0] = obstList[j+1+no].x - obstList[j+no].x;
            AB[1] = obstList[j+1+no].y - obstList[j+no].y;
            flag1 = (P0P1[0]*P0A[1]-P0A[0]*P0P1[1]) * (P0P1[0]*P0B[1]-P0B[0]*P0P1[1]);
            flag2 = (AB[0]*AP0[1]-AP0[0]*AB[1]) * (AB[0]*AP1[1]-AP1[0]*AB[1]);
                if(flag1<0 && flag2<0)
                {
                    newPoint.push_back(oldPoint[i-1]);
                    lastpoint.x = oldPoint[i-1].x;
                    lastpoint.y = oldPoint[i-1].y;
                    newpoint_num++;
                    ifcross = 1;
                    //相交了
                    break;
                }

                                 
                
            }
            if(ifcross == 1)
                break;
        }

    }


    newPoint.push_back(oldPoint[sub_num-1]);

    for(int i=0;i<(newpoint_num+1);i++)
    {
    	AnewPath.points.push_back(newPoint[i]);      	
        printf("New number is %d : x--%f, y--%f \n", i, newPoint[i].x, newPoint[i].y );
    }
//*****************************************************************
    /*
    for(int i=0;i<sub_num;i++)
    {
        printf("number is %d : x--%f, y--%f \n", i, oldPoint[i].x, oldPoint[i].y );
    }
    */
    /*
    for(int i=0;i<newPoint.size();i++)
    {
        printf("**** x--%f ***** y--%f *****\n",newPoint[i].x, newPoint[i].y);
    }
    */
    for(int i=0;i<(newpoint_num+1);i++)
    {
    	cancontrol_newpoints.publish(newPoint[i]);
    	usleep(1000000);
    }

    while(ros::ok())
    {
    	newpoints_publisher.publish(AnewPath);
       	usleep(1000000);
    }
    
    return 0;
}


