//*************************************************************
//这个节点是订阅rrt规划好的路径点，将旧的路径点删掉一部分，发布新的路径点
//（定心转向-直线运动-定心转向）小车沿直线运动，不转弯，避免安全事故，更适合于物流工业等场合
//优化思想：如果一个点与前面的点之间没有障碍物，则删除这个点
//起始点：（2,2），终止点：（95,95）
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

void initializeMarkers2(visualization_msgs::Marker &BSplinePath)
{
    //init headers
    BSplinePath.header.frame_id    = "path_planner";
    BSplinePath.header.stamp       = ros::Time::now();
    BSplinePath.ns                 = "path_planner";
    BSplinePath.action             = visualization_msgs::Marker::ADD;
    BSplinePath.pose.orientation.w = 1.0;

    //setting id for each marker
    BSplinePath.id    = 6;
    
    //defining types
    BSplinePath.type  = visualization_msgs::Marker::LINE_STRIP;
    
    //setting scale
    BSplinePath.scale.x = 0.2;
    
    //assigning colors
    BSplinePath.color.r = 0.0f;
    BSplinePath.color.g = 0.0f;
    BSplinePath.color.b = 0.0f;

    BSplinePath.color.a = 1.0f;
}


vector<geometry_msgs::Point> oldPoint;  //原始点
vector<geometry_msgs::Point> newPoint;  //新点 newPoint[i].x
vector<geometry_msgs::Point> newPoint_atLast;  //新点的新点
vector<geometry_msgs::Point> obstList;  //障碍物
vector<geometry_msgs::Point> BSplinePoints;  //需要做曲线平滑的离散点，要使曲线过起始点和终止点--5.6
vector<geometry_msgs::Point> AfterBSpline;  //平滑之后的点
vector<geometry_msgs::Point> TestBSpline;  //测试用的点
geometry_msgs::Point subpoint;
geometry_msgs::Point lastpoint;
geometry_msgs::Point obstpoint;
geometry_msgs::Point BSpline;  //qu xian ping hua
geometry_msgs::Point jianchaPoint;
visualization_msgs::Marker AnewPath; //显示新的路径
visualization_msgs::Marker BSplinePath; //显示样条曲线拟合之后的路径
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

vector<geometry_msgs::Point> BSplineFit(vector<geometry_msgs::Point> discretePoints, bool closed, double stride = 0.01);

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
    ros::Publisher BSplinePoints_publisher = n.advertise<visualization_msgs::Marker>("/BSpline_new_points", 1);
    //====================================================================================================
    ros::Publisher cancontrol_newpoints = n.advertise<geometry_msgs::Point>("/cancontrol_points", 1);
    //====================================================================================================
    ros::Publisher BSpline_publisher = n.advertise<geometry_msgs::Point>("/BSpline_points", 1);
    //====================================================================================================

	initializeMarkers(AnewPath);
    initializeMarkers2(BSplinePath);
    int flag=1; //标志位，标志是否接收完成
    int NewPointsNum; //一共有多少个新点

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
        NewPointsNum = i+1;
    }

    //给三次B样条曲线赋值
 
    //从第二个控制点开始循环遍历，找到距离曲线控制点较近的障碍物顶点，
    //分别计算这个顶点到上一个控制点的距离dis1和这个控制点到上一个控制点的距离dis2，距离小的放在前面
    // newPoint_atLast.push_back(newPoint[0]);
    // int bbq_flag=1;
    // for(int i=1; i<NewPointsNum; i++)
    // {
    //     float dis_obs_control;
    //     for(int j=0;j<15;j++)
    //     {
    //         if(j==4 || j==9 || j==14)
    //             continue;
    //         dis_obs_control = sqrt(pow(newPoint[i].x - obstList[j].x ,2) + pow(newPoint[i].y - obstList[j].y ,2));
    //         if(dis_obs_control < 14) //距离
    //         {
    //             int dis1,dis2;
    //             dis1 = sqrt(pow(newPoint[i-1].x - obstList[j].x ,2) + pow(newPoint[i-1].y - obstList[j].y ,2));
    //             dis2 = sqrt(pow(newPoint[i-1].x - newPoint[i].x ,2) + pow(newPoint[i-1].y - newPoint[i].y ,2));
    //             if(dis1 > dis2)
    //             {
    //                 newPoint_atLast.push_back(newPoint[i]);
    //                 newPoint_atLast.push_back(obstList[j]);
    //             }
    //             else
    //             {
    //                 newPoint_atLast.push_back(obstList[j]);
    //                 newPoint_atLast.push_back(newPoint[i]);
    //             }
    //             bbq_flag = 0;
    //             break;
    //         }
    //         else
    //             bbq_flag = 1;
    //     }
    //     if(bbq_flag)
    //         newPoint_atLast.push_back(newPoint[i]);
    // }

    //将控制点重新排列
    //略

    BSpline.x = newPoint[0].x - 1;
    BSpline.y = newPoint[0].y - 1;
    BSplinePoints.push_back(BSpline);

    // BSplinePoints[0].x = newPoint[0].x - 1;
    // BSplinePoints[0].y = newPoint[0].y - 1;

    BSplinePoints.push_back(newPoint[0]);

    // BSplinePoints[1] = newPoint[0];

    BSpline.x = newPoint[0].x + 1;
    BSpline.y = newPoint[0].y + 1;
    BSplinePoints.push_back(BSpline);

    // BSplinePoints[2].x = newPoint[0].x + 1;
    // BSplinePoints[2].y = newPoint[0].y + 1;

    for(int i=0; i<(NewPointsNum-2); i++)
    // for(int i=0; i<(newPoint_atLast.size()-2); i++)
    {
        // BSplinePoints[i+3] = newPoint[i+1];
        BSplinePoints.push_back(newPoint[i+1]);
    }

    BSpline.x = newPoint[NewPointsNum-1].x - 1;
    BSpline.y = newPoint[NewPointsNum-1].y - 1;
    BSplinePoints.push_back(BSpline);
    
    // BSplinePoints[1+NewPointsNum].x = newPoint[NewPointsNum-1].x - 1;
    // BSplinePoints[1+NewPointsNum].y = newPoint[NewPointsNum-1].y - 1;

    BSplinePoints.push_back(newPoint[NewPointsNum-1]);

    // BSplinePoints[2+NewPointsNum] = newPoint[NewPointsNum-1];

    BSpline.x = newPoint[NewPointsNum-1].x + 1;
    BSpline.y = newPoint[NewPointsNum-1].y + 1;
    BSplinePoints.push_back(BSpline);

    // BSplinePoints[3+NewPointsNum].x = newPoint[NewPointsNum-1].x + 1;
    // BSplinePoints[3+NewPointsNum].y = newPoint[NewPointsNum-1].y + 1;
    printf("finish give! --%d \n",BSplinePoints.size());
    for(int i=0; i<BSplinePoints.size(); i++)
    {
        printf("point-->%f,%f \n", BSplinePoints[i].x, BSplinePoints[i].y );      
    }
    
    //完成赋值
    AfterBSpline = BSplineFit(BSplinePoints, false, 0.05);

    //检查新点与障碍物有无碰撞
    int obs1,obs2,obs3;
    obs1 = 1;
    obs2 = 1;
    obs3 = 1;
    while(obs1 || obs2 || obs3)
    {
        obs1 = 0;
        obs2 = 0;
        obs3 = 0;
        for (int i = 0; i < AfterBSpline.size(); i++)
        {
            float obsdis1,obsdis2,obsdis3,obsdis4,mindis;
            if((AfterBSpline[i].x>50)&&(AfterBSpline[i].x<80)&&(AfterBSpline[i].y>50)&&(AfterBSpline[i].y<70))
            {
                //与第一个碰撞 1,2,3,4->(50,50)(50,70)(80,50)(80,70)
                obs1 = 1;
                obsdis1 = sqrt(pow(AfterBSpline[i].x - 50,2) + pow(AfterBSpline[i].y - 50 ,2));
                obsdis2 = sqrt(pow(AfterBSpline[i].x - 50,2) + pow(AfterBSpline[i].y - 70 ,2));
                obsdis3 = sqrt(pow(AfterBSpline[i].x - 80,2) + pow(AfterBSpline[i].y - 50 ,2));
                obsdis4 = sqrt(pow(AfterBSpline[i].x - 80,2) + pow(AfterBSpline[i].y - 70 ,2));
                mindis = obsdis1;
                jianchaPoint.x = 50;
                jianchaPoint.y = 50;
                if(obsdis2<mindis)
                {
                    mindis = obsdis2;
                    jianchaPoint.x = 50;
                    jianchaPoint.y = 70;
                }
                if(obsdis3<mindis)
                {
                    mindis = obsdis3;
                    jianchaPoint.x = 80;
                    jianchaPoint.y = 50;
                }
                if(obsdis4<mindis)
                {
                    mindis = obsdis4;
                    jianchaPoint.x = 80;
                    jianchaPoint.y = 70;
                }
                BSplinePoints.push_back(jianchaPoint);
                break;
            }
            if((AfterBSpline[i].x>20)&&(AfterBSpline[i].x<70)&&(AfterBSpline[i].y>5)&&(AfterBSpline[i].y<15))
            {
                //与第二个碰撞 1,2,3,4->(20,5)(70,5)(70,15)(20,15)
                obs2 = 1;
                obsdis1 = sqrt(pow(AfterBSpline[i].x - 20,2) + pow(AfterBSpline[i].y - 5 ,2));
                obsdis2 = sqrt(pow(AfterBSpline[i].x - 70,2) + pow(AfterBSpline[i].y - 5 ,2));
                obsdis3 = sqrt(pow(AfterBSpline[i].x - 70,2) + pow(AfterBSpline[i].y - 15 ,2));
                obsdis4 = sqrt(pow(AfterBSpline[i].x - 20,2) + pow(AfterBSpline[i].y - 15 ,2));
                mindis = obsdis1;
                jianchaPoint.x = 20;
                jianchaPoint.y = 5;
                if(obsdis2<mindis)
                {
                    mindis = obsdis2;
                    jianchaPoint.x = 70;
                    jianchaPoint.y = 5;
                }
                if(obsdis3<mindis)
                {
                    mindis = obsdis3;
                    jianchaPoint.x = 70;
                    jianchaPoint.y = 15;
                }
                if(obsdis4<mindis)
                {
                    mindis = obsdis4;
                    jianchaPoint.x = 20;
                    jianchaPoint.y = 15;
                }
                BSplinePoints.push_back(jianchaPoint);
                break;
            }
            if((AfterBSpline[i].x>10)&&(AfterBSpline[i].x<30)&&(AfterBSpline[i].y>60)&&(AfterBSpline[i].y<80))
            {
                //与第三个碰撞 1,2,3,4->(10,60)(30,60)(30,80)(10,80)
                obs3 = 1;
                obsdis1 = sqrt(pow(AfterBSpline[i].x - 10,2) + pow(AfterBSpline[i].y - 60 ,2));
                obsdis2 = sqrt(pow(AfterBSpline[i].x - 30,2) + pow(AfterBSpline[i].y - 60 ,2));
                obsdis3 = sqrt(pow(AfterBSpline[i].x - 30,2) + pow(AfterBSpline[i].y - 80 ,2));
                obsdis4 = sqrt(pow(AfterBSpline[i].x - 10,2) + pow(AfterBSpline[i].y - 80 ,2));
                mindis = obsdis1;
                jianchaPoint.x = 10;
                jianchaPoint.y = 60;
                if(obsdis2<mindis)
                {
                    mindis = obsdis2;
                    jianchaPoint.x = 30;
                    jianchaPoint.y = 60;
                }
                if(obsdis3<mindis)
                {
                    mindis = obsdis3;
                    jianchaPoint.x = 30;
                    jianchaPoint.y = 80;
                }
                if(obsdis4<mindis)
                {
                    mindis = obsdis4;
                    jianchaPoint.x = 10;
                    jianchaPoint.y = 80;
                }
                BSplinePoints.push_back(jianchaPoint);
                break;
            }
        }  
        if(obs1 || obs2 || obs3) //重新拟合 用BSplinePoints
        {
            float dis,lastdis;
            //mindis = sqrt(pow(BSplinePoints[0].x ,2) + pow(BSplinePoints[0].y ,2));
            //从第二个点开始
            for(int i=1; i<BSplinePoints.size(); i++)
            {
                for(int j=0;j<i;j++)
                {
                    dis = sqrt(pow(BSplinePoints[i-j].x,2) + pow(BSplinePoints[i-j].y,2));
                    lastdis = sqrt(pow(BSplinePoints[i-j-1].x,2) + pow(BSplinePoints[i-j-1].y,2));
                    if(dis<lastdis)
                    {
                        jianchaPoint.x = BSplinePoints[i-j-1].x;
                        jianchaPoint.y = BSplinePoints[i-j-1].y;
                        BSplinePoints[i-j-1].x = BSplinePoints[i-j].x;
                        BSplinePoints[i-j-1].y = BSplinePoints[i-j].y;
                        BSplinePoints[i-j].x = jianchaPoint.x;
                        BSplinePoints[i-j].y = jianchaPoint.y;
                    } 
                }
            }
            AfterBSpline = BSplineFit(BSplinePoints, false, 0.05);
        }

    }
     
//测试===================================================
    
    //     BSpline.x = 2;
    //     BSpline.y = 2;
    //     TestBSpline.push_back(BSpline);
    
    //     BSpline.x = 100;
    //     BSpline.y = 100;
    //     TestBSpline.push_back(BSpline);

    //     BSpline.x = 200;
    //     BSpline.y = 200;
    //     TestBSpline.push_back(BSpline);

    //     BSpline.x = 400; //95
    //     BSpline.y = 400; //95
    //     TestBSpline.push_back(BSpline);
    
    // AfterBSpline = BSplineFit(TestBSpline, false, 0.08);



//-----------------------------------------------------------------------------

    for (int i = 0; i < AfterBSpline.size(); i++)
    {
        BSpline_publisher.publish(AfterBSpline[i]);
        usleep(500000);  //1000000->500000
        printf("No.data->%d, x->%f, y->%f \n", i, AfterBSpline[i].x, AfterBSpline[i].y);
        BSplinePath.points.push_back(AfterBSpline[i]);
    }
    BSpline.x = 95;
    BSpline.y = 95;
    BSpline_publisher.publish(BSpline);
//-------------------------------------------------------------------------------

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
        BSplinePoints_publisher.publish(BSplinePath);
       	usleep(1000000);
    }
    
    return 0;
}

//discretePoints —— 输入的离散点，至少4个
//closed —— 是否拟合闭合曲线，true--闭合，false--不闭合
//stride —— 拟合精度=0.01
// vector<Point2f> BSplineFit(vector<Point2f> discretePoints, bool closed, double stride = 0.01) 
vector<geometry_msgs::Point> BSplineFit(vector<geometry_msgs::Point> discretePoints, bool closed, double stride ) 
{
    vector<geometry_msgs::Point> fittingPoints;
    for (int i = 0; i < (closed ? discretePoints.size() : discretePoints.size() - 3); i++) 
    {
        geometry_msgs::Point xy[4];
        geometry_msgs::Point totalPoints;
        xy[0].x = (discretePoints[i].x + 4 * discretePoints[(i + 1) % discretePoints.size()].x + discretePoints[(i + 2) % discretePoints.size()].x) / 6;
        xy[0].y = (discretePoints[i].y + 4 * discretePoints[(i + 1) % discretePoints.size()].y + discretePoints[(i + 2) % discretePoints.size()].y) / 6;
        xy[1].x = -(discretePoints[i].x - discretePoints[(i + 2) % discretePoints.size()].x) / 2;
        xy[1].y = -(discretePoints[i].y - discretePoints[(i + 2) % discretePoints.size()].y) / 2;
        xy[2].x = (discretePoints[i].x - 2 * discretePoints[(i + 1) % discretePoints.size()].x + discretePoints[(i + 2) % discretePoints.size()].x) / 2;
        xy[2].y = (discretePoints[i].y - 2 * discretePoints[(i + 1) % discretePoints.size()].y + discretePoints[(i + 2) % discretePoints.size()].y) / 2;
        xy[3].x = -(discretePoints[i].x - 3 * discretePoints[(i + 1) % discretePoints.size()].x + 3 * discretePoints[(i + 2) % discretePoints.size()].x - discretePoints[(i + 3) % discretePoints.size()].x) / 6;
        xy[3].y = -(discretePoints[i].y - 3 * discretePoints[(i + 1) % discretePoints.size()].y + 3 * discretePoints[(i + 2) % discretePoints.size()].y - discretePoints[(i + 3) % discretePoints.size()].y) / 6;
        for (double t = 0; t <= 1; t += stride) 
        {
            totalPoints.x = 0;
            totalPoints.y = 0;
            for (int j = 0; j < 4; j++) 
            {
                totalPoints.x += xy[j].x * pow(t, j);
                totalPoints.y += xy[j].y * pow(t, j);
            }
            fittingPoints.push_back(totalPoints);
        }
    }
    return fittingPoints;
}
