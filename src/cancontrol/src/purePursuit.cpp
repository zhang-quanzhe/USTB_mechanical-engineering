//输入：小车初始位置，跟踪点
//输出：小车的线速度，角速度  
//纯跟踪法
#include <sys/types.h>
#include <sys/stat.h>
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

vector<geometry_msgs::Point> RefPos;  //跟踪点
vector<geometry_msgs::Point> pos_actual;  //小车实际位置
geometry_msgs::Point pos;  //小车位置
geometry_msgs::Point lookaheadPoint;
float targetSpeed = 0.2; //目标距离 m/s
float Kv = 0.1; //前视距离系数
float Kp = 0.8; //速度P控制器系数
float Ld0 = 0.125; //预瞄距离下限值
float dt = 0.1; //时间间隔

int findaheadPoint(geometry_msgs::Point pos, float v, vector<geometry_msgs::Point> RefPos, float Kv, float Ld0)
{
	geometry_msgs::Point minPoint;
	float minDistence = 100;
	float dis = 0;
	int idx,idx_target;
	for(int i=0; i<RefPos.size(); i++)
	{
		dis = sqrt(pow(pos.x - RefPos[i].x,2)+pow(pos.y - RefPos[i].y,2));
		if(dis<minDistence)
		{
			minDistence = dis;
			minPoint.x = RefPos[i].x;
			minPoint.y = RefPos[i].y;
			idx = i;
		}
	}
	float L_steps = 0; // 参考轨迹上几个相邻点的累计距离
	float Ld; //预瞄距离
	Ld = Kv*v + Ld0;
	while((L_steps<Ld) && (idx<RefPos.size()))
	{
		L_steps = L_steps + sqrt(pow(RefPos[idx+1].x - RefPos[idx].x,2)+pow(RefPos[idx+1].y - RefPos[idx].y,2));
		idx = idx + 1;
	}
	idx_target = idx;
	return idx_target;
}
float pure_pursuit_control(geometry_msgs::Point lookaheadPoint, int idx_target, geometry_msgs::Point pos, 
		float heading, float v, vector<geometry_msgs::Point> RefPos, float Kv, float Ld0)
{
	geometry_msgs::Point Point_temp;
	float w_control;
	float alpha;
	float Ld;
	float R;

	if(idx_target<RefPos.size())
	{
		Point_temp.x = lookaheadPoint.x;
		Point_temp.y = lookaheadPoint.y;
	}
	else
	{
		Point_temp.x = RefPos.back().x;
		Point_temp.y = RefPos.back().y;
	}
	alpha = atan2(Point_temp.y-pos.y,Point_temp.x-pos.x) - heading;
	Ld = Kv*v + Ld0;

	R = Ld/(2*sin(alpha));
	w_control = v/R;

	return w_control;
}	//rad/s


//主程序
int main(int argc,char** argv)
{
	//相关参数定义
	float v;
	float heading;
	float heading_actual;
	float v_actual;
	int idx_target;
	float w_control;
	float a;
	int fuzhi_v;
	int fuzhi_w;
	
	//初始状态定义
	pos.x = RefPos[0].x;
	pos.y = RefPos[0].y;
	v = 0;
	heading = 0;

	//初始状态写入实际状态->画图
	// pos_actual.push_back(pos);
	// heading_actual = heading;
	// v_actual = v;
	idx_now = 0;

	// 循环遍历
	while(idx_now < RefPos.size()-1)
	{
		//---------------------------------------------------------
		idx_now = findaheadPoint(pos, v, RefPos, Kv, Ld0);
		lookaheadPoint.x = RefPos[idx_now].x;
		lookaheadPoint.y = RefPos[idx_now].y;
		//---------------------------------------------------------
		w_control = pure_pursuit_control(lookaheadPoint, idx_now, pos, heading, v, RefPos, Kv, Ld0);
		//---------------------------------------------------------
		a = Kp*(targetSpeed-v)/dt;
		//---------------------------------------------------------

		if(v<0.1) fuzhi_v=100;
		if(v>0.2) fuzhi_v=200;
		else fuzhi_v=v*1000.0;
		V = v_control(fuzhi_v);
		// w_folow = 1000*w_folow;
		if(w_control<0) fuzhi_w=-300;
		if(w_control>0) fuzhi_w=300;
		if(w_control==0) fuzhi_w=0;
		W = w_control(fuzhi_w);

		pos.x = pos.x + v*cos(heading)*dt;
		pos.y = pos.y + v*sin(heading)*dt;
		heading = heading + w_control*dt;
		v = v + a*dt;
		//----------------------------------------------------------
		pos_actual.push_back(pos);
	}


	return 0;
}