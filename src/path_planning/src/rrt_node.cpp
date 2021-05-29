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

#define success false
#define running true

using namespace rrt;

bool status = running;
bool pro_status = running;

void initializeMarkers(visualization_msgs::Marker &sourcePoint,
    visualization_msgs::Marker &goalPoint,
    visualization_msgs::Marker &randomPoint,
    visualization_msgs::Marker &rrtTreeMarker,
    visualization_msgs::Marker &finalPath)
{
    //init headers
	sourcePoint.header.frame_id    = goalPoint.header.frame_id    = randomPoint.header.frame_id    = rrtTreeMarker.header.frame_id    = finalPath.header.frame_id    = "path_planner";
	sourcePoint.header.stamp       = goalPoint.header.stamp       = randomPoint.header.stamp       = rrtTreeMarker.header.stamp       = finalPath.header.stamp       = ros::Time::now();
	sourcePoint.ns                 = goalPoint.ns                 = randomPoint.ns                 = rrtTreeMarker.ns                 = finalPath.ns                 = "path_planner";
	sourcePoint.action             = goalPoint.action             = randomPoint.action             = rrtTreeMarker.action             = finalPath.action             = visualization_msgs::Marker::ADD;
	sourcePoint.pose.orientation.w = goalPoint.pose.orientation.w = randomPoint.pose.orientation.w = rrtTreeMarker.pose.orientation.w = finalPath.pose.orientation.w = 1.0;

    //setting id for each marker
    sourcePoint.id    = 0;
	goalPoint.id      = 1;
	randomPoint.id    = 2;
	rrtTreeMarker.id  = 3;
    finalPath.id      = 4;

	//defining types
	rrtTreeMarker.type                                    = visualization_msgs::Marker::LINE_LIST;
	finalPath.type                                        = visualization_msgs::Marker::LINE_STRIP;
	sourcePoint.type  = goalPoint.type = randomPoint.type = visualization_msgs::Marker::SPHERE;

	//setting scale
	rrtTreeMarker.scale.x = 0.2;
	finalPath.scale.x     = 1;
	sourcePoint.scale.x   = goalPoint.scale.x = randomPoint.scale.x = 2;
    sourcePoint.scale.y   = goalPoint.scale.y = randomPoint.scale.y = 2;
    sourcePoint.scale.z   = goalPoint.scale.z = randomPoint.scale.z = 1;

    //assigning colors
	sourcePoint.color.r   = 1.0f;
	goalPoint.color.g     = 1.0f;
    randomPoint.color.b   = 1.0f;

	rrtTreeMarker.color.r = 0.8f;
	rrtTreeMarker.color.g = 0.4f;

	finalPath.color.r = 0.2f;
	finalPath.color.g = 0.2f;
	finalPath.color.b = 1.0f;

	sourcePoint.color.a = goalPoint.color.a = randomPoint.color.a = rrtTreeMarker.color.a = finalPath.color.a = 1.0f;
}

vector< vector<geometry_msgs::Point> > getObstacles()
{
    obstacles obst;
    return obst.getObstacleArray();
}

void addBranchtoRRTTree(visualization_msgs::Marker &rrtTreeMarker, RRT::rrtNode &tempNode, RRT &myRRT)
{

geometry_msgs::Point point;

point.x = tempNode.posX;
point.y = tempNode.posY;
point.z = 0;
rrtTreeMarker.points.push_back(point);

RRT::rrtNode parentNode = myRRT.getParent(tempNode.nodeID);

point.x = parentNode.posX;
point.y = parentNode.posY;
point.z = 0;

rrtTreeMarker.points.push_back(point);
}

bool checkIfInsideBoundary(RRT::rrtNode &tempNode)
{
    if(tempNode.posX < 0 || tempNode.posY < 0  || tempNode.posX > 100 || tempNode.posY > 100 ) return false;
    else return true;
}

bool checkIfOutsideObstacles(vector< vector<geometry_msgs::Point> > &obstArray, RRT::rrtNode &tempNode)
{
    double AB, AD, AMAB, AMAD;

    for(int i=0; i<obstArray.size(); i++)
    {
    	//=============================================================================================================
    	for(int j=0; j<11; j=j+5) //j小于障碍物的数量
    	{
    	//=============================================================================================================
        AB = (pow(obstArray[i][j].x - obstArray[i][j+1].x,2) + pow(obstArray[i][j].y - obstArray[i][j+1].y,2));
        AD = (pow(obstArray[i][j].x - obstArray[i][j+3].x,2) + pow(obstArray[i][j].y - obstArray[i][j+3].y,2));
        AMAB = (((tempNode.posX - obstArray[i][j].x) * (obstArray[i][j+1].x - obstArray[i][j].x)) + (( tempNode.posY - obstArray[i][j].y) * (obstArray[i][j+1].y - obstArray[i][j].y)));
        AMAD = (((tempNode.posX - obstArray[i][j].x) * (obstArray[i][j+3].x - obstArray[i][j].x)) + (( tempNode.posY - obstArray[i][j].y) * (obstArray[i][j+3].y - obstArray[i][j].y)));
        //(0<AM⋅AB<AB⋅AB)∧(0<AM⋅AD<AD⋅AD)
        	if((0 < AMAB) && (AMAB < AB) && (0 < AMAD) && (AMAD < AD))
        	{
            	return false;
        	}
    	}
    } 
    return true;
}

void generateTempPoint(RRT::rrtNode &tempNode, int goal_x, int goal_y)
{
    int x = rand() % 150 + 1;  //产生1-150之间的随机数
    int y = rand() % 150 + 1;  //产生1-150之间的随机数
    //std::cout<<"Random X: "<<x <<endl<<"Random Y: "<<y<<endl;
    if(x<20) //13%的概率
    {
        tempNode.posX = goal_x;
        tempNode.posY = goal_y;
    }
    else
    {
        tempNode.posX = x;
        tempNode.posY = y;
    }
}


//判断目标点在分割的地图的哪一部分,假设地图是方的,在地图中心建立直角坐标系
int judge_goalPoint(int bondx, int bondy, int goal_x, int goal_y)
{
	int result;
	int median_x, median_y;
	median_x = bondx/2;
	median_y = bondy/2;
	if( (goal_x > median_x)&&(goal_y > median_y) )
		result = 1;
	else if( (goal_x < median_x)&&(goal_y > median_y) )
		result = 2;
	else if( (goal_x < median_x)&&(goal_y < median_y) )
		result = 3;
	else if( (goal_x > median_x)&&(goal_y < median_y) )
		result = 4;
	return result;
}





bool addNewPointtoRRT(RRT &myRRT, RRT::rrtNode &tempNode, int rrtStepSize, vector< vector<geometry_msgs::Point> > &obstArray, int PointWhere)
{
    int nearestNodeID = myRRT.getNearestNodeID(tempNode.posX,tempNode.posY);

    RRT::rrtNode nearestNode = myRRT.getNode(nearestNodeID);

    int nearestPosition;
    nearestPosition = judge_goalPoint(100,100,(int)nearestNode.posX,(int)nearestNode.posY);

    double theta = atan2(tempNode.posY - nearestNode.posY,tempNode.posX - nearestNode.posX);

    if(nearestPosition==PointWhere)
    {
    	tempNode.posX = nearestNode.posX + (rrtStepSize * cos(theta));
    	tempNode.posY = nearestNode.posY + (rrtStepSize * sin(theta));
    }
    else
    {
    	tempNode.posX = nearestNode.posX + (5 * cos(theta));
    	tempNode.posY = nearestNode.posY + (5 * sin(theta));
    	// tempNode.posX = nearestNode.posX + (rrtStepSize * cos(theta));
    	// tempNode.posY = nearestNode.posY + (rrtStepSize * sin(theta));
    }
    

    if(checkIfInsideBoundary(tempNode) && checkIfOutsideObstacles(obstArray,tempNode))
    {
        tempNode.parentID = nearestNodeID;
        tempNode.nodeID = myRRT.getTreeSize();
        myRRT.addNewNode(tempNode);
        return true;
    }                    //检查障碍物========
    else
        return false;
}

bool checkNodetoGoal(int X, int Y, RRT::rrtNode &tempNode)
{
    double distance = sqrt(pow(X-tempNode.posX,2)+pow(Y-tempNode.posY,2));
    if(distance < 3)
    {
        return true;
    }
    return false;
}

void setFinalPathData(vector< vector<int> > &rrtPaths, RRT &myRRT, int i, visualization_msgs::Marker &finalpath, int goalX, int goalY)
{
    RRT::rrtNode tempNode;
    geometry_msgs::Point point;

    //===========================================================================
    //geometry_msgs::Point point_pub;
    //===========================================================================

    for(int j=0; j<rrtPaths[i].size();j++)
    {
        tempNode = myRRT.getNode(rrtPaths[i][j]);

        point.x = tempNode.posX;
        point.y = tempNode.posY;
        point.z = 0;

        finalpath.points.push_back(point);

        //=======================================================================
        //point_pub.x = tempNode.posX;
        //point_pub.y = tempNode.posY;
        //point_pub.z = 0;
        //turtle_vel_pub.publish(vel_msg);
        //=======================================================================
    }

    point.x = goalX;
    point.y = goalY;
    finalpath.points.push_back(point);
}



int main(int argc,char** argv)
{
    //initializing ROS
    ros::init(argc,argv,"rrt_node");
	ros::NodeHandle n;

	//defining Publisher
	ros::Publisher rrt_publisher = n.advertise<visualization_msgs::Marker>("path_planner_rrt",1);
    //====================================================================================================
    ros::Publisher points_publisher = n.advertise<geometry_msgs::Point>("/planning_rrt_points", 1);
    //====================================================================================================
    //====================================================================================================
    ros::Publisher obstpoints_publisher = n.advertise<geometry_msgs::Point>("/obstacle_points", 1);
    //====================================================================================================
	//defining markers
    visualization_msgs::Marker sourcePoint;
    visualization_msgs::Marker goalPoint;
    visualization_msgs::Marker randomPoint;
    visualization_msgs::Marker rrtTreeMarker;
    visualization_msgs::Marker finalPath;

    geometry_msgs::Point obstic_pub;

    initializeMarkers(sourcePoint, goalPoint, randomPoint, rrtTreeMarker, finalPath);

    //setting source and goal
    sourcePoint.pose.position.x = 2;
    sourcePoint.pose.position.y = 2;

    goalPoint.pose.position.x = 95;
    goalPoint.pose.position.y = 95;

    rrt_publisher.publish(sourcePoint);
    rrt_publisher.publish(goalPoint);
    ros::spinOnce();
    ros::Duration(0.01).sleep();

    srand (time(NULL));
    //initialize rrt specific variables

    //initializing rrtTree
    RRT myRRT(2.0,2.0);
    int goalX, goalY;
    goalX = goalY = 95;

    int rrtStepSize = 3;

    vector< vector<int> > rrtPaths;
    vector<int> path;
    int rrtPathLimit = 1;

    int shortestPathLength = 9999;
    int shortestPath = -1;

    RRT::rrtNode tempNode;

    vector< vector<geometry_msgs::Point> >  obstacleList = getObstacles();


    bool addNodeResult = false, nodeToGoal = false;


    obstic_pub.x=0;
    obstic_pub.y=0;
    obstic_pub.z=0;
    obstpoints_publisher.publish(obstic_pub);
    usleep(1000000);
    for(int i=0;i<15;i++)
    {
        obstic_pub.x=obstacleList[0][i].x;
        obstic_pub.y=obstacleList[0][i].y;
        obstic_pub.z=0;
        obstpoints_publisher.publish(obstic_pub);
        printf("ok!--%d, x:%f y:%f \n", i, obstic_pub.x, obstic_pub.y );
        ros::spinOnce();
        usleep(1000000);
    }

   int GoalPointInWhere;
   GoalPointInWhere = judge_goalPoint(100,100,goalX,goalY);

    while(ros::ok() && status)
    {

        

        if(rrtPaths.size() < rrtPathLimit)
        {
            generateTempPoint(tempNode,goalX,goalY);
            //std::cout<<"tempnode generated"<<endl;
            addNodeResult = addNewPointtoRRT(myRRT,tempNode,rrtStepSize,obstacleList,GoalPointInWhere);
            if(addNodeResult)
            {
               // std::cout<<"tempnode accepted"<<endl;
                addBranchtoRRTTree(rrtTreeMarker,tempNode,myRRT);
               // std::cout<<"tempnode printed"<<endl;
                nodeToGoal = checkNodetoGoal(goalX, goalY,tempNode);
                if(nodeToGoal)
                {
                    path = myRRT.getRootToEndPath(tempNode.nodeID);
                    rrtPaths.push_back(path);
                    std::cout<<"New Path Found. Total paths "<<rrtPaths.size()<<endl;
                    //ros::Duration(10).sleep();
                    //std::cout<<"got Root Path"<<endl;
                }
            }
        }
        else //if(rrtPaths.size() >= rrtPathLimit)
        {
            status = success;
            std::cout<<"Finding Optimal Path"<<endl;
            for(int i=0; i<rrtPaths.size();i++)
            {
                if(rrtPaths[i].size() < shortestPath)
                {
                    shortestPath = i;
                    shortestPathLength = rrtPaths[i].size();
                }
            }
            setFinalPathData(rrtPaths, myRRT, shortestPath, finalPath, goalX, goalY);
            rrt_publisher.publish(finalPath);
//========================================================================================
            geometry_msgs::Point point_pub;
            for(int j=0; j<rrtPaths[shortestPath].size();j++)
            {
               tempNode = myRRT.getNode(rrtPaths[shortestPath][j]);              
                point_pub.x = tempNode.posX;
                point_pub.y = tempNode.posY;
                point_pub.z = 0;
                points_publisher.publish(point_pub);
                usleep(100000);
                ROS_INFO("No.%d----Publsh:%f-x, %f-y, %f-z", 
                    j,point_pub.x, point_pub.y, point_pub.z);
            }
            point_pub.x = goalX;
            point_pub.y = goalY;
            point_pub.z = 0;
            points_publisher.publish(point_pub);
            ROS_INFO("Publsh:%f-x, %f-y, %f-z \n", 
                point_pub.x, point_pub.y, point_pub.z);
//========================================================================================
        }


        rrt_publisher.publish(sourcePoint);
        rrt_publisher.publish(goalPoint);
        rrt_publisher.publish(rrtTreeMarker);
        //rrt_publisher.publish(finalPath);
        ros::spinOnce();
        ros::Duration(0.01).sleep();
    }
    
	return 1;
}
