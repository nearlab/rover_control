#include <ros/ros.h>
#include <time.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Header.h>
#include <tf/transform_datatypes.h>
#include "quaternion.h"
#include "kinova_driver/kinova_comm.h"
#include <iostream>
#include <unistd.h>
#include <cstdlib>
#include <math.h>
#include <stdio.h>
#include <nav_msgs/Odometry.h>
//#include "orbot_utils.h"
#include <serial/serial.h>//to install, use sudo apt-get install libserial-dev
#include <actionlib/client/simple_action_client.h>
#include <kinova_msgs/ArmPoseAction.h>
#include <kinova_msgs/HomeArm.h>
#include <kinova_msgs/HomeArmRequest.h>
#include <mutex>
#include <csignal>
#include <tf2/LinearMath/Quaternion.h>
using namespace kinova;

ros::Subscriber armcmd;
ros::Subscriber velcmd, viconspeed;
ros::Publisher odom_pub;
bool update = false;

const float pi=3.14159265359;

geometry_msgs::Twist goalVelocity;
bool GoalTimeout = false;
ros::Time prevGoal;

KinovaPose position;
//Orbot orbot;
ros::Time armTime;
ros::Duration armDelay(1);
kinova_msgs::ArmPoseGoal goal;



void armCallback(geometry_msgs::PoseStamped pose)
{
	//double r,p,y;
	tf::Quaternion quat;
	std_msgs::Header header;
	header.frame_id = "j2n6s300_link_base";
	double x,y,z;
	goal.pose.header = pose.header;
	x = pose.pose.position.x;
	y = pose.pose.position.y;
	z = pose.pose.position.z;
	double range = sqrt(x*x + y*y + z*z);
	double mult = 0.5/range;
	tf2::Quaternion myQuaternion;
    myQuaternion.setRPY( 1.57, x*1.57/range, 0 ); 
	goal.pose.pose.position.x = mult*x;
	goal.pose.pose.position.y = mult*y;
	goal.pose.pose.position.z = 1.2*mult*z;
	goal.pose.pose.orientation.x = myQuaternion.getX();//pose.pose.orientation.x;
	goal.pose.pose.orientation.y = myQuaternion.getY();
	goal.pose.pose.orientation.z = myQuaternion.getZ();
	goal.pose.pose.orientation.w = myQuaternion.getW();
	update = true;
}


int main(int argc, char** argv){
	//signal(SIGINT,int_handler);
	ros::init(argc,argv,"orbot_client");
	ros::NodeHandle nh;
	armcmd = nh.subscribe("/tag_pose_base_frame",10,armCallback);
	//armcmd = nh.subscribe("/orbot/arm_pose",10,armCallback);
	ros::Rate loop_rate(10);//TODO: split up the updating and writing rates with time class
	
	ros::Time prevWrite=ros::Time::now();
	ros::Time start_time=ros::Time::now();
	ros::Duration writeDelay(.1);
	ros::Duration goalDelay(2);
	ros::Duration d;
	
	
	actionlib::SimpleActionClient<kinova_msgs::ArmPoseAction> ac("/j2n6s300_driver/pose_action/tool_pose", true);
	ROS_INFO("Waiting for action server to start.");
	ac.waitForServer(); //will wait for infinite time
	ROS_INFO("Action server started, sending goal.");
	ros::AsyncSpinner spinner(5);
	int writecounter = 0;
	while(ros::ok()){
		
		bool write=false;
		
		if(ros::Time::now()-armTime>armDelay && update)
		{
			update = false;
			ac.cancelGoal();
			ac.sendGoal(goal);
			armTime = ros::Time::now();
			bool finished_before_timeout = ac.waitForResult(ros::Duration(3.0));		
			if (finished_before_timeout)
		  		{
		    	actionlib::SimpleClientGoalState state = ac.getState();
		    	ROS_INFO("Action finished: %s",state.toString().c_str());
		  		}
		  	else
		    	ROS_INFO("Action did not finish before the time out.");
		}

		ros::spinOnce();
		loop_rate.sleep();
	}
	
	return 0;
}