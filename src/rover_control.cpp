/*
DESCRIPTION: 

FUNCTIONS:
void 	messageCallbackVicon(geometry_msgs::TransformStamped)
	Subscribes to the orbot object on vicon at /vicon/orbot/orbot and assigns the pose to loc and att
void	messageCallbackTarget(geometry_msgs::TransformStamped)
	Subscribes to the target on /orbot_server/target and assigns the target pose to tarLoc and tarAtt
int 	main(int, char**)
	Starts the subscribers and publisher, publishes delta between target and current pose to the /orbot_server/orbot_delta

NOTES:
	1. 	Could try and ensure rotation is mostly taken care of by the time that the bot reaches within a certain radius of its goal, 
		then fix rotation and translation separately until it converges. This involves giving rotation a superficially large priority based
		on distance to the goal (pretty much a multiplier, just need to ensure it's less than 42/14 times y at all times).
	2.	Could also attempt to have it run regularly until it's less than .5 meters out then go to translation and rotation separately
*/
#include <ros/ros.h>
#include <time.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
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
using namespace kinova;

ros::Subscriber armcmd;
ros::Subscriber velcmd, viconspeed;
bool update = false;

const float pi=3.14159265359;
const float wMax=122*2*pi/60;
const float vMax=.0762*wMax/1.424;// m/s
float rates[4], real_vel[3];

geometry_msgs::Twist goalVelocity;
bool GoalTimeout = false;
ros::Time prevGoal;

KinovaPose position;
//Orbot orbot;
ros::Time armTime;
ros::Duration armDelay(0.1);
kinova_msgs::ArmPoseGoal goal;


void getRotationRates(float* rates,float* goal_vel, float* real_vel, float x_cm=0, float y_cm=0, float length=.04, float width=.037, float radius=.0076);




void armCallback(geometry_msgs::Pose pose)
{
	double r,p,y;
	tf::Quaternion quat;
	std_msgs::Header header;
	header.frame_id = "m1n6s300_link_base";
	
	goal.pose.header = header;
	goal.pose.pose.position.x = pose.position.x;
	goal.pose.pose.position.y = pose.position.y;
	goal.pose.pose.position.z = pose.position.z;
	goal.pose.pose.orientation.x = pose.orientation.x;
	goal.pose.pose.orientation.y = pose.orientation.y;
	goal.pose.pose.orientation.z = pose.orientation.z;
	goal.pose.pose.orientation.w = pose.orientation.w;
	update = true;
}


void viconCallback(nav_msgs::Odometry roverspeed)
{

	real_vel[0] = roverspeed.twist.twist.linear.x;
	real_vel[1] = roverspeed.twist.twist.linear.y;
	real_vel[2] = roverspeed.twist.twist.angular.z;
	ROS_INFO("Vicon");

}

void velocityCallback(geometry_msgs::Twist twist)
{
	float goal_vel[3];
	goal_vel[0] = twist.linear.x;
	goal_vel[1] = twist.linear.y;
	goal_vel[2] = twist.angular.z;
	getRotationRates(rates,goal_vel,real_vel);
	for(int i=0;i<4;i++){
	rates[i]=rates[i]/pi/2*60;
		if(rates[i]>120)
			rates[i]=120;
		if(rates[i]<-120)
			rates[i]=-120;
		rates[i]=round(rates[i]/122*700);
		if(rates[i]<100 && rates[i]>-100)
			rates[i] = 0;
	}

	prevGoal = ros::Time::now();
	GoalTimeout = false;
	//std::cout<<twist.linear.x-real_vel[0]<<" "<<twist.linear.y-real_vel[1]<<" "<<twist.angular.z-real_vel[2]<<std::endl;

}

void writeToPort(serial::Serial *ser, char* write)
{
	bool written=false;
	int count = 0;
	while(!written && count<10){
		count++;
		try{
			ser->write(write);
			written=true;
			std::cout<<"wrote\n";
		}
		catch(std::exception& e){
			std::cout<<"exception\n"<<count;
			std::cerr<<e.what();
			continue;
		}
		
	}
	if (written==false)
	{
		ser->close();
		ser->open();
		std::cout<<"Could not write\n";
	}
}

void writeDeltas(serial::Serial *ser1, serial::Serial *ser2, float *rates){
		
		//TODO: make this more straightforward
		char output_buffer[64];
		int len=sprintf(output_buffer,"!g 1 %d\r",(int)rates[0]);
		char* write =(char*) malloc(len+1);
		strncpy(write,output_buffer,len);
		write[len]='\0';
		std::cout<<"write to port 1\n";
		writeToPort(ser1,write);
		std::cout<<"wrote 1";//<<write<<"\n";

		len=sprintf(output_buffer,"!g 2 %d\r",(int)rates[1]);
		write =(char*) malloc(len+1);
		strncpy(write,output_buffer,len);
		write[len]='\0';
		std::cout<<"write to port 2\n";
		writeToPort(ser1,write);
		//std::cout<<"wrote "<<write<<"\n";
		std::cout<<"wrote 2";
		
		len=sprintf(output_buffer,"!g 1 %d\r",(int)rates[2]);
		write =(char*) malloc(len+1);
		strncpy(write,output_buffer,len);
		write[len]='\0';
		std::cout<<"write to port 3\n";
		writeToPort(ser2,write);
		//std::cout<<"wrote "<<write<<"\n";
		std::cout<<"wrote 3";

		len=sprintf(output_buffer,"!g 2 %d\r",(int)rates[3]);
		write =(char*) malloc(len+1);
		strncpy(write,output_buffer,len);
		write[len]='\0';
		std::cout<<"write to port 4\n";
		writeToPort(ser2,write);
		//std::cout<<"wrote "<<write<<"\n";
		std::cout<<"wrote 4"<<std::endl;
}


int main(int argc, char** argv){
	//signal(SIGINT,int_handler);
	ros::init(argc,argv,"orbot_client");
	ros::NodeHandle nh;
	//viconSub=nh.subscribe("/vicon/Orbot/Orbot",1000,messageCallbackVicon);
	velcmd = nh.subscribe("/orbot/cmd_vel",10,velocityCallback);
	viconspeed = nh.subscribe("/orbot_rover/local_odom",10,viconCallback);
	armcmd = nh.subscribe("/orbot/arm_pose",10,armCallback);
	ros::Rate loop_rate(10);//TODO: split up the updating and writing rates with time class
	
	
	ros::Time prevWrite=ros::Time::now();
	ros::Time start_time=ros::Time::now();
	ros::Duration writeDelay(.1);
	ros::Duration goalDelay(0.5);
	ros::Duration d;
	
	double t;
	serial::Serial *ser1;//("/dev/ttyACM0");
	serial::Serial *ser2;//("/dev/ttyACM1");
	ser1 = new serial::Serial();
	ser2 = new serial::Serial();
	ser1->setPort("/dev/ttyACM0");
	ser1->open();
	ser1->setBaudrate(115200);
	
	ser2->setPort("/dev/ttyACM1");
	ser2->open();
	ser2->setBaudrate(115200);
	
	actionlib::SimpleActionClient<kinova_msgs::ArmPoseAction> ac("/m1n6s300_driver/pose_action/tool_pose", true);
	ROS_INFO("Waiting for action server to start.");
	ac.waitForServer(); //will wait for infinite time
	ROS_INFO("Action server started, sending goal.");
	//ros::AsyncSpinner spinner(5);
	while(ros::ok()){
		
		//spinner.start();
		bool write=false;
		//std::cout<<"Still Running\n";
		if(ros::Time::now()-prevGoal>goalDelay)
			GoalTimeout = true;

		if(ros::Time::now()-prevWrite>writeDelay){ //&& !GoalTimeout){
			
			write=false;
			std::cout<<"Going to write deltas\n";
			writeDeltas(ser1,ser2,rates); //Rover Uncomment
			prevWrite=ros::Time::now();
			std::cout<<"Wrote speeds: "<<rates[0]<<" "<<rates[1]<<" "<<rates[2]<<" "<<rates[3]<<"\n";
		}
		
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
	//spinner.stop();
	ser1->close();// Rover Uncomment
	ser2->close();// Rover Uncomment
	return 0;
}



void getRotationRates(float* rates, float* goal_vel,float* real_vel, float x_cm, float y_cm, float length, float width, float radius)
{
	const float pi=3.14159265359;
	float wheel_locs[2][4];
	for(uint8_t i=0;i<4;i++){
		wheel_locs[0][i]=((i&2)>0?-length/2:length/2)-x_cm;//hardcoding is for dummies
		wheel_locs[1][i]=(((i&2)>>1)^(i&1)>0?-width/2:width/2)-y_cm;
	}
	float b[] = {0,0,pi,pi};
	for(uint8_t i=0;i<4;i++){
		float a=atan2(wheel_locs[1][i],wheel_locs[0][i]);//about pi/4 or -pi/4
		//float b=(((i&2)>>1)^(i&1))>0?-pi/2:pi/2;
		float c=i&1>0?-pi/4:pi/4;
		float l=sqrt(wheel_locs[0][i]*wheel_locs[0][i]+wheel_locs[1][i]*wheel_locs[1][i]);
		rates[i]=(goal_vel[0] + 0*(goal_vel[0] - real_vel[0]))/radius*cos(b[i]-c)/sin(c);
		rates[i]-=(goal_vel[1] + 0*(goal_vel[1] - real_vel[1]))/radius*sin(b[i]-c)/sin(c);
		rates[i]-=(goal_vel[2] + 0*(goal_vel[2] - real_vel[2]))/radius*l*sin(b[i]-c+a)/sin(c);
	}

}