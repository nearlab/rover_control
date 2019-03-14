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
#include <iostream>
#include <unistd.h>
#include <cstdlib>
#include <math.h>
#include <stdio.h>
#include <nav_msgs/Odometry.h>
//#include "orbot_utils.h"
#include <SerialPort.h>//to install, use sudo apt-get install libserial-dev


ros::Subscriber velcmd, viconspeed;
bool update;

const float pi=3.14159265359;
const float wMax=122*2*pi/60;
const float vMax=.0762*wMax/1.424;// m/s
float rates[4], real_vel[3];
geometry_msgs::Twist goalVelocity;
bool GoalTimeout = false;
ros::Time prevGoal;
void getRotationRates(float* rates,float* goal_vel, float* real_vel, float x_cm=0, float y_cm=0, float length=.04, float width=.037, float radius=.0076);


void viconCallback(nav_msgs::Odometry roverspeed)
{

	real_vel[0] = roverspeed.twist.twist.linear.x;
	real_vel[1] = roverspeed.twist.twist.linear.y;
	real_vel[2] = roverspeed.twist.twist.angular.z;

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
		rates[i]=round(rates[i]/122*1000);
		if(rates[i]<300 && rates[i]>-300)
			rates[i] = 0;
	}

	prevGoal = ros::Time::now();
	GoalTimeout = false;
	std::cout<<twist.linear.x-real_vel[0]<<" "<<twist.linear.y-real_vel[1]<<" "<<twist.angular.z-real_vel[2]<<std::endl;

}

void writeToPort(SerialPort *ser, char* write)
{
	bool written=false;
	while(!written){
		try{
			ser->Write(write);
			written=true;
		}
		catch(std::exception& e){
			continue;
		}
	}
}

void writeDeltas(SerialPort *ser1, SerialPort *ser2, float *rates){
		
		//TODO: make this more straightforward
		char output_buffer[64];
		int len=sprintf(output_buffer,"!g 1 %d\r",(int)rates[0]);
		char* write =(char*) malloc(len+1);
		strncpy(write,output_buffer,len);
		write[len]='\0';
		writeToPort(ser1,write);
		//std::cout<<"wrote "<<write<<"\n";

		len=sprintf(output_buffer,"!g 2 %d\r",(int)rates[1]);
		write =(char*) malloc(len+1);
		strncpy(write,output_buffer,len);
		write[len]='\0';
		writeToPort(ser1,write);
		//std::cout<<"wrote "<<write<<"\n";

		len=sprintf(output_buffer,"!g 1 %d\r",(int)rates[2]);
		write =(char*) malloc(len+1);
		strncpy(write,output_buffer,len);
		write[len]='\0';
		writeToPort(ser2,write);
		//std::cout<<"wrote "<<write<<"\n";

		len=sprintf(output_buffer,"!g 2 %d\r",(int)rates[3]);
		write =(char*) malloc(len+1);
		strncpy(write,output_buffer,len);
		write[len]='\0';
		writeToPort(ser2,write);
		//std::cout<<"wrote "<<write<<"\n";
}

int main(int argc, char** argv){

	ros::init(argc,argv,"orbot_client");
	ros::NodeHandle nh;
	//viconSub=nh.subscribe("/vicon/Orbot/Orbot",1000,messageCallbackVicon);
	velcmd = nh.subscribe("/orbot/cmd_vel",1000,velocityCallback);
	viconspeed = nh.subscribe("/orbot_rover/local_odom",1000,viconCallback);
	ros::Rate loop_rate(30);//TODO: spiit up the updating and writing rates with time class
	
	
	ros::Time prevWrite=ros::Time::now();
	ros::Time start_time=ros::Time::now();
	ros::Duration writeDelay(.01);
	ros::Duration goalDelay(2);
	ros::Duration d;
	
	double t;
	
	SerialPort ser1("/dev/ttyACM0");
	ser1.Open();
	ser1.SetBaudRate(SerialPort::BAUD_115200);
	
	SerialPort ser2("/dev/ttyACM1");
	ser2.Open();
	ser2.SetBaudRate(SerialPort::BAUD_115200);
		

	while(ros::ok()){
		
		bool write=false;
		//std::cout<<"Still Running\n";
		if(ros::Time::now()-prevGoal>goalDelay)
			GoalTimeout = true;

		if(ros::Time::now()-prevWrite>writeDelay && !GoalTimeout){
			
			write=false;
			writeDeltas(&ser1,&ser2,rates); //Rover Uncomment
			prevWrite=ros::Time::now();
			//std::cout<<"Wrote speeds\n";
		}
		
//		loop_rate.sleep();
		ros::spinOnce();
		loop_rate.sleep();
	}
	ser1.Close();// Rover Uncomment
	ser2.Close();// Rover Uncomment
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

	for(uint8_t i=0;i<4;i++){
		float a=atan2(wheel_locs[1][i],wheel_locs[0][i]);//about pi/4 or -pi/4
		float b=(((i&2)>>1)^(i&1))>0?-pi/2:pi/2;
		float c=i&1>0?-pi/4:pi/4;
		float l=sqrt(wheel_locs[0][i]*wheel_locs[0][i]+wheel_locs[1][i]*wheel_locs[1][i]);
		rates[i]=(goal_vel[0] + 0.1*(goal_vel[0] - real_vel[0]))/radius*cos(b-c)/sin(c);
		rates[i]-=(goal_vel[1] + 0.1*(goal_vel[1] - real_vel[1]))/radius*sin(b-c)/sin(c);
		rates[i]-=(goal_vel[2] + 0.5*(goal_vel[2] - real_vel[2]))/radius*l*sin(b-c+a)/sin(c);
	}

}