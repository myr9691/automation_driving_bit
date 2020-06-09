#include <ros/ros.h>
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <wiringPi.h>
#include <signal.h>

#include <tof.h> // time of flight sensor library

#include "geometry_msgs/Twist.h"
#include "std_msgs/Int32.h"

#define SPEED_0 0.0
#define SPEED_N 0.05
#define SPEED_P 0.01
#define SPEED_L -0.01
#define ANGLE_0 0.0

#define ON 1
#define OFF 0

enum POSISION_ARRANGE {
	find_pos = 0,
	rearrange
};

enum PARKING {
	pause_0 = 0,
	turn_right,
	pause_1,
	turn_left1,
	turn_left2,
	pause_2,
	nothing
};

enum WAYOUT {
	start = 0,
	go_back,
	go_left1,
	go_left2,
	go_straight
};

void mySigintHandler(int sig)
{
	geometry_msgs::Twist twist;
	twist.linear.x = SPEED_0;
	twist.angular.z = ANGLE_0;
	ros::shutdown();
}

using namespace std;

int main(int argc, char **argv){
	int i2c_bus, i2c_address, long_range = 0;
	int right_laser = ON;
	double poll_rate = 0, distance;

	ros::init(argc, argv, "vl53l0x");
	ros::NodeHandle nh, nh_priv("~");
	
	//Publisher
	ros::Publisher pub_cmd_vel = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10); // motor contorl
	
	//msgs
	geometry_msgs::Twist twist;
	
	//wiringPi
	wiringPiSetupGpio();			
	pinMode(20, OUTPUT);		
	pinMode(21, OUTPUT);
	digitalWrite(20, ON);
	digitalWrite(21, OFF);
	
	nh_priv.param("long_range", long_range, 0);
	nh_priv.param("poll_rate", poll_rate, 100.0);
	nh_priv.param("i2c_bus", i2c_bus, 1);
	nh_priv.param("i2c_address", i2c_address, 0x29);

	tofInit(i2c_bus, i2c_address, long_range);
	
	twist.linear.y=SPEED_0;
	twist.linear.z=SPEED_0;
	twist.angular.x=ANGLE_0;
	twist.angular.y=ANGLE_0;
	
	ros::Rate r(poll_rate);
	int count = 0;
	int wait_flag = 0;
	
	enum POSISION_ARRANGE POS_ARRANGE = find_pos;
	enum PARKING PARKING_ANGLE = pause_0;
	enum WAYOUT OUT_ANGLE = start;
	
	while (ros::ok()) 
	{
	    distance = tofReadDistance();
	    ROS_INFO("POS_ARRANGE = %d", POS_ARRANGE);
	    ROS_INFO("PARKING_ANGLE = %d", PARKING_ANGLE);
		ROS_INFO("distance = %f", distance);
		ROS_INFO("SPEED = %f", twist.linear.x);
		ROS_INFO("ANGLE = %f", twist.angular.z);
		
		/*****POS_ARRANGE*****/
		//right_laser = ON, back_laser = OFF
		if(right_laser == ON)
		{
			//Find Position
			if(POS_ARRANGE == find_pos)
			{
				twist.linear.x = SPEED_N;
				twist.angular.z = ANGLE_0;
				pub_cmd_vel.publish(twist);
				if(distance > 140.0 & distance < 450)
					count += 1;
			}
			
			ROS_INFO("count = %d\n", count);
			
			//NO Obstacle
			if (distance < 100.0 & count > 100)
			{
				twist.linear.x = SPEED_0;
				twist.angular.z = ANGLE_0;
				pub_cmd_vel.publish(twist);
				POS_ARRANGE = rearrange;
			}
			
			//Rearrange
			if (POS_ARRANGE == rearrange)
			{
				twist.linear.x = SPEED_L;
				twist.angular.z = ANGLE_0;
				pub_cmd_vel.publish(twist);
				if(distance > 110 & distance < 450 & count > 100)
				{
					digitalWrite(20, OFF);  //right
					digitalWrite(21, ON);   //back
					right_laser = OFF;
					tofInit(i2c_bus, i2c_address, long_range);
					PARKING_ANGLE = pause_0;
				}
			}
		}
		/*****PARKING*****/
		//right_laser = OFF, back_laser = ON
		else 
		{
			//sleep once
			if(wait_flag == 0)
			{
				sleep(1);
				wait_flag = 1;
			}
			
			//PARKING START
			if (PARKING_ANGLE == pause_0)
			{
				cout <<  "waiting for back laser" << endl;
				cout << "distance = " << distance << endl;
				if (distance > 300)
					PARKING_ANGLE = turn_right;
			}
			else if (PARKING_ANGLE == turn_right)
			{
				twist.linear.x = -SPEED_P;
				twist.angular.z = 0.05;
				pub_cmd_vel.publish(twist);
				if (distance < 70)
					PARKING_ANGLE = pause_1;
			}
			else if(PARKING_ANGLE == pause_1)
			{
				sleep(1);
				PARKING_ANGLE = turn_left1;
			}
			else if (PARKING_ANGLE == turn_left1)
			{
				twist.linear.x = -SPEED_P;
				twist.angular.z = -0.2;
				pub_cmd_vel.publish(twist);
				if(distance > 130)
					PARKING_ANGLE = turn_left2;
			}
			else if (PARKING_ANGLE == turn_left2)
			{
				twist.linear.x = -SPEED_P;
				twist.angular.z = -0.2;
				pub_cmd_vel.publish(twist);
				if(distance < 75)
					PARKING_ANGLE = pause_2;
			}
			else if(PARKING_ANGLE == pause_2)
			{
				twist.linear.x = SPEED_0;
				twist.angular.z = ANGLE_0;
				pub_cmd_vel.publish(twist);
				OUT_ANGLE = go_back;
				PARKING_ANGLE = nothing;
			}
			if(OUT_ANGLE == go_back)
			{
				twist.linear.x = -SPEED_P;
				twist.angular.z = ANGLE_0;
				pub_cmd_vel.publish(twist);
				if(distance < 50)
					OUT_ANGLE = go_left1;
			}
			else if(OUT_ANGLE == go_left1)
			{
				twist.linear.x = SPEED_0;
				twist.angular.z = 0.2;
				pub_cmd_vel.publish(twist);
				if(distance > 100)
					OUT_ANGLE = go_left2;
			}
			else if(OUT_ANGLE == go_left2)
			{
				twist.linear.x = SPEED_0;
				twist.angular.z = 0.2;
				pub_cmd_vel.publish(twist);
				if(distance < 50)
					OUT_ANGLE = go_straight;
			}
			else if(OUT_ANGLE == go_straight)
			{
				for(int i = 0; i < 300; i++)
				{
					twist.linear.x = SPEED_N;
					twist.angular.z = ANGLE_0;
					pub_cmd_vel.publish(twist);
				}
			}
			signal(SIGINT, mySigintHandler);
			pub_cmd_vel.publish(twist);
		}
	    ros::spinOnce();
	}

	ROS_INFO("VL53L0X: stop ranging");

	return 0;
}






