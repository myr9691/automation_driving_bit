#include <ros/ros.h>
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>

#include <tof.h> // time of flight sensor library

#include "geometry_msgs/Twist.h"
#include "std_msgs/Int32.h"
#include <sensor_msgs/Range.h>

int main(int argc, char **argv){
	int i;
	double back_distance;
	int model, revision;
	int long_range = 0;
	int i2c_bus, i2c_address;
	double poll_rate = 0;

	ros::init(argc, argv, "vl53l0x");
	ros::NodeHandle nh, nh_priv("~");
	
	ros::Publisher pub_cmd_vel = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10); // motor contorl
	ros::Publisher range_pub = nh.advertise<sensor_msgs::Range>("back", 20);
	
	sensor_msgs::Range range;
	geometry_msgs::Twist twist;
	
	nh_priv.param("long_range", long_range, 0);
	nh_priv.param("poll_rate", poll_rate, 100.0);
	nh_priv.param("i2c_bus", i2c_bus, 1);
	nh_priv.param("i2c_address", i2c_address, 0x29);
	nh_priv.param<std::string>("frame_id", range.header.frame_id, "");

	i = tofInit(i2c_bus, i2c_address, long_range);
	if (i != 1){
	    std::cout << "Sensor Init Error " << i << "\n";
	    return -1;
	}

	i = tofGetModel(&model, &revision);
	std::cout << "VL53L0X device successfully opened.\n";
	std::cout << "Model ID - " << model << "\n";
	std::cout << "Revision ID - " << revision << "\n";
	if(long_range)
	    std::cout << "Long Range - " << long_range << "\n";

	ROS_INFO("VL53L0X: start ranging");
	
	twist.linear.y=0.0;
	twist.linear.z=0.0;
	twist.angular.x=0.0;
	twist.angular.y=0.0;
	
	ros::Rate r(poll_rate);
	
	double speed_state = 0.0;
	double angular_state = 0.0;
	int state = 1;
	
	while (ros::ok()) {
	    back_distance = tofReadDistance()/10;
	    ROS_INFO("state = %d", state);
		ROS_INFO("back_distance = %f", back_distance);
		ROS_INFO("SPEED = %f", speed_state);
		ROS_INFO("ANGLE = %f", angular_state);
		range.range = back_distance;
		range_pub.publish(range);
		
		if (state == 1)
		{
			speed_state = -0.05;
        	angular_state = 0.2;
        	twist.linear.x = speed_state;
        	twist.angular.z = angular_state;
        	pub_cmd_vel.publish(twist);
        	if (back_distance < 12)
        		state = 2;
		}
		else if (state == 2)
		{
			speed_state = -0.05;
			angular_state = 0.0;
			twist.linear.x = speed_state;
			twist.angular.z = angular_state;
			pub_cmd_vel.publish(twist);
			if(back_distance < 8)
				state = 3;
		}
		
		else if (state == 3)
		{
			speed_state = -0.05;
        	angular_state = -0.5;
        	twist.linear.x = speed_state;
        	twist.angular.z = angular_state;
        	pub_cmd_vel.publish(twist);
        	if(back_distance < 5)
        		state = 4;
		}
		else
		{
			speed_state = 0.0;
        	angular_state = 0.0;
        	twist.linear.x = speed_state;
        	twist.angular.z = angular_state;
        	pub_cmd_vel.publish(twist);
		}

	    ros::spinOnce();
	}

	ROS_INFO("VL53L0X: stop ranging");

	return 0;
}




