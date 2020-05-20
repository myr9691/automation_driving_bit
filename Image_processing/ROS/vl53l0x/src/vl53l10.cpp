
#include <ros/ros.h>
#include <iostream>

#include <sensor_msgs/Range.h>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>

#include <tof.h> // time of flight sensor library

int main(int argc, char **argv){
	int i;
	double iDistance;
	int model, revision;
	int long_range = 0;
	int i2c_bus, i2c_address;
	double poll_rate = 0;

	ros::init(argc, argv, "vl53l0x");
	ros::NodeHandle nh, nh_priv("~");

	sensor_msgs::Range range;
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

	ros::Publisher range_pub = nh.advertise<sensor_msgs::Range>("back", 20);

	ros::Rate r(poll_rate);
	while (ros::ok()) {
	    iDistance = tofReadDistance()/10;
	    if (iDistance < 4096){
		std::cout << "Distance = " << iDistance << "\n";
		range.range = iDistance;
		range_pub.publish(range);
	    }

	    ros::spinOnce();
	}

	ROS_INFO("VL53L0X: stop ranging");

	return 0;
}




