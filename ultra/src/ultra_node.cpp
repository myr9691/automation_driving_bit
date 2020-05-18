#include "ros/ros.h"
#include "std_msgs/Int32.h"
//#include "geometry_msgs/Twist.h"
#include "wiringPi.h"
//#include "back_laser_sensor.h"
#include "ultrasonic_sensor.h"
//include "laser_sensor.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "Sensor");
	ros::NodeHandle nh;
	ros::Publisher back_pub = nh.advertise<std_msgs::Int32>("back_distance", 100);
	//ros::Publisher laser_pub = nh.advertise<std_msgs::Int32>("right_distance", 100);
	ros::Rate loop_rate(10);
	std_msgs::Int32 back_distance;
	//std_msgs::Int32 laser_distance;
	int back;
	int laser;
	
	wiringPiSetup();
	back_laser_sensor_setup();
	//laser_sensor_setup();
	while(ros::ok())
	{
		back = back_laser_sensor_start();
		back_distance.data = back;
		ROS_INFO("distance = %d", back);
		back_pub.publish(back_distance);
	//	laser = laser_sensor_start();
	//	laser_diatance = laser;
	//	laser_pub.publish(laser_distance);
		
		loop_rate.sleep();		
	}
	return 0;
}
