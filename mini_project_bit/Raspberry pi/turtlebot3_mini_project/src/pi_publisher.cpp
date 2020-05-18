#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/Twist.h"
#include "wiringPi.h"
#include "ultrasonic_sensor.h"
#include "speed_sensor.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "pi_publisher");
 	ros::NodeHandle nh;
	ros::Publisher pi_distance_pub = nh.advertise<std_msgs::Int32>("distance_left", 100);
	ros::Publisher pi_power_pub = nh.advertise<std_msgs::Int32>("power", 100);
	ros::Rate loop_rate(10);
	std_msgs::Int32 ultra_distance;
	std_msgs::Int32 pi_power;
	int distance;
	int power;
	
	wiringPiSetup();
	ultrasonic_sensor_setup();
	power_setup();
	
	while(ros::ok())
	{
		power = power_on_off();
		pi_power.data = power;
		pi_power_pub.publish(pi_power);
		distance = ultrasonic_sensor_start();
		ultra_distance.data = distance;
		pi_distance_pub.publish(ultra_distance);
		loop_rate.sleep();
	}
	return 0;
}
