#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "std_msgs/Int32.h"

void ultra_callback(const std_msgs::Int32::ConstPtr& back);

static int ultra_distance = 0;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "parking_node");
	ros::NodeHandle nh;
	
	ros::Subscriber sub_ultra = nh.subscribe<std_msgs::Int32>("back_distance", 10, ultra_callback);
	ros::Publisher pub_cmd_vel = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10); // motor contorl
	
	geometry_msgs::Twist twist;
	
	twist.linear.y=0.0;
	twist.linear.z=0.0;
	twist.angular.x=0.0;
	twist.angular.y=0.0;
	
	ros::Rate loop_rate(10);
	
	double speed_state = 0.0;
	double angular_state = 0.0;
	
	while(ros::ok())
	{
        ROS_INFO("SPEED = %lf", speed_state);
		
		if(ultra_distance < 6)
		{
			ROS_INFO("STOP STOP STOP");
			
			speed_state = 0.0;
			twist.linear.x = speed_state;
			twist.angular.z = angular_state;
			
			pub_cmd_vel.publish(twist);
			
			if(angular_state == 0.0)
				angular_state = 0.2;
			if(speed_state == 0.0)
				speed_state = -0.2;
		}
		else
		{
			ROS_INFO("GO BACK");
			speed_state = -0.2;
			angular_state = 0.0;
			
			twist.linear.x = speed_state;
			twist.angular.z = angular_state;
			
			pub_cmd_vel.publish(twist);
		}
		
                ROS_INFO("distance = %d", ultra_distance);
		ros::spinOnce(); 
		loop_rate.sleep();
	}
	return 0;
}

void ultra_callback(const std_msgs::Int32::ConstPtr& back)
{
	ultra_distance = back->data;
}
