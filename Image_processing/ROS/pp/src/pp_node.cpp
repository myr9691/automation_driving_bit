#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "std_msgs/Int32.h"
#include "sensor_msgs/Range.h"

void back_callback(const sensor_msgs::Range::ConstPtr& range);

static double back_distance = 250;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "parking_node");
	ros::NodeHandle nh;
	
	ros::Subscriber sub_laser = nh.subscribe<sensor_msgs::Range>("back", 20, back_callback);
    //ros::Subscriber sub_laser = nh.subscribe<std_msgs::Int32>("right_distance", 10, laser_callback);
	ros::Publisher pub_cmd_vel = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10); // motor contorl
	
	geometry_msgs::Twist twist;
	
	twist.linear.y=0.0;
	twist.linear.z=0.0;
	twist.angular.x=0.0;
	twist.angular.y=0.0;
	
	ros::Rate loop_rate(10);
	
	double speed_state = 0.0;
	double angular_state = 0.0;
	int state = 1;
	
	while(ros::ok())
	{
        ROS_INFO("state = %d", state);
        ROS_INFO("back_distance = %f", back_distance);
        ROS_INFO("SPEED = %f", speed_state);
        ROS_INFO("ANGLE = %f", angular_state);
		
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
		loop_rate.sleep();
	}
	return 0;
}

void back_callback(const sensor_msgs::Range::ConstPtr& range)
{
	back_distance = range->range;

}

void laser_callback(const std_msgs::Int32::ConstPtr& right)
{
        //laser_distance = right->data;
}
