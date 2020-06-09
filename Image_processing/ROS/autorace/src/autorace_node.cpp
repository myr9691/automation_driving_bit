#include <ros/ros.h>
#include "std_msgs/Int32.h"
#include "geometry_msgs/Twist.h"
#include <stdlib.h>
#include <iostream>

using namespace std;
ros::Publisher pub_cmd_vel;

//정의부 
class ControlLane
{
	public:
		//생성자
		ControlLane();
		~ControlLane();
		
		int lastError;
		int center;
		float MAX_VEL;
		
		void cbFollowLane(const std_msgs::Int32::ConstPtr& desired_center);
		void publish_CMD_VEL(ros::Publisher *cmd_vel);
};


//구현부 		
ControlLane::ControlLane() : lastError(0), MAX_VEL(0.05)
{
	//cout << "lastError = " << lastError << ", MAX_VEL = " << MAX_VEL << endl;
}

ControlLane::~ControlLane()
{
}

void ControlLane::cbFollowLane(const std_msgs::Int32::ConstPtr& desired_center)
{
	center = desired_center->data;
	cout << "center = " << center << endl;
}

void ControlLane::publish_CMD_VEL(ros::Publisher *cmd_vel)
{
	geometry_msgs::Twist twist;
	
	int error = center - 330;
	
	float Kp = 0.0025;
	float Kd = 0.02;
	
	float angular_z = Kp * error + Kd * (error - lastError);
	float angle = 0.5;
	
	lastError = error;
	twist.linear.x = std::min(MAX_VEL * pow(1 - abs(error) / 330, 2.2), 0.2);
	twist.linear.y = 0;
	twist.linear.z = 0;
	twist.angular.x = 0;
	twist.angular.y = 0;
	if (angular_z < 0)
		twist.angular.z = std::max(angular_z, -angle);
	else
		twist.angular.z = -std::min(angular_z, angle);
	
	cmd_vel -> publish(twist);
}


//MAIN
int main(int argc, char** argv)
{
	ros::init(argc, argv, "control_lane");
	ros::NodeHandle nh;
	
	ControlLane *controlane = new ControlLane();
	
	ros::Subscriber sub_lane = nh.subscribe<std_msgs::Int32>("pi/lane", 1, &ControlLane::cbFollowLane, controlane);
	ros::Publisher pub_cmd_vel = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

	ros::Rate loop_rate(10);
	
	controlane -> publish_CMD_VEL(&pub_cmd_vel);
	ros::spin();


	return 0;
}

/*
#include <ros/ros.h>
#include "std_msgs/Int32.h"
#include "geometry_msgs/Twist.h"
#include <stdlib.h>

void cbFollowLane(const std_msgs::Int32::ConstPtr& desired_center);
geometry_msgs::Twist twist;
ros::Publisher pub_cmd_vel;

int lastError = 0;
float MAX_VEL = 0.05;
int center = 0;
int error = 0;
float Kp = 0.0025;
float Kd = 0.02;
float angle = 0.5;
float angular_z = 0;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "control_lane");
	ros::NodeHandle nh;
	ros::Subscriber sub_lane = nh.subscribe<std_msgs::Int32>("pi/lane", 1, cbFollowLane);
	pub_cmd_vel = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	
	twist.linear.y = 0;
	twist.linear.z = 0;
	twist.angular.x = 0;
	twist.angular.y = 0;
	
	ros::Rate loop_rate(10);
	while(ros::ok())
	{
		twist.linear.x = std::min(MAX_VEL * pow(1 - abs(error) / 330, 2.2), 0.2);
	
		if (angular_z < 0)
			twist.angular.z = std::max(angular_z, -angle);
		else
			twist.angular.z = -std::min(angular_z, angle);
		pub_cmd_vel.publish(twist);
		
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	
	return 0;
}

void cbFollowLane(const std_msgs::Int32::ConstPtr& desired_center)
{
	center = desired_center->data;
	error = center - 330;
	
	angular_z = Kp * error + Kd * (error - lastError);
	
	lastError = error;
}*/
