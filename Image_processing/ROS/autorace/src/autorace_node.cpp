#include <ros/ros.h>
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include <stdlib.h>
#include <iostream>

using namespace std;

//정의부 
class ControlLane
{
	public:
		//생성자
		ControlLane();
		//소멸자
		~ControlLane();
		
		int lastError;
		int center;
		float MAX_VEL;
		float angular_z;
		int stop;
		
		void velocityCallback(const std_msgs::Float32::ConstPtr& velocity);
		void angularCallback(const std_msgs::Int32::ConstPtr& angle_stop);
		void cbFollowLane(const std_msgs::Int32::ConstPtr& desired_center);
		void publish_CMD_VEL(ros::Publisher *cmd_vel);
	private:
		int error;
		float Kp;
		float Kd;
		float angle;
};


//구현부 		
ControlLane::ControlLane() : lastError(0), stop(0)
{
}

ControlLane::~ControlLane()
{
}

void ControlLane::angularCallback(const std_msgs::Int32::ConstPtr& angle_stop)
{
	stop = angle_stop->data;
	if(stop)
		angular_z = 0;
	else
		angular_z = Kp * error + Kd * (error - lastError);
	cout << "angular_z = " << angular_z << endl;
}

void ControlLane::velocityCallback(const std_msgs::Float32::ConstPtr& velocity)
{
	MAX_VEL = velocity->data;
	cout << "MAX_VEL = " << MAX_VEL<<endl<<endl;
}

void ControlLane::cbFollowLane(const std_msgs::Int32::ConstPtr& desired_center)
{
	center = desired_center->data;
	//cout << "center = " << center << endl;
}

void ControlLane::publish_CMD_VEL(ros::Publisher *cmd_vel)
{
	geometry_msgs::Twist twist;
	
	error = center - 327;
	Kp = 0.0025;
	Kd = 0.02;

	angle = 0.5;
	
	lastError = error;
	twist.linear.x = std::min(MAX_VEL * pow(1 - abs(error) / 327, 2.2), 0.2);
	twist.linear.y = 0;
	twist.linear.z = 0;
	twist.angular.x = 0;
	twist.angular.y = 0;
	twist.angular.z = angular_z < 0 ? -std::max(angular_z, -angle) : -std::min(angular_z, angle);
	
	cmd_vel -> publish(twist);
}


//MAIN
int main(int argc, char** argv)
{
	ros::init(argc, argv, "control_lane");
	ros::NodeHandle nh;
	
	ControlLane *controlane = new ControlLane();
	
	ros::Subscriber sub_lane = nh.subscribe<std_msgs::Int32>("pi/lane", 1, &ControlLane::cbFollowLane, controlane);
	ros::Subscriber sub_MAX_VEL = nh.subscribe<std_msgs::Float32>("pi/MAX_VEL", 1, &ControlLane::velocityCallback, controlane);
	ros::Subscriber sub_angle_stop = nh.subscribe<std_msgs::Int32>("pi/ANGLE_STOP", 1, &ControlLane::angularCallback, controlane);
	ros::Publisher pub_cmd_vel = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

	ros::Rate loop_rate(10);
	
	while(nh.ok())
	{
		controlane -> publish_CMD_VEL(&pub_cmd_vel);
		
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	return 0;
}
