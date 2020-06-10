#include "ros/ros.h"
#include <iostream>
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include <unordered_map>

using namespace std;

enum Statement
{
	person,
	red,
	green,
	avoid_right,
	no_right_turn,
	speed_up,
	school_zone,
	school_zone_off,
	roundabout,
	tunnel,
	first_parking,
	second_parking,
	avoid_left
};

unordered_map<string, int> statement = {
	{"person", person},
	{"red", red},
	{"green", green},
	{"avoid_right", avoid_right},
	{"no_right_turn", no_right_turn},
	{"speed_up", speed_up},
	{"school_zone", school_zone},
	{"school_zone_off", school_zone_off},
	{"roundabout", roundabout},
	{"tunnel", tunnel},
	{"first_parking", first_parking},
	{"second_parking", second_parking},
	{"avoid_left", avoid_left}
};

string state;

void stateCallback(const std_msgs::String::ConstPtr& nano_state)
{
	state = nano_state->data.c_str();
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "course_control");
	ros::NodeHandle nh;
	
	ros::Subscriber sub_state = nh.subscribe<std_msgs::String>("nano/state", 1, stateCallback);
	ros::Publisher pub_MAX_VEL = nh.advertise<std_msgs::Float32>("pi/MAX_VEL", 1);
	
	std_msgs::Float32 MAX_VEL;
	enum Statement STATE;
	
	switch(statement[state])
	{
		case person:
		case red:
			MAX_VEL.data = 0;
			break;
		case green:
		case avoid_right:
		case no_right_turn:
		case school_zone_off:
		case roundabout:
		case tunnel:
		case first_parking:
		case second_parking:
		case avoid_left:
			MAX_VEL.data = 0.05;
			break;
		case speed_up:
			MAX_VEL.data = 0.1;
			break;
		case school_zone:
			MAX_VEL.data = 0.02;
			break;
		default :
			MAX_VEL.data = 0.05;
			break;
	}
	
	ros::Rate loop_rate(10);
	
	while(nh.ok())
	{
		pub_MAX_VEL.publish(MAX_VEL);
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	return 0;
}
