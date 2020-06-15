#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include <unordered_map>
#include "course_control.h"

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

Course::Course() : count(0), flag(0)
{
}

void Course::stateCallback(const std_msgs::String::ConstPtr& nano_state)
{
	state = nano_state->data.c_str();
	cout << state << endl;
}

void Course::stopCallback(const std_msgs::Int32::ConstPtr& stop_state)
{
	stop = stop_state->data;
}

void Course::publishAngle(ros::Publisher *angle_stop)
{
	std_msgs::Int32 ANGLE_STOP;
	enum Statement STATE;
	
	switch(statement[state])
	{
		case person:
		case red:
			flag = 1;
			ANGLE_STOP.data = 1;
			break;
		case green:
			flag = 1;
			ANGLE_STOP.data = 0;
			break;
		default:
			flag = 1;
			ANGLE_STOP.data = 0;
			break;
	}
	count = 0;
	
	if (stop == 1 && flag == 0)
	{
		cout << "stop line : ANGLE = " << count << endl;
		count++;
		ANGLE_STOP.data = 1;
		if (count > 20)
		{
			flag = 1;
		}
	}
	
	angle_stop -> publish(ANGLE_STOP);
}

void Course::publishMaxVel(ros::Publisher *velocity)
{
	std_msgs::Float32 MAX_VEL;
	enum Statement STATE;
	
	switch(statement[state])
	{
		case person:
		case red:
			flag = 1;
			MAX_VEL.data = 0.0;
			break;
		case green:
			flag = 1;
			MAX_VEL.data = 0.05;
		case avoid_right:
		case no_right_turn:
			flag = 0;
			MAX_VEL.data = 0.05;
			break;
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
			flag = 0;
			MAX_VEL.data = 0.05;
			break;
	}
	
	if (stop == 1 && flag == 0)
	{
		cout << "stop line : MAX = " << count << endl;
		count++;
		MAX_VEL.data = 0.0;
		if (count > 20)
		{
			flag = 1;
			count = 0;
		}
	}
	
	velocity -> publish(MAX_VEL);
	
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "course_control");
	ros::NodeHandle nh;
	
	Course *course = new Course();
	
	ros::Subscriber sub_state = nh.subscribe<std_msgs::String>("nano/status", 1, &Course::stateCallback, course);
	ros::Subscriber sub_stop = nh.subscribe<std_msgs::Int32>("pi/stop", 1, &Course::stopCallback, course);
	ros::Publisher pub_MAX_VEL = nh.advertise<std_msgs::Float32>("pi/MAX_VEL", 1);
	ros::Publisher pub_ANGLE_STOP = nh.advertise<std_msgs::Int32>("pi/ANGLE_STOP", 1);
	
	ros::Rate loop_rate(10);
	
	while(nh.ok())
	{
		course -> publishMaxVel(&pub_MAX_VEL);
		course -> publishAngle(&pub_ANGLE_STOP);
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	return 0;
}
