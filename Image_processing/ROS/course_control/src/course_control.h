#ifndef COURSE_CONTROL_H
#define COURSE_CONTROL_H

#include <iostream>

using namespace std;

class Course
{
	public :
		Course();
		
		string state;
		int stop = 0;
		int count = 0;
		int flag = 0;
		
		void stateCallback(const std_msgs::String::ConstPtr& nano_state);
		void stopCallback(const std_msgs::Int32::ConstPtr& stop_state);
		void publishMaxVel(ros::Publisher *velocity);
		void publishAngle(ros::Publisher *angular);
};

#endif
