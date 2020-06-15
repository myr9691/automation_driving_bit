#ifndef AUTORACE_H
#define AUTORACE_H

#include <iostream>

class ControlLane
{
	public:
		//생성자
		ControlLane();

		std::string state;
		int lastError = 0;
		int center = 0;
		float MAX_VEL = 0;
		float angular_z = 0;
		static int stop;
		
		void stateCallback(const std_msgs::String::ConstPtr& nano_state);
		void publishCmdVel(ros::Publisher *cmd_vel);

	private:
		int error = 0;
		float Kp = 0.0025;
		float Kd = 0.02;
		float angle = 0.5;
};

#endif
