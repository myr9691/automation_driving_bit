#ifndef AUTORACE_H
#define AUTORACE_H

#include <iostream>

class ControlLane
{
	public:
		//생성자
		ControlLane();
		
		enum POSISION_ARRANGE {
			find_pos = 0,
			rearrange
		};

		enum PARKING {
			pause_0 = 0,
			turn_right,
			pause_1,
			turn_left1,
			turn_left2,
			pause_2,
			nothing
		};

		enum WAYOUT {
			start = 0,
			go_back,
			go_left1,
			go_left2,
			go_straight
		};
		
		enum POSISION_ARRANGE POS_ARRANGE = find_pos;
		enum PARKING PARKING_ANGLE = pause_0;
		enum WAYOUT OUT_ANGLE = start;

		static std::string state;
		int lastError = 0;
		int center = 0;
		float MAX_VEL = 0;
		float angular_z = 0;
		static int stop;
		int count = 0;
		int wait_flag = 0;
		static int right_laser;
		double distance = 0;
		
		void stateCallback(const std_msgs::String::ConstPtr& nano_state);
		void publishCmdVel(ros::Publisher *cmd_vel);
		void firstParking(ros::Publisher *cmd_vel, int, int, int);
		void secondParking(ros::Publisher *cmd_vel);

	private:
		int error = 0;
		float Kp = 0.0025;
		float Kd = 0.02;
		float angle = 0.5;
};

#endif
