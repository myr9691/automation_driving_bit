#ifndef AUTORACE_H
#define AUTORACE_H

#include <iostream>
#define ON 1

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
			turn_right2,
			turn_right3,
			pause_1,
			turn_left1,
			turn_left2,
			pause_2,
			nothing
		};

		enum WAYOUT {
			start = 0,
			out_right1,
			out_right2,
			out_right3,
			out_left1,
			out_left2,
			out_pause,
			go_back,
			go_left,
			go_out_left,
			go_out_right,
			lane_follow
		};
		
		enum POSISION_ARRANGE POS_ARRANGE = find_pos;
		enum PARKING PARKING_ANGLE = pause_0;
		enum WAYOUT OUT_ANGLE = start;

		int lastError = 0;
		int center = 0;
		float MAX_VEL = 0;
		float angular_z = 0;
		int stop = 1;
		int count = 0;
		int wait_flag = 0;
		int right_laser = ON;
		double distance = 0;
		int parking2 = 0;
		int parking1 = 0;
		int state = 0;
		int pre_state = 0;
		
		void eventCallback(const std_msgs::String::ConstPtr& nano_event);
		void publishCmdVel(ros::Publisher *cmd_vel);
		void firstParking(ros::Publisher *cmd_vel, int, int, int);
		void secondParking(ros::Publisher *cmd_vel, int, int, int);

	private:
		int error = 0;
		float Kp = 0.0025;
		float Kd = 0.02;
		float angle = 0.5;
};

#endif
