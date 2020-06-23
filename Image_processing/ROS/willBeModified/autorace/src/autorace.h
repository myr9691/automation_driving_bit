#ifndef AUTORACE_H
#define AUTORACE_H

class ControlLane
{
	public:
		//생성자
		ControlLane();
		
		int lastError = 0;
		int center = 0;
		float MAX_VEL = 0;
		float angular_z = 0;
		int stop = 0;
		
		void velocityCallback(const std_msgs::Float32::ConstPtr& velocity);
		void angularCallback(const std_msgs::Int32::ConstPtr& angle_stop);
		void cbFollowLane(const std_msgs::Int32::ConstPtr& desired_center);
		void publishCmdVel(ros::Publisher *cmd_vel);
	private:
		int error = 0;
		float Kp = 0;
		float Kd = 0;
		float angle = 0;
};

#endif
