#include "ros/ros.h"							// ROS 기본 헤더 파일
#include "geometry_msgs/Twist.h"				// geometry_msgs/Twist 헤더 파일 (로봇 구동 메시지 Pub)
#include "std_msgs/Int32.h"						// std_msgs/Int32 헤더 파일 (거리값 Sub)
#include "turtlebot3_msgs/Sound.h"				// turtlebot3_msgs/Sound 헤더 파일 (에러 사운드 값 pub)

// Subscriber callback 함수 선언
void distance_right_callback(const std_msgs::Int32::ConstPtr& right); 
void distance_left_callback(const std_msgs::Int32::ConstPtr& left); 
void power_callback(const std_msgs::Int32::ConstPtr& status); 
void speed_button_callback(const std_msgs::Int32::ConstPtr& status); 

// check_button_press 함수 선언
int check_button_press(int button_status, int button_pre_status);

// callback 함수에서 사용하는 전역변수 선언
static int distance_right = 0;				
static int distance_left = 0;
static int power_status = 0;
static int speed_status = 0;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "mini_project_01"); 		// 노드명 초기화
	ros::NodeHandle nh; 							// ROS 시스템과 통신을 위한 노드 핸들 선언

	// sub_distance_right subscriber 선언, 토픽: distance_right, 큐 사이즈: 10개, callback함수: distance_right_callback
	ros::Subscriber sub_distance_right = nh.subscribe<std_msgs::Int32>("distance_right", 10, distance_right_callback);

	// sub_distance_left subscriber 선언, 토픽: distance_left, 큐 사이즈: 10개, callback함수: distance_left_callback
	ros::Subscriber sub_distance_left = nh.subscribe<std_msgs::Int32>("distance_left", 10, distance_left_callback);

	// sub_power subscriber 선언, 토픽: power, 큐 사이즈: 10개, callback함수: power_callback
	ros::Subscriber sub_power = nh.subscribe<std_msgs::Int32>("power", 10, power_callback);

	// sub_speed subscriber 선언, 토픽: speed_button, 큐 사이즈: 10개, callback함수: speed_button_callback
	ros::Subscriber sub_speed = nh.subscribe<std_msgs::Int32>("speed_button", 10, speed_button_callback);
	
	// pub_cmd_vel publisher 선언, 토픽: cmd_vel, 큐 사이즈: 10개
	ros::Publisher pub_cmd_vel = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);

	// pub_sound publisher 선언, 토픽: sound, 큐 사이즈: 10개
	ros::Publisher pub_sound = nh.advertise<turtlebot3_msgs::Sound>("sound", 10);  

	// geometry_msgs 안의 Twist 메시지 twist로 변수 선언
	geometry_msgs::Twist twist;

	// turtlebot3_msgs 안의 Sound 메시지 sound 변수 선언
	turtlebot3_msgs::Sound sound;

	twist.linear.y=0.0; twist.linear.z=0.0;			// linear.y,z = 0 사용하지 않음
	twist.angular.x=0.0; twist.angular.y=0.0;		// angular.x,y = 0 사용하지 않음

	// 루프 주기를 10Hz로 설정 0.1초 주기로 while문 실행
	ros::Rate loop_rate(10);
	
	// 지역변수 선언
	int power_pre_status = 0;						// 이전의 power 버튼 상태 저장 변수 선언 및 초기화
	int speed_pre_status = 0;						// 이전의 speed 버튼 상태 저장 변수 선언 및 초기화
	int power_count = 0;							// power 상태 on/off 상태 확인하기 위한 변수 선언 및 초기화	
	double speed_count = 0.0;						// 직진 speed 상태 확인하기 위한 변수 선언 및 초기화
	double angular_count = 0.0; 					// 회전 속도 count 변수 선언 및 초기화

	while(ros::ok())
	{
		ROS_INFO("POWER STATUS: %d", power_count & 1);
		ROS_INFO("SPEED STATUS: %0.2lf", speed_count);

		// 버튼 눌리면 power_count 1씩 증가
		power_count += check_button_press(power_status, power_pre_status);  
 
		if(power_count & 1)							// &연산으로 홀수면 1(ON), 짝수면 0(OFF) 
		{
			ROS_INFO("POWER ON");

			if(distance_right < 20)					// 왼쪽으로 회전할 때 
			{	
				ROS_INFO("distance_right < 20cm");

				// 오른쪽 거리가 20cm 보다 작으면 처음에는 STOP
				twist.linear.x = 0.0;
				twist.angular.z = angular_count;
		
				sound.value = 3;					// ERROR value 3 저장
				
				pub_sound.publish(sound);			// sound publish

				pub_cmd_vel.publish(twist); 		// twist publish
				
				if(angular_count == 0.0) 				
					angular_count = 1.0; 			// 왼쪽으로 회전(최대 회전 속도 = 2.84)
			}
			else if(distance_left < 20)				// 오른쪽으로 회전할 때 
			{
				ROS_INFO("distance_left < 20cm");

				// 왼쪽 거리가 20cm 보다 작으면 처음에는 STOP
				twist.linear.x = 0.0;
				twist.angular.z = angular_count;

				sound.value = 3;					// ERROR value 3 저장		
				
				pub_sound.publish(sound);			// sound publish

				pub_cmd_vel.publish(twist); 		// twist publish

				if(angular_count == 0.0)			 		
					angular_count = -1.0;			// 오른쪽으로 회전(최대 회전 속도 = -2.84)
			}
			else									// 직진할 때 
			{	
				// 직진할 상황일 때만 speed 버튼 확인해서 저장(속도 0.01씩 증가하기 위해 100으로 나눔)
				speed_count += (check_button_press(speed_status, speed_pre_status) / 100.0); 

				ROS_INFO("Go! Go! Go! Go! Go!");
			
				angular_count = 0.0;				// angular_count 변수 0으로 초기화
			
				twist.linear.x = speed_count;		// 왼쪽 오른쪽 거리가 20cm보다 크면 직진
				twist.angular.z = angular_count;	// 회전은 하지 않는다

				pub_cmd_vel.publish(twist); 		// twist publish
			}
		}	
		else
		{
				ROS_INFO("POWER OFF");

				speed_count = 0.0;					// speed_count 변수 0으로 초기화
				angular_count = 0.0;				// angular_count 변수 0으로 초기화
			
				twist.linear.x = speed_count;		
				twist.angular.z = angular_count;	

				pub_cmd_vel.publish(twist); 		// twist publishs
		}
		if(power_count > 20) power_count = 0; 		// power_count 20 넘으면 다시 0으로 초기화
		if(speed_count > 0.2) speed_count = 0;  	// speed_count 0.2 넘으면 다시 0으로 초기화(최대 속도=0.2)

		power_pre_status = power_status;			// 현재 파워 버튼 상태를 이전 파워 버튼 상태에 저장
		speed_pre_status = speed_status;			// 현재 스피드 버튼 상태를 이전 스피트 버튼 상태에 저장
		ros::spinOnce(); 							// 콜백함수 호출을 위한 함수, 메시지 수신되면 콜백함수 실행, spin() 함수와 다른점은 호출하고 반환되므로 다음 코드를 실행할 수 있다.
		loop_rate.sleep(); 							// 위에서 정한 루프 주기(10Hz)에 따라 슬립에 들어간다
	}
	
	return 0;
	
}

// sub_distance_right callback 함수 정의
void distance_right_callback(const std_msgs::Int32::ConstPtr& right)
{	
	distance_right = right->data;
	//ROS_INFO("distance_right: %d", distance_right);
}

// sub_distance_left callback 함수 정의
void distance_left_callback(const std_msgs::Int32::ConstPtr& left)
{	
	distance_left = left->data;
	//ROS_INFO("distance_left: %d", distance_left);
}

// sub_power callback 함수 정의
void power_callback(const std_msgs::Int32::ConstPtr& status)
{
	power_status = status->data;
}

// sub_speed callback 함수 정의
void speed_button_callback(const std_msgs::Int32::ConstPtr& status)
{
	speed_status = status->data;
}

// check button press 함수 정의
int check_button_press(int button_status, int button_pre_status)
{
	if((button_status == 1) && (button_pre_status == 0)) 	// 버튼의 상태가 0 -> 1로 바뀌었을 때(눌렀을 때) 실행
	{	
		return 1;
	}
	return 0;
}























