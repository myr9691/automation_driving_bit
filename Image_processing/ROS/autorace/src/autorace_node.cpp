#include <ros/ros.h>
#include <vector>
#include <wiringPi.h>
#include <unordered_map>
#include "geometry_msgs/Twist.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include "autorace.h"
#include "tof.h"
#include "pthread.h"

using namespace std;

#define SPEED_0 0.0
#define SPEED_N 0.05
#define SPEED_P 0.02
#define SPEED_L -0.01
#define ANGLE_0 0.0

#define ON 1
#define OFF 0

#define DBG

string event = "clear";

void* stopLineStopThread(void *ret);

enum Event
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
	avoid_left,
	clear
};

enum State
{
    driving_start,
    Drivex0_red,
    Drivex0_person,
    Drivex1,
    Drivex2,
    Drivehalf
};

unordered_map<string, int> event_map = {
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
	{"avoid_left", avoid_left},
	{"clear", clear},
	{"driving_start", driving_start}
};

//FUNCTION

ControlLane::ControlLane() : lastError(0)
{
}

void ControlLane::centerCallback(const std_msgs::Int32::ConstPtr& sub_center)
{
	center = sub_center->data;
}

void ControlLane::eventCallback(const std_msgs::String::ConstPtr& nano_event)
{
    event = nano_event->data.c_str();
}

void ControlLane::stopCallback(const std_msgs::Int32::ConstPtr& sub_stop)
{
    int err = 0;
    pthread_t pth;
    int t = 0;
    int input = 0;

	stopLinestop = sub_stop->data;
	if (stopLinestop)
	{
		if (err = pthread_create(&pth, NULL, stopLineStopThread, (void*)&input) < 0)
		{
			perror("Thread1 error : ");
			exit(2);
		}
		pthread_join(pth, (void**)t);
	}
}

void ControlLane::publishCmdVel(ros::Publisher *cmd_vel)
{
    geometry_msgs::Twist twist;
    
    static enum State state = driving_start;
    static enum State pre_state = driving_start;
    static enum Event event_enum;
    
    switch(state)
    {

	/****driving_start****/
	case driving_start:
#ifdef DBG
	    cout << "STATE = driving_start" << endl;
#endif
	    MAX_VEL = 0.0;
	    stop = 1;
	    if (event_map[event] == green)
		state = Drivex1;
	    else if (event_map[event] == red)
		state = Drivex0_red;
	    else if (event_map[event] == clear)
		state = driving_start;
	    break;
	
	/****Drivex1****/
	case Drivex1:
#ifdef DBG
	    cout << "STATE = Drivex1" << endl;
#endif
	    MAX_VEL = 0.06;
	    stop = 0;
	    if (event_map[event] == speed_up)
		state = Drivex2;
	    else if (event_map[event] == person)
	    {
		pre_state = Drivex1;
		state = Drivex0_person;
	    }
	    else if (event_map[event] == red)
		state = Drivex0_red;
	    else if (event_map[event] == roundabout)
		state = Drivex0_red;
	    else if (event_map[event] == first_parking)
		state = FParking;
	    else if (event_map[event] == second_parking)
		state = SParking;
	    else
		state = Drivex0;
	    break;
	
	/****Drivex2****/
	case Drivex2:
#ifdef DBG
	    cout << "STATE = Drivex2" << endl;
#endif
	    MAX_VEL = 0.1;
	    stop = 0;
	    if (event_map[event] == school_zone)
		state = Drivehalf;
	    else if (event_map[event] == person)
	    {
		pre_state = Drivex2;
		state = Drivex0_person;
	    }
	    else if (event_map[event] == clear)
		state = Drivex2;
	    break;
	
	/****Drivehalf****/
	case Drivehalf:
#ifdef DBG
	    cout << "STATE = Drivehalf" << endl;
#endif
	    MAX_VEL = 0.02;
	    stop = 0;
	    if (event_map[event] == school_zone_off)
		state = Drivex1;
	    else if (event_map[event] == person)
	    {
		pre_state = Drivehalf;
		state = Drivex0_person;
	    }
	    else if (event_map[event] == clear)
		state = Drivehalf;
	    break;
	
	/****Drivex0_person****/
	case Drivex0_person:
#ifdef DBG
	    cout << "STATE = Drivex0_person" << endl;
#endif
	    MAX_VEL = 0.0;
	    stop = 1;
	    if (event_map[event] == clear)
		state = pre_state;
	    else if (event_map[event] == green)
	    {
		state = Drivex1;
		parking1 = 0;
		parking2 = 0;
	    }
	    break;
	
	/****Drivex0_red****/
	case Drivex0_red:
#ifdef DBG
	    cout << "STATE = Drivex0_red" << endl;
#endif
	    MAX_VEL = 0.0;
	    stop = 1;
	    if (event_map[event] == green)
		state = Drivex1;
	    else if (event_map[event] == clear)
		state = Drivex0_red;
	    break;
    }

#ifdef DBG
    cout << "EVENT = " << event << endl;
    cout << "MAX_VEL = " << MAX_VEL << endl;
#endif
    
    error = center - 327;

    if(stop)
	    angular_z = 0;
    else
	    angular_z = Kp * error + Kd * (error - lastError);
    
    lastError = error;
    twist.linear.x = std::min(MAX_VEL * pow(1 - abs(error) / 327, 2.2), 0.2);
    twist.linear.y = 0;
    twist.linear.z = 0;
    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = angular_z < 0 ? -std::max(angular_z, -angle) : -std::min(angular_z, angle);
    
    cmd_vel -> publish(twist);
}

void ControlLane::firstParking(ros::Publisher *cmd_vel, int i2c_bus, int i2c_address, int long_range)
{     
    geometry_msgs::Twist twist;

    distance = tofReadDistance();
    ROS_INFO("distance = %f", distance);

    error = center - 327;
    angular_z = Kp * error + Kd * (error - lastError);
    lastError = error;
    
    if (parking1 == 0)
    {
	digitalWrite(20, ON);  //right
	digitalWrite(21, OFF);   //back
	right_laser = ON;
	tofInit(i2c_bus, i2c_address, long_range);
	count = 0;
	wait_flag = 0;
	POS_ARRANGE = find_pos;
	parking1 ++;
	sleep(1);
    }

    /*****POS_ARRANGE*****/
    //right_laser = ON, back_laser = OFF
    if(right_laser == ON)
    {
        //Find Position
        if(POS_ARRANGE == find_pos)
        {
            twist.linear.x = SPEED_N;
            twist.angular.z = angular_z < 0 ? -std::max(angular_z, -angle) : -std::min(angular_z, angle);
            cmd_vel -> publish(twist);
            if(distance > 140.0 & distance < 450)
                count += 1;
#ifdef DBG
	    cout << "POS_ARRANGE = 0" << endl;
#endif
        }
        
        ROS_INFO("count = %d\n", count);
        
        //NO Obstacle
        if (distance < 130.0 && count > 25 && POS_ARRANGE == find_pos)
        {
            twist.linear.x = SPEED_0;
            twist.angular.z = ANGLE_0;
            cmd_vel -> publish(twist);
            POS_ARRANGE = rearrange;
#ifdef DBG
	    cout << "POS_ARRANGE = 1" << endl;
#endif
        }
        
        //Rearrange
        if (POS_ARRANGE == rearrange)
        {
            twist.linear.x = SPEED_L;
            twist.angular.z = ANGLE_0;
            cmd_vel -> publish(twist);
#ifdef DBG
	    cout << "POS_ARRANGE = 2" << endl;
#endif
            if(distance > 110 && distance < 450 && count > 25)
            {
#ifdef DBG
		cout << "BACK LASER ON!" << endl;
#endif
                digitalWrite(20, OFF);  //right
                digitalWrite(21, ON);   //back
                right_laser = OFF;
                tofInit(i2c_bus, i2c_address, long_range);
                PARKING_ANGLE = pause_0;
		OUT_ANGLE = start;
            }
        }
    }
    /*****PARKING*****/
    //right_laser = OFF, back_laser = ON
    else 
    {
        //sleep once
        if(wait_flag == 0)
        {
            sleep(1);
            wait_flag = 1;
        }
        
        //PARKING START
        if (PARKING_ANGLE == pause_0)
        {
#ifdef DBG
            cout <<  "waiting for back laser" << endl;
#endif
            if (distance > 300)
                PARKING_ANGLE = turn_right;
        }
	else if (PARKING_ANGLE == turn_right)
	{
#ifdef DBG
	    cout << "PARKING_ANGLE = 0" << endl;
#endif
	    twist.linear.x = -SPEED_P;
	    twist.angular.z = 0.1;
	    cmd_vel -> publish(twist);
	    if (distance < 100)
		PARKING_ANGLE = pause_1;
	}
	else if (PARKING_ANGLE == pause_1)
	{
	    sleep(1);
            PARKING_ANGLE = turn_left1;
        }
	else if (PARKING_ANGLE == turn_left1)
        {
#ifdef DBG
	    cout << "PARKING_ANGLE = 1" << endl;
#endif
            twist.linear.x = -SPEED_P;
            twist.angular.z = -0.25;
            cmd_vel -> publish(twist);
            if(distance > 130)
                PARKING_ANGLE = turn_left2;
        }
	else if (PARKING_ANGLE == turn_left2)
	{
#ifdef DBG
	    cout << "PARKING_ANGLE = 2 " << endl;
#endif
	    twist.linear.x = -SPEED_P;
	    twist.angular.z = -0.25;
	    cmd_vel -> publish(twist);
	    if (distance < 70)
	    {
		PARKING_ANGLE = pause_2;
	    }
	}
	else if(PARKING_ANGLE == pause_2)
        {
#ifdef DBG
	    cout << "PARKING_ANGLE = 3 " << endl;
#endif
            twist.linear.x = SPEED_0;
            twist.angular.z = ANGLE_0;
            cmd_vel -> publish(twist);
	    sleep(3);
	    OUT_ANGLE = out_right1;
	    PARKING_ANGLE = nothing;
        }
	
	if(OUT_ANGLE == out_right1)
	{
#ifdef DBG
	    cout << "OUT_ANGLE = 0" << endl;
#endif
	    twist.linear.x = SPEED_P;
	    twist.angular.z = 0.3;
	    cmd_vel -> publish(twist);
	    if (distance > 80)
	    {
		OUT_ANGLE = out_right2;
	    }
	}
	else if(OUT_ANGLE == out_right2)
	{
#ifdef DBG
	    cout << "OUT_ANGLE = 1" << endl;
#endif
	    twist.linear.x = SPEED_P;
	    twist.angular.z = 0.3;
	    cmd_vel -> publish(twist);
	    if (distance < 70)
	    {
		OUT_ANGLE = out_left1;
	    }
	}
	else if(OUT_ANGLE == out_left1)
	{
#ifdef DBG
	    cout << "OUT_ANGLE = 1" << endl;
#endif
	    twist.linear.x = SPEED_P;
	    twist.angular.z = 0.1;
	    cmd_vel -> publish(twist);
	    if (distance > 90)
	    {
		OUT_ANGLE = out_pause;
	    }
	}
	else if (OUT_ANGLE == out_pause)
	{
	    sleep(1);
            OUT_ANGLE = out_left2;
	}
	else if (OUT_ANGLE == out_left2)
	{
#ifdef DBG
	    cout << "OUT_ANGLE = 2" << endl;
#endif
	    twist.linear.x = SPEED_P;
	    twist.angular.z = -0.1;
	    cmd_vel -> publish(twist);
	    if (distance > 8000)
	    {
		OUT_ANGLE = lane_follow;
	    }
	}
	else if (OUT_ANGLE == lane_follow)
	{
#ifdef DBG
	    cout << "OUT_ANGLE = 3" << endl;
#endif
	    twist.linear.x = SPEED_N;
	    twist.angular.z = angular_z < 0 ? -std::max(angular_z, -angle) : -std::min(angular_z, angle);
	    cmd_vel -> publish(twist);
	}
    }
    cmd_vel -> publish(twist);
}

void ControlLane::secondParking(ros::Publisher *cmd_vel, int i2c_bus, int i2c_address, int long_range)
{
    geometry_msgs::Twist twist;

    distance = tofReadDistance();
    ROS_INFO("distance = %f", distance);

    error = center - 327;
    angular_z = Kp * error + Kd * (error - lastError);
    lastError = error;
    
    if (parking2 == 0)
    {
	digitalWrite(20, ON);  //right
	digitalWrite(21, OFF);   //back
	right_laser = ON;
	tofInit(i2c_bus, i2c_address, long_range);
	count = 0;
	wait_flag = 0;
	POS_ARRANGE = find_pos;
	OUT_ANGLE = start;
	parking2 ++;
	sleep(1);
    }

    /*****POS_ARRANGE*****/
    //right_laser = ON, back_laser = OFF
    if(right_laser == ON)
    {
        //Find Position
        if(POS_ARRANGE == find_pos)
        {
            twist.linear.x = SPEED_N;
            twist.angular.z = angular_z < 0 ? -max(angular_z, -angle) : -min(angular_z, angle);
            cmd_vel -> publish(twist);
            if(distance > 140.0 & distance < 450)
                count += 1;
#ifdef DBG
	    cout << "POS_ARRANGE = 0" << endl;
#endif
        }
        
        ROS_INFO("count = %d\n", count);
        
        //NO Obstacle
        if (distance < 130.0 && count > 15 && POS_ARRANGE == find_pos)
        {
            twist.linear.x = SPEED_0;
            twist.angular.z = ANGLE_0;
            cmd_vel -> publish(twist);
            POS_ARRANGE = rearrange;
#ifdef DBG
	    cout << "POS_ARRANGE = 1" << endl;
#endif
        }
        
        //Rearrange
        if (POS_ARRANGE == rearrange)
        {
            twist.linear.x = SPEED_L;
            twist.angular.z = ANGLE_0;
            cmd_vel -> publish(twist);
#ifdef DBG
	    cout << "POS_ARRANGE = 2" << endl;
#endif
            if(distance > 110 && distance < 450 && count > 15)
            {
#ifdef DBG
		cout << "BACK LASER ON!" << endl;
#endif
                digitalWrite(20, OFF);  //right
                digitalWrite(21, ON);   //back
                right_laser = OFF;
                tofInit(i2c_bus, i2c_address, long_range);
                PARKING_ANGLE = pause_0;
            }
        }
    }
    /*****PARKING*****/
    //right_laser = OFF, back_laser = ON
    else 
    {
        //sleep once
        if(wait_flag == 0)
        {
            sleep(1);
            wait_flag = 1;
        }
        
        //PARKING START
        if (PARKING_ANGLE == pause_0)
        {
#ifdef DBG
            cout <<  "waiting for back laser" << endl;
#endif
            if (distance > 300)
                PARKING_ANGLE = turn_right;
        }
        else if (PARKING_ANGLE == turn_right)
        {
#ifdef DBG
	    cout << "PARKING_ANGLE = 0" << endl;
#endif
            twist.linear.x = -SPEED_P;
            twist.angular.z = 0.1;
            cmd_vel -> publish(twist);
            if (distance < 160)
                PARKING_ANGLE = turn_right2;
        }
	else if(PARKING_ANGLE == turn_right2)
	{
#ifdef DBG
	    cout << "PARKING_ANGLE = 1" << endl;
#endif
            twist.linear.x = -SPEED_P;
            twist.angular.z = 0.1;
            cmd_vel -> publish(twist);
            if (distance < 120)
                PARKING_ANGLE = pause_1;
	}
	else if(PARKING_ANGLE == pause_1)
        {
            sleep(3);
            OUT_ANGLE = out_right1;
	    PARKING_ANGLE = nothing;
        }
	
	if(OUT_ANGLE == out_right1)
	{
#ifdef DBG
	    cout << "OUT_ANGLE = 0" << endl;
#endif
	    twist.linear.x = SPEED_N;
	    twist.angular.z = ANGLE_0;
	    cmd_vel -> publish(twist);
	    if(distance > 160)
		OUT_ANGLE = out_right2;
	}
	else if(OUT_ANGLE == out_right2)
	{
#ifdef DBG
	    cout << "OUT_ANGLE = 1" << endl;
#endif
	    twist.linear.x = SPEED_P;
	    twist.angular.z = -0.15;
	    cmd_vel -> publish(twist);
	    if(distance > 8000)
		OUT_ANGLE = out_pause;
	}
	else if(OUT_ANGLE == out_pause)
	{
#ifdef DBG
	    cout << "OUT_ANGLE = 2" << endl;
#endif
	    sleep(3);
	}
    }
    
    cmd_vel -> publish(twist);
}

void* stopLineStopThread(void *input1)
{
    double t;
    int temp;
    
    sleep(5);
    
    return (void*)temp;
}

//MAIN

int main(int argc, char** argv)
{
    int i2c_bus = 0;
    int i2c_address = 0;
    int long_range = 0;
    double poll_rate = 0;

    ros::init(argc, argv, "control_lane");
    ros::NodeHandle nh, nh_priv("~");
    
    ControlLane *controllane = new ControlLane();
    
    ros::Subscriber sub_lane = nh.subscribe<std_msgs::Int32>("pi/center", 1, &ControlLane::centerCallback, controllane);
    ros::Subscriber sub_event = nh.subscribe<std_msgs::String>("nano/event", 1, &ControlLane::eventCallback, controllane); 
    ros::Subscriber sub_stop = nh.subscribe<std_msgs::Int32>("pi/stop", 1, &ControlLane::stopCallback, controllane);
    ros::Publisher pub_cmd_vel = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    ros::Rate loop_rate(10);

    nh_priv.param("long_range", long_range, 0);
    nh_priv.param("poll_rate", poll_rate, 100.0);
    nh_priv.param("i2c_bus", i2c_bus, 1);
    nh_priv.param("i2c_address", i2c_address, 0x29);
    
    wiringPiSetupGpio();			
    pinMode(20, OUTPUT);		
    pinMode(21, OUTPUT);
    digitalWrite(20, ON);
    digitalWrite(21, OFF);
    
    tofInit(i2c_bus, i2c_address, long_range);
    
    while(nh.ok())
    {
	switch(event_map[event])
	{
		    /***first_parking***/
	    case first_parking:
#ifdef DBG
		cout << "FIRST PARKING!" << endl;
#endif
		controllane -> firstParking(&pub_cmd_vel, i2c_bus, i2c_address, long_range);
		break;

		    /***second_parking***/
	    case second_parking:
#ifdef DBG
		cout << "SECOND PARKING!"<< endl;
#endif
		controllane -> secondParking(&pub_cmd_vel, i2c_bus, i2c_address, long_range);
		break;

		    /***default***/
	    default:
		controllane -> publishCmdVel(&pub_cmd_vel);
		break;
	}
	    
	ros::spinOnce();
	loop_rate.sleep();
    }
    
    return 0;
}
