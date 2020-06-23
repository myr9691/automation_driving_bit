#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <wiringPi.h>
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"
#include "autorace.h"
#include "lane_detection_pub.h"
#include "tof.h"
#include "time.h"
#include "pthread.h"

#define SPEED_0 0.0
#define SPEED_N 0.05
#define SPEED_P 0.02
#define SPEED_L -0.01
#define ANGLE_0 0.0

#define ON 1
#define OFF 0

//#define DBG

using namespace std;

void* stopLineStopThread(void *ret);
void* stopLinePassThread(void *ret);

bool stop_line_detect = true;
bool thread_run = false;

std::string event = "clear";

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

/*********LaneDetector**********/

LaneDetector::LaneDetector() : h(480), w(640)
{
}

int LaneDetector::center = 0;

cv::Mat LaneDetector::colorFilter(const cv::Mat &image) const
{
    cv::Mat hsv, v, white, yellow, or_img, md;
    cv::Mat yellow_lower = (cv::Mat1d(1, 3) << 5, 90, 100);
    cv::Mat yellow_upper = (cv::Mat1d(1, 3) << 200, 255, 255);
    vector<cv::Mat> hsv_split;
    cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
    cv::split(hsv, hsv_split);
    v = hsv_split[2];
    cv::inRange(v, 245, 255, white);
    cv::inRange(hsv, yellow_lower, yellow_upper, yellow);
    cv::bitwise_or(white, yellow, or_img);
    cv::flip(yellow, yellow, -1);
    cv::flip(white, white, -1);

    return or_img;
}

cv::Mat LaneDetector::lightRemoval(const cv::Mat &image)
{
    cv::Mat lab, l, md, remove_img;
    vector<cv::Mat> lab_split;
    cv::cvtColor(image, lab, cv::COLOR_BGR2Lab);
    cv::split(lab, lab_split);
    l = lab_split[0]; 
    cv::medianBlur(l, md, 91);
    
    remove_img = l - md*0.8;
    
    return remove_img;
}

LaneDetector::warpped_ret LaneDetector::warping(const cv::Mat &image) const
{
    warpped_ret r;
    cv::Mat w_img, transform_matrix, minv;
    vector<cv::Point2f> source = {cv::Point2f(0.4 * w, 0.45 * h), cv::Point2f(0.6 * w, 0.45 * h), cv::Point2f(0.0, h), cv::Point2f(w, h)};
    vector<cv::Point2f> destination = {cv::Point2f(0.27 * w, 0), cv::Point2f(0.65 * w, 0), cv::Point2f(0.27 * w, h), cv::Point2f(0.73 * w, h)};
    transform_matrix = cv::getPerspectiveTransform(source, destination);
    minv = cv::getPerspectiveTransform(destination, source);
    cv::warpPerspective(image, w_img, transform_matrix, cv::Size(w, h));
    r.w_img = w_img;
    r.minverse = minv;

    return r;
}

cv::Mat LaneDetector::roi(const cv::Mat &image) const
{
    cv::Mat mask = cv::Mat::zeros(int(h), int(w), CV_8U);
    cv::Mat masked_img;
    vector<vector<cv::Point2i>> shape = { { {0, 0},{int(w) - 100, 0},{int(w) - 100, int(h)},{0, int(h)} } };
    cv::fillPoly(mask, shape, 255);
    cv::bitwise_and(image, mask, masked_img);

    return masked_img;
}

/*cv::Mat LaneDetector::windowRoi(const cv::Mat &binary_img, int num)
{
    cv::Mat out_img, img_cp, nonzero_x, nonzero_y, good_left, good_right, mean_left;
    vector<int> max_wleft, max_wright, left_v, right_v;
    int max_left = 0, max_right = 0, max = 0, margin = 30, minpix = 50, thickness = 2;
    cv::Scalar left_color(255, 0, 0);
    cv::Scalar right_color(0, 0, 255);
    const double *next;
    vector<cv::Mat> temp = {binary_img, binary_img, binary_img};
    merge(temp, out_img);
    vector<cv::Point> nonzero; 
    cv::Point pnt;
    cv::Scalar ml;
    int line_center = 130;
    
    /*cv::findNonZero(binary_img, nonzero);
    
    for(int i=0; i<nonzero.size(); i++)
    {
	pnt = nonzero[i];
	nonzero_x.push_back(pnt.x);
	nonzero_y.push_back(pnt.y);
    }
    
    for (int wd=0;wd<num;wd++)
    {
        cv::Mat hist;
	double max_l = 0, max_r = 0;
	max_left = 0; max_right = 0; max = 0;
        
        cv::Rect rect(0, h-40*(wd+1), w, 40);
        img_cp = binary_img(rect);
	
	for (int i = 0; i < img_cp.cols; i++)
	hist.push_back(cv::sum(img_cp.col(i))[0]);

	for (int r = 0; r < hist.rows/2; r++) 
	{
	    next = hist.ptr<double>(r, 1);
	    if (max_l < *next) {
		max_l = *next;
		max_left = r;
	    }
	}
	
	for (int r = hist.rows/2; r < hist.rows; r++) 
	{
	    next = hist.ptr<double>(r, 1);
	    if (max_r < *next) {
		max_r = *next;
		max_right = r;
	    }
	}

	if (max_left == 0)
	    max_left = 240;
	if (max_right == 639 || max_right == 0)
	    max_right = 506;
        
        max_wleft.push_back(max_left);
        max_wright.push_back(max_right);
	left_v.push_back(max_l);
	right_v.push_back(max_r);
    
		/*good_left = ((nonzero_y >= win_y_top) & (nonzero_y < win_y_bottom) & (nonzero_x >= win_lt) & (nonzero_x < win_lb));
	good_right = ((nonzero_y >= win_y_top) & (nonzero_y < win_y_bottom) & (nonzero_x >= win_rt) & (nonzero_x < win_rb));
	
	if (good_left.rows > minpix)
	{
	    for (int i = 0; i < good_left.rows; i++)
	    {
		mean_left.push_back(*nonzero_x.ptr<int>(i));
	    }
	    ml = cv::mean(mean_left);
	}
    }
    
    //vector<int> leftfit = polyfit_Eigen(max_wleft, nonzero_y, 2);
    //vector<int> rightfit = polyfit_Eigen(max_wright, nonzero_y, 2);
    
    //cout << leftfit[0] << " " << leftfit[1] << " " << leftfit[2] << endl;
    
    //우회전 (왼쪽 선 참조)
    if (max_wleft[1] - max_wleft[0] > 0)
    {
        cout << "LEFT LINE" << endl;
        if (max_wleft[5] > 0)
            max = max_wleft[5];
        center = max + line_center;
        cout << center << endl;
	for (int wd=0;wd<num;wd++)
	{
	    int win_y_top = int(h)-(40*(wd+1));
	    int win_y_bottom = win_y_top + 40;
	    int win_lt = max_wleft[wd] - margin;
	    int win_lb = max_wleft[wd] + margin;
	    if (left_v[wd] > 400 || max_wleft[wd] > max_wleft[0])
	    {
		cv::rectangle(out_img, cv::Point(win_lt, win_y_top), cv::Point(win_lb, win_y_bottom), left_color, thickness);
		cv::circle(out_img, cv::Point(max_wleft[wd] + line_center, int(h)-(40*(wd+1))), 5, cv::Scalar(255, 0, 0), -1);
	    }
	}
    }
    
    //좌회전 (오른쪽 선 참조)
    else if (max_wright[0] - max_wright[1] > 0)
    {
        cout << "RIGHT LINE" << endl;
        if (max_wright[5] > 0)
            max = max_wright[5];
        center = max - line_center;
        cout << center << endl;
	for (int wd=0;wd<num;wd++)
	{
	    int win_y_top = int(h)-(40*(wd+1));
	    int win_y_bottom = win_y_top + 40;
	    int win_rt = max_wright[wd] - margin;
	    int win_rb = max_wright[wd] + margin;
	    if (right_v[wd] > 400 || max_wright[wd] > max_wright[0])
	    {
		cv::rectangle(out_img, cv::Point(win_rt, win_y_top), cv::Point(win_rb, win_y_bottom), right_color, thickness);
		cv::circle(out_img, cv::Point(max_wright[wd] - line_center, int(h)-(40*(wd+1))), 5, cv::Scalar(0, 0, 255), -1);
	    }
	}
    }
    
    else
    {
        cout << "LEFT LINE " <<endl;
        if (max_wleft[5] > 0)
            max = max_wleft[5];
        center = max + line_center;
        cout << center << endl;
	for (int wd=0;wd<num;wd++)
	{
	    int win_y_top = int(h)-(40*(wd+1));
	    int win_y_bottom = win_y_top + 40;
	    int win_lt = max_wleft[wd] - margin;
	    int win_lb = max_wleft[wd] + margin;
	    int win_rt = max_wright[wd] - margin;
	    int win_rb = max_wright[wd] + margin;
	    if (left_v[wd] > 400)
	    {
		cv::rectangle(out_img, cv::Point(win_lt, win_y_top), cv::Point(win_lb, win_y_bottom), left_color, thickness);
		cv::circle(out_img, cv::Point(max_wleft[wd] + line_center, int(h)-(40*(wd+1))), 5, cv::Scalar(255, 0, 0), -1);
	    }
	    if (right_v[wd] > 400)
		cv::rectangle(out_img, cv::Point(win_rt, win_y_top), cv::Point(win_rb, win_y_bottom), right_color, thickness);
	}
    }
    
    cout << max_wleft[4] << endl;
    cout << max_wright[4] << endl;
    
    
    return out_img;
}*/


cv::Mat LaneDetector::windowRoi(const cv::Mat &binary_img, int num) 
{
    cv::Mat out_img = binary_img.clone(), img_cp;
    vector<int> max_wleft, max_wright;
    int max_left = 0, max_right = 0, max = 0, margin = 30, minpix = 50, thickness = 2;
    cv::Scalar left_color(255, 0, 0);
    cv::Scalar right_color(0, 0, 255);
    const double *next;

    for (int wd=0;wd<num;wd++)
    {
        cv::Mat hist;
        double max_l = 0, max_r = 0;
        max_left = 0; max_right = 0; max = 0;
        int win_y_top = int(h)-(40*(wd+1));
        int win_y_bottom = win_y_top + 40;
        
        cv::Rect rect(0, h-40*(wd+1), w, 40);
        img_cp = binary_img(rect);
        
        for (int i = 0; i < img_cp.cols; i++)
            hist.push_back(cv::sum(img_cp.col(i))[0]);

        for (int r = 0; r < hist.rows/2; r++) 
        {
            next = hist.ptr<double>(r, 1);
            if (max_l < *next) {
                max_l = *next;
                max_left = r;
            }
        }
        
        for (int r = hist.rows/2; r < hist.rows; r++) 
        {
            next = hist.ptr<double>(r, 1);
            if (max_r < *next) {
                max_r = *next;
                max_right = r;
            }
        }
        
        max_wleft.push_back(max_left);
        max_wright.push_back(max_right);
        
        if (wd < 3)
        {
            int win_lt = max_left - margin;
            int win_lb = max_left + margin;
            int win_rt = max_right - margin;
            int win_rb = max_right + margin;
            cv::rectangle(out_img, cv::Point(win_lt, win_y_top), cv::Point(win_lb, win_y_bottom), left_color, thickness);
            cv::rectangle(out_img, cv::Point(win_rt, win_y_top), cv::Point(win_rb, win_y_bottom), right_color, thickness);
        }
    }
    
    //우회전 (왼쪽 선 참조)
    if (max_wleft[1] - max_wleft[0] > 0)
    {
        //cout << "LEFT LINE" << endl;
        if (max_wleft[3] > 0)
            max = max_wleft[3];
        else
            max = max_wleft[2];
        center = max + 102;
    }
    
    //좌회전 (오른쪽 선 참조)
    else if (max_wright[0] - max_wright[1] > 0)
    {
        //cout << "RIGHT LINE" << endl;
        if (max_wright[3] > 0)
            max = max_wright[3];
        else
            max = max_wright[2];
        center = max - 102;
    }
    
    else
    {
        //cout << "LEFT LINE " <<endl;
        if (max_wleft[3] > 0)
            max = max_wleft[3];
        else
            max = max_wleft[2];
        center = max + 102;
    }

    return out_img;
}

void LaneDetector::stopLine(const cv::Mat &binary_img)
{
    int horiz_size, err = 0;
    pthread_t pth;
    int t = 0;
    int input = 0;
    cv::Mat hist, img_cp, horizontal, horizontal_img;
    
    cv::Rect rect(center - 102, h-120, 102, 40);
    img_cp = binary_img(rect);
    horizontal = img_cp.clone();
    
    horiz_size = img_cp.cols/20;
    
    horizontal_img = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(horiz_size, 1));
    cv::erode(horizontal, horizontal, horizontal_img, cv::Point(-1, -1));
    cv::dilate(horizontal, horizontal, horizontal_img, cv::Point(-1, -1));
    
    for (int i = 0; i < horizontal.cols; i++)
        hist.push_back(cv::sum(img_cp.col(i))[0]);
	
    if(*hist.ptr<double>(hist.rows/2, 1) > 400)
    {
#ifdef DBG
	cout << "S T O P !!" << endl;
#endif
	
	if (err = pthread_create(&pth, NULL, stopLineStopThread, (void*)&input) < 0)
	{
	    perror("Thread1 error : ");
	    exit(2);
	}
	pthread_join(pth, (void**)t);
	stop_line_detect = false;
    }
}

void* stopLineStopThread(void *input1)
{
    double t;
    int temp;
    
    sleep(5);
    
    return (void*)temp;
}

void* stopLinePassThread(void *input2)
{
    int temp;
    thread_run = true;
    
    sleep(2);
    
    stop_line_detect = false;
    thread_run = false;
    
    return (void*)temp;
}


/*********ControlLane**********/

ControlLane::ControlLane() : lastError(0)
{
}

void ControlLane::eventCallback(const std_msgs::String::ConstPtr& nano_event)
{
    event = nano_event->data.c_str();
}

void ControlLane::publishCmdVel(ros::Publisher *cmd_vel)
{
    geometry_msgs::Twist twist;
    
    static enum State state = driving_start;
    static enum State pre_state = driving_start;
    static enum Event event_enum;
    
    switch(state)
    {
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
	case Drivex1:
#ifdef DBG
	    cout << "STATE = Drivex1" << endl;
#endif
	    MAX_VEL = 0.05;
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
	    else if (event_map[event] == clear)
		state = Drivex1;
	    else if (event_map[event] == no_right_turn)
		state = Drivex1;
	    else if (event_map[event] == roundabout)
		state = Drivex0_red;
	    else if (event_map[event] == tunnel)
		state = Drivex1;
	    break;
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
    
    error = LaneDetector::center - 327;

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

    error = LaneDetector::center - 327;
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

    error = LaneDetector::center - 327;
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



int main(int argc, char** argv)
{
    int i2c_bus = 0;
    int i2c_address = 0;
    int long_range = 0;
    double poll_rate = 0;
    int err = 0;
    pthread_t pth;
    int input = 0;
    int t = 0;
    
    ros::init(argc, argv, "AutoDriving");
    ros::NodeHandle nh, nh_priv("~");
    image_transport::ImageTransport it(nh);
    image_transport::Publisher img_pub = it.advertise("pi/image", 1);
    
    LaneDetector *lane_detector = new LaneDetector();
    ControlLane *controllane = new ControlLane();
    
    ros::Publisher pub_cmd_vel = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    ros::Subscriber sub_event = nh.subscribe<std_msgs::String>("nano/event", 1, &ControlLane::eventCallback, controllane);

    cv::VideoCapture cap(0);

    cv::Mat mtx = (cv::Mat1d(3, 3) << 375.02024751, 0., 316.52572289, 0., 490.14999206, 288.56330145, 0., 0., 1.);
    cv::Mat dist = (cv::Mat1d(1, 5) << -0.30130634,  0.09320542, - 0.00809047,  0.00165312, - 0.00639115);
    cv::Mat newcameramtx = (cv::Mat1d(3, 3) << 273.75825806, 0., 318.4331204, 0., 391.74940796, 283.77532838, 0., 0., 1.);
    
    LaneDetector::warpped_ret r1;
    
    sensor_msgs::ImagePtr Image;

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
        
        cv::Mat src;
        
        cap.read(src);

        if(!src.empty())
        {
            double st=static_cast<double>(cv::getTickCount());
            cv::Mat filtered_img, remove_img, thresh, rotate_img, img, warped_img, minv, roi_img, out, save_img;
	    
            //Color filter
            filtered_img = lane_detector -> colorFilter(src);
	    
	    //Light Remove
	    remove_img = lane_detector -> lightRemoval(src);
	    
	    cv::threshold(remove_img, thresh, 90, 255, CV_THRESH_BINARY);
	    
            //180도 rotation
            cv::flip(thresh, rotate_img, -1);

            //Fisheye lense calibration
            cv::undistort(rotate_img, img, mtx, dist, newcameramtx);
            
	    //Warpped img
            r1 = lane_detector -> warping(img);
            warped_img = r1.w_img;
            minv = r1.minverse;

            //ROI img
            roi_img = lane_detector -> roi(warped_img);
            
            //Window ROI
            out = lane_detector -> windowRoi(roi_img, 5);
            
            //Image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", out).toImageMsg();
            //img_pub.publish(Image);
            
	    /*if (stop_line_detect)
		lane_detector -> stopLine(img);
	    else if(!stop_line_detect && !thread_run)
	    {
		if (err = pthread_create(&pth, NULL, stopLinePassThread, (void*)&input) < 0)
		{
		    perror("Thread2 error : ");
		    exit(2);
		}
		pthread_detach(pth);
	    }
	    if(stop_line_detect == true){
#ifdef DBG
		cout << "STOP LINE DETECT = 1" << endl;
#endif}
	    else{
#ifdef DBG
		cout << "STOP LINE DETECT = 0" << endl;
#endif}
#ifdef DBG
	    cout << "THREAD RUN = " << thread_run << endl;
#endif*/

            cv::imshow("result", out);
	    
	    double ed = static_cast<double>(cv::getTickCount());
#ifdef DBG
	    cout << "FRAME = " << ed - st << endl;
#endif

            int key = cv::waitKey(2);
            if(key == 27)
                break;
        }

        switch(event_map[event])
        {
            case first_parking:
#ifdef DBG
		cout << "FIRST PARKING!" << endl;
#endif
                controllane -> firstParking(&pub_cmd_vel, i2c_bus, i2c_address, long_range);
                break;
            case second_parking:
#ifdef DBG
		cout << "SECOND PARKING!"<< endl;
#endif
                controllane -> secondParking(&pub_cmd_vel, i2c_bus, i2c_address, long_range);
                break;
            default:
                controllane -> publishCmdVel(&pub_cmd_vel);
                break;
        }
#ifdef DBG
	cout << "--------------------------" << endl;
#endif

        ros::spinOnce();
    }

    return 0;
}
