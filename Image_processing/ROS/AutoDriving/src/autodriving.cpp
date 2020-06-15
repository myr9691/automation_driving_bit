#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"
#include "autorace.h"
#include "lane_detection_pub.h"

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
	avoid_left,
	clear
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
	{"avoid_left", avoid_left},
	{"clear", clear}
};

/*********LaneDetector**********/

LaneDetector::LaneDetector() : h(480), w(640)
{
}

int LaneDetector::center = 0;
int LaneDetector::flag = 0;

cv::Mat LaneDetector::colorFilter(const cv::Mat &image) const
{
    cv::Mat hsv, v, white, yellow, or_img;
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

cv::Mat LaneDetector::windowRoi(const cv::Mat &binary_img, int num)
{
    cv::Mat out_img, img_cp;
    vector<int> max_wleft, max_wright;
    int max_left = 0, max_right = 0, max = 0, margin = 30, minpix = 50, thickness = 2;
    cv::Scalar left_color(255, 0, 0);
    cv::Scalar right_color(0, 0, 255);
    const double *next;
    vector<cv::Mat> temp = {binary_img, binary_img, binary_img};
    merge(temp, out_img);

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
        cout << center << endl;
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
        cout << center << endl;
    }
    
    else
    {
        //cout << "LEFT LINE " <<endl;
        if (max_wleft[3] > 0)
            max = max_wleft[3];
        else
            max = max_wleft[2];
        center = max + 102;
        cout << center << endl;
    }

    return out_img;
}

void LaneDetector::stopLine(const cv::Mat &binary_img)
{
    int horiz_size;
    cv::Mat hist, img_cp, horizontal, horizontal_img;
    
    cv::Rect rect(0, h-80, w, 10);
    img_cp = binary_img(rect);
    horizontal = img_cp.clone();
    
    horiz_size = img_cp.cols/20;
    
    horizontal_img = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(horiz_size, 1));
    cv::erode(horizontal, horizontal, horizontal_img, cv::Point(-1, -1));
    cv::dilate(horizontal, horizontal, horizontal_img, cv::Point(-1, -1));
    
    for (int i = 0; i < horizontal.cols; i++)
        hist.push_back(cv::sum(img_cp.col(i))[0]);
    if(flag == 0 && *hist.ptr<double>(hist.rows/2, 1) > 800)
    {
	cout << *hist.ptr<double>(hist.rows/2, 1) << endl;
	cout << "S T O P !!" << endl;
        sleep(5);
	flag = 1;
    }
}

/*********ControlLane**********/

ControlLane::ControlLane() : lastError(0)
{
}

int ControlLane::stop = 1;

void ControlLane::stateCallback(const std_msgs::String::ConstPtr& nano_state)
{
    enum Statement STATE;

	state = nano_state->data.c_str();

    switch(statement[state])
	{
		case person:
		case red:
		    stop = 1;
		    break;
		case green:
		    stop = 0;
		    break;
		default:
		    stop = 0;
		    break;
	}
}

void ControlLane::publishCmdVel(ros::Publisher *cmd_vel)
{
	geometry_msgs::Twist twist;

    enum Statement STATE;
	
	switch(statement[state])
	{
		case person:
		case red:
			MAX_VEL = 0.0;
			break;
		case green:
			MAX_VEL = 0.05;
		case avoid_right:
		case no_right_turn:
			LaneDetector::flag = 0;
			MAX_VEL = 0.05;
			break;
		case school_zone_off:
		case roundabout:
		case tunnel:
		case first_parking:
		case second_parking:
		case avoid_left:
			LaneDetector::flag = 0;
			MAX_VEL = 0.05;
			break;
		case speed_up:
			LaneDetector::flag = 0;
			MAX_VEL = 0.1;
			break;
		case school_zone:
			LaneDetector::flag = 0;
			MAX_VEL = 0.02;
			break;
		case clear:
			MAX_VEL = 0.05;
			break;
		default :
			MAX_VEL = 0.05;
			break;
	}

	cout << "MAX_VEL = " << MAX_VEL << endl;
	
	error = LaneDetector::center - 327;

	if(stop)
		angular_z = 0;
	else
		angular_z = Kp * error + Kd * (error - lastError);

	cout << "angular_z = " << angular_z << endl;
	cout << "stop = " << stop << endl;
	
	lastError = error;
	twist.linear.x = std::min(MAX_VEL * pow(1 - abs(error) / 327, 2.2), 0.2);
	twist.linear.y = 0;
	twist.linear.z = 0;
	twist.angular.x = 0;
	twist.angular.y = 0;
	twist.angular.z = angular_z < 0 ? -std::max(angular_z, -angle) : -std::min(angular_z, angle);
	
	cmd_vel -> publish(twist);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "AutoDriving");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher img_pub = it.advertise("pi/image", 1);

    LaneDetector *lane_detector = new LaneDetector();
    ControlLane *controllane = new ControlLane();

    ros::Publisher pub_cmd_vel = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    ros::Subscriber sub_state = nh.subscribe<std_msgs::String>("nano/status", 1, &ControlLane::stateCallback, controllane);

    cv::VideoCapture cap(0);

    cv::Mat mtx = (cv::Mat1d(3, 3) << 375.02024751, 0., 316.52572289, 0., 490.14999206, 288.56330145, 0., 0., 1.);
    cv::Mat dist = (cv::Mat1d(1, 5) << -0.30130634,  0.09320542, - 0.00809047,  0.00165312, - 0.00639115);
    cv::Mat newcameramtx = (cv::Mat1d(3, 3) << 273.75825806, 0., 318.4331204, 0., 391.74940796, 283.77532838, 0., 0., 1.);
    
    LaneDetector::warpped_ret r1;
    
    sensor_msgs::ImagePtr Image;

    while(nh.ok())
    {
        
        cv::Mat src;
        
        cap.read(src);

        if(!src.empty())
        {
            //double st=static_cast<double>(cv::getTickCount());
            cv::Mat filtered_img, rotate_img, img, warped_img, minv, roi_img, out, save_img;

            //Color filter
            filtered_img = lane_detector -> colorFilter(src);

            //180도 rotation
            cv::flip(filtered_img, rotate_img, -1);

            //Fisheye lense calibration
            cv::undistort(rotate_img, img, mtx, dist, newcameramtx);
            
            //Warpped img
            r1 = lane_detector -> warping(img);
            warped_img = r1.w_img;
            minv = r1.minverse;

            //ROI img
            roi_img = lane_detector -> roi(warped_img);
            
            //Window ROI
            out = lane_detector -> windowRoi(roi_img, 4);
            
            Image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", out).toImageMsg();
            img_pub.publish(Image);
            
            //Find Stop Line
            lane_detector -> stopLine(roi_img);
        
            cv::imshow("result", out);

            int key = cv::waitKey(2);
            if(key == 27)
                break;
        }

        controllane -> publishCmdVel(&pub_cmd_vel);

        ros::spinOnce();
    }

    return 0;
}
