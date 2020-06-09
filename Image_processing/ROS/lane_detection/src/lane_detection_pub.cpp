#include <iostream>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/Int32.h>
#include <vector>

using namespace std;

static double ym_per_pix = 30.0 / 480;
static double xm_per_pix = 3.7 / 480;
static double h = 480;
static double w = 640;
static int center = 0;

typedef struct warpped_ret
{
    cv::Mat w_img;
    cv::Mat minverse;
}ret1;

//*******FUNCTION**********
cv::Mat color_filter(const cv::Mat &);
ret1 warpping(const cv::Mat &);
cv::Mat roi(const cv::Mat &);
cv::Mat window_roi(const cv::Mat &binary_img, int num);
//*************************

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lane_detect");
    ros::NodeHandle nh;
    ros::Publisher img_pub = nh.advertise<std_msgs::UInt8MultiArray>("pi/image", 1);
    ros::Publisher center_pub = nh.advertise<std_msgs::Int32>("pi/lane", 1);

    cv::VideoCapture cap(0);

    cv::Mat mtx = (cv::Mat1d(3, 3) << 375.02024751, 0., 316.52572289, 0., 490.14999206, 288.56330145, 0., 0., 1.);
    cv::Mat dist = (cv::Mat1d(1, 5) << -0.30130634,  0.09320542, - 0.00809047,  0.00165312, - 0.00639115);
    cv::Mat newcameramtx = (cv::Mat1d(3, 3) << 273.75825806, 0., 318.4331204, 0., 391.74940796, 283.77532838, 0., 0., 1.);

    ret1 r1;

    while(nh.ok())
    {
        cv::Mat src;
        
        cap.read(src);

        if(!src.empty())
        {
            cv::Mat filtered_img, rotate_img, img, warped_img, minv, roi_img, out;
            vector<uchar> encode;
            vector<int> encode_param;

            encode_param.push_back(cv::IMWRITE_JPEG_QUALITY);  //jpg format compressing
            encode_param.push_back(20);  //compressed in 20%

            //Color filter
            filtered_img = color_filter(src);

            //180도 rotation
            cv::flip(filtered_img, rotate_img, -1);

            //Fisheye lense calibration
            cv::undistort(rotate_img, img, mtx, dist, newcameramtx);

            //Warpped img
            r1 = warpping(img);
            warped_img = r1.w_img;
            minv = r1.minverse;

            //ROI img
            roi_img = roi(warped_img);
            
            //Window ROI
            out = window_roi(roi_img, 4);

            cv::imshow("result", out);

            //cv::imencode(".jpg", out, encode, encode_param);  //encode -> unsigned char array

            //std_msgs::UInt8MultiArray msgArray;
            std_msgs::Int32 lane_center;
            lane_center.data = center;
            //msgArray.data.clear();
            //msgArray.data.resize(encode.size());
            //std::copy(encode.begin(), encode.end(), msgArray.data.begin());  //copy to msgArray

            //img_pub.publish(msgArray);
            center_pub.publish(lane_center);

            int key = cv::waitKey(2);
            if(key == 27)
                break;
        }
        ros::spinOnce();
    }

    return 0;
}

cv::Mat color_filter(const cv::Mat &image)
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

ret1 warpping(const cv::Mat &image)
{
    ret1 r;
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

cv::Mat roi(const cv::Mat &image)
{
    cv::Mat mask = cv::Mat::zeros(int(h), int(w), CV_8U);
    cv::Mat masked_img;
    vector<vector<cv::Point2i>> shape = { { {0, 0},{int(w) - 100, 0},{int(w) - 100, int(h)},{0, int(h)} } };
    cv::fillPoly(mask, shape, 255);
    cv::bitwise_and(image, mask, masked_img);

    return masked_img;
}

cv::Mat window_roi(const cv::Mat &binary_img, int num) {
    cv::Mat out_img, img_cp;
    vector<int> max_wleft, max_wright;
    int max_left = 0, max_right = 0, max = 0, margin = 30, minpix = 50, thickness = 2;
    cv::Scalar left_color(255, 0, 0);
    cv::Scalar right_color(0, 0, 255);
    vector<cv::Mat> temp = {binary_img, binary_img, binary_img};
    merge(temp, out_img);
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
        cout << "LEFT LINE" << endl;
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
        cout << "RIGHT LINE" << endl;
        if (max_wright[3] > 0)
            max = max_wright[3];
        else
            max = max_wright[2];
        center = max - 102;
        cout << center << endl;
    }
    
    else
    {
        cout << "LEFT LINE " <<endl;
        if (max_wleft[3] > 0)
            max = max_wleft[3];
        else
            max = max_wleft[2];
        center = max + 102;
        cout << center << endl;
    }

    return out_img;
}

