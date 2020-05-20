#include <iostream>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/UInt8MultiArray.h>
#include <vector>

using namespace std;

static double ym_per_pix = 30.0 / 480;
static double xm_per_pix = 3.7 / 480;
static double h = 480;
static double w = 640;
static int midpoint = 300;

typedef struct warpped_ret
{
    cv::Mat w_img;
    cv::Mat minverse;
}ret1;

//*******FUNCTION**********
//ret1 calibration(void);
ret1 warpping(const cv::Mat &image);
cv::Mat roi(const cv::Mat &image);
cv::Mat plothistogram(const cv::Mat &);
cv::Mat window_roi(const cv::Mat &binary_img, const cv::Mat &hist, int midpoint, const cv::Mat &warped_img, int num);
//*************************

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lane_detect");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::UInt8MultiArray>("camera/image", 1);

    cv::VideoCapture cap("/home/moon/Downloads/drive_test_01.avi");

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
            cv::Mat gray, not_gray, img, warped_img, minv, roi_img, thresh, hist, out;
            vector<uchar> encode;
            vector<int> encode_param;

            encode_param.push_back(cv::IMWRITE_JPEG_QUALITY);  //jpg format compressing
            encode_param.push_back(20);  //compressed in 20%

            //Gray scale
            cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);
            not_gray = ~gray;

            //Fisheye lense calibration
            cv::undistort(not_gray, img, mtx, dist, newcameramtx);

            //Warpped img
            r1 = warpping(img);
            warped_img = r1.w_img;
            minv = r1.minverse;

            //ROI img
            roi_img = roi(warped_img);

            //Threshold
            cv::threshold(roi_img, thresh, 180, 255, cv::THRESH_BINARY);

            //Histogram
            hist = plothistogram(thresh);

            //Window ROI
            out = window_roi(thresh, hist, midpoint, warped_img, 1);

            cv::imshow("result", out);

            cv::imencode(".jpg", out, encode, encode_param);  //encode -> unsigned char array

            std_msgs::UInt8MultiArray msgArray;
            msgArray.data.clear();
            std::copy(encode.begin(), encode.end(), msgArray.data.begin());  //copy to msgArray

            pub.publish(msgArray);

            int key = cv::waitKey(2);
            if(key == 27)
                break;
        }
        ros::spinOnce();
    }
    return 0;
}

ret1 warpping(const cv::Mat &image)
{
    ret1 r;
    cv::Mat w_img, transform_matrix, minv;
    vector<cv::Point2f> source = {cv::Point2f(0.4 * w, 0.4 * h), cv::Point2f(0.6 * w, 0.4 * h), cv::Point2f(0.0, h), cv::Point2f(w, h)};
    vector<cv::Point2f> destination = {cv::Point2f(0.27 * w, 0), cv::Point2f(0.67 * w, 0), cv::Point2f(0.27 * w, h), cv::Point2f(0.73 * w, h)};
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
    vector<vector<cv::Point2i>> shape = { { {int(0.2 * w), int(h - 200)},{int(0.8 * w), int(h - 200)},{int(0.8 * w), int(h)},{int(0.2 * w), int(h)} } };
    cv::fillPoly(mask, shape, 255);
    cv::bitwise_and(image, mask, masked_img);
    return masked_img;
}

cv::Mat plothistogram(const cv::Mat &image)
{
    cv::Mat hist;
    for (int i = 0; i < image.cols; i++)
        hist.push_back(cv::sum(image.col(i))[0]);
    return hist;
}

cv::Mat window_roi(const cv::Mat &binary_img, const cv::Mat &hist, int midpoint, const cv::Mat &warped_img, int num) {
    cv::Mat out_img, nonzero, nonzero_x, nonzero_y;
    //binary_img.copyTo(out_img);
    int max_left = 0, max_right = 0, margin = 30, minpix = 50, thickness = 2;
    cv::Scalar color(0, 255, 0);
    vector<cv::Mat> temp = {binary_img, binary_img, binary_img};
    merge(temp, out_img);
    const double *next;
    double max = 0;
    for (int r = 0; r < hist.rows / 2; r++) {
        next = hist.ptr<double>(r, 1);
        if (max < *next) {
            max = *next;
            max_left = r;
        }
    }
    max = 0;
    for (int r = hist.rows / 2; r < hist.rows; r++) {
        next = hist.ptr<double>(r, 1);
        if (max < *next) {
            max = *next;
            max_right = r;
        }
    }
    if (max_left < 100)
        max_left = 195;
    if (max_right < 310)
        max_right = 415;

    for (int wd=0;wd<num;wd++)
    {
        int win_y_top = int(h)-(100*(int(wd)+1));
        int win_y_bottom = int(h)-(100*int(wd));
        if(max_left < 150 | max_left > 250)
            max_left = max_right - 230;
        if(max_right < 380 | max_right > 450)
            max_right = max_left + 230;
        int win_xleft_top = max_left - margin;
        int win_xleft_bottom = max_left + margin;
        int win_xright_top = max_right - margin;
        int win_xright_bottom = max_right + margin;

        cv::rectangle(out_img, cv::Point(win_xleft_top, win_y_top), cv::Point(win_xleft_bottom, win_y_bottom), color, thickness);
        cv::rectangle(out_img, cv::Point(win_xright_top, win_y_top), cv::Point(win_xright_bottom, win_y_bottom), color, thickness);

    }

    return out_img;
}

