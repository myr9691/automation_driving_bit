#ifndef LANE_DETECTION_PUB_H
#define LANE_DETECTION_PUB_H

class LANEDETECTOR
{
    public:
        //생성자
        LANEDETECTOR();
        
        struct warpped_ret
        {
            cv::Mat w_img;
            cv::Mat minverse;
        };
        
        //FUNCTION
        cv::Mat colorFilter(const cv::Mat &);
        warpped_ret warping(const cv::Mat &);
        cv::Mat roi(const cv::Mat &);
        cv::Mat windowRoi(const cv::Mat &binary_img, int num, ros::Publisher *center_pub);
        void stopLine(const cv::Mat &binary_img, ros::Publisher *stop_pub);
        
    private:
        double h = 0;
        double w = 0;
};

#endif