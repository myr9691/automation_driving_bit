#ifndef LANE_DETECTION_PUB_H
#define LANE_DETECTION_PUB_H

class LaneDetector
{
    public:
        //생성자
        LaneDetector();
        
        struct warpped_ret
        {
            cv::Mat w_img;
            cv::Mat minverse;
        };

        static int center;
        static int flag;
        
        //FUNCTION
        cv::Mat colorFilter(const cv::Mat &) const;
        warpped_ret warping(const cv::Mat &) const;
        cv::Mat roi(const cv::Mat &) const;
        cv::Mat windowRoi(const cv::Mat &binary_img, int num);
        void stopLine(const cv::Mat &binary_img);
        
    private:
        double h = 0;
        double w = 0;
};

#endif
