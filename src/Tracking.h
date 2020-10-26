#ifndef TRACKING_H
#define TRACKING_H

#include "util.h"
#include "Settings.h"

namespace LT_SLAM
{

const int BORDER_SIZE = 32; //consistent with brief
//const int BORDER_SIZE = 20 ; //consistent with brief


class TrackingFrame
{
public:
    TrackingFrame(){
        ;
    }
    TrackingFrame(vector<uchar>& r_status,
                  cv::Mat& img_l,
                  cv::Mat& img_r,
                  vector<cv::Point2f>& cur_pts,
                  vector<cv::Point2f>& cur_pts_right,
                  vector<unsigned int>& ids,
                  int num,
                  unsigned int frameID,
                  MYTIMESTAMP& cur_time):
        r_status(r_status),
        cur_pts(cur_pts),
        cur_pts_right(cur_pts_right),
        ids(ids),
        num(num),
        frameID(frameID),
        cur_time(cur_time)
    {
       cur_img = img_l.clone();
       cur_img_r = img_r.clone();
    }
    vector<uchar> r_status;
    cv::Mat cur_img, cur_img_r;
    vector<cv::Point2f> cur_pts, cur_pts_right;
    vector<unsigned int> ids;
    int num ;
    unsigned int frameID;
    MYTIMESTAMP cur_time;
};

class Tracking
{  
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    static unsigned int featureNextID ;
    static unsigned int frameNextID ;

    Tracking();
    // Computes image bounds for the undistorted image (called in the constructor).
    void ComputeImageBounds(const cv::Mat &imLeft, bool calibrated);
    // Preprocess the input and call Track(). Extract features and performs stereo matching.
    void GrabImageStereo(const cv::Mat &imRectLeft, const cv::Mat &imRectRight, cv::Mat &depth, const MYTIMESTAMP &timestamp);

    std::mutex trackingFrameQueueMutex ;
    std::list<TrackingFrame> trackingFrameQueue ;

    // Current Frame
    cv::Mat mImGray;
    cv::Mat mImGrayRight ;
    cv::Mat imDepth;

    // Main tracking function. It is independent of the input sensor.
    void Track();
    int TrackFeatures(cv::Mat& mImGray, cv::Mat& imGrayRight);
    void rejectWithF();
    void setMask(int ROW, int COL);
    static bool inBorder(const cv::Point2f &pt);

    MYTIMESTAMP cur_time ;
    int trackFeaturesFlag ;
    int trackCnt ;
    int stereoMatchCnt ;

    Eigen::Matrix3d cur_R ;
    Eigen::Vector3d cur_t ;
    vector<uchar> r_status;
    vector<float> r_err;
    cv::Mat mask;
    cv::Mat prev_img, cur_img, forw_img;
    vector<cv::Point2f> n_pts, cur_pts, cur_pts_right, forw_pts, prev_pts ;
    int pre_frame_id ;
    vector<unsigned int> ids;
    vector<int> track_cnt;
    cv::Ptr<cv::CLAHE> clahe;

    vector<cv::Point2f> extra_pts ;
    int extra_pts_num ;
};

} //namespace LT_SLAM

#endif // TRACKING_H
