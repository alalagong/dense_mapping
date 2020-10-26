#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <memory>
#include <functional>
#include<list>
#include<vector>
#include<ros/ros.h>
#include<cv_bridge/cv_bridge.h>
#include"boost/thread.hpp"
#include<opencv2/core/core.hpp>
#include<rosbag/bag.h>
#include<rosbag/chunked_file.h>
#include<rosbag/view.h>
#include<rosbag/query.h>
#include"src/System.h"
#include <stdio.h>

#define BACKWARD_HAS_DW 1
#include <backward.hpp>
namespace backward
{
backward::SignalHandling sh;
} // namespace backward

class ImageGrabber
{
public:
    ImageGrabber(){;}
    ~ImageGrabber(){;}

    LT_SLAM::System* mpSLAM;
    bool do_rectify = true;
    cv::Mat M1l,M2l,M1r,M2r;
}igb;

class ImageMeasurement
{
public:
    ros::Time t;
    cv::Mat   image;

    ImageMeasurement(const ros::Time& _t, const cv::Mat& _image)
    {
        t     = _t;
        image = _image.clone();
    }

    ImageMeasurement(const ImageMeasurement& i)
    {
        t     = i.t;
        image = i.image.clone();
    }

    ~ImageMeasurement() { ;}
};

std::list<ImageMeasurement> image0Buf;
std::list<ImageMeasurement> image1Buf;
std::mutex mMutexImg0;
std::mutex mMutexImg1;
LT_SLAM::System* pSystem ;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lt_slam");
    ros::start();
    ros::NodeHandle nh("~") ;
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Warn);

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun LT_SLAM stereo path_to_vocabulary path_to_settings" << endl;
        ros::shutdown();
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    LT_SLAM::System SLAM(argv[1],argv[2],LT_SLAM::STEREO);
    igb.mpSLAM = &SLAM ;

    // Load settings related to stereo calibration
    cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        cerr << "ERROR: Wrong path to settings" << endl;
        return -1;
    }
    int preRectified = fsSettings["preRectified"] ;
    if ( preRectified > 0 ){
        igb.do_rectify = false ;
    }
    else{
        igb.do_rectify = true ;
    }
    if(igb.do_rectify)
    {
        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
        fsSettings["LEFT.K"] >> K_l;
        fsSettings["RIGHT.K"] >> K_r;

        fsSettings["LEFT.P"] >> P_l;
        fsSettings["RIGHT.P"] >> P_r;

        fsSettings["LEFT.R"] >> R_l;
        fsSettings["RIGHT.R"] >> R_r;

        fsSettings["LEFT.D"] >> D_l;
        fsSettings["RIGHT.D"] >> D_r;

        int rows_l = fsSettings["LEFT.height"];
        int cols_l = fsSettings["LEFT.width"];
        int rows_r = fsSettings["RIGHT.height"];
        int cols_r = fsSettings["RIGHT.width"];

        if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
                rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0)
        {
            cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
            return -1;
        }

        cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,igb.M1l,igb.M2l);
        cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,igb.M1r,igb.M2r);
    }

    int flipStereo = fsSettings["flipStereo"];

    //open the bag
    string bagPath = fsSettings["bagPath"] ;
    rosbag::Bag bag(bagPath, rosbag::bagmode::Read);

    double startTime = fsSettings["startTime"] ;

    fsSettings.release();

    std::string imu_topic("/imu0");
    rosbag::View view_imu(
                bag,
                rosbag::TopicQuery(imu_topic));

    std::string camera_topic ;
    if ( flipStereo > 0 ){
        camera_topic = "/cam"+std::to_string(1)+"/image_raw" ;
    }
    else {
        camera_topic = "/cam"+std::to_string(0)+"/image_raw" ;
    }
    rosbag::View view_image0(
                bag,
                rosbag::TopicQuery(camera_topic));

    if ( flipStereo > 0 ){
        camera_topic = "/cam"+std::to_string(0)+"/image_raw" ;
    }
    else {
        camera_topic = "/cam"+std::to_string(1)+"/image_raw" ;
    }
    rosbag::View view_image1(
                bag,
                rosbag::TopicQuery(camera_topic));

    rosbag::View::iterator view_cam_iter0 = view_image0.begin() ;
    rosbag::View::iterator view_cam_iter1 = view_image1.begin() ;

//    double* mp[1000] ;
//    for( int i = 0 ; i < 1000; i++ ){
//        mp[i] = NULL ;
//    }
//    unsigned int pID = 0 ;
//    Eigen::Vector3d Pos ;
//    Pos.setZero();
//    while ( ros::ok() )
//    {
//        if ( mp[0] != NULL )
//        {
//            for( int i = 0 ; i < 1000; i++ ){
//                delete[] mp[i] ;
//                mp[i] = NULL ;
//            }
//        }
//        else {
//            for( int i = 0 ; i < 1000; i++ ){
//                mp[i] = new double[80000];
//                for( int j = 0 ; j < 80000; j++ ){
//                    mp[i][j] = j ;
//                }
//            }
//        }
//        usleep(1000*1000);
//    }
//    return 0;

    cv::Mat imLeft, imRight, depth;
    bool init = false ;
    LT_SLAM::MYTIMESTAMP timeStamp ;
    while( ros::ok() )
    {
        if ( view_cam_iter0 == view_image0.end() ){
            break ;
        }
        if ( view_cam_iter1 == view_image1.end() ){
            break ;
        }
        sensor_msgs::ImageConstPtr msg0 = view_cam_iter0
                ->instantiate<sensor_msgs::Image>();

        sensor_msgs::ImageConstPtr msg1 = view_cam_iter1
                ->instantiate<sensor_msgs::Image>();

        double t0 = msg0->header.stamp.toSec() ;
        double t1 = msg1->header.stamp.toSec() ;

        if ( init == false )
        {
            if ( t0 > t1 ){
                startTime += t0 ;
            }
            else {
                startTime += t1 ;
            }
            init = true ;
        }

        if ( t0 < startTime ){
            view_cam_iter0++ ;
            continue ;
        }
        if ( t1 < startTime ){
            view_cam_iter1++ ;
            continue ;
        }

        timeStamp.sec = msg0->header.stamp.sec;
        timeStamp.nsec = msg0->header.stamp.nsec;


        TicToc tc ;

        int width = msg0->width ;
        int height = msg0->height ;
        cv::Mat left(height, width, CV_8U ) ;
        cv::Mat right(height, width, CV_8U ) ;
        int k = 0 ;
        for( int i = 0 ; i < height ; i++ )
        {
            for ( int j = 0 ; j < width ; j++ )
            {
                left.at<uchar>(i, j) = msg0->data[k] ;
                right.at<uchar>(i, j) = msg1->data[k] ;
                k++ ;
            }
        }

        //        std::cout << msg0->header.stamp << " " << msg1->header.stamp << "\n" ;

//        cv::Mat left = cv_bridge::toCvShare(msg0, std::string("mono8"))->image;
//        cv::Mat right = cv_bridge::toCvShare(msg1, std::string("mono8"))->image;

        //double t = (double)cvGetTickCount();

        if ( igb.do_rectify ){
            cv::remap(left, imLeft, igb.M1l, igb.M2l, cv::INTER_LINEAR);
            cv::remap(right, imRight, igb.M1r, igb.M2r, cv::INTER_LINEAR);
        }
        else {
            imLeft = left.clone();
            imRight = right.clone();
        }
        //printf("rectification time: %f\n", ((double)cvGetTickCount() - t) / (cvGetTickFrequency() * 1000));


        igb.mpSLAM->TrackStereo(imLeft, imRight, depth, timeStamp );

        view_cam_iter0++ ;
        view_cam_iter1++ ;

//        while ( igb.mpSLAM->mpLocalMesher->onView == true ){
//            usleep(1000);
//        }
        double spend_t = tc.toc();
        if ( spend_t < 40 ){
            usleep((40-spend_t)*1000);
        }
    }

    std::getchar();
    // Stop all threads
    SLAM.Shutdown();

    return 0;
}
