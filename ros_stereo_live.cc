#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <memory>
#include <functional>
#include<list>
#include<vector>
#include<ros/ros.h>
#include<ros/subscriber.h>
#include<ros/publisher.h>
#include<cv_bridge/cv_bridge.h>
#include"boost/thread.hpp"
#include<opencv2/core/core.hpp>
#include<rosbag/bag.h>
#include<rosbag/chunked_file.h>
#include<rosbag/view.h>
#include<rosbag/query.h>
#include"src/System.h"
#include <stdio.h>
#include "visensor_node/visensor_imu.h"

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
std::list<visensor_node::visensor_imu> imuBuf ;
std::mutex image0_queue_mtx;
std::mutex image1_queue_mtx;
std::mutex imu_queue_mtx ;
LT_SLAM::System* pSystem ;

ros::Subscriber sub_image[2];
ros::Subscriber sub_imu;

void convertMsgToMatMono(const sensor_msgs::ImageConstPtr& msg, cv::Mat& img)
{
    int width = msg->width ;
    int height = msg->height ;
    img = cv::Mat(height, width, CV_8U);
    int k = 0 ;
    for( int i = 0 ; i < height ; i++ )
    {
        for ( int j = 0 ; j < width ; j++ )
        {
            img.at<uchar>(i, j) = msg->data[k] ;
            k++ ;
        }
    }
}

void image0CallBack(const sensor_msgs::ImageConstPtr& msg)
{
    ros::Time tImage = msg->header.stamp;
    cv::Mat   image ;
    convertMsgToMatMono(msg, image);

    image0_queue_mtx.lock();
    image0Buf.push_back(ImageMeasurement(tImage, image));
    image0_queue_mtx.unlock();
}

void image1CallBack(const sensor_msgs::ImageConstPtr& msg)
{
    ros::Time tImage = msg->header.stamp;
    cv::Mat   image ;
    convertMsgToMatMono(msg, image);

    image1_queue_mtx.lock();
    image1Buf.push_back(ImageMeasurement(tImage, image));
    image1_queue_mtx.unlock();
}

void imuCallBack(const visensor_node::visensor_imu& imu_msg )
{
    imu_queue_mtx.lock();
    imuBuf.push_back( imu_msg );
    imu_queue_mtx.unlock();
}

void Loop()
{
    unsigned int image0BufSize ;
    unsigned int image1BufSize ;
    unsigned int imuBufSize ;
    std::list<ImageMeasurement>::iterator iter0 ;
    std::list<ImageMeasurement>::iterator iter1 ;
    std::list<visensor_node::visensor_imu>::reverse_iterator reverse_iterImu ;
    std::list<visensor_node::visensor_imu>::iterator currentIMU_iter;
    ros::Time tImage ;
    cv::Mat   image0 ;
    cv::Mat   image1 ;
    ros::Rate r(1000.0);
    cv::Mat imLeft ;
    cv::Mat imRight ;
    cv::Mat depth;
    LT_SLAM::MYTIMESTAMP timeStamp ;

    while ( ros::ok() )
    {
        image0_queue_mtx.lock();
        image1_queue_mtx.lock();
        imu_queue_mtx.lock();
        image0BufSize = image0Buf.size();
        image1BufSize = image1Buf.size();
        imuBufSize = imuBuf.size();
        //printf("%d %d %d\n",image0BufSize, image1BufSize, imuBufSize ) ;
        if ( image0BufSize == 0 || image1BufSize == 0 || imuBufSize == 0 ){
            image0_queue_mtx.unlock();
            image1_queue_mtx.unlock();
            imu_queue_mtx.unlock();
            r.sleep() ;
            continue ;
        }
        iter0 = image0Buf.begin();
        iter1 = image1Buf.begin();
        while ( iter1 != image1Buf.end() && iter0->t > iter1->t ){
            iter1 =  image1Buf.erase( iter1 ) ;
        }
        while ( iter0 != image0Buf.end() && iter0->t < iter1->t ){
            iter0 =  image0Buf.erase( iter0 ) ;
        }
        if ( iter1 == image1Buf.end() || iter0 == image0Buf.end() ){
            image0_queue_mtx.unlock();
            image1_queue_mtx.unlock();
            imu_queue_mtx.unlock();
            r.sleep() ;
            continue ;
        }
        tImage = iter0->t;
        reverse_iterImu = imuBuf.rbegin() ;
        if ( reverse_iterImu->header.stamp < tImage ){
            image0_queue_mtx.unlock();
            image1_queue_mtx.unlock();
            imu_queue_mtx.unlock();
            r.sleep() ;
            continue ;
        }
        imu_queue_mtx.unlock();

        image0 = iter0->image.clone();
        image1 = iter1->image.clone();
        iter1 =  image1Buf.erase( iter1 ) ;
        iter0 =  image0Buf.erase( iter0 ) ;
        image0_queue_mtx.unlock();
        image1_queue_mtx.unlock();

        imu_queue_mtx.lock();
        currentIMU_iter = imuBuf.begin() ;
//        Quaternionf q, dq ;
//        q.setIdentity() ;
        while ( currentIMU_iter->header.stamp < tImage )
        {
//            float pre_t = currentIMU_iter->header.stamp.toSec();
            currentIMU_iter = imuBuf.erase(currentIMU_iter);
//            float next_t = currentIMU_iter->header.stamp.toSec();
//            float dt = next_t - pre_t ;

//            //prediction for dense tracking
//            dq.x() = currentIMU_iter->angular_velocity.x*dt*0.5 ;
//            dq.y() = currentIMU_iter->angular_velocity.y*dt*0.5 ;
//            dq.z() = currentIMU_iter->angular_velocity.z*dt*0.5 ;
//            dq.w() =  sqrt( 1 - SQ(dq.x()) * SQ(dq.y()) * SQ(dq.z()) ) ;
//            q = (q * dq).normalized();
        }
        imu_queue_mtx.unlock();

        timeStamp.sec = tImage.sec ;
        timeStamp.nsec = tImage.nsec ;

        cv::remap(image0, imLeft,  igb.M1l, igb.M2l, cv::INTER_LINEAR);
        cv::remap(image1, imRight, igb.M1r, igb.M2r, cv::INTER_LINEAR);

        //printf("%d %d %d %d\n", image0.rows, image0.cols, imLeft.rows, imLeft.cols );
        igb.mpSLAM->TrackStereo(imLeft, imRight, depth, timeStamp );
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lt_slam");
    ros::start();
    ros::NodeHandle nh("~") ;
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Warn);

    string packagePath = ros::package::getPath("lt_slam");
    string dictPath = packagePath + "//Vocabulary//ORBvoc.bin" ;
    string configPath = packagePath + "/config/myVI.yaml";

    // Create SLAM system. It initializes all system threads and gets ready to process frames.

    LT_SLAM::System SLAM(dictPath, configPath, LT_SLAM::STEREO);
    igb.mpSLAM = &SLAM ;

    // Load settings related to stereo calibration
    cv::FileStorage fsSettings(configPath, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        cerr << "ERROR: Wrong path to settings" << endl;
        return -1;
    }

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

    sub_image[0] = nh.subscribe("/cam1/image_raw", 100, &image0CallBack );
    sub_image[1] = nh.subscribe("/cam0/image_raw", 100, &image1CallBack );
    sub_imu = nh.subscribe("/cust_imu0", 1000, &imuCallBack ) ;

    boost::thread ptrProcessImageThread = boost::thread(&Loop);
    ros::spin();

    std::getchar();
    // Stop all threads

    ptrProcessImageThread.join();
    SLAM.Shutdown();

    return 0;
}
