#include<chrono>
#include<ros/ros.h>
#include<rosbag/bag.h>
#include<rosbag/chunked_file.h>
#include<rosbag/view.h>
#include<rosbag/query.h>
#include"src/System.h"
#include <stdio.h>
#include <boost/filesystem.hpp>
#include "sys/types.h"
#include "sys/sysinfo.h"

using namespace std;

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

    if ( argc > 1 ){
        //       ROS_WARN("%s", argv[0] ) ;
        //       ROS_WARN("%s", argv[1] ) ;
    }

    //ros::start();
    ros::NodeHandle nh("~") ;
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Warn);

#ifdef EIGEN_DONT_PARALLELIZE
    ROS_INFO("EIGEN_DONT_PARALLELIZE");
#endif


    //    printf("Size of MapPoint = %d\n", sizeof(ORB_SLAM2::MapPoint) ) ;
    //    printf("Size of KeyFrame = %d\n", sizeof(ORB_SLAM2::KeyFrame) ) ;
    //    return 0 ;


    struct sysinfo memInfo;
    sysinfo (&memInfo);

    string packagePath = ros::package::getPath("lt_slam");

    string dictPath = packagePath + "//Vocabulary//ORBvoc.bin" ;
    string configPath;
    if ( argc < 2 ){
        configPath = packagePath + "/config/KITTI05.yaml";
    }
    else {
        configPath = packagePath + "/config/KITTI" + argv[1] + ".yaml";
    }
    //system("rm /home/ygling2008/graph_data/*");

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

    //open the bag
    string bagPath = fsSettings["bagPath"] ;
    string lidarPath = fsSettings["lidarPath"];
    int imageTotalNum = fsSettings["imageTotalNum"] ;
    double TrackerTotalTime = fsSettings["TrackerTotalTime"] ;



    fsSettings.release();
    char filePath[1024] ;
    double t0 = 0 ;
    double pre_t, spend_t ;
    cv::Mat curImg(185, 613, CV_8U ) ;
    curImg.setTo(0) ;

    //    cv::imshow("Current depth", curImg) ;
    //    cv::moveWindow("Current depth", 0, 700) ;
    //    cv::imshow("Current Keyframe", curImg) ;
    //    cv::moveWindow("Current Keyframe", 700, 700) ;

    cv::imshow("Current Keyframe", curImg ) ;
    cv::imshow("Loop-Img", curImg ) ;
    cv::waitKey(0) ;

    //cv::imshow("Current Keyframe", curImg) ;
    //cv::moveWindow("Current Keyframe", 0, 700) ;
    //cv::waitKey(0) ;
    //usleep(6000000) ;
    LT_SLAM::MYTIMESTAMP timeStamp ;

    //LT_SLAM::debug_f.open("/home/ygling2008/time.txt");
    for( int i=0 ; i <= imageTotalNum && ros::ok(); i++ )
    {
        pre_t = (double)cvGetTickCount() ;

        sprintf(filePath, "%s//image_0//%06d.png",  bagPath.c_str(), i ) ;
        cv::Mat imLeft = cv::imread(filePath, CV_LOAD_IMAGE_GRAYSCALE );
        sprintf(filePath, "%s//image_1//%06d.png",  bagPath.c_str(), i ) ;
        cv::Mat imRight = cv::imread(filePath, CV_LOAD_IMAGE_GRAYSCALE );

        sprintf(filePath, "%06d.bin", i );
        cv::Mat depth_lidar, disparity_lidar ;
        //readLidarData(lidarPath+filePath, configPath, depth_lidar, disparity_lidar ) ;

        //printf("%d %d\n", depth_lidar.rows, depth_lidar.cols ) ;

        timeStamp.sec = i ;
        timeStamp.nsec = 0 ;
        igb.mpSLAM->TrackStereo(imLeft, imRight, depth_lidar, timeStamp );
        spend_t = ( (double)cvGetTickCount()- pre_t) / (cvGetTickFrequency() * 1000) ;

        //        while ( igb.mpSLAM->mpLocalMesher->onView == true){
        //            usleep(1000);
        //        }

        char key = cv::waitKey(1);
        if ( key == 's' ){
            for( ; ; )
            {
                usleep(1000);
                key = cv::waitKey(1);
                if ( key == 's' ){
                    break ;
                }
            }
        }

        //        if ( (i%5) == 0 ){
        //            sysinfo (&memInfo);
        //            long long physMemUsed = memInfo.totalram - memInfo.freeram;
        //            ROS_WARN("i = %d Memory Used: %lld", i, physMemUsed ) ;
        ////            printf("MapPoint Total Num: %d NextId: %d\n", igb.mpSLAM->mpMap->mspMapPoints.size(), ORB_SLAM2::MapPoint::nNextId ) ;
        ////            printf("KeyFrame Num: %d\n", LT_SLAM::KeyFrame::nNextId ) ;
        //        }
        //        while ( igb.mpSLAM->mpLocalMesher->mlpKeyFrameQueue.size() > 1 ){
        //            usleep(1000);
        //        }
        //        while ( igb.mpSLAM->mpLoopCloser->mbRunningGBA ){
        //            usleep(1000);
        //        }
        //usleep(100000) ;
        //printf("spend_t = %lf TrackerTotalTime=%lf\n", spend_t, TrackerTotalTime ) ;

        if ( spend_t < TrackerTotalTime ){
            usleep((TrackerTotalTime-spend_t)*1000);
        }
    }
    LT_SLAM::debug_f.close();

//    string save_name = "KITTI" ;
//    if ( argc < 2 ){
//        save_name = save_name + "05" ;
//    }
//    else{
//        save_name = save_name + argv[1] ;
//    }
//  SLAM.SaveDataKITTI(save_name);

    // std::getchar();
    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    //SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    puts("ends") ;


    return 0;
}
