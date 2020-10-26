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
#include "src/ramRead.h"

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
    ros::start();
    ros::NodeHandle nh("~") ;
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Warn);

#ifdef EIGEN_DONT_PARALLELIZE
    ROS_INFO("EIGEN_DONT_PARALLELIZE");
#endif

    struct sysinfo memInfo;
    sysinfo (&memInfo);

    string packagePath = ros::package::getPath("lt_slam");
    string dictPath = packagePath + "/Vocabulary/ORBvoc.bin" ;
    string configPath = packagePath + "/config/stereo_NewCollege.yaml";
    // Load settings related to stereo calibration
    cv::FileStorage fsSettings(configPath, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        cerr << "ERROR: Wrong path to settings" << endl;
        return -1;
    }
    string listPath = fsSettings["listPath"] ;

    double TrackerTotalTime = fsSettings["TrackerTotalTime"] ;

    FILE * file;
    file = std::fopen( listPath.c_str() , "r");
    char filePath[1024] ;
    char fileName[1024] ;
    double t0 = 0 ;
    double pre_t, spend_t ;
    int k = 0 ;
    int cnt = 0 ;
    vector<string> fileListName(60000) ;
    while ( fscanf(file, "%s", fileName) != EOF && ros::ok() )
    {
        //printf("k=%d\n", k ) ;
        if ( fileName[31] != 'l' ){
            continue ;
        }
        cnt++ ;
        if ( cnt < 760 ){
//            sprintf(filePath, "/media/ygling2008/82ECA253ECA24171/newCollege/%s", fileName ) ;
//            cv::Mat imLeft = cv::imread(filePath, CV_LOAD_IMAGE_GRAYSCALE );

//            cv::imshow("pre", imLeft ) ;
//            cv::waitKey(10) ;
//            printf("k = %d ", cnt ) ;

            continue ;
        }
        string str(fileName);

        fileListName[k] = str;
//        if ( k < 100  ){
//            cout << fileName << "\n" ;
//            cout << fileListName[k-1] << "\n" ;
//        }
        k++ ;
    }


    // Create SLAM system. It initializes all system threads and gets ready to process frames.
        ROS_WARN("reach-a");
    LT_SLAM::System SLAM(dictPath,configPath, LT_SLAM::STEREO);
    igb.mpSLAM = &SLAM ;


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


    fsSettings.release();
    cv::Mat curImg(185, 613, CV_8U ) ;
    curImg.setTo(0) ;

//    cv::imshow("Current depth", curImg) ;
//    cv::moveWindow("Current depth", 0, 700) ;
//    cv::imshow("Current Keyframe", curImg) ;
//    cv::moveWindow("Current Keyframe", 700, 700) ;

    //cv::imshow("Current Keyframe", curImg) ;
    //cv::moveWindow("Current Keyframe", 0, 700) ;
    //cv::waitKey(0) ;
    //usleep(6000000) ;

    ROS_WARN("reach");

    int imageTotalNum = k;
    ProcessList pl ;
    LinuxProcessList_scanMemoryInfo(&pl) ;
    long long startRam = pl.usedMem;
    //imageTotalNum = fileListName.size()-1;
    cv::Mat imLeft;
    cv::Mat imRight;
    cv::Mat depth_lidar, disparity_lidar ;
    LT_SLAM::MYTIMESTAMP timeStamp ;
    for( int i = 0 ; i < imageTotalNum && ros::ok(); i += 2 )
    {
        if ( i > 35320 && i < 35350 ){
            continue ;
        }

        if ( LT_SLAM::pSetting->loopStop && igb.mpSLAM->mpLocalMapper->onGlobalOptimization )
        {
            for( ; ros::ok(); )
            {
                usleep(10000);
                if ( igb.mpSLAM->mpLocalMapper->onGlobalOptimization == false
                     && igb.mpSLAM->mpLocalMapper->doneGlobalOptimization == false ){
                    break ;
                }
            }
        }

        pre_t = (double)cvGetTickCount() ;
        sprintf(filePath, "/media/rolandling/DATA/newCollege/rectified_%s", fileListName[i].c_str() ) ;
        imLeft = cv::imread(filePath, CV_LOAD_IMAGE_GRAYSCALE );

        std::strcpy(&filePath[std::strlen(filePath)-8], "right.pnm" ) ;
        imRight = cv::imread(filePath, CV_LOAD_IMAGE_GRAYSCALE );

        //sprintf(filePath, "%06d.bin", i );

        //readLidarData(lidarPath+filePath, configPath, depth_lidar, disparity_lidar ) ;

        //printf("%d %d\n", depth_lidar.rows, depth_lidar.cols ) ;

        timeStamp.sec = i ;
        timeStamp.nsec = 0 ;

        t0 = i ;
        igb.mpSLAM->TrackStereo(imLeft, imRight, depth_lidar, timeStamp );
        spend_t = ( (double)cvGetTickCount()- pre_t) / (cvGetTickFrequency() * 1000) ;

//        while ( igb.mpSLAM->mpLocalMesher->onView == true && ros::ok()){
//            usleep(1000);
//        }

        char key = cv::waitKey(1);
        if ( key == 's' ){
            for( ; ros::ok(); )
            {
                usleep(1000);
                key = cv::waitKey(1);
                if ( key == 's' ){
                    break ;
                }
            }
        }

        if ( (i%500) == 0 ){
            //sysinfo (&memInfo);
            //long long physMemUsed = (memInfo.totalram - memInfo.freeram) - startRam;
            LinuxProcessList_scanMemoryInfo(&pl) ;
            //cout << pl.usedMem << " " << pl.buffersMem << " " << pl.cachedMem << " " ;
            printf("Used: %llu ", (pl.usedMem-startRam)/1024 ) ;

            printf("MP Num:%u KF Num:%u ",
                   igb.mpSLAM->mpMap->mspMapPoints.size(),
                   igb.mpSLAM->mpMap->mspKeyFrames.size());
            printf("i = %d, spend_t = %lf\n", i, spend_t ) ;
        }
//        while ( igb.mpSLAM->mpLocalMesher->mlpKeyFrameQueue.size() > 1 ){
//            usleep(1000);
//        }
//        while ( igb.mpSLAM->mpLoopCloser->mbRunningGBA ){
//            usleep(1000);
//        }
        //usleep(100000) ;

        if ( spend_t < TrackerTotalTime ){
            usleep((TrackerTotalTime-spend_t)*1000);
        }
    }

    SLAM.SaveData("/home/rolandling");

    std::getchar();
    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    //SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}
