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

    string seqName = "20170323" ;
    string dataPath = "/media/nova/DATA/" ;
    string configPath = dataPath + seqName + "/myVI.yaml";
    string listPath = dataPath + seqName + "/stereo_list.txt";
    string imuPath = dataPath + seqName + "/imu.txt" ;
    string bagPath = dataPath + seqName + "/stereo/" ;

    // Load settings related to stereo calibration
    cv::FileStorage fsSettings(configPath, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        cerr << "ERROR: Wrong path to settings" << endl;
        return -1;
    }
    double TrackerTotalTime = 40 ;

    FILE * file;
    file = std::fopen( listPath.c_str() , "r");
    char filePath[1024] ;
    char fileName[1024] ;
    double t0 = 0 ;
    double pre_t, spend_t ;
    vector<string> fileListName ;
    fileListName.reserve(200000);
    while ( fscanf(file, "%s", fileName) != EOF && ros::ok() )
    {
        string str(fileName);
        if ( str.size() < 26 ){
            str.insert(11, "0");
        }
        fileListName.push_back(str);
    }
    std::fclose(file) ;
    //std::sort(fileListName.begin(), fileListName.end() );

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    LT_SLAM::System SLAM(dictPath, configPath, LT_SLAM::STEREO);
    igb.mpSLAM = &SLAM ;

    igb.do_rectify = true ;
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
    fsSettings.release();

    igb.mpSLAM->mpLocalMapper->imuVector.clear();
    file = std::fopen( imuPath.c_str() , "r");
    LT_SLAM::IMU_DATA tmpIMU ;
    while ( std::fgets(fileName, 256, file) != NULL && ros::ok() )
    {
        //cout << fileName << "\n" ;
        sscanf(fileName, "%u_%u %lf %lf %lf %lf %lf %lf",
               &tmpIMU.t.sec, &tmpIMU.t.nsec,
               &tmpIMU.ax, &tmpIMU.ay, &tmpIMU.az,
               &tmpIMU.wx, &tmpIMU.wy, &tmpIMU.wz ) ;
        igb.mpSLAM->mpLocalMapper->imuVector.push_back(tmpIMU);
        //tmpIMU.print();
    }
    std::fclose(file) ;

    //printf("imuSz = %d\n", igb.mpSLAM->mpLocalMapper->imuVector.size()  ) ;



//    cv::imshow("Current depth", curImg) ;
//    cv::moveWindow("Current depth", 0, 700) ;
//    cv::imshow("Current Keyframe", curImg) ;
//    cv::moveWindow("Current Keyframe", 700, 700) ;

    //cv::imshow("Current Keyframe", curImg) ;
    //cv::moveWindow("Current Keyframe", 0, 700) ;
    //cv::waitKey(0) ;
    //usleep(6000000) ;

    cv::Mat imLeft;
    cv::Mat imRight;
    cv::Mat depth_lidar, disparity_lidar ;
    LT_SLAM::MYTIMESTAMP timeStamp ;
    for( int i = 0, seq_num = fileListName.size() ; i < seq_num && ros::ok(); i += 2 )
    {
//        if ( i < 500 ){
//            continue ;
//        }
//        if ( i > 35320 && i < 35350 ){
//            continue ;
//        }

//        if ( fileListName[i].at(11) == '0' ){
//             fileListName[i].erase(11, 1) ;
//             fileListName[i+1].erase(11, 1) ;
//        }

        sscanf(fileListName[i].c_str(), "%u_%u_l.png", &timeStamp.sec, &timeStamp.nsec ) ;
        //printf("%u.%u\n", timeStamp.sec, timeStamp.nsec ) ;

        pre_t = (double)cvGetTickCount() ;
        sprintf(filePath, "%s%s", bagPath.c_str(), fileListName[i].c_str() ) ;
        imLeft = cv::imread(filePath, CV_LOAD_IMAGE_GRAYSCALE );

        sprintf(filePath, "%s%s", bagPath.c_str(), fileListName[i+1].c_str() ) ;
        imRight = cv::imread(filePath, CV_LOAD_IMAGE_GRAYSCALE );

        //sprintf(filePath, "%06d.bin", i );

        //readLidarData(lidarPath+filePath, configPath, depth_lidar, disparity_lidar ) ;

        //printf("%d %d\n", depth_lidar.rows, depth_lidar.cols ) ;

        cv::remap(imLeft, imLeft,  igb.M1l, igb.M2l, cv::INTER_LINEAR);
        cv::remap(imRight, imRight, igb.M1r, igb.M2r, cv::INTER_LINEAR);

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

        if ( (i%50) == 0 )
        {
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

        if ( spend_t < 20 ){
            usleep((20-spend_t)*1000);
        }
    }

    SLAM.SaveData(dataPath + seqName);

    std::getchar();
    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    //SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}
