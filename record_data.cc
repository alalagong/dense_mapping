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
#include"boost/thread.hpp"
#include<opencv2/core/core.hpp>
#include<rosbag/bag.h>
#include<rosbag/chunked_file.h>
#include<rosbag/view.h>
#include<rosbag/query.h>
#include"src/System.h"
#include <stdio.h>
#include <X11/Xlib.h>
#include "visensor_node/visensor_imu.h"

#define BACKWARD_HAS_DW 1
#include <backward.hpp>
namespace backward
{
backward::SignalHandling sh;
} // namespace backward

std::list<visensor_node::visensor_imu> imuBuf ;

ros::Subscriber sub_image[2];
ros::Subscriber sub_imu;

char dirPath[256] = "//home//ygling2008//save//" ;

vector<int> compression_params;


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

//    double pre_t = (double)cvGetTickCount() ;

    char fileName[256] ;
    sprintf(fileName, "%s%u_%u_l.png", dirPath, tImage.sec, tImage.nsec ) ;
    cv::imwrite(fileName, image) ;

//    double spend_t = ( (double)cvGetTickCount()- pre_t) / (cvGetTickFrequency() * 1000) ;
//    printf("spend_t = %lf\n", spend_t ) ;
}

void image1CallBack(const sensor_msgs::ImageConstPtr& msg)
{
    ros::Time tImage = msg->header.stamp;
    cv::Mat   image ;
    convertMsgToMatMono(msg, image);

    char fileName[256] ;
    sprintf(fileName, "%s%u_%u_r.png", dirPath, tImage.sec, tImage.nsec ) ;
    cv::imwrite(fileName, image) ;
}

void imuCallBack(const visensor_node::visensor_imu& imu_msg )
{
    //imuBuf.push_back( imu_msg );


    char fileName[256] = "//home//ygling2008//imu.txt";

    FILE * pFile;
    pFile = fopen(fileName,"a");
    fprintf (pFile, "%u_%u %lf %lf %lf %lf %lf %lf\n",
             imu_msg.header.stamp.sec,
             imu_msg.header.stamp.nsec,
             imu_msg.linear_acceleration.x,
             imu_msg.linear_acceleration.y,
             imu_msg.linear_acceleration.z,
             imu_msg.angular_velocity.x,
             imu_msg.angular_velocity.y,
             imu_msg.angular_velocity.z);
    fclose (pFile);
}

int main(int argc, char **argv)
{
    XInitThreads();

    ros::init(argc, argv, "lt_slam");
    ros::start();
    ros::NodeHandle nh("~") ;
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Warn);

    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(3);

    system("rm //home//ygling2008//save//*") ;
    system("rm //home//ygling2008//imu.txt") ;

    sub_image[0] = nh.subscribe("/cam1/image_raw", 100, &image0CallBack );
    sub_image[1] = nh.subscribe("/cam0/image_raw", 100, &image1CallBack );
    sub_imu = nh.subscribe("/cust_imu0", 1000, &imuCallBack ) ;

    ros::spin();


//    char fileName[256] = "//home//ygling2008//imu.txt";

//    FILE * pFile;
//    pFile = fopen(fileName,"a");
//    std::list<visensor_node::visensor_imu>::iterator iter ;
//    for (iter = imuBuf.begin() ; iter != imuBuf.end() ; iter++)
//    {
//        fprintf (pFile, "%u_%u %lf %lf %lf %lf %lf %lf\n",
//                 iter->header.stamp.sec,
//                 iter->header.stamp.nsec,
//                 iter->linear_acceleration.x,
//                 iter->linear_acceleration.y,
//                 iter->linear_acceleration.z,
//                 iter->angular_velocity.x,
//                 iter->angular_velocity.y,
//                 iter->angular_velocity.z);
//    }
//    fclose (pFile);


    return 0;
}
