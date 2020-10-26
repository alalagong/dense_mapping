#ifndef LOCALMESHING_H
#define LOCALMESHING_H

#include "util.h"
#include "KeyFrame.h"
#include "System.h"
#include "open_chisel/ProjectionIntegrator.h"
#include "open_chisel/Chisel.h"
#include "open_chisel/weighting/ConstantWeighter.h"
#include "open_chisel/truncation/QuadraticTruncator.h"
#include "open_chisel/truncation/InverseTruncator.h"

namespace LT_SLAM
{

typedef float DepthData;
typedef uint8_t ColorData;

class KeyFrame;
class System;

struct meshingFrame
{
    KeyFrame* pKF;
    std::vector<KeyFrame*> neighboringKFs ;
};

class LocalMeshing
{
public:
EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    LocalMeshing(System *sys);
    ~LocalMeshing();
    System* pSystem;
    bool mbFinished;

    cv::Mat curDepth ;
    cv::Mat curImg ;

    std::mutex mutexLoopImg ;
    bool loopImgFlag = false ;
    cv::Mat loopImg;

    float curImgTime ;

    tf::TransformBroadcaster pubTf_;
    ros::Publisher pubMesh_, pubPose_, pubObometry_, pubFreeSpace_ ;
    ros::Publisher pub_point_cloud2, pub_disp_img, pub_color_img ;
    ros::Publisher pub_color_img_info, pub_disp_img_info ;
    ros::Publisher meshPublisher, chunkBoxPublisher ;

    // Main function
    void Run();
    void Update();
    void ProjectNeighboringKFsNew();
    void extractMeshes(KeyFrame* mpCurrentKF);
    void FillMarkerTopicWithMeshesLocal();
    void setViewingInfo();
    std::list<meshingFrame> mlpKeyFrameQueue;
    std::mutex mMutexKeyFrameQueue;
    meshingFrame curFrame;

    visualization_msgs::Marker marker;
    vector<geometry_msgs::Point> localChunkBoxes;

    bool onView = false ;

    chisel::ChiselPtr chiselMap;
    chisel::ProjectionIntegrator projectionIntegrator;

    int numChannels;
    bool initColorImage ;
    std::shared_ptr<chisel::ColorImage<ColorData> > ColorImage;
    std::shared_ptr<chisel::DepthImage<DepthData> > DepthImage;
    chisel::PinholeCamera cameraModel, colorModel, depthModel;
    chisel::Intrinsics intrinsics;

    int SADWindowSize;
    SGM2 sgm2 ;
};

} //namespace LT_SLAM

#endif // LOCALMESHING_H
