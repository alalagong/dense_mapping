#ifndef MAPDRAWER_H
#define MAPDRAWER_H

#include "util.h"
#include "System.h"
#include "Settings.h"

namespace LT_SLAM
{

class System ;

class MapDrawer
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    MapDrawer(System* sys);
    ~MapDrawer();
    System* pSystem ;
    std::mutex mMutexCamera;

    std::mutex mMutexLocalMap ;
    std::vector<cv::Point3f> localMapPoints;
    std::vector<cv::Point3f> allMapPoints;

    std::mutex mMutexPose ;
    bool initPose ;
    Eigen::Matrix3d cur_R ;
    Eigen::Vector3d cur_t ;
    std::set<unsigned int> neigborhoodKF_ids ;
    std::vector<unsigned int> NonneigborhoodKF_ids ;
    vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> neigborhoodKF_T ;
    vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> NonneigborhoodKF_T ;

    std::mutex mMutexLocalDenseMap;
    visualization_msgs::Marker marker;
    vector<geometry_msgs::Point> localChunkBoxes;

    void DrawReferencePoints();
    void DrawChunkBoxes(float chunkResolution, float currentHeight, bool flag, double threshold);
    void DrawChunkBoxesDist(float chunkResolution, Eigen::Vector3f t, double threshold);
    void DrawKeyFramesLocal(float scale);
    void DrawMeshesLocal();
    void DrawMapPoints();
    void DrawCube( float x, float y, float z, float cubicBound);
    void DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph, float scale);
    void DrawCurrentCamera(pangolin::OpenGlMatrix &Twc, float scale);
    void SetCurrentCameraPose(const cv::Mat &Tcw);
    void SetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M, const cv::Mat& mCameraPose );
    void SetReferenceKeyFrame(KeyFrame *pKF);
    void GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M);

    void GetPose(Eigen::Matrix3f& R, Eigen::Vector3f& t);
};

} //namespace LT_SLAM

#endif // MAPDRAWER_H
