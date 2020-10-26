#ifndef MAPPOINT_H
#define MAPPOINT_H

#include"util.h"
#include"Settings.h"
#include"KeyFrame.h"

namespace LT_SLAM
{

class KeyFrame;

class MapPoint
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    MapPoint(unsigned int pID, Eigen::Vector3d Pos, bool flag);
    MapPoint(MapPoint* p);
    ~MapPoint();
    static bool triangulate(const vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d> > &R_b_2_w,
                            const vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > &T_b_2_w,
                            const vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > &observation,
                            const vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > &shift,
                            Eigen::Vector3d& result_p );
    Eigen::Vector3d pos ;//in world coordinate
    Eigen::Vector3d mNormalVector;
    unsigned int mnId;
    bool init ;
    short numOfMonoObservations;
    short numOfStereoObservations;

    // Keyframes observing the point and associated index in keyframe
    std::map<KeyFrame*, unsigned int> mObservations;

    // Best descriptor to fast matching
    cv::Mat mDescriptor;
};

} //namespace LT_SLAM

#endif // MAPPOINT_H
