#ifndef MAP_H
#define MAP_H

#include "util.h"
#include "System.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include "KeyFrameDatabase.h"

namespace LT_SLAM
{

class MapPoint;
class KeyFrame;
class System;
class KeyFrameDatabase;

class Map
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    //typedef std::unordered_map<KeyFrame*, int> KFMAPTYPE ;
    typedef std::map<int, KeyFrame*> KFMAPTYPE;
    typedef std::map<int, KeyFrame*>::iterator KFMAPTYPE_ITER;


    Map(System* pSys);
    ~Map();

    MapPoint* createMapPoint(unsigned int pID,
                             const Eigen::Vector3d Pos,
                             bool init);
    void eraseMapPoint(MapPoint *mp);
    bool mergeMapPoint(unsigned int fromID, unsigned int toID) ;
    void updateMapPoint(MapPoint *mp) ;
    void updateDescriptor(MapPoint *mp) ;
    MapPoint* getMapPoint(unsigned int pID) ;

    void addObservation(MapPoint *mp, KeyFrame* pKF, unsigned int index, unsigned int flag) ;
    bool eraseObservation(MapPoint* mp, KeyFrame* pKF );

    KeyFrame* createKeyFrame(const Eigen::Vector3d &t, const Eigen::Matrix3d &R );
    void EraseKeyFrame(KeyFrame* pKF, bool eraseObservationFlag);
    void AddKeyFrame(KeyFrame* pKF);

    void clear();

    System* pSystem;

    std::unordered_map<unsigned int, MapPoint*> mspMapPoints ;
    std::unordered_map<unsigned int, unsigned int> mergeMap ;

    std::mutex mMutexKeyFramePose ;
    KFMAPTYPE mspKeyFrames ;
    //std::list<KeyFrame*> keyFrameBuffer;
    int numOfKeyFrames;
    //std::set<KeyFrame*> mspKeyFrames;

//    std::mutex mMutexMapUpdate;
//    std::mutex mMutexMap;

    // KeyFrame database for place recognition (relocalization and loop detection).
    // std::mutex mMutexKeyFrameDatabase ;
    KeyFrameDatabase* mpKeyFrameDatabase;

    int numOfEraseKF ;
    int numOfEraseMP ;
};

} //namespace LT_SLAM

#endif // MAP_H
