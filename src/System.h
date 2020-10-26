#ifndef SYSTEM_H
#define SYSTEM_H

#include "util.h"
#include "Settings.h"
#include "Tracking.h"
#include "LocalMapping.h"
#include "Viewer.h"
#include "KeyFrameDatabase.h"
#include "Map.h"
#include "LocalMeshing.h"
#include "MapDrawer.h"

namespace LT_SLAM
{

class Viewer;
class MapDrawer;

class Map;

class Tracking;
class LocalMapping;
class LocalMeshing;

class System
{
public:
    // Initialize the SLAM system. It launches the Local Mapping, Loop Closing and Viewer threads.
    System(const string &strVocFile, const string &strSettingsFile, const eSensor mSensor);

    // Proccess the given stereo frame. Images must be synchronized and rectified.
    // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
    // Returns the camera pose (empty if tracking fails).
    void TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, cv::Mat& depth, const MYTIMESTAMP &timestamp);

    // Reset the system (clear map)
    void Reset();

    // All threads will be requested to finish.
    // It waits until all threads have finished.
    // This function must be called before saving the trajectory.
    void Shutdown();
    void SaveDataKITTI(string kitti_name);
    void SaveData(string fileName);

    MapDrawer* mpMapDrawer ;

    // Map structure that stores the pointers to all KeyFrames and MapPoints.
    Map* mpMap;

    // Tracker. It receives a frame and computes the associated camera pose.
    // It also decides when to insert a new keyframe, create some new MapPoints and
    // performs relocalization if tracking fails.
    Tracking* mpTracker;

    // Local Mapper. It manages the local map and performs local bundle adjustment.
    LocalMapping* mpLocalMapper;
    std::thread mptLocalMapping;
    std::thread mptGlobalOpt;

//    //mpLocalMesher. It generates local dense mesh map
    LocalMeshing* mpLocalMesher;
    std::thread mptLocalMeshing;

    // The viewer draws the map and the current camera pose. It uses Pangolin.
    Viewer* mpViewer;
    std::thread mptViewer;

    // Reset flag
    std::mutex mMutexReset;
    bool mbReset;
};

}// namespace LT_SLAM

#endif // SYSTEM_H
