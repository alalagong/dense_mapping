#ifndef VIEWER_H
#define VIEWER_H

#include "System.h"

namespace LT_SLAM
{

class System;

class Viewer
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Viewer(System* pSystem);
    ~Viewer();
    System* pSystem;

    // Main thread function. Draw points, keyframes, the current camera pose and the last processed
    // frame. Drawing is refreshed according to the camera fps. We use Pangolin.
    void test();
    void Run();
    void Run_Multi();
    void Run_Multi_Move();

    void RequestFinish();

    void RequestStop();

    bool isFinished();

    bool isStopped();

    void Release();

    bool Stop();

    bool CheckFinish();
    void SetFinish();
    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;

    bool mbStopped;
    bool mbStopRequested;
    std::mutex mMutexStop;
};

}

#endif // VIEWER_H


