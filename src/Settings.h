#pragma once
#include "util.h"

namespace LT_SLAM
{

#define FRAME_GRID_ROWS 48
#define FRAME_GRID_COLS 64
const int WINDOW_SIZE = 16 ;
const int NUM_OF_CAM = 1;
const int NUM_OF_F = 2000 ;
const int MAX_KF = 50000 ;

// Input sensor
enum eSensor{
    MONOCULAR=0,
    STEREO=1,
    RGBD=2
};

enum SIZE_PARAMETERIZATION
{
    SIZE_POSE = 7,
    SIZE_SPEEDBIAS = 9,
    SIZE_FEATURE = 3
};

enum StateOrder
{
    O_P = 0,
    O_R = 3,
    O_V = 6,
    O_BA = 9,
    O_BG = 12
};

enum NoiseOrder
{
    O_AN = 0,
    O_GN = 3,
    O_AW = 6,
    O_GW = 9
};


class Setting
{
public:
    Setting(const string &strSettingsFile) ;
    ~Setting(){
        ;
    }
    eSensor mSensor;

    int mbRGB;

    //Camera parameters
    cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
    cv::Mat P_l_inv ;
    float fx, fy, cx, cy, invfx, invfy, bf, baseline, mThDepth;

    //Tracking Parameters
    int equalized;
    int mMinFrames;
    int mMaxFrames;
    int skipFrames;
    int FREQ ;
    int MIN_DIST;
    double F_THRESHOLD;
    int SHOW_TRACK;
    double MIN_PARALLAX;
    double TrackerTotalTime;
    double BATime ;
    int SHOW_PNP ;

    //SGM Parameters
    int SADWindowSize, maxDisparity ;
    int useCUDA ;
    int sgmFlag ;
    int upMargin, downMargin, leftMargin, rightMargin;
    float fxDepthMap, fyDepthMap, cxDepthMap, cyDepthMap, bfDepthMap ;

    //Mesh Parameters
    int bMeshing;
    int renderingK ;
    int useColor ;
    double nearPlaneDist ;
    double farPlaneDist ;
    float cubicBound ;
    int updateMeshFlag ;
    int chunkSizeX ;
    int chunkSizeY ;
    int chunkSizeZ ;
    double voxelResolution ;//in meters
    int weight ;
    double carvingDist ;
    int useCarving ;
    double truncationDistConst ;
    double truncationDistLinear ;
    double truncationDistQuad ;
    double truncationDistScale ;
    int downSampleTimes ;
    int updateMeshInterval ;
    float delayLoopTime ;
    int updateNonKF ;
    float cvWaitTime;
    int displayImmediateResult;

    //ORB Parameters
    int maxFeatures;
    float scaleFactor;
    int nLevels;
    int iniThFAST, minThFAST;
    int TH_HIGH;
    int TH_LOW;
    int HISTO_LENGTH;
    std::vector<cv::Point> pattern;

    //Debug Info Parameters
    int memoryLimit;
    int loopStop;

    //Viewer Parameters    
    int bDisplayViewer ;
    int followPose ;
    float mKeyFrameSize;
    float mKeyFrameLineWidth;
    float mGraphLineWidth;
    float mPointSize;
    float mCameraSize;
    float mCameraLineWidth;
    float mMeshLineWidth;
    float mViewpointX;
    float mViewpointY;
    float mViewpointZ;
    float mViewpointF;
    float mViewpointTopDownZ;

    //Frame Parameters
    bool mbInitialComputations;
    float mfGridElementWidthInv;
    float mfGridElementHeightInv;

    float mnMinX;
    float mnMaxX;
    float mnMinY;
    float mnMaxY;
    int height;
    int width;

    int mnScaleLevels;
    float mfScaleFactor;
    float mfLogScaleFactor;
    vector<float> mvScaleFactors;
    vector<float> mvInvScaleFactors;
    vector<float> mvLevelSigma2;
    vector<float> mvInvLevelSigma2;

    //loop closing parameters
    int covisiblityTh ;
    int mnCovisibilityConsistencyTh;
    int loop_MAX_FEATURES;
};

//
extern Setting* pSetting;

// ORB vocabulary used for place recognition and feature matching.
typedef DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB> ORBVocabulary;
extern ORBVocabulary* mpVocabulary ;

//
extern ORB_SLAM2::ORBextractor* mpORBextractorLeft;
extern ORB_SLAM2::ORBextractor* mpORBextractorRight;
extern ORB_SLAM2::ORBextractor* mpIniORBextractor;

extern ofstream debug_f;
}
