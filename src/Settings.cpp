#include "Settings.h"

namespace LT_SLAM
{

Setting* pSetting;
ORBVocabulary* mpVocabulary ;
ORB_SLAM2::ORBextractor* mpORBextractorLeft;
ORB_SLAM2::ORBextractor* mpORBextractorRight;
ORB_SLAM2::ORBextractor* mpIniORBextractor ;
ofstream debug_f;

Setting::Setting(const string &strSettingsFile)
{
    cout << "strSettingPath: " << strSettingsFile << "\n"  ;

    cv::FileStorage fSettings(strSettingsFile, cv::FileStorage::READ);
    if(!fSettings.isOpened())
    {
        cerr << "Failed to open settings file at: " << strSettingsFile << endl;
        exit(-1);
    }

    //Camera Parameters
    fx = fSettings["Camera.fx"];
    invfx = 1/fx ;
    fy = fSettings["Camera.fy"];
    invfy = 1/fy ;
    cx = fSettings["Camera.cx"];
    cy = fSettings["Camera.cy"];
    bf = fSettings["Camera.bf"];
    baseline = bf/fx ;
    mThDepth = baseline*(float)fSettings["ThDepth"];

    K_l = cv::Mat::eye(3, 3, CV_64F);
    K_l.at<double>(0, 0) = fx;
    K_l.at<double>(1, 1) = fy;
    K_l.at<double>(0, 2) = cx;
    K_l.at<double>(1, 2) = cy;
    //K_r;
    P_l = K_l.clone();
    //P_r;
    //R_l;
    //R_r;
    //D_l;
    //D_r;

    //Tracking Paramters
    skipFrames = fSettings["skipFrames"];
    mMinFrames = 0 ;
    mMaxFrames = fSettings["Camera.fps"];
    FREQ = fSettings["FREQ"];
    mbRGB = fSettings["Camera.RGB"];
    TrackerTotalTime = fSettings["TrackerTotalTime"] ;
    BATime = fSettings["BATime"] ;
    if ( BATime < 0.0001 ){
        BATime = 0.1 ;
    }

    //Debug Info Paramters
    bDisplayViewer = fSettings["displayViewer"];
    followPose = fSettings["followPose"] ;
    memoryLimit = fSettings["memoryLimit"] ;
    if ( memoryLimit < 1 ){
        memoryLimit = 10000 ;
    }
    loopStop = fSettings["loopStop"] ;

    //SGM Parameters
    sgmFlag = 1;
    useCUDA = fSettings["useCUDA"] ;
    leftMargin = fSettings["leftMargin"];
    rightMargin = fSettings["rightMargin"];
    upMargin = fSettings["upMargin"];
    downMargin = fSettings["downMargin"];
    maxDisparity = fSettings["maxDisparity"];
    downSampleTimes = fSettings["downSampleTimes"] ;
    SADWindowSize = fSettings["SADWindowSize"] ;

    fxDepthMap = fx;
    fyDepthMap = fy;
    cxDepthMap = cx;
    cyDepthMap = cy;
    bfDepthMap = bf;
    for( int i = 0 ; i < downSampleTimes; i++ ){
        fxDepthMap /= 2 ;
        fyDepthMap /= 2 ;
        cxDepthMap = (cxDepthMap+0.5)/2.0 - 0.5 ;
        cyDepthMap = (cyDepthMap+0.5)/2.0 - 0.5 ;
        bfDepthMap /= 2;
        leftMargin /= 2;
        rightMargin /= 2;
        upMargin /= 2;
        downMargin /=  2;
    }

    //Meshing Paramters
    bMeshing = fSettings["enableMeshing"] ;
    cvWaitTime = fSettings["cvWaitTime"] ;
    renderingK = fSettings["renderingK"] ;
    downSampleTimes = fSettings["downSampleTimes"] ;
    updateMeshFlag = fSettings["updateMeshFlag"] ;
    updateNonKF = fSettings["updateNonKF"] ;
    updateMeshInterval = fSettings["updateMeshInterval"] ;
    cubicBound = fSettings["cubicBound"] ;
    useColor = fSettings["useColor"] ;
    nearPlaneDist = fSettings["nearPlaneDist"];
    farPlaneDist = fSettings["farPlaneDist"] ;
    chunkSizeX = fSettings["chunkSizeX"] ;
    chunkSizeY = fSettings["chunkSizeY"] ;
    chunkSizeZ = fSettings["chunkSizeZ"] ;
    voxelResolution = fSettings["voxelResolution"] ;//in meters
    weight = fSettings["weight"] ;
    useCarving = fSettings["useCarving"] ;
    carvingDist = fSettings["carvingDist"] ;
    truncationDistConst = fSettings["truncationDistConst"];
    truncationDistLinear = fSettings["truncationDistLinear"];
    truncationDistQuad = fSettings["truncationDistQuad"];
    truncationDistScale = fSettings["truncationDistScale"] ;
    delayLoopTime = fSettings["delayLoopTime"] ;
    displayImmediateResult = fSettings["displayImmediateResult"] ;

    //ORB Parameters
    maxFeatures = fSettings["ORBextractor.nFeatures"];
    scaleFactor = fSettings["ORBextractor.scaleFactor"];
    nLevels = fSettings["ORBextractor.nLevels"];
    iniThFAST = fSettings["ORBextractor.iniThFAST"];
    minThFAST = fSettings["ORBextractor.minThFAST"];

    TH_HIGH = 100;
    TH_LOW = 50;
    HISTO_LENGTH = 30;

    //Viewer Parameters
    mViewpointX = fSettings["Viewer.ViewpointX"];
    mViewpointY = fSettings["Viewer.ViewpointY"];
    mViewpointZ = fSettings["Viewer.ViewpointZ"];
    mViewpointF = fSettings["Viewer.ViewpointF"];
    mViewpointTopDownZ = fSettings["Viewer.TopDownZ"];
    mKeyFrameSize = fSettings["Viewer.KeyFrameSize"];
    mKeyFrameLineWidth = fSettings["Viewer.KeyFrameLineWidth"];
    mGraphLineWidth = fSettings["Viewer.GraphLineWidth"];
    mPointSize = fSettings["Viewer.PointSize"];
    mCameraSize = fSettings["Viewer.CameraSize"];
    mCameraLineWidth = fSettings["Viewer.CameraLineWidth"];
    mMeshLineWidth = fSettings["Viewer.MeshLineWidth"] ;

    //Frame Paramters
    mbInitialComputations = false ;

    equalized = fSettings["equalized"] ;
    MIN_DIST = fSettings["MIN_DIST"];
    F_THRESHOLD = fSettings["F_THRESHOLD"] ;
    SHOW_TRACK = fSettings["SHOW_TRACK"] ;
    MIN_PARALLAX = fSettings["MIN_PARALLAX"] ;
    SHOW_PNP = fSettings["SHOW_PNP"] ;
    //MIN_PARALLAX /= fx;

    const int npoints = 512;
    const cv::Point* pattern0 = (const cv::Point*)ORB_SLAM2::bit_pattern_31_;
    std::copy(pattern0, pattern0 + npoints, std::back_inserter(pattern));

    covisiblityTh = fSettings["covisiblityTh"] ;
    mnCovisibilityConsistencyTh = 3;
    loop_MAX_FEATURES = fSettings["loop_MAX_FEATURES"] ;
    if ( loop_MAX_FEATURES < 1 ){
        loop_MAX_FEATURES = 500 ;
    }
}

}
