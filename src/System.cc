#include "System.h"
#include <iomanip>

bool has_suffix(const std::string &str, const std::string &suffix) {
    std::size_t index = str.find(suffix, str.size() - suffix.size());
    return (index != std::string::npos);
}

namespace LT_SLAM
{

System::System(const string &strVocFile, const string &strSettingsFile, const eSensor mSensor):mbReset(false)
{
    // Output welcome message
    cout << "Input sensor was set to: ";

    if(mSensor==MONOCULAR)
        cout << "Monocular" << endl;
    else if(mSensor==STEREO)
        cout << "Stereo" << endl;
    else if(mSensor==RGBD)
        cout << "RGB-D" << endl;

    //Read settings file
    pSetting = new Setting(strSettingsFile) ;
    pSetting->mSensor = mSensor ;

    //Load ORB Vocabulary
    cout << endl << "Loading ORB Vocabulary. This could take a while..." << endl;
    clock_t tStart = clock();
    mpVocabulary = new ORBVocabulary();
    bool bVocLoad = false; // chose loading method based on file extension
    if (has_suffix(strVocFile, ".txt"))
        bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
    else
        bVocLoad = mpVocabulary->loadFromBinaryFile(strVocFile);
    if(!bVocLoad)
    {
        cerr << "Wrong path to vocabulary. " << endl;
        cerr << "Failed to open at: " << strVocFile << endl;
        exit(-1);
    }
    printf("Vocabulary loaded in %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);

    //ORBextractor
    mpORBextractorLeft = new ORB_SLAM2::ORBextractor(
                pSetting->maxFeatures,
                pSetting->scaleFactor,
                pSetting->nLevels,
                pSetting->iniThFAST,
                pSetting->minThFAST);

    if( pSetting->mSensor == STEREO ){
        mpORBextractorRight = new ORB_SLAM2::ORBextractor(
                    pSetting->maxFeatures,
                    pSetting->scaleFactor,
                    pSetting->nLevels,
                    pSetting->iniThFAST,
                    pSetting->minThFAST);
    }

    if( pSetting->mSensor== MONOCULAR ){
        mpIniORBextractor = new ORB_SLAM2::ORBextractor(
                    2*pSetting->maxFeatures,
                    pSetting->scaleFactor,
                    pSetting->nLevels,
                    pSetting->iniThFAST,
                    pSetting->minThFAST);
    }
    // Scale Level Info
    pSetting->mnScaleLevels = mpORBextractorLeft->GetLevels();
    pSetting->mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
    pSetting->mfLogScaleFactor = log(pSetting->mfScaleFactor);
    pSetting->mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
    pSetting->mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
    pSetting->mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
    pSetting->mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();



    //Create the Map
    mpMap = new Map(this);
    KeyFrame::mpMap = mpMap ;


    //Create Drawers. These are used by the Viewer
    mpMapDrawer = new MapDrawer(this);
    //    mpFrameDrawer = new FrameDrawer(mpMap);

    //Initialize the Tracking thread
    //(it will live in the main thread of execution, the one that called this constructor)
    mpTracker = new Tracking();


    //Initialize the Local Mapping thread and launch
    mpLocalMapper = new LocalMapping(this);


    mptLocalMapping = thread(&LT_SLAM::LocalMapping::Run,mpLocalMapper);


    mptGlobalOpt = thread(&LT_SLAM::LocalMapping::GlobalOptimization_Loop, mpLocalMapper);


//    //Initialize the Local Meshing thread and launch
    if ( pSetting->bMeshing ){
        mpLocalMesher = new LocalMeshing(this) ;
        mptLocalMeshing = thread(&LT_SLAM::LocalMeshing::Run, mpLocalMesher) ;
    }

    //Initialize the Viewer thread and launch
    if( pSetting->bDisplayViewer ){
        mpViewer = new Viewer(this);
        mptViewer = thread(&Viewer::Run_Multi, mpViewer);
        //mptViewer = thread(&Viewer::Run, mpViewer);
    }
}

void System::TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, cv::Mat& depth, const MYTIMESTAMP &timestamp)
{
    mpTracker->GrabImageStereo(imLeft,imRight, depth, timestamp);
}

void System::Reset()
{
    unique_lock<mutex> lock(mMutexReset);
    mbReset = true;
}

void System::SaveData(string fileName)
{
    ofstream f;

    printf("SaveData : %s\n", fileName.c_str() ) ;
    fileName = fileName + "/lt_slam.txt" ;
    f.open(fileName.c_str());
    if ( f.is_open() )
    {
        KeyFrame*pKF = NULL ;
        bool init = false ;
        Eigen::Matrix3d base_R ;
        Eigen::Vector3d base_t ;

        for ( Map::KFMAPTYPE_ITER iter = mpMap->mspKeyFrames.begin();
              iter != mpMap->mspKeyFrames.end(); iter++ )
        {
            pKF = iter->second ;

            Eigen::Matrix3d R = pKF->GetRotation();
            Eigen::Vector3d t = pKF->GetTranslation();

            if ( init == false ){
                init = true ;
                base_R = R ;
                base_t = t ;
            }
            Eigen::Matrix3d R_k_2_0 = base_R.transpose()*R;
            Eigen::Vector3d t_k_2_0 = base_R.transpose()*(t-base_t);

            Eigen::Quaterniond q(R_k_2_0) ;

            f << pKF->mTimeStamp.sec << "." << pKF->mTimeStamp.nsec <<  " "
              << setprecision(8) << pKF->totalKF << " " << pKF->totalMapPoint
              << setprecision(10)
              << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w()
              << " " << t_k_2_0(0) << " " << t_k_2_0(1) << " " << t_k_2_0(2)
              << " " << setprecision(6) << pKF->erasedKF << " " << pKF->erasedMP << "\n" ;
        }
    }
    f.close();
}

void System::SaveDataKITTI(string kitti_name)
{
    ROS_WARN("SaveDataKITTI") ;

    ofstream f;
    string file_name = "/home/ygling2008/" + kitti_name + ".txt" ;

    while( mpTracker->trackingFrameQueue.size() > 0 && ros::ok()){
        usleep(5000) ;
    }
    mpLocalMapper->mbFinished = true;
    while( mpLocalMapper->mbStop == false && ros::ok() ){
        usleep(5000) ;
    }

    f.open(file_name.c_str());
    if ( f.is_open() )
    {
        KeyFrame*pKF = NULL ;
        bool init = false ;
        Eigen::Matrix3d base_R ;
        Eigen::Vector3d base_t ;

        for ( Map::KFMAPTYPE_ITER iter = mpMap->mspKeyFrames.begin();
              iter != mpMap->mspKeyFrames.end(); iter++ )
        {
            printf("%d ", iter->first ) ;

            pKF = iter->second ;


            Eigen::Matrix3d R = pKF->GetRotation();
            Eigen::Vector3d t = pKF->GetTranslation();

            if ( init == false ){
                init = true ;
                base_R = R ;
                base_t = t ;
            }
            Eigen::Matrix3d R_k_2_0 = base_R.transpose()*R;
            Eigen::Vector3d t_k_2_0 = base_R.transpose()*(t-base_t);

            f << pKF->mTimeStamp.sec << " " << setprecision(10)
              << R_k_2_0(0, 0) << " " << R_k_2_0(0, 1) << " " << R_k_2_0(0, 2) << " " << t_k_2_0(0) << " "
              << R_k_2_0(1, 0) << " " << R_k_2_0(1, 1) << " " << R_k_2_0(1, 2) << " " << t_k_2_0(1) << " "
              << R_k_2_0(2, 0) << " " << R_k_2_0(2, 1) << " " << R_k_2_0(2, 2) << " " << t_k_2_0(2) << "\n";
        }
    }
    f.close();
}

void System::Shutdown()
{
    //SaveData();

    puts("shut down system") ;
    if ( pSetting->bMeshing ){
        mpLocalMesher->mbFinished = true;
    }
    mpLocalMapper->mbFinished = true;
//    /mpLocalMapper->onGlobalOptimization = true ;
    mpLocalMapper->cvGlobalOptimization.notify_all();
    if ( mpViewer != NULL ){
        mpViewer->RequestFinish();
    }

    // Wait until all thread have effectively stopped
    while(!mpLocalMapper->mbStop ||
          (mpViewer != NULL && !mpViewer->isFinished()) )
    {
        usleep(1000);
    }

    if ( pSetting->bMeshing ){
        mptLocalMeshing.join();
    }

    mptLocalMapping.join();

    mptGlobalOpt.join();

    if( pSetting->bDisplayViewer ){
        mptViewer.join();
    }

//    if ( mpVocabulary ){
//        delete mpVocabulary ;
//    }

//    if ( mpMap ){
//        delete mpMap ;
//    }

//    if ( mpMapDrawer ){
//        delete mpMapDrawer ;
//    }
//    if ( mpViewer ){
//        delete mpViewer ;
//    }

//    if ( mpTracker ){
//        delete mpTracker ;
//    }
//    if ( mpLocalMapper ){
//        delete mpLocalMapper ;
//    }
//    if ( mpLocalMesher ){
//        delete mpLocalMesher ;
//    }
}

} //namespace LT_SLAM
