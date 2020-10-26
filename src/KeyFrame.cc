#include "KeyFrame.h"

namespace LT_SLAM
{

Map* KeyFrame::mpMap = NULL;

KeyFrame::KeyFrame()
{
    T_b_2_w.setIdentity();
    R_b_2_w.setIdentity();
    t_b_2_w.setZero();
    mbFirstConnection = true ;
    mbBad = false ;

    mnBALocalForKF = -1;
    mnBAFixedForKF = -1;

    isolated = false ;
}

KeyFrame::KeyFrame(unsigned int id, Eigen::Matrix3d &R, Eigen::Vector3d &t,
                   const cv::Mat &imLeft, const cv::Mat &imRight,
                   const bool calibrated, const MYTIMESTAMP &timeStamp)
    :mnId(id), mTimeStamp(timeStamp), calibrated(calibrated), mpReferenceKF(static_cast<KeyFrame*>(NULL))
{
    SetPose(R, t) ;
    img = imLeft.clone();
    img_right = imRight.clone();
    mbFirstConnection = true ;
    mbBad = false ;
    initConnectKeyFrames = false ;

    mnLoopQuery = 0;
    mnLoopWords = 0;
    mLoopScore = 0;
    mnRelocQuery = 0;
    mnRelocWords= 0;
    mRelocScore = 0;

    isolated = false ;
    mDescriptors.release();

//    // ORB extraction
//    std::thread threadLeft(&Frame::ExtractORB,this,0,imLeft);
//    std::thread threadRight(&Frame::ExtractORB,this,1,imRight);
//    threadLeft.join();
//    threadRight.join();

//    if(mvKeys.empty()){
//        return;
//    }
//    N = mvKeys.size();

//    UndistortKeyPoints();
//    ComputeStereoMatches();
//    AssignFeaturesToGrid();
    mnBALocalForKF = -1;
    mnBAFixedForKF = -1;
}

KeyFrame::KeyFrame(KeyFrame* frame)
{
    mnId = frame->mnId;
    mTimeStamp = frame->mTimeStamp;
    calibrated = frame->calibrated;
    R_b_2_w = frame->R_b_2_w;
    t_b_2_w = frame->t_b_2_w;
    T_b_2_w = frame->T_b_2_w;
    N = frame->N;
    pIDs = frame->pIDs;

    mDescriptors = frame->mDescriptors.clone();
    mvKeys = frame->mvKeys;
    mvKeysRight = frame->mvKeysRight;
    mvKeysUn = frame->mvKeysUn;
    mvKeysRightUn = frame->mvKeysRightUn;


    for(int i=0;i<FRAME_GRID_COLS;i++)
        for(int j=0; j<FRAME_GRID_ROWS; j++)
            mGrid[i][j]=frame->mGrid[i][j];

    mBowVec = frame->mBowVec;
    mFeatVec = frame->mFeatVec;
    mpReferenceKF = frame->mpReferenceKF;
    mbFirstConnection = frame->mbFirstConnection ;
    mbBad = frame->mbBad;

    mnBALocalForKF = frame->mnBALocalForKF;
    mnBAFixedForKF = frame->mnBAFixedForKF;

    mnLoopQuery = frame->mnLoopQuery ;
    mnLoopWords = frame->mnLoopWords ;
    mLoopScore =  frame->mLoopScore ;
    mnRelocQuery =frame->mnRelocQuery;
    mnRelocWords =frame->mnRelocWords;
    mRelocScore = frame->mRelocScore ;
    isolated = frame->isolated;
}

KeyFrame::~KeyFrame()
{
//    img.release();
//    img_right.release();
//    depth.release();
}

vector<KeyFrame*> KeyFrame::GetBestCovisibilityKeyFrames(const int &N)
{
    if ( initConnectKeyFrames == false ){
        UpdateConnections();
    }
    std::vector<KeyFrame*> tmp ;
    if((int)mvpOrderedConnectedKeyFrames.size()<N){
        tmp = mvpOrderedConnectedKeyFrames;
        //printf("tmp_sz=%d\n", tmp.size() ) ;
    }
    else{
        tmp = vector<KeyFrame*>(mvpOrderedConnectedKeyFrames.begin(),mvpOrderedConnectedKeyFrames.begin()+N);
    }
    /*
    const set<KeyFrame*> sLoopEdges = GetLoopEdges();
    for( KeyFrame* KF: sLoopEdges ){
        tmp.push_back(KF);
    }*/

    return tmp ;
}

void KeyFrame::UpdateConnections()
{
    if ( initConnectKeyFrames ){
        return ;
    }

    map<KeyFrame*,int> KFcounter;
    for( int i = 0, sz = pIDs.size(); i< sz; i++ )
    {
        unsigned int pID = pIDs[i] ;
        if ( pID == 0 ){
            continue ;
        }
        MapPoint* mp = mpMap->getMapPoint(pID) ;
        map<KeyFrame*,unsigned int> observations = mp->mObservations;
        for(map<KeyFrame*,unsigned int>::iterator mit=observations.begin(), mend=observations.end();
            mit!=mend; mit++)
        {
            if(mit->first->mnId==mnId){
                continue;
            }
            else
            {
                if ( KFcounter.find(mit->first) != KFcounter.end() ){
                    KFcounter[mit->first]++;
                }
                else {
                     KFcounter[mit->first] = 1 ;
                }
            }
        }
    }

    if ( KFcounter.size() < 1 )
    {
        // mspConnectedKeyFrames = spConnectedKeyFrames;
        mConnectedKeyFrameWeights.clear();
        mvpOrderedConnectedKeyFrames.clear();
        mvOrderedWeights.clear();
        initConnectKeyFrames = true ;
        return ;
    }

//    if ( mnId == 1 )
//    {
//        ROS_WARN("KFcounter.size()=%d", KFcounter.size());
//    }

    int nmax=0;
    KeyFrame* pKFmax=NULL;
    std::vector<pair<int,KeyFrame*> > vPairs;
    vPairs.reserve(KFcounter.size());
    for(map<KeyFrame*,int>::iterator mit=KFcounter.begin();
        mit!=KFcounter.end();
        mit++)
    {
        if(mit->second>nmax)
        {
            nmax=mit->second;
            pKFmax=mit->first;
        }
        if(mit->second >= pSetting->covisiblityTh )
        {
            vPairs.push_back(make_pair(mit->second,mit->first));
        }
    }
    if(vPairs.empty()){
        vPairs.push_back(make_pair(nmax,pKFmax));
    }
    sort(vPairs.begin(),vPairs.end());

    list<KeyFrame*> lKFs;
    list<int> lWs;
    for(size_t i=0; i<vPairs.size();i++)
    {
        lKFs.push_front(vPairs[i].second);
        lWs.push_front(vPairs[i].first);
    }

    {
        // mspConnectedKeyFrames = spConnectedKeyFrames;
        mConnectedKeyFrameWeights = KFcounter;
        mvpOrderedConnectedKeyFrames = std::vector<KeyFrame*>(lKFs.begin(),lKFs.end());
        mvOrderedWeights = std::vector<int>(lWs.begin(), lWs.end());
    }
    initConnectKeyFrames = true ;
}

set<KeyFrame*> KeyFrame::GetConnectedKeyFrames()
{
    if ( initConnectKeyFrames == false ){
        UpdateConnections();
    }
    set<KeyFrame*> s;
    for(map<KeyFrame*,int>::iterator mit=mConnectedKeyFrameWeights.begin();mit!=mConnectedKeyFrameWeights.end();mit++)
        s.insert(mit->first);
    return s;
}

void KeyFrame::SetBadFlag()
{   



//    puts("SetKeyFrameBadFlag") ;
}

void KeyFrame::AssignFeaturesToGrid()
{
    for(unsigned int i=0; i<FRAME_GRID_COLS;i++){
        for (unsigned int j=0; j<FRAME_GRID_ROWS;j++){
            mGrid[i][j].clear();
        }
    }

    for(int i=0;i<N;i++)
    {
        int nGridPosX, nGridPosY;
        if(PosInGrid(mvKeysUn[i],nGridPosX,nGridPosY)){
            mGrid[nGridPosX][nGridPosY].push_back(i);
        }
    }
}

void KeyFrame::SetPose(const Eigen::Matrix3d& R, const Eigen::Vector3d& t)
{
    R_b_2_w = R ;
    t_b_2_w = t ;
    T_b_2_w.setIdentity();
    T_b_2_w.block(0, 0, 3, 3) = R_b_2_w;
    T_b_2_w.block(0, 3, 3, 1) = t_b_2_w;
}

void KeyFrame::SetPose(const Eigen::Matrix4d& T)
{
    T_b_2_w = T ;
    R_b_2_w = T.block(0, 0, 3, 3) ;
    t_b_2_w = T.block(0, 3, 3, 1) ;
}

Eigen::Matrix3d KeyFrame::GetRotation()
{
    return R_b_2_w;
}

Eigen::Vector3d KeyFrame::GetTranslation()
{
    return t_b_2_w;
}

Eigen::Matrix4d KeyFrame::GetTransformation()
{
    return T_b_2_w;
}

bool KeyFrame::isInFrustum(MapPoint *pMP, float viewingCosLimit)
{
//    pMP->mbTrackInView = false;

//    // 3D in absolute coordinates
//    cv::Mat P = pMP->GetWorldPos();

//    // 3D in camera coordinates
//    const cv::Mat Pc = mRcw*P+mtcw;
//    const float &PcX = Pc.at<float>(0);
//    const float &PcY= Pc.at<float>(1);
//    const float &PcZ = Pc.at<float>(2);

//    // Check positive depth
//    if(PcZ<0.0f)
//        return false;

//    // Project in image and check it is not outside
//    const float invz = 1.0f/PcZ;
//    const float u=fx*PcX*invz+cx;
//    const float v=fy*PcY*invz+cy;

//    if(u<mnMinX || u>mnMaxX)
//        return false;
//    if(v<mnMinY || v>mnMaxY)
//        return false;

//    // Check distance is in the scale invariance region of the MapPoint
//    const float maxDistance = pMP->GetMaxDistanceInvariance();
//    const float minDistance = pMP->GetMinDistanceInvariance();
//    const cv::Mat PO = P-mOw;
//    const float dist = cv::norm(PO);

//    if(dist<minDistance || dist>maxDistance)
//        return false;

//   // Check viewing angle
//    cv::Mat Pn = pMP->GetNormal();

//    const float viewCos = PO.dot(Pn)/dist;

//    if(viewCos<viewingCosLimit)
//        return false;

//    // Predict scale in the image
//    const int nPredictedLevel = pMP->PredictScale(dist,mfLogScaleFactor);

//    // Data used by the tracking
//    pMP->mbTrackInView = true;
//    pMP->mTrackProjX = u;
//    pMP->mTrackProjXR = u - mbf*invz;
//    pMP->mTrackProjY = v;
//    pMP->mnTrackScaleLevel= nPredictedLevel;
//    pMP->mTrackViewCos = viewCos;

    return true;
}

bool KeyFrame::PosInGrid(const cv::Point2f &kp, int &posX, int &posY)
{
    posX = round((kp.x - pSetting->mnMinX)*pSetting->mfGridElementWidthInv);
    posY = round((kp.y - pSetting->mnMinY)*pSetting->mfGridElementHeightInv);

    //Keypoint's coordinates are undistorted, which could cause to go out of the image
    if(posX<0 || posX>=FRAME_GRID_COLS || posY<0 || posY>=FRAME_GRID_ROWS)
        return false;

    return true;
}

void KeyFrame::initDescriptors()
{
    std::vector<cv::KeyPoint> keypoints;
    keypoints.reserve(mvKeysUn.size());
    for( size_t i = 0; i < mvKeysUn.size(); i++ ) {
      keypoints.push_back(cv::KeyPoint(mvKeysUn[i], 1.f));
    }
    ORB_SLAM2::computeDescriptors(img, keypoints, mDescriptors, pSetting->pattern) ;
}


void KeyFrame::ComputeBoW(cv::Mat &mDescriptors)
{
    if(mBowVec.empty())
    {
        std::vector<cv::Mat> vCurrentDesc;
        vCurrentDesc.reserve(mDescriptors.rows);
        for (int j=0;j<mDescriptors.rows;j++){
            vCurrentDesc.push_back(mDescriptors.row(j));
        }
        mpVocabulary->transform(vCurrentDesc,mBowVec,mFeatVec,4);
    }
}

void KeyFrame::UndistortKeyPoints()
{
    if( calibrated )
    {
        mvKeysUn=mvKeys;
        return;
    }

//    // Fill matrix with points
//    cv::Mat mat(N,2,CV_32F);
//    for(int i=0; i<N; i++)
//    {
//        mat.at<float>(i,0)=mvKeys[i].pt.x;
//        mat.at<float>(i,1)=mvKeys[i].pt.y;
//    }

//    // Undistort points
//    mat=mat.reshape(2);
//    cv::undistortPoints(mat,mat,mK,mDistCoef,cv::Mat(),mK);
//    mat=mat.reshape(1);

//    // Fill undistorted keypoint vector
//    mvKeysUn.resize(N);
//    for(int i=0; i<N; i++)
//    {
//        cv::KeyPoint kp = mvKeys[i];
//        kp.pt.x=mat.at<float>(i,0);
//        kp.pt.y=mat.at<float>(i,1);
//        mvKeysUn[i]=kp;
//        //std::cout << "kp" << kp.pt << std::endl ;
//    }
}

bool KeyFrame::IsInImage(const float &x, const float &y)
{
    return (x>=pSetting->mnMinX && x<pSetting->mnMaxX && y>=pSetting->mnMinY && y<pSetting->mnMaxY);
}

} //namespace LT_SLAM
