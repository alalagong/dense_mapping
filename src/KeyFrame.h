#ifndef KEYFRAME_H
#define KEYFRAME_H

#include "util.h"
#include "Map.h"
#include "MapPoint.h"
#include "Settings.h"
#include "open_chisel/ProjectionIntegrator.h"
#include "open_chisel/Chisel.h"
#include "open_chisel/weighting/ConstantWeighter.h"
#include "open_chisel/truncation/QuadraticTruncator.h"
#include "open_chisel/truncation/InverseTruncator.h"

namespace LT_SLAM
{

class MapPoint;
class Map;

class KeyFrame
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    KeyFrame();
    KeyFrame(unsigned int id, Eigen::Matrix3d& R, Eigen::Vector3d&t,
             const cv::Mat &imLeft, const cv::Mat &imRight,
             const bool calibrated, const MYTIMESTAMP &timeStamp);
    KeyFrame(KeyFrame *frame);
    ~KeyFrame();

    //Frame information
    static Map* mpMap;
    cv::Mat img, img_right ;
    cv::Mat depth ;
    bool isolated ;

    void initDescriptors();
    void ComputeBoW(cv::Mat& mDescriptors);    // Compute Bag of Words representation.

    // Pose functions
    //std::mutex mMutexPose;
    void SetPose(const Eigen::Matrix3d &R, const Eigen::Vector3d &t);
    void SetPose(const Eigen::Matrix4d &T);
    Eigen::Matrix3d GetRotation();
    Eigen::Vector3d GetTranslation();
    Eigen::Matrix4d GetTransformation();

    // Check if a MapPoint is in the frustum of the camera
    // and fill variables of the MapPoint to be used by the tracking
    bool isInFrustum(MapPoint* pMP, float viewingCosLimit);

    // Compute the cell of a keypoint (return false if outside the grid)
    bool PosInGrid(const cv::Point2f &kp, int &posX, int &posY);

    // Frame timestamp.
    unsigned int mnId;
    MYTIMESTAMP mTimeStamp;
    bool calibrated;

    // Camera pose.
    Eigen::Matrix3d R_b_2_w ;
    Eigen::Vector3d t_b_2_w ;
    Eigen::Matrix4d T_b_2_w ;

    int N ;
    // ORB descriptor, each row associated to a keypoint.
    cv::Mat mDescriptors ;
    vector<unsigned int> pIDs;
    // Vector of keypoints (original for visualization) and undistorted (actually used by the system).
    // In the stereo case, mvKeysUn is redundant as images must be rectified.
    // In the RGB-D case, RGB images can be distorted.
    std::vector<cv::Point2f> mvKeys, mvKeysRight;
    std::vector<cv::Point2f> mvKeysUn, mvKeysRightUn;
    vector<uchar> r_status;

    std::list<int> mGrid[FRAME_GRID_COLS][FRAME_GRID_ROWS] ;

    // Bag of Words Vector structures.
    DBoW2::BowVector mBowVec;
    DBoW2::FeatureVector mFeatVec;

    // Reference Keyframe.
    KeyFrame* mpReferenceKF;

    // Undistort keypoints given OpenCV distortion parameters.
    // Only for the RGB-D case. Stereo must be already rectified!
    // (called in the constructor).
    void UndistortKeyPoints();

    // Assign keypoints to the grid for speed up feature matching (called in the constructor).
    void AssignFeaturesToGrid();

    void UpdateConnections();
    set<KeyFrame*> GetConnectedKeyFrames();
    std::vector<KeyFrame*> GetBestCovisibilityKeyFrames(const int &N);

    // Set/check bad flag
    void SetBadFlag();

    // Image
    bool IsInImage(const float &x, const float &y);

    static bool weightComp( int a, int b){
        return a>b;
    }

    static bool lId(KeyFrame* pKF1, KeyFrame* pKF2){
        return pKF1->mnId < pKF2->mnId;
    }

    std::list<chisel::MeshPtr> attachedMeshes;
    std::list<chisel::ChunkPtr> attachedChunks;

    // Variables used by the tracking
    unsigned int mnTrackReferenceForFrame;
    unsigned int mnFuseTargetForKF;

    // Variables used by the local mapping
    unsigned int mnBALocalForKF;
    unsigned int mnBAFixedForKF;

    // Variables used by the keyframe database
    long unsigned int mnLoopQuery;
    int mnLoopWords;
    float mLoopScore;
    long unsigned int mnRelocQuery;
    int mnRelocWords;
    float mRelocScore;

    // Variables used by loop closing
    unsigned int mnBAGlobalForKF;

    // Pose relative to parent (this is computed when bad flag is activated)
    Eigen::Matrix3d delta_R_c_2_p ;
    Eigen::Vector3d delta_t_c_2_p ;

    bool initConnectKeyFrames;
    std::map<KeyFrame*, int> mConnectedKeyFrameWeights;
    std::vector<KeyFrame*> mvpOrderedConnectedKeyFrames;
    std::vector<int> mvOrderedWeights;

    // Spanning Tree and Loop Edges
    bool mbFirstConnection;
    KeyFrame* mpParent;
    std::set<KeyFrame*> mspChildrens;
    std::set<KeyFrame*> mspLoopEdges;

    //std::list<EDGELINK> loopEdges ;

    // Bad flags
    bool mbNotErase;
    bool mbToBeErased;
    bool mbBad;

    int totalKF ;
    int totalMapPoint ;
    int erasedKF ;
    int erasedMP ;
};

} //namespace LT_SLAM

#endif // KEYFRAME_H
