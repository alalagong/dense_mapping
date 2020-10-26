#ifndef LOCALMAPPING_H
#define LOCALMAPPING_H

#include "System.h"
#include "KeyFrame.h"
#include "Tracking.h"
#include "MapPoint.h"
#include "Map.h"
#include "ramRead.h"

using namespace Eigen;

namespace LT_SLAM
{

class System;
class KeyFrame;
class Map;
class MapPoint;

struct IMU_DATA
{
    MYTIMESTAMP t ;
    double ax, ay, az ;//acceleration
    double wx, wy, wz ;//angular
    void print(){
        printf("%u.%u %lf %lf %lf %lf %lf %lf\n", t.sec, t.nsec, ax, ay, az, wx, wy, wz ) ;
    }
};



class LocalMapping
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef pair<set<KeyFrame*>,int> ConsistentGroup;
    LocalMapping(System* sys);

    // Main function
    void updateTrackingData();
    void Run();
    void LocalBundleAdjustment();
    void initLocalMap();

    void processNewKeyFrame();
    KeyFrame* CreateNewKeyFrame(Matrix3d cur_R, Vector3d cur_t);
    bool checkParallax();

    void addFrameInfo(KeyFrame *pKF);

    int projectLocalMap();
    void updateLocalMap(KeyFrame *pKF, vector<KeyFrame*>& vpNeighKFs, std::set<unsigned int>& pIDset);

    std::list<KeyFrame*> EraseKFsToChecked;
    void KeyFrameCulling();

    void StereoInitialization();
    bool Relocalization();

    void marginalize();

    void setMeshingInfo();
    void setViewingInfo();

    void motionOnlyBA(Eigen::Matrix3d& cur_R, Vector3d &cur_t);
    void PnPwithRANSAC(Eigen::Matrix3d& cur_R, Vector3d &cur_t);
    void PnPwithOpenCVNoRANSAC(Eigen::Matrix3d& cur_R, Vector3d &cur_t);
    bool removeOutlier();
    void initNewKeyFrame(KeyFrame* pKF);

    std::list<IMU_DATA> imuVector;

    TrackingFrame curFrame;

    bool mbFinished;
    bool mbStop;
    bool mbAbortBA;

    System* pSystem;
    Map* mpMap;
    Tracking* pTracker ;

    KeyFrame* mpCurrentKeyFrame;
    vector<KeyFrame*> KeyFramesInLocalMap;
    int mpToBeErasedNum;
    std::set<MapPoint*> MapPointsInLocalMap;

    bool firstKeyFrame ;
    int initKeyFrameID ;
    unsigned int mLastLoopKFid;

    //    // from imu_3dm_gx4
    double acc_density = 1.0e-3;
    double gyr_density = 8.73e-5;
    double update_rate = 200.0;
    Matrix3d acc_cov ;
    Matrix3d gyr_cov ;
    Matrix2d pts_cov ;

    // used in nonlinear, fixed first state
    double prior_p_std = 0.0001;
    double prior_q_std = 0.01 / 180.0 * M_PI;

    // used in nonlinear week assumption
    // const double tic_std = .005;
    // const double ric_std = .3 / 180.0 * M_PI;

    // used in nonlinear strong assumption
    const double tic_std = 0.005;
    const double ric_std = 0.01 / 180.0 * M_PI;

    const Matrix3d gra_cov = 0.001 * 0.001 * Matrix3d::Identity();

    enum SolverFlag
    {
        CALIBRATION,
        LINEAR,
        NON_LINEAR
    };

    enum eTrackingState{
        SYSTEM_NOT_READY=-1,
        NO_IMAGES_YET=0,
        NOT_INITIALIZED=1,
        OK=2,
        LOST=3
    };
    eTrackingState mState;

    bool marginalization_flag;//1 insert KF, 0 otherwise
    bool loop_flag ;

    double para_Pose[WINDOW_SIZE + 1][SIZE_POSE];
    double para_Feature[NUM_OF_F][SIZE_FEATURE];
    double par_pose_PnP[SIZE_POSE] ;


//    ceres::Problem problemPnP ;
//    ceres::Solver::Options ceres_options;
//    ceres::LossFunction *loss_function;
//    ceres::Solver::Summary summary;
//        ceres::Problem problem;

    //    MarginalizationFactor *last_marginalization_factor;
    //    vector<double *> last_marginalization_parameter_blocks;

    void solve_ceres();
    void old2new();
    void new2old();

    int frame_count ;
    KeyFrame* keyFrameQueue[WINDOW_SIZE+1] ;
    std::unordered_map<KeyFrame*, int> hashKeyFrameTable;

    Eigen::Matrix3d latest_R ;
    Eigen::Matrix3d latest_t ;

    Eigen::Matrix4d current_T ;
    Eigen::Matrix4d latest_T ;
    Eigen::Matrix4d detla_T ;

    int initKFrameID ;
    bool onGlobalOptimization ;
    bool doneGlobalOptimization ;
    std::mutex mutexGlobalOptimization ;
    std::condition_variable cvGlobalOptimization ;
    KeyFrame* mpMatchedKF ;
    Eigen::Matrix3d pnp_R;
    Eigen::Vector3d pnp_t;
    double para_loop_Pose[MAX_KF][SIZE_POSE] ;
    KeyFrame* fix_KF ;
    int fix_KF_index ;
    int totalInitKFinPoseGraph ;
    std::multiset<KeyFrame*> loopKFset ;
    std::unordered_map<KeyFrame*, int> KF_index;
    ceres::Problem *problemPoseGraph ;
    std::vector<ConsistentGroup> mvConsistentGroups;
    std::vector<KeyFrame*> mvpEnoughConsistentCandidates;
    std::set<MapPoint*> mvpLoopMapPoints;
    bool detectLoop();
    bool checkLoop(Eigen::Matrix3d& pnp_R, Eigen::Vector3d& pnp_t);
    void GlobalOptimization_Loop();
    void poseGraphOptimization(KeyFrame* currentKF,
                                KeyFrame* current_loopKF,
                                const Eigen::Matrix3d& R_l_2_c,
                                const Eigen::Vector3d& t_l_2_c);
    void getBackGlobalOptimizationResult();
    int searchByBow(KeyFrame* pKF,
                    KeyFrame* Frame,
                    float mfNNratio,
                    bool mbCheckOrientation,
                    std::vector<cv::Point2i> &matches);
    int p3Dto2DRansac(KeyFrame* Frame,
                      std::vector<cv::Point2i> &matches,
                      Eigen::Matrix3d& R_pnp,
                      Eigen::Vector3d& t_pnp, bool initial_guess);

    int searchByProjection(KeyFrame* pKF,
                           Matrix3d& R_pnp,
                           Vector3d& t_pnp,
                           std::vector<MapPoint*>& mp_Vector,
                           std::vector<cv::Point2i> &matches,
                           int descriptorTh,
                           double distTh);

    std::vector<int> plot_id;//for debug only

    std::mutex mMutexNotErasedKFs ;
    std::multiset<KeyFrame*> NotErasedKFs ;
};

} //namespace LT_SLAM

#endif // LOCALMAPPING_H
