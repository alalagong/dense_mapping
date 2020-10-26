#include "LocalMapping.h"
#include "ros/assert.h"

namespace LT_SLAM
{

#define OPENCV_3 1

LocalMapping::LocalMapping(System* sys)
{
    pSystem = sys;
    mpMap = sys->mpMap;
    pTracker = sys->mpTracker;
    firstKeyFrame = true ;

    // from imu_3dm_gx4
    acc_density = 1.0e-3;
    gyr_density = 8.73e-5;
    update_rate = 200.0;

    acc_cov = std::pow(acc_density * std::sqrt(update_rate), 2.0) * Matrix3d::Identity(); // 0.014
    gyr_cov = std::pow(gyr_density * std::sqrt(update_rate), 2.0) * Matrix3d::Identity(); // 0.0012
    pts_cov = (0.5 / pSetting->fx) * (0.5 / pSetting->fx) * Matrix2d::Identity();

    // used in nonlinear, fixed first state
    prior_p_std = 0.0001;
    prior_q_std = 0.01 / 180.0 * M_PI;


    //problem.SetParameterBlockConstant(para_Pose[0]);
    //    for (int i = 0; i < NUM_OF_CAM; i++)
    //    {
    //        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
    //        problem.AddParameterBlock(para_Ex_Pose[i], SIZE_POSE, local_parameterization);
    //        if (!ESTIMATE_EXTRINSIC)
    //        {
    //            ROS_WARN("fix extrinsic param");
    //            problem.SetParameterBlockConstant(para_Ex_Pose[i]);
    //        }
    //        else
    //            ROS_WARN("estimate extrinsic param");
    //    }

    //problemPnP.AddParameterBlock(par_pose_PnP, SIZE_POSE, new PoseLocalParameterization() );
    problemPoseGraph = new ceres::Problem();
    onGlobalOptimization = false ;
    doneGlobalOptimization = true ;

    frame_count = 0 ;

    NotErasedKFs.clear();

    totalInitKFinPoseGraph = -1;
}

void LocalMapping::old2new()
{
    Eigen::Vector3d cur_t ;
    Eigen::Matrix3d cur_R ;
    for (int i = 0; i <= frame_count; i++)
    {
        KeyFrame* pKF = keyFrameQueue[i] ;
        cur_t = pKF->GetTranslation();
        cur_R = pKF->GetRotation();
        para_Pose[i][0] = cur_t.x();
        para_Pose[i][1] = cur_t.y();
        para_Pose[i][2] = cur_t.z();
        Quaterniond q{cur_R};
        para_Pose[i][3] = q.x();
        para_Pose[i][4] = q.y();
        para_Pose[i][5] = q.z();
        para_Pose[i][6] = q.w();
    }
    int k = 0;
    for( MapPoint* mp: MapPointsInLocalMap ){
        para_Feature[k][0] = mp->pos(0) ;
        para_Feature[k][1] = mp->pos(1) ;
        para_Feature[k][2] = mp->pos(2) ;
        k++ ;
    }
}

void LocalMapping::new2old()
{
    Eigen::Vector3d cur_t ;
    Eigen::Matrix3d cur_R ;
    Eigen::Quaterniond cur_q ;
    for (int i = 0; i <= frame_count; i++)
    {
        cur_t << para_Pose[i][0], para_Pose[i][1], para_Pose[i][2];
        cur_q.x() = para_Pose[i][3];
        cur_q.y() = para_Pose[i][4];
        cur_q.z() = para_Pose[i][5];
        cur_q.w() = para_Pose[i][6];
        cur_R = cur_q.normalized().toRotationMatrix();

        KeyFrame* pKF = keyFrameQueue[i] ;
        pKF->SetPose(cur_R, cur_t);
    }

    int k = 0;
    for( MapPoint* mp: MapPointsInLocalMap ){
        mp->pos << para_Feature[k][0],
                para_Feature[k][1],
                para_Feature[k][2];
        k++ ;
    }
}

void LocalMapping::initLocalMap()
{
    //set of local map
    std::set<unsigned int> mpToBeErased;

    mpToBeErasedNum = 0;
    MapPointsInLocalMap.clear();
    for( int i=0; i < frame_count; i++ )
    {
        KeyFrame* pKF = keyFrameQueue[i] ;
        for( unsigned int& pID: pKF->pIDs)
        {
            if ( pID == 0 ){
                continue ;
            }
            if ( mpMap->mspMapPoints.find(pID) == mpMap->mspMapPoints.end() ){
                ROS_WARN("can not find map point = %d KFid= %d", pID, pKF->mnId ) ;
            }
            MapPoint* mp = mpMap->getMapPoint(pID);
            MapPointsInLocalMap.insert(mp) ;
        }
    }
    //printf("MapPointsInLocalMap.sz 0= %d\n", MapPointsInLocalMap.size() ) ;

    //delete map point with that is not initialized or observed more than 2 keyframes in the window
    for( std::set<MapPoint*>::iterator pIter = MapPointsInLocalMap.begin();
         pIter != MapPointsInLocalMap.end(); )
    {
        MapPoint* mp = *pIter;
        if ( mp->init == false )
        {
            if ( mp->mObservations.find(keyFrameQueue[frame_count]) == mp->mObservations.end() ){
                mpMap->eraseMapPoint(mp);
            }
            pIter = MapPointsInLocalMap.erase(pIter) ;
            mpToBeErasedNum++;
            //printf("%d-%d ", mp->numOfMonoObservations, mp->numOfStereoObservations);
            continue ;
        }

        //delete Mappoint with less than 3 observations
        if ( mp->mObservations.find(keyFrameQueue[frame_count]) == mp->mObservations.end()
             && mp->mObservations.size() < 2 ){
            mpMap->eraseMapPoint(mp);
            pIter = MapPointsInLocalMap.erase(pIter) ;
            mpToBeErasedNum++;
            continue ;
        }

        int cnt = 0;
        for(std::map<KeyFrame*, unsigned int>::iterator obs = mp->mObservations.begin() ;
            obs != mp->mObservations.end(); obs++ )
        {
            if ( hashKeyFrameTable.find(obs->first) != hashKeyFrameTable.end() ){
                cnt++ ;
            }
        }
        if ( cnt < 2 ){
            pIter = MapPointsInLocalMap.erase(pIter) ;
        }
        else{
            pIter++;
        }

        //cout << mp->pos.transpose() << "\n" ;
    }
    //printf("MapPointsInLocalMap.sz 1= %d\n", MapPointsInLocalMap.size() ) ;

    //mamximum feature constaints
    for( std::set<MapPoint*>::iterator pIter = MapPointsInLocalMap.begin();
         pIter != MapPointsInLocalMap.end(); )
    {
        if ( MapPointsInLocalMap.size() < NUM_OF_F-5 ){
            break ;
        }
        pIter = MapPointsInLocalMap.erase(pIter) ;
    }
    //printf("MapPointsInLocalMap.sz 2= %d\n", MapPointsInLocalMap.size() ) ;
}

void LocalMapping::LocalBundleAdjustment()
{
    old2new();

    ceres::Solver::Options ceres_options;
    ceres::Solver::Summary summary;
    ceres::Problem problem;

    //ceres setup
    double reprojectLoss = 5.991*SQ(1.0/pSetting->fx);
    ceres::LossFunction * loss_function = new ceres::HuberLoss(reprojectLoss);
    ceres_options.max_solver_time_in_seconds = pSetting->BATime;
    ceres_options.num_threads = 2 ;
    ceres_options.max_num_iterations = 6;
    //ceres_options.function_tolerance = 1.0e-6;
    ceres_options.minimizer_type = ceres::TRUST_REGION;
    ceres_options.linear_solver_type = ceres::DENSE_SCHUR;
    ceres_options.trust_region_strategy_type = ceres::DOGLEG;
    //ceres_options.sparse_linear_algebra_library_type = ceres::SUITE_SPARSE;

    for (int i = 0; i < WINDOW_SIZE + 1; i++)
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Pose[i], SIZE_POSE, local_parameterization);
        //problem.AddParameterBlock(para_SpeedBias[i], SIZE_SPEEDBIAS);
    }


    //fix the pose that is the oldest
    int oldest_pose_index = 0 ;
    int oldest_pose_id = keyFrameQueue[frame_count]->mnId ;
    for( int i=0; i <= frame_count; i++ )
    {
        if ( keyFrameQueue[i]->mnId < oldest_pose_id ){
            oldest_pose_id = keyFrameQueue[i]->mnId;
            oldest_pose_index = i ;
        }
    }
    problem.SetParameterBlockConstant(para_Pose[oldest_pose_index]);
    for( int i=0; i <= frame_count; i++ )
    {
        if ( keyFrameQueue[i]->isolated ){
            problem.SetParameterBlockConstant(para_Pose[i]);
        }
    }

    //begin ceres problem construction
    int MapPoint_index = 0;
    //add constraints
    std::vector<MapPoint*> mpList ;
    std::vector<KeyFrame*> kfList ;
    std::vector<unsigned char> obList;
    std::vector<ceres::ResidualBlockId> constraintList ;
    ceres::ResidualBlockId rid;
    double u_x, u_y, shift ;
    for( MapPoint* mp: MapPointsInLocalMap )
    {
        for(std::map<KeyFrame*, unsigned int>::iterator obs = mp->mObservations.begin() ;
            obs != mp->mObservations.end(); obs++ )
        {
            KeyFrame* pKFi = obs->first;
            if ( hashKeyFrameTable.find(pKFi) == hashKeyFrameTable.end() ){
                continue ;
            }
            int KF_index = hashKeyFrameTable[pKFi] ;
            if ( KF_index < 0 || KF_index > frame_count ){
                ROS_WARN("KF_index=%d", KF_index ) ;
            }
            unsigned int flag = obs->second >> 16 ;
            unsigned int index = obs->second & 0xffff ;

            //printf("%d->%d[%d] ", KF_index, MapPoint_index, (flag>0) ) ;
            if( flag == 0 )// Monocular observation
            {
                u_x = (pKFi->mvKeysUn[index].x-pSetting->cx)/pSetting->fx;
                u_y = (pKFi->mvKeysUn[index].y-pSetting->cy)/pSetting->fy;
                shift = 0 ;

                rid = problem.AddResidualBlock(
                            new ProjectionFactor(u_x, u_y, shift),
                            loss_function,
                            para_Pose[KF_index],
                            para_Feature[MapPoint_index] );
                constraintList.push_back(rid);

                mpList.push_back(mp);
                kfList.push_back(pKFi);
                obList.push_back((unsigned char)flag);
            }
            else // Stereo observation
            {
                //printf("%d->%d[S] ", KF_index, MapPoint_index ) ;
                if ( pKFi->r_status[index] == 0 ){
                    ROS_WARN("stereo observation error!!") ;
                }
                u_x = (pKFi->mvKeysUn[index].x-pSetting->cx)/pSetting->fx;
                u_y = (pKFi->mvKeysUn[index].y-pSetting->cy)/pSetting->fy;
                shift = 0 ;

                rid = problem.AddResidualBlock(
                            new ProjectionFactor(u_x, u_y, shift),
                            loss_function,
                            para_Pose[KF_index],
                            para_Feature[MapPoint_index] );
                constraintList.push_back(rid);


                u_x = (pKFi->mvKeysRightUn[index].x-pSetting->cx)/pSetting->fx;
                u_y = (pKFi->mvKeysRightUn[index].y-pSetting->cy)/pSetting->fy;
                shift = -pSetting->baseline;
                ceres::ResidualBlockId rid = problem.AddResidualBlock(
                            new ProjectionFactor(u_x, u_y, shift),
                            loss_function,
                            para_Pose[KF_index],
                            para_Feature[MapPoint_index] );
                constraintList.push_back(rid);

                mpList.push_back(mp);
                kfList.push_back(pKFi);
                obList.push_back((unsigned char)flag);
            }
        }
        MapPoint_index++ ;
    }


    ceres::Solve(ceres_options, &problem, &summary);


    //cout << summary.BriefReport() << endl;
    new2old();


//    //release the fixed pose parameters block
//    problem.SetParameterBlockVariable(para_Pose[oldest_pose_index]);
//    for( int i=0; i <= frame_count; i++ )
//    {
//        if ( keyFrameQueue[i]->isolated ){
//            problem.SetParameterBlockVariable(para_Pose[i]);
//        }
//    }

    //        //remove outlier observation
    //        ceres::Problem::EvaluateOptions evalOpts;
    //        evalOpts.parameter_blocks ;
    //        evalOpts.residual_blocks = constraintList;
    //        evalOpts.apply_loss_function = false ;
    //        std::vector<double> res;
    //        double cost;
    //        problem.Evaluate(evalOpts, &cost, &res, NULL, NULL);
    //        int mBad = 0 ;
    //        double monoLoss = 5.991*SQ(1.0/pSetting->fx);
    //        double stereoLoss = 7.815*SQ(1.0/pSetting->fx);
    //        //printf("%d %d %d", constraintList.size(), res.size(), idList.size() ) ;
    //        size_t k = 0 ;
    //        for( size_t i = 0, sz = constraintList.size(); i < sz; )
    //        {
    //            if ( obList[k] == 0 )
    //            {
    //                if ( SQ(res[i]) > monoLoss)
    //                {
    //                    mBad++ ;
    //                    if ( mpMap->mspMapPoints.find(mpList[k]->mnId) != mpMap->mspMapPoints.end() ){
    //                        mpMap->eraseObservation(mpList[k], kfList[k]) ;
    //                    }
    //                }
    //                i++ ;
    //            }
    //            else
    //            {
    //                if ( SQ(res[i]) + SQ(res[i+1]) > stereoLoss )
    //                {
    //                    mBad++ ;
    //                    if ( mpMap->mspMapPoints.find(mpList[k]->mnId) != mpMap->mspMapPoints.end() ){
    //                        mpMap->eraseObservation(mpList[k], kfList[k]) ;
    //                    }
    //                }
    //                i += 2 ;
    //            }
    //            k++ ;
    //        }
    //        ROS_WARN("[Outlier]k=%d total=%d mBad=%d", k, mpList.size(), mBad ) ;
}

void LocalMapping::updateTrackingData()
{
    //update track feature id
    std::unordered_map<unsigned int, unsigned int>::iterator mit ;
    for( int i=0; i < curFrame.ids.size(); i++ )
    {
        mit = mpMap->mergeMap.find(curFrame.ids[i]) ;
        if ( mit == mpMap->mergeMap.end() ){
            continue ;
        }
        curFrame.ids[i] = mit->second ;
    }
}

void LocalMapping::Run()
{
    puts("start local mapping thread") ;
    mbFinished = false;
    mbStop = false ;
    bool flag = false ;

    mState = NOT_INITIALIZED;

    //    ProcessList pl ;
    //    LinuxProcessList_scanMemoryInfo(&pl) ;
    //    unsigned long long startRam = pl.usedMem;
    //    unsigned long long used ;

    while( ros::ok())
    {
        //Global Optimization result check
        if ( onGlobalOptimization && doneGlobalOptimization )
        {
            getBackGlobalOptimizationResult();
            onGlobalOptimization = false ;
            doneGlobalOptimization = false ;

            if ( pSetting->bMeshing > 0 )
            {
                unique_lock<mutex> lock( pSystem->mpLocalMesher->mutexLoopImg ) ;
                pSystem->mpLocalMesher->loopImgFlag = false;
            }
        }

        //momory check
        //        LinuxProcessList_scanMemoryInfo(&pl) ;
        //        used = (pl.usedMem-startRam)/1024 ;
        int limitKF = 100000;
        if ( !onGlobalOptimization && mpMap->mspKeyFrames.size() > limitKF )
        {
            vector<KeyFrame*> kfList ;
            kfList.reserve( mpMap->mspKeyFrames.size());
            for( Map::KFMAPTYPE_ITER kfIter = mpMap->mspKeyFrames.begin();
                 kfIter != mpMap->mspKeyFrames.end();
                 kfIter++ )
            {
                kfList.push_back(kfIter->second);
            }

            for( int i = 0, sz = kfList.size(); i < sz ; i++ )
            {
                KeyFrame* pKF = kfList[i] ;
                int kfID = pKF->mnId ;

                if ( sz - i < limitKF - 10 ){
                    printf("\n") ;
                    break ;
                }

                printf("earseKF=%d ", kfID ) ;
                mpMap->numOfEraseKF++ ;

                for(KeyFrame* loopKF: pKF->mspLoopEdges ){
                    loopKF->mspLoopEdges.erase(pKF) ;
                }
                pKF->mspLoopEdges.clear();

                if ( loopKFset.find(pKF) != loopKFset.end() ){
                    loopKFset.erase(pKF) ;
                }

                mpMap->mpKeyFrameDatabase->erase(pKF);
                mpMap->EraseKeyFrame(pKF, true);

                //                LinuxProcessList_scanMemoryInfo(&pl) ;
                //                used = (pl.usedMem-startRam)/1024 ;
                //                if ( used < pSetting->memoryLimit/2 ){
                //                    break ;
                //                }
            }

        }

        //new data check
        {
            flag = true ;
            if ( pSetting->loopStop && onGlobalOptimization && doneGlobalOptimization == false )
            {
                char key;
                for( ; ros::ok(); )
                {
                    key = cv::waitKey(1);
                    //printf("2 key=%c\n", key) ;
                    if ( key == 'g' && doneGlobalOptimization == true ){
                        break ;
                    }
                }
            }

            std::unique_lock<std::mutex> lk(pTracker->trackingFrameQueueMutex) ;
            if ( pTracker->trackingFrameQueue.size() > 0 && flag )
            {
                curFrame = pTracker->trackingFrameQueue.front();
                //curFrame = tmp;
                //curFrame.cur_img = tmp.cur_img.clone();
                //curFrame.cur_img_r = tmp.cur_img_r.clone();
                //                curFrame.cur_img.release();
                //                curFrame.cur_img_r.release();

                pTracker->trackingFrameQueue.pop_front();
            }
            else {
                flag = false ;
            }
        }

        if ( flag )
        {
            if(mState == NOT_INITIALIZED)
            {
                if( pSetting->mSensor == STEREO )
                    StereoInitialization();
                else{
                    //MonocularInitialization();
                }
            }
            else
            {

                TicToc tc ;
                updateTrackingData();
                //                ROS_INFO("updateTrackingData costs: %fms", tc.toc());

                processNewKeyFrame();
                ROS_INFO("processNewKeyFrame costs: %fms", tc.toc());

                marginalization_flag = checkParallax();
                ROS_INFO("frame_count = %d marginalization_flag = %d", frame_count, marginalization_flag);
                ROS_INFO("checkParallax costs: %fms", tc.toc());

                if ( marginalization_flag )
                {
                    if ( frame_count > 1 ){
                        initNewKeyFrame(keyFrameQueue[frame_count-1]);
                        ROS_INFO("initNewKeyFrame costs: %fms", tc.toc() );
                    }
                }

                initLocalMap();
                ROS_INFO("initLocalMap costs: %fms, deleteMP=%d LocalMP=%d", tc.toc(), mpToBeErasedNum, MapPointsInLocalMap.size()  );
                //                projectLocalMap();
                //                ROS_INFO("projectLocalMap costs: %fms", tc.toc());

                //cout << "beforeBA:\n" << mpCurrentKeyFrame->GetTransformation() << "\n";
                ROS_INFO("kF.sz=%d MP.sz=%d", mpMap->numOfKeyFrames, mpMap->mspMapPoints.size() );
                LocalBundleAdjustment();
                ROS_INFO("LocalBundleAdjustment costs: %fms", tc.toc());
                //cout << "afterBA:\n" << mpCurrentKeyFrame->GetTransformation() << "\n";

                setViewingInfo();

                //record the last
                current_T = keyFrameQueue[frame_count]->GetTransformation() ;
                latest_T = keyFrameQueue[frame_count-1]->GetTransformation() ;
                detla_T = latest_T.inverse()*current_T ;

                //marginalization_flag = false ;
                marginalize();
                ROS_INFO("marginalize costs: %fms", tc.toc());

                //                int mergeNum = projectLocalMap();
                //                if ( mergeNum == 0 ){
                //                    ROS_INFO("projectLocalMap costs: %fms mergeNum=%d", tc.toc(), mergeNum );
                //                }
                //                else{
                //                    ROS_WARN("projectLocalMap costs: %fms mergeNum=%d", tc.toc(), mergeNum );
                //                }
                if ( debug_f.is_open() ){
                    debug_f << "1 " << tc.toc() << "\n";
                }


                TicToc loop_tc ;
                loop_flag = detectLoop();
                if ( debug_f.is_open() ){
                    debug_f << "2 " << loop_tc.toc() << "\n";
                }

                if ( loop_flag )
                {
                    loop_flag = checkLoop(pnp_R, pnp_t);
                    if ( loop_flag ){
                        ROS_WARN("found loop");
                    }
                    ROS_WARN("loop cost: %fms", loop_tc.toc() ) ;
                }
                if ( loop_flag )
                {
                    //loop_tc.tic();
                    poseGraphOptimization(keyFrameQueue[frame_count-1], mpMatchedKF, pnp_R, pnp_t);
                    //ROS_WARN("poseGraphOptimization cost: %fms", loop_tc.toc() ) ;

                    //for viewing purpose only
                    if ( pSetting->bMeshing > 0 )
                    {
                        unique_lock<mutex> lock( pSystem->mpLocalMesher->mutexLoopImg ) ;
                        pSystem->mpLocalMesher->loopImg = mpMatchedKF->img.clone();
                        pSystem->mpLocalMesher->loopImgFlag = true ;
                        for( int i = 0 ; i < 1; i++ )
                        {
                            cv::pyrDown(pSystem->mpLocalMesher->loopImg,
                                        pSystem->mpLocalMesher->loopImg,
                                        cv::Size(pSystem->mpLocalMesher->loopImg.cols/2,
                                                 pSystem->mpLocalMesher->loopImg.rows/2) ) ;
                        }
                    }
                }

                setMeshingInfo();

                //ROS_INFO("setInfo costs: %fms", tc.toc());

                KeyFrameCulling();

                keyFrameQueue[frame_count]->totalKF = mpMap->numOfKeyFrames ;
                keyFrameQueue[frame_count]->totalMapPoint = mpMap->mspMapPoints.size() ;
                keyFrameQueue[frame_count]->erasedKF = mpMap->numOfEraseKF ;
                keyFrameQueue[frame_count]->erasedMP = mpMap->numOfEraseMP ;
            }
        }
        else {
            usleep(1000);
        }
        if( mbFinished ){
            break;
        }
    }

    ROS_WARN("End of local mapping") ;

    //Global Optimization result check
    while ( onGlobalOptimization && doneGlobalOptimization == false && ros::ok() ){
        usleep(1000) ;
    }
    if ( onGlobalOptimization && doneGlobalOptimization && ros::ok() )
    {
        getBackGlobalOptimizationResult();
        onGlobalOptimization = false ;
        doneGlobalOptimization = false ;

        if ( pSetting->bMeshing > 0 )
        {
            unique_lock<mutex> lock( pSystem->mpLocalMesher->mutexLoopImg ) ;
            pSystem->mpLocalMesher->loopImgFlag = false;
        }
    }


    initNewKeyFrame(keyFrameQueue[frame_count]);
    initLocalMap();
    LocalBundleAdjustment();

    setViewingInfo();

    mbStop = true ;
}


void LocalMapping::updateLocalMap(KeyFrame* pKF,
                                  vector<KeyFrame*>& vpNeighKFs,
                                  std::set<unsigned int>& pIDset )
{
    vpNeighKFs.clear();
    pIDset.clear();

    std::set<KeyFrame*> vst ;
    std::list<KeyFrame*> queue ;
    vector<KeyFrame*> KFs = pKF->GetBestCovisibilityKeyFrames(1000);
    for( KeyFrame* pKFi: pKF->mspLoopEdges ){
        queue.push_back(pKFi);
    }
    for(KeyFrame* pKFi: KFs ){
        queue.push_back(pKFi);
    }
    while( queue.empty() == false && ( vpNeighKFs.size() < 80 || pIDset.size()< 1000) )
    {
        KeyFrame* curKF = queue.front();
        queue.pop_front();
        if ( curKF->mbBad ){
            continue ;
        }
        if ( vst.find(curKF) != vst.end() ){
            continue ;
        }

        for(size_t i=0, sz = curKF->pIDs.size() ; i < sz; i++ )
        {
            if ( curKF->pIDs[i] == 0 ){
                continue ;
            }
            pIDset.insert( curKF->pIDs[i] ) ;
        }
        vpNeighKFs.push_back(curKF);
        vst.insert(curKF) ;

        for( KeyFrame* pKFi: curKF->mspLoopEdges ){
            queue.push_back(pKFi);
        }
        vector<KeyFrame*> expandKFs = curKF->GetBestCovisibilityKeyFrames(1000);
        for( int i = 0; i< expandKFs.size(); i++ ){
            queue.push_back(expandKFs[i]);
        }
    }
}

bool LocalMapping::checkParallax()
{
    if ( frame_count < 2 ){
        return true ;
    }
    std::map<unsigned int, int> pIDset ;
    std::map<unsigned int, int> pID2index_pKF_1 ;
    std::map<unsigned int, int> pID2index_pKF_2 ;
    std::map<unsigned int, int>::iterator pIter ;
    KeyFrame* pKF_1;
    KeyFrame* pKF_2;
    KeyFrame* pKF ;

    //find the set of features in frame-1 and frame-2
    pKF_1 = keyFrameQueue[frame_count-1] ;
    pKF_2 = keyFrameQueue[frame_count-2] ;

    pKF = pKF_1 ;
    for( int i = 0 ; i < pKF->N; i++ )
    {
        if ( pKF->pIDs[i] == 0 ){
            continue ;
        }
        pID2index_pKF_1[pKF->pIDs[i]] = i ;
        pIter = pIDset.find( pKF->pIDs[i] ) ;
        if ( pIter == pIDset.end() ){
            pIDset[ pKF->pIDs[i] ] = 1 ;
        }
        else {
            pIter->second++ ;
        }
    }

    pKF = pKF_2 ;
    for( int i = 0 ; i < pKF->N; i++ )
    {
        if ( pKF->pIDs[i] == 0 ){
            continue ;
        }
        pID2index_pKF_2[pKF->pIDs[i]] = i ;
        pIter = pIDset.find( pKF->pIDs[i] ) ;
        if ( pIter == pIDset.end() ){
            pIDset[ pKF->pIDs[i] ] = 1 ;
        }
        else {
            pIter->second++ ;
        }
    }

    double parallax_sum = 0 ;
    int parallax_num = 0 ;
    int cnt = 0;
    Eigen::Vector3d p1, p2, p_2_comp ;
    Eigen::Matrix3d R_1_to_2 = pKF_1->GetRotation().transpose()*pKF_2->GetRotation();
    //ROS_WARN("pIDset.sz=%d", pIDset.size() ) ;
    for( pIter = pIDset.begin(); pIter != pIDset.end(); pIter++ )
    {
        if ( pIter->second != 2 ){
            continue ;
        }
        int index_1 = pID2index_pKF_1[pIter->first] ;
        p1 << (pKF_1->mvKeysUn[index_1].x-pSetting->cx)/pSetting->fx,
                (pKF_1->mvKeysUn[index_1].y-pSetting->cy)/pSetting->fy,
                1 ;

        int index_2 = pID2index_pKF_2[pIter->first] ;
        p2 << (pKF_2->mvKeysUn[index_2].x-pSetting->cx)/pSetting->fx,
                (pKF_2->mvKeysUn[index_2].y-pSetting->cy)/pSetting->fy,
                1 ;

        p_2_comp = R_1_to_2*p2 ;
        p_2_comp = p_2_comp/p_2_comp(2) ;
        parallax_sum += std::min( (p1-p_2_comp).norm(), (p1-p2).norm() );
        parallax_num++ ;
    }

    ROS_DEBUG("parallax_sum: %lf, parallax_num: %d", parallax_sum, parallax_num);
    ROS_DEBUG("current parallax: %lf", parallax_sum*pSetting->fx / parallax_num);

    bool flag = (parallax_sum * pSetting->fx / (double)parallax_num) > (pSetting->MIN_PARALLAX) ;
    flag |= (parallax_num < 20) ;

    return flag;
}

void LocalMapping::addFrameInfo(KeyFrame* pKF)
{
    Eigen::Matrix3d cur_R = pKF->GetRotation();
    Eigen::Vector3d cur_t = pKF->GetTranslation();
    Eigen::Vector3d x3D ;
    float fx = pSetting->fx;
    float fy = pSetting->fy;
    float cx = pSetting->cx;
    float cy = pSetting->cy;
    float bf = pSetting->bf;
    for(int i=0; i < pKF->N; i++)
    {
        if ( pKF->pIDs[i] == 0 ){
            continue ;
        }
        else //create new map points
        {
            std::unordered_map<unsigned int, MapPoint*>::iterator pIter =
                    mpMap->mspMapPoints.find(pKF->pIDs[i]) ;
            if ( pIter == mpMap->mspMapPoints.end() )
            {//create new map points

                if ( pKF->r_status[i] == 0 )
                {
                    x3D.setZero();
                    MapPoint* mp = mpMap->createMapPoint( pKF->pIDs[i], x3D, false) ;
                    mpMap->addObservation(mp, pKF, i, 0) ;

                    //ROS_WARN("create new map point, %d->%d:0", mp->mnId, pKF->mnId ) ;
                }
                else
                {
                    float disparity = pKF->mvKeysUn[i].x - pKF->mvKeysRightUn[i].x ;
                    if ( disparity < 1.5 ){
                        x3D.setZero();
                        MapPoint* mp = mpMap->createMapPoint( pKF->pIDs[i], x3D, false) ;
                        mpMap->addObservation(mp, pKF, i, 0) ;
                    }
                    else
                    {
                        float d = bf/disparity ;
                        //        if ( d > pSetting->mThDepth ){
                        //            continue ;
                        //        }
                        x3D << (pKF->mvKeysUn[i].x-cx)*d/fx,
                                (pKF->mvKeysUn[i].y-cy)*d/fy,
                                d ;
                        x3D = cur_R*x3D+ cur_t ;
                        MapPoint* mp = mpMap->createMapPoint(pKF->pIDs[i], x3D, true) ;
                        mpMap->addObservation(mp, pKF, i, 1) ;

                        //ROS_WARN("create new map point, %d->%d:1", mp->mnId, pKF->mnId ) ;
                    }
                }
            }
            else
            {
                MapPoint* mp = pIter->second ;
                int flag = (pKF->r_status[i] > 0) ;
                mpMap->addObservation(mp, pKF, i, flag);
            }
        }
    }
}

int LocalMapping::projectLocalMap()
{
    if ( marginalization_flag == false ){
        return 0;
    }
    KeyFrame* pKF = keyFrameQueue[frame_count-1];
    Eigen::Matrix3d R_w_2_b = pKF->GetRotation();
    Eigen::Vector3d t_w_2_b = pKF->GetTranslation();
    R_w_2_b.transposeInPlace() ;
    t_w_2_b = -R_w_2_b*t_w_2_b ;

    std::vector<cv::Point2i> matches;
    std::set<MapPoint*> tmpLocalMap = MapPointsInLocalMap;
    for( int i = 0 ; i < pKF->N; i++ )
    {
        if ( pKF->pIDs[i] == 0 ){
            continue ;
        }
        MapPoint* mp = mpMap->getMapPoint(pKF->pIDs[i]) ;
        if ( mp->init == false ){
            continue ;
        }
        std::set<MapPoint*>::iterator pIter = tmpLocalMap.find(mp);
        if ( pIter == tmpLocalMap.end() ){
            ROS_WARN("can not find map point in projectLocalMap()") ;
        }
        else {
            tmpLocalMap.erase(pIter) ;
        }
    }
    std::vector<MapPoint*> mp_vector(tmpLocalMap.begin(), tmpLocalMap.end() );
    searchByProjection(pKF,
                       R_w_2_b, t_w_2_b, mp_vector, matches, pSetting->TH_HIGH, 1000000.0 ) ;
    //    searchByProjection(pKF,
    //            R_w_2_b, t_w_2_b, mp_vector, matches, pSetting->TH_LOW, 400.0 ) ;

    //merge two map points
    int sz = matches.size();
    int from_mpID, to_mpID ;
    int outlier = 0 ;
    for( int i = 0 ; i < sz; i++ )
    {
        to_mpID = matches[i].x ;
        from_mpID = pKF->pIDs[matches[i].y] ;
        if ( from_mpID == 0 )
        {

            MapPoint* mp = mpMap->getMapPoint(to_mpID);
            pKF->pIDs[matches[i].y] = to_mpID ;
            mpMap->addObservation(mp, pKF, matches[i].y, (pKF->r_status[matches[i].y]>0) );
        }
        else{
            outlier += (mpMap->mergeMapPoint(from_mpID, to_mpID) == 0);
        }
    }

    return sz-outlier;
    //    {
    //        unique_lock<mutex> lock(mMutexNewKFs);
    //        mpCurrentKeyFrame = mlNewKeyFrames.front();
    //        mlNewKeyFrames.pop_front();
    //    }

    //    printf("mpCurrentKeyFrame->mnId = %d\n", mpCurrentKeyFrame->mnId ) ;
    //    //cout << mpCurrentKeyFrame->GetTransformation() << "\n" ;
    //    if ( firstKeyFrame ){
    //        firstKeyFrame = false ;
    //        initKeyFrameID = mpCurrentKeyFrame->mnId;
    //        return false;
    //    }

    //    // Create MapPoints and asscoiate to KeyFrame
    //    float fx = pSetting->fx;
    //    float fy = pSetting->fy;
    //    float cx = pSetting->cx;
    //    float cy = pSetting->cy;
    //    float bf = pSetting->bf;
    //    int insertNum = 0 ;

    //    KeyFrame* pKF = mpCurrentKeyFrame ;
    //    for(int i=0; i < pKF->N; i++)
    //    {
    //        if ( pKF->pIDs[i] == 0 ){
    //            continue ;
    //        }
    //        std::unordered_map<unsigned int, unsigned int>::iterator
    //                mergeMapIter = mpMap->mergeMap.find(pKF->pIDs[i]) ;
    //        if ( mergeMapIter != mpMap->mergeMap.end() ){
    //            pKF->pIDs[i] = mergeMapIter->second ;
    //        }
    //        if ( mpMap->MapPointInUse(pKF->pIDs[i]) )
    //        {
    //            if ( pKF->r_status[i] > 0 ){//stereo
    //                mpMap->addObservation(pKF->pIDs[i], pKF, i, 1) ;
    //            }
    //            else{//mono
    //                mpMap->addObservation(pKF->pIDs[i], pKF, i, 0) ;
    //            }
    //            mpMap->updateMapPoint(pKF->pIDs[i]) ;
    //        }
    //        else //create new map points
    //        {
    //            Eigen::Vector3d x3D ;
    //            if ( pKF->r_status[i] > 0 )//stereo obseravtion
    //            {
    //                float disparity = pKF->mvKeysUn[i].x - pKF->mvKeysRightUn[i].x ;
    //                if ( disparity < 3 ){
    //                    continue ;
    //                }
    //                float d = bf/disparity ;
    //                //        if ( d > pSetting->mThDepth ){
    //                //            continue ;
    //                //        }
    //                x3D << (pKF->mvKeysUn[i].x-cx)*d/fx,
    //                        (pKF->mvKeysUn[i].y-cy)*d/fy,
    //                        d ;
    //                x3D = pKF->R_b_2_w*x3D+ pKF->t_b_2_w ;

    //                insertNum++ ;
    //                mpMap->createMapPoint(pKF->pIDs[i], x3D, true) ;
    //                mpMap->addObservation(pKF->pIDs[i], pKF, i, 1) ;
    //                mpMap->updateMapPoint(pKF->pIDs[i]) ;
    //            }
    //            else
    //            {
    //                x3D.setZero();
    //                //printf("mId=%d ", pKF->pIDs[i] ) ;
    //                mpMap->createMapPoint(pKF->pIDs[i], x3D, false) ;
    //                mpMap->addObservation(pKF->pIDs[i], pKF, i, 0) ;
    //                mpMap->updateMapPoint(pKF->pIDs[i]) ;
    //            }
    //        }
    //    }
    //    //ROS_INFO("%d points created in keyframe creation", insertNum );

    //    // Update links in the Covisibility Graph
    //    mpCurrentKeyFrame->UpdateConnections();

    //    // Project map points around into the the current keyframe
    //    //1. update local maps
    //    updateLocalMap(mpCurrentKeyFrame, KeyFramesInLocalMap, MapPointsInLocalMap) ;
    //    ROS_WARN("localKFs=%d, localMPs=%d", KeyFramesInLocalMap.size(), MapPointsInLocalMap.size() ) ;

    //    //2. remove the part of features that have been tracked
    //    std::set<unsigned int> pIDset ;
    //    pIDset = MapPointsInLocalMap ;
    //    for( int i = 0, sz = mpCurrentKeyFrame->pIDs.size() ; i < sz ; i++ )
    //    {
    //        if ( mpCurrentKeyFrame->pIDs[i] == 0 ){
    //            continue ;
    //        }
    //        std::set<unsigned int>::iterator pIter =
    //                pIDset.find( mpCurrentKeyFrame->pIDs[i] ) ;
    //        if ( pIter == pIDset.end() ){
    //            continue ;
    //        }
    //        else {
    //            pIDset.erase(pIter) ;
    //        }
    //    }

    //    TicToc tic_merge;
    //    //3. begin merge check
    //    Eigen::Vector3d p3D, p3D_c ;
    //    for( unsigned int pID: pIDset )
    //    {
    //        cv::Mat vDescriptors ;
    //        if ( mpMap->getMapPoint(pID, p3D) == false ){
    //            continue;
    //        }
    //        if ( mpMap->getDescriptor(pID, vDescriptors) == false )
    //        {
    //            ROS_WARN("can not find pID getDescriptor = %d", pID )  ;
    //            if ( mpMap->mergeMap.find(pID) != mpMap->mergeMap.end() ){
    //                ROS_WARN("in mergeMap")  ;
    //            }
    //            else {
    //                ROS_WARN("not in mergeMap")  ;
    //            }
    //            continue ;
    //        }
    //        if ( vDescriptors.cols == 0 || vDescriptors.rows == 0 ){
    //            ROS_WARN("vDescriptors error = %d", pID )  ;
    //            continue ;
    //        }

    //        p3D_c = mpCurrentKeyFrame->R_b_2_w*(p3D - mpCurrentKeyFrame->t_b_2_w) ;
    //        float x = p3D_c(0)/p3D_c(2)*pSetting->fx + pSetting->cx ;
    //        float y = p3D_c(1)/p3D_c(2)*pSetting->fy + pSetting->cy ;
    //        float x2 = (p3D_c(0)-pSetting->baseline)/p3D_c(2)*pSetting->fx + pSetting->cx ;

    //        int nGridPosX, nGridPosY;
    //        if ( mpCurrentKeyFrame->PosInGrid( cv::Point2f(x, y), nGridPosX,nGridPosY ) )
    //        {
    //            int bestID = -1 ;
    //            int bestDist = pSetting->TH_LOW ;
    //            for( int fID: mpCurrentKeyFrame->mGrid[nGridPosX][nGridPosY] )
    //            {
    //                if ( mpCurrentKeyFrame->mDescriptors.row(fID).rows != vDescriptors.rows
    //                     || mpCurrentKeyFrame->mDescriptors.row(fID).cols != vDescriptors.cols )
    //                {
    //                    ROS_WARN("row0=%d row1=%d col0=%d col1=%d\n",
    //                             mpCurrentKeyFrame->mDescriptors.row(fID).rows,
    //                             vDescriptors.rows,
    //                             mpCurrentKeyFrame->mDescriptors.row(fID).cols,
    //                             vDescriptors.cols ) ;
    //                }
    //                if ( mpCurrentKeyFrame->r_status[fID] == 0 ) //monocular observation
    //                {
    //                    float errX1 = x - mpCurrentKeyFrame->mvKeysUn[fID].x;
    //                    float errY1 = y - mpCurrentKeyFrame->mvKeysUn[fID].y;
    //                    if((errX1*errX1+errY1*errY1)>5.991)
    //                    {
    //                        ROS_WARN("[mono] dist not satisfied, %f", errX1*errX1+errY1*errY1 ) ;
    //                        continue;
    //                    }
    //                    else
    //                    {
    //                        int dist = DescriptorDistance(vDescriptors,
    //                                                      mpCurrentKeyFrame->mDescriptors.row(fID));


    //                        if( dist > bestDist){
    //                            continue;
    //                        }
    //                        else{
    //                            bestDist = dist;
    //                            bestID = fID ;
    //                        }
    //                    }
    //                }
    //                else// stereo observation
    //                {
    //                    float errX1 = x - mpCurrentKeyFrame->mvKeysUn[fID].x;
    //                    float errY1 = y - mpCurrentKeyFrame->mvKeysUn[fID].y;
    //                    float errX1_r = x2 - mpCurrentKeyFrame->mvKeysRightUn[fID].x;
    //                    if((errX1*errX1+errY1*errY1+errX1_r*errX1_r)>7.8)
    //                    {
    //                        ROS_WARN("[stereo] dist not satisfied, %f %f", errX1*errX1+errY1*errY1,
    //                                 errX1*errX1+errY1*errY1+errX1_r*errX1_r ) ;
    //                        continue;
    //                    }
    //                    else
    //                    {
    //                        int dist = DescriptorDistance(vDescriptors,
    //                                                      mpCurrentKeyFrame->mDescriptors.row(fID));
    //                        if( dist > bestDist){
    //                            continue;
    //                        }
    //                        else{
    //                            bestDist = dist;
    //                            bestID = fID ;
    //                        }
    //                    }
    //                }
    //            }
    //            if ( bestID > -1 )//merge two points
    //            {
    //                mpMap->mergeMapPoint( mpCurrentKeyFrame->pIDs[bestID], pID) ;
    //                ROS_WARN("[merge] from = %d, to = %d", mpCurrentKeyFrame->pIDs[bestID], pID ) ;
    //            }
    //        }
    //    }
    //    ROS_INFO("Merge Search costs: %fms", tic_merge.toc());
}

void LocalMapping::KeyFrameCulling()
{
    if ( frame_count < 2 ){
        return ;
    }

    KeyFrame* mpCurrentKF = EraseKFsToChecked.front() ;
    bool flag = false ;
    {
        unique_lock<mutex> lk(mMutexNotErasedKFs) ;

        if ( NotErasedKFs.find(mpCurrentKF) == NotErasedKFs.end() ){
            flag = true ;
        }
    }
    if ( flag == false ){
        return ;
    }
    if ( mpCurrentKF->mnId + pSetting->mMaxFrames > keyFrameQueue[0]->mnId ){
        return ;
    }
    if ( hashKeyFrameTable.find(mpCurrentKF) != hashKeyFrameTable.end() ){
        return ;
    }

    EraseKFsToChecked.pop_front();
    if ( mpCurrentKF->mnId == initKFrameID ){
        return ;
    }
    if ( loopKFset.find(mpCurrentKF) != loopKFset.end() ){
        return ;
    }

    int sum_use = 0 ;
    int sum_redundant = 0 ;
    for( int i = 0 ; i < mpCurrentKF->N ; i++ )
    {
        if ( mpCurrentKF->pIDs[i] == 0 ){
            continue ;
        }
        sum_use++ ;
        std::unordered_map<unsigned int, MapPoint*>::iterator
                pIter = mpMap->mspMapPoints.find( mpCurrentKF->pIDs[i]) ;
        if ( pIter == mpMap->mspMapPoints.end() ){
            ROS_WARN("[Bug] can not find map point = %d", mpCurrentKF->pIDs[i] ) ;
        }
        MapPoint* mp = pIter->second;
        if ( mp->mObservations.size() > 3 ){
            sum_redundant++ ;
        }
    }

    if ( sum_redundant > 30 && (float)sum_redundant/(float)sum_use > 0.8 )
    {
        //ROS_WARN("[Erase] KF=%d Red=%d Sum=%d", mpCurrentKF->mnId, sum_redundant,  sum_use ) ;
        mpMap->mpKeyFrameDatabase->erase(mpCurrentKF);
        mpMap->EraseKeyFrame( mpCurrentKF, true );

        mpMap->numOfEraseKF++ ;
    }
}

void LocalMapping::marginalize()
{
    if ( marginalization_flag )//insert keyframe
    {
        if ( frame_count < WINDOW_SIZE ){
            return;
        }

        KeyFrame* pKFToBeDropped = keyFrameQueue[0] ;
        hashKeyFrameTable.erase(pKFToBeDropped) ;
        for( int i = 0; i < frame_count; i++ )
        {
            KeyFrame* pKFNext = keyFrameQueue[i+1] ;
            keyFrameQueue[i] = pKFNext ;
            hashKeyFrameTable[pKFNext] = i ;
        }
        frame_count--;
    }
    else //drop the second newest frame
    {
        if ( frame_count < 2 ){
            return ;
        }
        KeyFrame* pKFCur = keyFrameQueue[frame_count] ;
        KeyFrame* pKFToBeDropped = keyFrameQueue[frame_count-1] ;
        keyFrameQueue[frame_count-1] = pKFCur ;

        hashKeyFrameTable.erase(pKFToBeDropped) ;
        hashKeyFrameTable[pKFCur] = frame_count-1 ;

        mpMap->EraseKeyFrame(pKFToBeDropped, true);
        frame_count-- ;
    }
}

void LocalMapping::setMeshingInfo()
{
    if ( pSetting->bMeshing == 0 ||
         pSystem->mpLocalMesher == NULL ||
         marginalization_flag == false ||
         frame_count < 2 ){
        return ;
    }

    KeyFrame* mpCurrentKF = keyFrameQueue[frame_count-1];

    //find the neighborhood KFs
    vector<KeyFrame*> KFs = mpCurrentKF->GetBestCovisibilityKeyFrames(10);
    std::set<KeyFrame*> vst ;
    std::list<KeyFrame*> queue ;
    for( KeyFrame* KF:  mpCurrentKF->mspLoopEdges ){
        queue.push_back(KF);
    }
    for( int i = 0 ; i < KFs.size(); i++ ){
        queue.push_back(KFs[i]);
    }
    KFs.clear();
    while( queue.empty() == false && KFs.size() < pSetting->renderingK )
    {
        KeyFrame* curKF = queue.front();
        queue.pop_front();
        if ( curKF == NULL || curKF->mbBad ){
            continue ;
        }
        if ( curKF->mnId > mpCurrentKF->mnId ){
            continue ;
        }
        if ( vst.find(curKF) != vst.end() ){
            continue ;
        }
        KFs.push_back(curKF);
        vst.insert(curKF) ;
        for( KeyFrame* KF: curKF->mspLoopEdges ){
            queue.push_back(KF);
        }
        vector<KeyFrame*> expandKFs = curKF->GetBestCovisibilityKeyFrames(10);
        for( int i = 0; i< expandKFs.size(); i++ ){
            queue.push_back(expandKFs[i]);
        }
    }

    meshingFrame curFrame;
    curFrame.pKF = mpCurrentKF;
    curFrame.neighboringKFs = KFs ;

    {
        //neighboring KFs
        unique_lock<mutex> lock(mMutexNotErasedKFs) ;
        NotErasedKFs.insert(mpCurrentKF) ;
        for( KeyFrame* pKF: curFrame.neighboringKFs ){
            NotErasedKFs.insert(pKF) ;
        }
    }

    {
        unique_lock<mutex> lock( pSystem->mpLocalMesher->mMutexKeyFrameQueue ) ;
        pSystem->mpLocalMesher->mlpKeyFrameQueue.push_back(curFrame);
    }
}

void LocalMapping::setViewingInfo()
{
    if ( pSetting->bDisplayViewer == 0 ){
        return ;
    }

    //set current pose
    {
        std::unique_lock<std::mutex> lk( pSystem->mpMapDrawer->mMutexPose ) ;
        KeyFrame* pKF = keyFrameQueue[frame_count] ;

        pSystem->mpMapDrawer->initPose = true ;
        pSystem->mpMapDrawer->cur_R = pKF->GetRotation();
        pSystem->mpMapDrawer->cur_t = pKF->GetTranslation();

        //        pSystem->mpMapDrawer->neigborhoodKF_T.clear();
        //        for( int i = 0; i < frame_count; i++ )
        //        {
        //            pSystem->mpMapDrawer->neigborhoodKF_T.push_back(
        //                        keyFrameQueue[i]->GetTransformation().transpose()) ;
        //        }

        pSystem->mpMapDrawer->NonneigborhoodKF_T.clear();
        pSystem->mpMapDrawer->NonneigborhoodKF_ids.clear();
        for ( Map::KFMAPTYPE_ITER kfIter = mpMap->mspKeyFrames.begin() ;
              kfIter != mpMap->mspKeyFrames.end() ;
              kfIter++ )
        {
            KeyFrame* kf = kfIter->second ;
            if ( kf->mnId >= pKF->mnId ){
                continue ;
            }
            //            if ( hashKeyFrameTable.find(pKF) != hashKeyFrameTable.end() ){
            //                continue ;
            //            }
            pSystem->mpMapDrawer->NonneigborhoodKF_T.push_back(
                        kf->GetTransformation().transpose()) ;
            pSystem->mpMapDrawer->NonneigborhoodKF_ids.push_back(kf->mnId) ;
        }
        //printf("NonneigborhoodKF_T.sz=%d\n", pSystem->mpMapDrawer->NonneigborhoodKF_T.size() ) ;
    }

    //set local map point
    {
        std::unique_lock<std::mutex> lk( pSystem->mpMapDrawer->mMutexLocalMap) ;
        cv::Point3f tmp ;

        //std::vector<cv::Point3f> localMapPoints;
        pSystem->mpMapDrawer->localMapPoints.clear();
        pSystem->mpMapDrawer->localMapPoints.reserve(MapPointsInLocalMap.size());
        for( std::set<MapPoint*>::iterator pIter = MapPointsInLocalMap.begin();
             pIter != MapPointsInLocalMap.end(); pIter++ )
        {
            MapPoint* mp = *pIter ;
            tmp.x = mp->pos(0) ;
            tmp.y = mp->pos(1) ;
            tmp.z = mp->pos(2) ;
            pSystem->mpMapDrawer->localMapPoints.push_back(tmp);

            //cout << mp->pos.transpose() << "\n" ;
        }

        //        pSystem->mpMapDrawer->allMapPoints.clear();
        //        pSystem->mpMapDrawer->allMapPoints.reserve(mpMap->mspMapPoints.size());
        //        for( std::unordered_map<unsigned int, MapPoint*>::iterator pIter = mpMap->mspMapPoints.begin();
        //                pIter != mpMap->mspMapPoints.end() ; pIter++ )
        //        {
        //            MapPoint* mp = pIter->second ;
        //            tmp.x = mp->pos(0) ;
        //            tmp.y = mp->pos(1) ;
        //            tmp.z = mp->pos(2) ;
        //            pSystem->mpMapDrawer->allMapPoints.push_back(tmp);
        //        }
    }
}

bool LocalMapping::Relocalization()
{
    return true;
}

void LocalMapping::StereoInitialization()
{
    int usefulMatchCnt = 0 ;
    for(int i=0, sz = curFrame.cur_pts.size(); i < sz; i++)
    {
        if ( curFrame.r_status[i] == 0 ){
            continue ;
        }
        float disparity = curFrame.cur_pts[i].x - curFrame.cur_pts_right[i].x ;
        if ( disparity < 3 ){
            continue ;
        }
        //        float d = pSetting->bf/disparity ;
        //        if ( d > pSetting->mThDepth ){
        //            continue ;
        //        }
        usefulMatchCnt++ ;
    }
    if ( usefulMatchCnt <= 50 ){
        return ;
    }
    Eigen::Matrix3d cur_R ;
    Eigen::Vector3d cur_t ;
    cur_R.setIdentity();
    cur_t.setZero();

    KeyFrame* pKFini = CreateNewKeyFrame(cur_R, cur_t);
    addFrameInfo(pKFini) ;
    mpMap->AddKeyFrame(pKFini);
    initNewKeyFrame(pKFini) ;
    cout << "New map created with " << mpMap->mspMapPoints.size() << " points" << endl;

    frame_count = 0 ;
    hashKeyFrameTable[pKFini] = 0 ;
    keyFrameQueue[frame_count] = pKFini ;

    mState = OK;
    mpMap->mpKeyFrameDatabase->add(pKFini);
    mLastLoopKFid = pKFini->mnId ;

    initKFrameID = pKFini->mnId ;

    latest_T.setIdentity() ;
    detla_T.setIdentity() ;

    for ( std::list<IMU_DATA>::iterator imuIter = imuVector.begin() ;
          imuIter != imuVector.end() ; )
    {
        if ( imuIter->t < pKFini->mTimeStamp ){
            imuIter = imuVector.erase(imuIter) ;
        }
        else {
            break ;
        }
    }
}

void LocalMapping::processNewKeyFrame()
{
    int succeedNum = 0;
    for( int i = 0; i < curFrame.num; i++ )
    {
        if ( curFrame.ids[i] == 0 ){
            continue ;
        }
        if ( mpMap->mspMapPoints.find( curFrame.ids[i] ) != mpMap->mspMapPoints.end() ){
            succeedNum++ ;
        }
    }
    bool flag = true;
    if ( succeedNum <= 10 )//tracking fail
    {
        flag = false ;
        //        std::vector<unsigned int> ids;
        //        std::vector<cv::Point2f> cur_pts, forw_pts;
        //        std::vector<uchar> status;
        //        std::vector<float> err;
        //        for( int i = frame_count-1-(!marginalization_flag) ; i >= 0 ; i-- )
        //        {
        //            KeyFrame* pKF = keyFrameQueue[i] ;
        //            cur_pts = pKF->mvKeysUn;
        //            ids = pKF->pIDs ;
        //            cv::calcOpticalFlowPyrLK(pKF->img,
        //                                     curFrame.cur_img,
        //                                     cur_pts,
        //                                     forw_pts,
        //                                     status,
        //                                     err,
        //                                     cv::Size(21, 21), 3,
        //                                     cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01),
        //                                     0, 1e-4 );
        //            for (int i = 0, sz = forw_pts.size(); i < sz; i++)
        //            {
        //                if (ids[i] == 0 || status[i] && !Tracking::inBorder(forw_pts[i])){
        //                    status[i] = 0;
        //                }
        //            }
        //            reduceVector(cur_pts, status);
        //            reduceVector(forw_pts, status);
        //            reduceVector(ids, status);

        //            if ( forw_pts.size() < 10 ){
        //                continue ;
        //            }

        //            cv::findFundamentalMat(cur_pts, forw_pts, cv::FM_RANSAC, pSetting->F_THRESHOLD, 0.99, status);
        //            reduceVector(cur_pts, status);
        //            reduceVector(forw_pts, status);
        //            reduceVector(ids, status);
        //            if ( forw_pts.size() > 30 )
        //            {
        //                curFrame.cur_pts = forw_pts ;
        //                curFrame.ids = ids ;
        //                curFrame.num = ids.size() ;

        //                cv::calcOpticalFlowPyrLK(curFrame.cur_img, curFrame.cur_img_r,
        //                                         curFrame.cur_pts, curFrame.cur_pts_right,
        //                                         curFrame.r_status, err,
        //                                         cv::Size(21, 21), 3,
        //                                         cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01),
        //                                         0, 1e-4 );

        //                vector<cv::Point2f> ll, rr;
        //                vector<int> idx;
        //                for (int i = 0, sz = curFrame.r_status.size(); i < sz; i++)
        //                {
        //                    if (curFrame.r_status[i])
        //                    {
        //                        idx.push_back(i);
        //                        ll.push_back(curFrame.cur_pts[i]) ;
        //                        rr.push_back(curFrame.cur_pts_right[i]);
        //                    }
        //                }
        //                if (ll.size() >= 8)
        //                {
        //                    cv::findFundamentalMat(ll, rr, cv::FM_RANSAC, pSetting->F_THRESHOLD, 0.99, status);
        //                    for (unsigned int i = 0; i < status.size(); i++)
        //                    {
        //                        if (status[i] == 0){
        //                            curFrame.r_status[idx[i]] = 0;
        //                        }
        //                    }
        //                }

        //                flag = true ;
        //                ROS_WARN("[relocalized]") ;
        //                break ;
        //            }
        //        }
    }
    else{
        flag = true ;
    }

    Eigen::Matrix3d cur_R;
    Eigen::Vector3d cur_t ;
    Eigen::Quaterniond q, dq ;
    q.setIdentity() ;
    double dt ;
    bool useIMU = false ;
    for ( std::list<IMU_DATA>::iterator imuIter = imuVector.begin() ;
          imuIter != imuVector.end() ; )
    {
        if ( imuIter->t < curFrame.cur_time )
        {
            std::list<IMU_DATA>::iterator imuIterNext = std::next(imuIter, 1);

            if ( imuIterNext == imuVector.end() ){
                break ;
            }
            dt = imuIterNext->t.toSec() - imuIter->t.toSec() ;
            //printf("dt=%lf ", dt ) ;

            dq.x() = (imuIter->wx + imuIterNext->wx)/2.0 *dt*0.5 ;
            dq.y() = (imuIter->wy + imuIterNext->wy)/2.0 *dt*0.5 ;
            dq.z() = (imuIter->wz + imuIterNext->wz)/2.0 *dt*0.5 ;
            dq.w() =  sqrt( 1 - SQ(dq.x()) * SQ(dq.y()) * SQ(dq.z()) ) ;
            q = (q * dq).normalized();
            imuIter = imuVector.erase(imuIter) ;

            useIMU = true ;
        }
        else {
            break ;
        }
    }


    if ( true ){//prediction
        current_T = keyFrameQueue[frame_count]->GetTransformation()*detla_T ;
        cur_R = current_T.block(0, 0, 3, 3) ;
        cur_t = current_T.block(0, 3, 3, 1) ;

        if ( useIMU ){
            cur_R = keyFrameQueue[frame_count]->GetRotation()*q.toRotationMatrix();
        }
        //cout << q.toRotationMatrix() << "\n" ;
    }
    else{
        KeyFrame* pLastKF = keyFrameQueue[frame_count] ;
        cur_R = pLastKF->GetRotation();
        cur_t = pLastKF->GetTranslation();
    }

    frame_count++ ;
    //ROS_WARN_STREAM("cur_R =\n" << cur_R << "\n cur_t = " << cur_t.transpose() ) ;

    //calculate initial guess
    //motionOnlyBA(cur_R, cur_t);
    //PnPwithRANSAC(cur_R, cur_t);

    if ( flag == true )
    {
        flag = removeOutlier() ;
        //        if ( flag ){
        //            motionOnlyBA(cur_R, cur_t);
        //        }
    }

    //create new frame
    KeyFrame* pKF = CreateNewKeyFrame(cur_R, cur_t);
    addFrameInfo(pKF) ;
    mpMap->AddKeyFrame(pKF);

    if ( flag == false ){
        pKF->isolated = true ;
        ROS_WARN("[tracking fail]") ;
    }

    hashKeyFrameTable[pKF] = frame_count ;
    keyFrameQueue[frame_count] = pKF ;
}

KeyFrame* LocalMapping::CreateNewKeyFrame(Eigen::Matrix3d cur_R, Eigen::Vector3d cur_t)
{
    KeyFrame* pKF = new KeyFrame(curFrame.frameID,
                                 cur_R,
                                 cur_t,
                                 curFrame.cur_img,
                                 curFrame.cur_img_r,
                                 true, curFrame.cur_time);
    pKF->mvKeysUn = curFrame.cur_pts;
    pKF->mvKeysRightUn = curFrame.cur_pts_right;
    pKF->r_status = curFrame.r_status ;
    pKF->pIDs = curFrame.ids ;
    pKF->N = curFrame.num;

    return pKF ;
}

void LocalMapping::initNewKeyFrame(KeyFrame* pKF)
{
    //init initDescriptors
    pKF->initDescriptors();
    pKF->AssignFeaturesToGrid();

    std::vector<cv::KeyPoint> kps ;
    cv::Mat descriptors_frame;

#ifdef OPENCV_3
    cv::Ptr<cv::ORB> orb_detector =
            cv::ORB::create(pSetting->loop_MAX_FEATURES, 1.2f, 8, 31, 0, 2, cv::ORB::HARRIS_SCORE, 31, 20 );
    orb_detector->detect(pKF->img, kps);
    orb_detector->compute(pKF->img, kps, descriptors_frame);
#else
    cv::Ptr<cv::FeatureDetector> orb_detector =
            cv::FeatureDetector::create("ORB") ;
    cv::Ptr<cv::DescriptorExtractor> descriptor =
            cv::DescriptorExtractor::create("ORB") ;
    orb_detector->detect(pKF->img, kps);
    descriptor->compute(pKF->img, kps, descriptors_frame);
#endif

    pKF->ComputeBoW(descriptors_frame);

    //    addFrameInfo(pKF) ;
    //    mpMap->AddKeyFrame(pKF);

    EraseKFsToChecked.push_back(pKF);
}

void LocalMapping::PnPwithOpenCVNoRANSAC(Eigen::Matrix3d& cur_R, Vector3d &cur_t)
{
#ifdef OPENCV_3
    Eigen::Matrix3d w_R = cur_R.transpose();
    Eigen::Vector3d w_t = -w_R*cur_t;

    cv::Mat K = (cv::Mat_<double>(3, 3) << pSetting->fx, 0, pSetting->cx,
                 0, pSetting->fy, pSetting->cy,
                 0, 0, 1);
    cv::Mat D;
    std::vector<cv::Point3d> pts_3_set;
    std::vector<cv::Point2d> pts_2_set;
    pts_3_set.reserve(curFrame.num);
    pts_2_set.reserve(curFrame.num);

    vector<int> index ;
    index.reserve(curFrame.num);
    for( int i = 0, sz = curFrame.num ; i < sz ; i++ )
    {
        unsigned int pID = curFrame.ids[i] ;
        if ( pID == 0 ){
            ROS_WARN("pID=0 in PnPwithOpenCVNoRANSAC") ;
            ROS_BREAK();
        }
        std::unordered_map<unsigned int, MapPoint*>::iterator pIter =
                mpMap->mspMapPoints.find(pID);
        if ( pIter == mpMap->mspMapPoints.end() ){
            continue ;
        }
        MapPoint* mp = pIter->second ;
        //        if ( mp->mObservations.size() < 3 ){
        //            continue ;
        //        }
        cv::Point3d pts_3(mp->pos(0), mp->pos(1), mp->pos(2));
        pts_3_set.push_back(pts_3);

        cv::Point2d pts_2(curFrame.cur_pts[i].x, curFrame.cur_pts[i].y);
        pts_2_set.push_back(pts_2);
    }

    int flags = cv::SOLVEPNP_ITERATIVE;
    cv::Mat rvec, tvec ;
    cv::Mat r;
    cv::eigen2cv(w_R, r) ;
    cv::Rodrigues(r, rvec);
    cv::eigen2cv(w_t, tvec) ;

    cv::solvePnP(pts_3_set, pts_2_set, K, D, rvec, tvec, true, flags);

    cv::Rodrigues(rvec, r);
    //cout << "r " << endl << r << endl;
    cv::cv2eigen(r, w_R);
    cv::cv2eigen(tvec, w_t);

    cur_R = w_R.transpose() ;
    cur_t = -cur_R*w_t;

#endif
}

void LocalMapping::PnPwithRANSAC(Eigen::Matrix3d& cur_R, Vector3d &cur_t)
{
#ifdef OPENCV_3
    Eigen::Matrix3d w_R = cur_R.transpose();
    Eigen::Vector3d w_t = -w_R*cur_t;

    cv::Mat K = (cv::Mat_<double>(3, 3) << pSetting->fx, 0, pSetting->cx,
                 0, pSetting->fy, pSetting->cy,
                 0, 0, 1);
    cv::Mat D;
    std::vector<cv::Point3f> pts_3_set;
    std::vector<cv::Point2f> pts_2_set;
    std::vector<uchar> useFlags ;
    pts_3_set.reserve(curFrame.num);
    pts_2_set.reserve(curFrame.num);
    useFlags.reserve(curFrame.num);
    int sumUse = 0 ;

    vector<int> index ;
    index.reserve(curFrame.num);
    for( int i = 0, sz = curFrame.num ; i < sz ; i++ )
    {
        unsigned int pID = curFrame.ids[i] ;
        if ( pID == 0 ){
            ROS_WARN("pID=0 in PnPwithRANSAC") ;
            //ROS_BREAK();
        }
        useFlags.push_back(1);
        std::unordered_map<unsigned int, MapPoint*>::iterator pIter =
                mpMap->mspMapPoints.find(pID);
        if ( pIter == mpMap->mspMapPoints.end() ){
            continue ;
        }
        MapPoint* mp = pIter->second ;
        //        if ( mp->mObservations.size() < 3 ){
        //            continue ;
        //        }

        index[sumUse] = i;

        sumUse++ ;

        cv::Point3f pts_3(mp->pos(0), mp->pos(1), mp->pos(2));
        pts_3_set.push_back(pts_3);

        cv::Point2f pts_2(curFrame.cur_pts[i].x, curFrame.cur_pts[i].y);
        pts_2_set.push_back(pts_2);
    }

    int flags = cv::SOLVEPNP_P3P;
    //cv::Mat inliers;
    vector<uchar> inliers;
    int iterationsCount= 100;
    float reprojectionError = 3.0;
    double confidence = 0.99 ;
    cv::Mat rvec, tvec ;
    cv::Mat r;
    cv::eigen2cv(w_R, r) ;
    cv::Rodrigues(r, rvec);
    cv::eigen2cv(w_t, tvec) ;

    cv::solvePnPRansac(pts_3_set, pts_2_set, K, D, rvec, tvec,
                       true, iterationsCount, reprojectionError, confidence,
                       inliers, flags);

    cv::Rodrigues(rvec, r);
    //cout << "r " << endl << r << endl;
    cv::cv2eigen(r, w_R);
    cv::cv2eigen(tvec, w_t);

    cur_R = w_R.transpose() ;
    cur_t = -cur_R*w_t;

    int inlierNum = 0 ;
    for( int i = 0, sz = inliers.size(); i < sz ; i++ )
    {
        if ( inliers[i] ){
            inlierNum++ ;
        }
        else{
            useFlags[index[i]] = 0;
        }
    }

    reduceVector(curFrame.cur_pts, useFlags);
    reduceVector(curFrame.cur_pts_right, useFlags);
    reduceVector(curFrame.ids, useFlags);
    reduceVector(curFrame.r_status, useFlags);
    curFrame.num = curFrame.ids.size();

    if ( inlierNum*2 < sumUse ){
        ROS_WARN("[PnP] total = %d, inliners = %d", sumUse, inlierNum ) ;
    }
#endif
}

bool LocalMapping::removeOutlier()
{
    std::vector<cv::Point3f> pts_3_set;
    std::vector<cv::Point2f> pts_2_set;
    std::vector<uchar> useFlags ;
    vector<int> index ;
    pts_3_set.reserve(curFrame.num);
    pts_2_set.reserve(curFrame.num);
    useFlags.reserve(curFrame.num);
    index.reserve(curFrame.num);

    KeyFrame* preKF = NULL ;
    if ( marginalization_flag || frame_count < 2 ){
        preKF = keyFrameQueue[frame_count-1] ;
    }
    else{
        preKF = keyFrameQueue[frame_count-2] ;
    }

    vector<cv::Point2f> preKF_pts;
    //    for( int ith = frame_count-1-(!marginalization_flag); ith >= 0 ; ith-- )
    //    {
    //        preKF = keyFrameQueue[ith] ;

    //printf("ith = %d id = %d\n", ith, preKF->mnId ) ;

    int sumUse = 0 ;
    index.clear();
    pts_3_set.clear();
    pts_2_set.clear();
    useFlags.clear();
    preKF_pts.clear();
    preKF_pts.reserve(preKF->N);
    for( int i = 0, sz = curFrame.num ; i < sz ; i++ )
    {
        unsigned int pID = curFrame.ids[i] ;
        if ( pID == 0 ){
            ROS_WARN("pID=0 in PnPwithRANSAC") ;
            //ROS_BREAK();
        }
        useFlags.push_back(1);
        std::unordered_map<unsigned int, MapPoint*>::iterator pIter =
                mpMap->mspMapPoints.find(pID);
        if ( pIter == mpMap->mspMapPoints.end() ){
            continue ;
        }
        MapPoint* mp = pIter->second ;
        std::map<KeyFrame*, unsigned int>::iterator bIter
                = mp->mObservations.find(preKF);
        if ( bIter == mp->mObservations.end() ){
            continue ;
        }
        index[sumUse] = i;
        sumUse++ ;

        unsigned int pKF_index = bIter->second & 0xffff ;
        preKF_pts.push_back( preKF->mvKeysUn[pKF_index] );

        cv::Point2f pts_2(curFrame.cur_pts[i].x, curFrame.cur_pts[i].y);
        pts_2_set.push_back(pts_2);
    }

    //        if ( preKF_pts.size() < curFrame.num*0.8 || preKF_pts.size() < 30 ){
    //            break ;
    //        }

    vector<uchar> inliers;
    if ( preKF_pts.size() > 10 )
    {
        cv::findFundamentalMat(preKF_pts, pts_2_set, cv::FM_RANSAC, pSetting->F_THRESHOLD, 0.99, inliers);

        int inlierNum = 0 ;
        for( int j = 0, sz = inliers.size(); j < sz ; j++ )
        {
            if ( inliers[j] ){
                inlierNum++ ;
            }
            else{
                useFlags[index[j]] = 0;
            }
        }

        reduceVector(curFrame.cur_pts, useFlags);
        reduceVector(curFrame.cur_pts_right, useFlags);
        reduceVector(curFrame.ids, useFlags);
        reduceVector(curFrame.r_status, useFlags);
        curFrame.num = curFrame.ids.size();

        if ( inlierNum*2 < sumUse || sumUse < 10 ){
            ROS_WARN("[trackedWithRANSAC] flag=%d total = %d, inliners = %d", marginalization_flag, sumUse, inlierNum ) ;

            return false ;
        }
    }
    //   }
    return true ;
}

void LocalMapping::motionOnlyBA(Eigen::Matrix3d& cur_R, Eigen::Vector3d& cur_t)
{
    const int N = curFrame.num;

    cv::Mat pnpImg ;
    if ( pSetting->SHOW_PNP ){
        cv::cvtColor(curFrame.cur_img, pnpImg, CV_GRAY2BGR) ;
    }

    ceres::Problem problemPnP ;

    double reprojectLoss = 5.991*SQ(1.0/pSetting->fx);
    ceres::LossFunction *loss_function = new ceres::HuberLoss(reprojectLoss);
    //ceres::LossFunction *loss_function = new ceres::CauchyLoss(1.0);
    ceres::Solver::Options ceres_options;
    ceres_options.max_solver_time_in_seconds = 0.006;
    ceres_options.num_threads = 1 ;
    ceres_options.max_num_iterations = 6 ;
    ceres_options.function_tolerance = 1.0e-2;
    ceres_options.minimizer_type = ceres::TRUST_REGION;
    //ceres_options.linear_solver_type = ceres::DENSE_QR;
    ceres_options.linear_solver_type = ceres::DENSE_SCHUR;
    ceres_options.trust_region_strategy_type = ceres::DOGLEG;
    //ceres_options.sparse_linear_algebra_library_type = ceres::SUITE_SPARSE;

    int SIZE_POSE = 7 ;
    Eigen::Quaterniond cur_q(cur_R) ;
    par_pose_PnP[0]= cur_t(0);
    par_pose_PnP[1]= cur_t(1);
    par_pose_PnP[2]= cur_t(2);
    par_pose_PnP[3]= cur_q.x();
    par_pose_PnP[4]= cur_q.y();
    par_pose_PnP[5]= cur_q.z();
    par_pose_PnP[6]= cur_q.w();

    int nInitialCorrespondences = 0;
    std::vector<ceres::ResidualBlockId> constraintList ;
    ceres::ResidualBlockId rid;
    double u_x, u_y, shift ;
    for(int i=0; i< N ; i++)
    {
        unsigned int pID = curFrame.ids[i] ;
        if ( pID == 0 ){
            continue ;
        }
        if ( mpMap->mspMapPoints.find(pID) == mpMap->mspMapPoints.end() ){
            continue ;
        }
        MapPoint* mp = mpMap->getMapPoint(pID) ;
        if ( mp->init == false ){
            continue ;
        }
        //        if ( mp->mObservations.size() < 3 ){
        //            continue ;
        //        }
        Eigen::Vector3d p = mp->pos ;
        //cout << "id= " << pID << " index= " << mpMapPointIndex << " " << p.transpose() << "\n" ;

        //printf("pID=%d ", pID ) ;

        if ( pSetting->SHOW_PNP ){
            cv::circle(pnpImg, curFrame.cur_pts[i], 2, cv::Scalar(0, 30, 255), 2 ) ;
        }

        if ( curFrame.r_status[i] == 0 )// Monocular observation
        {
            nInitialCorrespondences++;

            u_x = (curFrame.cur_pts[i].x-pSetting->cx)/pSetting->fx;
            u_y = (curFrame.cur_pts[i].y-pSetting->cy)/pSetting->fy;
            shift = 0 ;

            rid = problemPnP.AddResidualBlock(
                        new PnPFactor(u_x, u_y, shift, p),
                        loss_function,
                        par_pose_PnP);
            constraintList.push_back(rid);
        }
        else// Stereo observation
        {
            nInitialCorrespondences++;

            u_x = (curFrame.cur_pts[i].x-pSetting->cx)/pSetting->fx;
            u_y = (curFrame.cur_pts[i].y-pSetting->cy)/pSetting->fy;
            shift = 0 ;

            rid = problemPnP.AddResidualBlock(
                        new PnPFactor(u_x, u_y, shift, p),
                        loss_function,
                        par_pose_PnP);
            constraintList.push_back(rid);

            u_x = (curFrame.cur_pts_right[i].x-pSetting->cx)/pSetting->fx;
            u_y = (curFrame.cur_pts_right[i].y-pSetting->cy)/pSetting->fy;
            shift = -pSetting->baseline;
            rid = problemPnP.AddResidualBlock(
                        new PnPFactor(u_x, u_y, shift, p),
                        loss_function,
                        par_pose_PnP);
            constraintList.push_back(rid);
        }
    }
    ceres::Solver::Summary summary;
    ceres::Solve(ceres_options, &problemPnP, &summary);
    //std::cout << summary.BriefReport() << "\n";

    cur_t << par_pose_PnP[0],
            par_pose_PnP[1],
            par_pose_PnP[2];
    cur_q.x() = par_pose_PnP[3];
    cur_q.y() = par_pose_PnP[4];
    cur_q.z() = par_pose_PnP[5];
    cur_q.w() = par_pose_PnP[6];
    cur_R = cur_q.toRotationMatrix() ;

    if ( pSetting->SHOW_PNP ){
        cv::imshow("pnpImg", pnpImg ) ;
        cv::waitKey(5) ;
    }
}

bool LocalMapping::detectLoop()
{
    if ( marginalization_flag == false || frame_count < 2 || onGlobalOptimization ){
        return false;
    }

    KeyFrame* mpCurrentKF = keyFrameQueue[frame_count-1];

    if ( keyFrameQueue[0]->mnId <= mLastLoopKFid + pSetting->mMaxFrames ){
        mpMap->mpKeyFrameDatabase->add(mpCurrentKF);
        return false;
    }

    mpCurrentKF->UpdateConnections();
    const DBoW2::BowVector &CurrentBowVec = mpCurrentKF->mBowVec;
    float minScore = 1;
    for(std::map<KeyFrame*, int>::iterator kIter = mpCurrentKF->mConnectedKeyFrameWeights.begin();
        kIter != mpCurrentKF->mConnectedKeyFrameWeights.end();
        kIter++ )
    {
        //        if ( kIter->second < pSetting->covisiblityTh ){
        //            continue ;
        //        }
        if ( kIter->second < mpCurrentKF->N/5 ){
            continue ;
        }
        KeyFrame* pKF = kIter->first;
        const DBoW2::BowVector &BowVec = pKF->mBowVec;

        float score = mpVocabulary->score(CurrentBowVec, BowVec);

        //ROS_WARN("pKF.ID=%d , score=%f", pKF->mnId, score ) ;

        if(score < minScore){
            minScore = score;
        }
    }
    // Query the database imposing the minimum score
    vector<KeyFrame*> vpCandidateKFs =
            mpMap->mpKeyFrameDatabase->DetectLoopCandidates(mpCurrentKF, minScore);

    // If there are no loop candidates, just add new keyframe and return false
    if(vpCandidateKFs.empty())
    {
        mpMap->mpKeyFrameDatabase->add(mpCurrentKF);
        mvConsistentGroups.clear();
        return false;
    }

    // For each loop candidate check consistency with previous loop candidates
    // Each candidate expands a covisibility group (keyframes connected to the loop candidate in the covisibility graph)
    // A group is consistent with a previous group if they share at least a keyframe
    // We must detect a consistent loop in several consecutive keyframes to accept it
    mvpEnoughConsistentCandidates.clear();
    vector<ConsistentGroup> vCurrentConsistentGroups;
    vector<bool> vbConsistentGroup(mvConsistentGroups.size(),false);
    for(size_t i=0, iend=vpCandidateKFs.size(); i<iend; i++)
    {
        KeyFrame* pCandidateKF = vpCandidateKFs[i];

        set<KeyFrame*> spCandidateGroup = pCandidateKF->GetConnectedKeyFrames();
        spCandidateGroup.insert(pCandidateKF);

        bool bEnoughConsistent = false;
        bool bConsistentForSomeGroup = false;
        for(size_t iG=0, iendG=mvConsistentGroups.size(); iG<iendG; iG++)
        {
            set<KeyFrame*> sPreviousGroup = mvConsistentGroups[iG].first;

            bool bConsistent = false;
            for(set<KeyFrame*>::iterator sit=spCandidateGroup.begin(), send=spCandidateGroup.end(); sit!=send;sit++)
            {
                if(sPreviousGroup.count(*sit))
                {
                    bConsistent=true;
                    bConsistentForSomeGroup=true;
                    break;
                }
            }

            if(bConsistent)
            {
                int nPreviousConsistency = mvConsistentGroups[iG].second;
                int nCurrentConsistency = nPreviousConsistency + 1;
                if(!vbConsistentGroup[iG])
                {
                    ConsistentGroup cg = make_pair(spCandidateGroup,nCurrentConsistency);
                    vCurrentConsistentGroups.push_back(cg);
                    vbConsistentGroup[iG]=true; //this avoid to include the same group more than once
                }
                if(nCurrentConsistency>= pSetting->mnCovisibilityConsistencyTh && !bEnoughConsistent)
                {
                    mvpEnoughConsistentCandidates.push_back(pCandidateKF);
                    bEnoughConsistent=true; //this avoid to insert the same candidate more than once
                }
            }
        }

        // If the group is not consistent with any previous group insert with consistency counter set to zero
        if(!bConsistentForSomeGroup)
        {
            ConsistentGroup cg = make_pair(spCandidateGroup,0);
            vCurrentConsistentGroups.push_back(cg);
        }
    }

    // Update Covisibility Consistent Groups
    mvConsistentGroups = vCurrentConsistentGroups;

    // Add Current Keyframe to database
    mpMap->mpKeyFrameDatabase->add(mpCurrentKF);

    //    static std::vector<int> plot_id;
    //    if ( mvpEnoughConsistentCandidates.size() > 0 )
    //    {
    //        if ( plot_id.size() != 0 ){
    //            for( int& id: plot_id ){
    //                cv::destroyWindow(std::to_string(id));
    //            }
    //            plot_id.clear();
    //        }
    //        ROS_WARN("currentKF.ID = %d minScore=%f", mpCurrentKF->mnId, minScore ) ;
    //        for( size_t i = 0 ; i < mvpEnoughConsistentCandidates.size(); i++ ){
    //            ROS_WARN("pKF.ID=%d , score=%f", mvpEnoughConsistentCandidates[i]->mnId, mvpEnoughConsistentCandidates[i]->mLoopScore ) ;
    //            cv::imshow(std::to_string(mvpEnoughConsistentCandidates[i]->mnId), mvpEnoughConsistentCandidates[i]->img ) ;
    //            plot_id.push_back(mvpEnoughConsistentCandidates[i]->mnId);
    //        }
    //        cv::waitKey(10) ;
    //    }

    if(mvpEnoughConsistentCandidates.empty()){
        return false;
    }
    else{
        return true;
    }
}

int LocalMapping::searchByBow(KeyFrame* pKF,
                              KeyFrame* Frame,
                              float mfNNratio,
                              bool mbCheckOrientation,
                              std::vector<cv::Point2i> &matches)
{


    //            ROS_WARN("currentKF.ID = %d minScore=%f", mpCurrentKF->mnId, minScore ) ;
    //            for( size_t i = 0 ; i < mvpEnoughConsistentCandidates.size(); i++ ){
    //                ROS_WARN("pKF.ID=%d , score=%f", mvpEnoughConsistentCandidates[i]->mnId, mvpEnoughConsistentCandidates[i]->mLoopScore ) ;
    //                cv::imshow(std::to_string(mvpEnoughConsistentCandidates[i]->mnId), mvpEnoughConsistentCandidates[i]->img ) ;
    //                plot_id.push_back(mvpEnoughConsistentCandidates[i]->mnId);
    //            }
    //            cv::waitKey(10) ;
    bool debug = true ;
    cv::Mat pKF_img ;
    cv::cvtColor(pKF->img, pKF_img, CV_GRAY2RGB);

    matches.clear();
    const DBoW2::FeatureVector &vFeatVecKF = pKF->mFeatVec;
    const DBoW2::FeatureVector &vFeatVecFrame = Frame->mFeatVec;

    // We perform the matching over ORB that belong to the same vocabulary node (at a certain level)
    DBoW2::FeatureVector::const_iterator KFit = vFeatVecKF.begin();
    DBoW2::FeatureVector::const_iterator KFend = vFeatVecKF.end();
    DBoW2::FeatureVector::const_iterator Fit = vFeatVecFrame.begin();
    DBoW2::FeatureVector::const_iterator Fend = vFeatVecFrame.end();

    while(KFit != KFend && Fit != Fend)
    {
        if(KFit->first == Fit->first)
        {
            const vector<unsigned int>& vIndicesKF = KFit->second;
            const vector<unsigned int>& vIndicesF = Fit->second;

            for(size_t iKF=0; iKF<vIndicesKF.size(); iKF++)
            {
                const unsigned int realIdxKF = vIndicesKF[iKF];

                if ( pKF->pIDs[realIdxKF] == 0 ){
                    continue ;
                }
                const cv::Mat &dKF= pKF->mDescriptors.row(realIdxKF);

                if ( debug ){
                    cv::circle(pKF_img, pKF->mvKeysUn[realIdxKF], 2, cv::Scalar(250, 0, 0), 2 ) ;
                }

                int bestDist1=256;
                int bestIdxF =-1 ;
                int bestDist2=256;

                for(size_t iF=0; iF<vIndicesF.size(); iF++)
                {
                    const unsigned int realIdxF = vIndicesF[iF];

                    if ( Frame->pIDs[realIdxF] == 0 ){
                        continue ;
                    }

                    const cv::Mat &dF = Frame->mDescriptors.row(realIdxF);

                    const int dist = DescriptorDistance(dKF,dF);

                    if(dist<bestDist1)
                    {
                        bestDist2=bestDist1;
                        bestDist1=dist;
                        bestIdxF=realIdxF;
                    }
                    else if(dist<bestDist2)
                    {
                        bestDist2=dist;
                    }
                }

                if(bestDist1 <= pSetting->TH_HIGH)
                {
                    if(static_cast<float>(bestDist1)<mfNNratio*static_cast<float>(bestDist2))
                    {
                        cv::Point2i tmp ;
                        tmp.x = pKF->pIDs[realIdxKF] ;
                        tmp.y = bestIdxF ;
                        matches.push_back(tmp);

                        if ( debug ){
                            cv::circle(pKF_img, pKF->mvKeysUn[realIdxKF], 2, cv::Scalar(0, 255, 0), 2 ) ;
                        }
                    }
                }

            }

            KFit++;
            Fit++;
        }
        else if(KFit->first < Fit->first)
        {
            KFit = vFeatVecKF.lower_bound(Fit->first);
        }
        else
        {
            Fit = vFeatVecFrame.lower_bound(KFit->first);
        }
    }

    if ( debug ){
        cv::imshow(std::to_string(pKF->mnId), pKF_img ) ;
        cv::waitKey(10) ;
        plot_id.push_back(pKF->mnId);
    }

    return (int)matches.size();
}

int LocalMapping::p3Dto2DRansac(KeyFrame* Frame,
                                std::vector<cv::Point2i> &matches,
                                Eigen::Matrix3d& R_pnp,
                                Eigen::Vector3d& t_pnp,
                                bool initial_guess)
{
#ifdef OPENCV_3
    cv::Mat K = (cv::Mat_<double>(3, 3) << pSetting->fx, 0, pSetting->cx,
                 0, pSetting->fy, pSetting->cy,
                 0, 0, 1);
    cv::Mat D;
    std::vector<cv::Point3f> pts_3_set;
    std::vector<cv::Point2f> pts_2_set;
    cv::Mat rvec, tvec ;

    if ( initial_guess ){
        cv::Mat tmp_r ;
        cv::eigen2cv(R_pnp, tmp_r);
        cv::Rodrigues(tmp_r, rvec);
        cv::eigen2cv(t_pnp, tvec);

        cout << "before RANSAC\n" ;
        cout << "R_pnp=\n" << R_pnp << "\n" ;
        cout << "t_pnp=\n" << t_pnp.transpose() << "\n";
    }

    int flags = cv::SOLVEPNP_ITERATIVE;
    //cv::Mat inliers;
    vector<uchar> inliers;
    int iterationsCount= 300;
    float reprojectionError = 8.0 ;
    double confidence = 0.99 ;

    for( cv::Point2i& match:matches )
    {
        int pID = match.x;
        int F_index = match.y;
        if ( pID == 0 || Frame->pIDs[F_index] == 0 ){
            continue ;
        }
        MapPoint* mp = mpMap->getMapPoint(pID) ;
        if ( mp->init == false || mp->mObservations.size() < 2 ){
            continue ;
        }
        cv::Point3f pts_3(mp->pos(0), mp->pos(1), mp->pos(2) );
        pts_3_set.push_back(pts_3);

        cv::Point2f pts_2( Frame->mvKeysUn[F_index].x, Frame->mvKeysUn[F_index].y ) ;
        pts_2_set.push_back(pts_2);
    }
    cv::solvePnPRansac(pts_3_set, pts_2_set, K, D, rvec, tvec,
                       initial_guess, iterationsCount, reprojectionError, confidence,
                       inliers, flags);
    cv::Mat r;
    cv::Rodrigues(rvec, r);
    //cout << "r " << endl << r << endl;
    cv::cv2eigen(r, R_pnp);
    cv::cv2eigen(tvec, t_pnp);

    int num = 0 ;
    for( int i = 0 ; i < inliers.size(); i++ )
    {
        if ( inliers[i] > 0 ){
            num++ ;
        }
    }

    return num ;
#endif
}

bool LocalMapping::checkLoop(Eigen::Matrix3d& pnp_R, Eigen::Vector3d& pnp_t)
{
    KeyFrame* mpCurrentKF = keyFrameQueue[frame_count-1];
    const int nInitialCandidates = mvpEnoughConsistentCandidates.size();

    std::vector<cv::Point2i> matches;

    vector<vector<MapPoint*> > vvpMapPointMatches;
    vvpMapPointMatches.resize(nInitialCandidates);

    vector<bool> vbDiscarded;
    vbDiscarded.resize(nInitialCandidates);

    //int nCandidates=0; //candidates with enough matches

    bool bMatch = false ;
    if ( plot_id.size() != 0 ){
        for( int& id: plot_id ){
            cv::destroyWindow(std::to_string(id));
        }
        plot_id.clear();
    }
    mpMatchedKF = NULL;
    for(int k=0; k<nInitialCandidates; k++)
    {
        KeyFrame* pKF = mvpEnoughConsistentCandidates[k];
        if ( pKF->GetBestCovisibilityKeyFrames(10).size() < 1 ){
            //do not link KFs without neighbhoods
            continue ;
        }

        vector<uchar> status;
        vector<float> err;
        vector<cv::Point2f> forw_pts;
        vector<cv::Point2f> cur_pts;
        vector<unsigned int> ids ;
        for ( int i = 0 ; i < pKF->N; i++ )
        {
            if ( pKF->pIDs[i] == 0 ){
                continue ;
            }
            MapPoint* mp = mpMap->getMapPoint(pKF->pIDs[i]) ;
            if ( mp->init == false ){
                continue ;
            }
            cur_pts.push_back(pKF->mvKeysUn[i]);
            ids.push_back(pKF->pIDs[i]);
        }
        if ( ids.size() < 60 ){
            continue ;
        }
        cv::calcOpticalFlowPyrLK(pKF->img,
                                 mpCurrentKF->img,
                                 cur_pts,
                                 forw_pts,
                                 status,
                                 err,
                                 cv::Size(21, 21),
                                 5);
        int nmatches = 0 ;
        if (forw_pts.size() >= 10)
        {
            int sumOfUse = 0;
            int sumOfMatch = 0 ;
            matches.clear();

            cv::Mat K = (cv::Mat_<double>(3, 3) << pSetting->fx, 0, pSetting->cx,
                         0, pSetting->fy, pSetting->cy,
                         0, 0, 1);
            cv::Mat D;
            std::vector<cv::Point3f> pts_3_set;
            std::vector<cv::Point2f> pts_2_set;
            cv::Mat rvec, tvec ;

            Eigen::Matrix3d cur_R = pKF->GetRotation();
            Eigen::Matrix3d cur_R_T = cur_R.transpose();
            Eigen::Vector3d cur_t = pKF->GetTranslation();
            for( int i = 0, sz = status.size() ; i < sz ; i++ )
            {
                if ( status[i] == 0 ){
                    continue ;
                }
                sumOfUse++ ;
                MapPoint* mp = mpMap->getMapPoint(ids[i]) ;
                Eigen::Vector3d p = cur_R_T*(mp->pos-cur_t);

                cv::Point3f pts_3(p(0), p(1), p(2));
                pts_3_set.push_back(pts_3);

                cv::Point2f pts_2( forw_pts[i].x, forw_pts[i].y ) ;
                pts_2_set.push_back(pts_2);
            }
            vector<uchar> inliers;
            int iterationsCount= 100;
            float reprojectionError = 3.0;
            double confidence = 0.99 ;

            if ( pts_3_set.size() < 10 ){
                continue ;
            }

#ifdef OPENCV_3
            int flags = cv::SOLVEPNP_P3P;
            cv::solvePnPRansac(pts_3_set, pts_2_set, K, D, rvec, tvec,
                               false, iterationsCount, reprojectionError, confidence,
                               inliers, flags);
#else
            int flags = CV_P3P;
            cv::solvePnPRansac(pts_3_set, pts_2_set, K, D, rvec, tvec,
                               false, iterationsCount, reprojectionError, pts_3_set.size()*confidence,
                               inliers, flags);
#endif
            cv::Mat r;
            cv::Rodrigues(rvec, r);
            //cout << "r " << endl << r << endl;
            cv::cv2eigen(r, pnp_R);
            cv::cv2eigen(tvec, pnp_t);

            int num = 0 ;
            for( int i = 0 ; i < inliers.size(); i++ )
            {
                if ( inliers[i] > 0 ){
                    num++ ;
                }
            }

            //            cout << "after RANSAC, inliner = " << num << " \n" ;
            //            cout << "R_pnp=\n" << pnp_R << "\n" ;
            //            cout << "t_pnp=\n" << pnp_t.transpose() << "\n";

            sumOfMatch = num ;

            //            vector<uchar> status;
            //            cv::findFundamentalMat(cur_pts,
            //                                   forw_pts,
            //                                   cv::FM_RANSAC,
            //                                   pSetting->F_THRESHOLD,
            //                                   0.99,
            //                                   status);


            //            for( size_t i = 0, sz = status.size() ; i < sz; i++ )
            //            {
            //                if ( status[i] > 0  ){
            //                    sumOfMatch++;
            //                }
            //            }

            if ( sumOfMatch*2 > sumOfUse && sumOfMatch > 60 )
            {
                ROS_WARN("pKFId=%d sumOfMatch=%d sumOfUse=%d sumN=%d", pKF->mnId, sumOfMatch, sumOfUse, pKF->N ) ;
                //                cv::imshow(std::to_string(pKF->mnId), pKF->img ) ;
                //                cv::waitKey(10) ;
                //                plot_id.push_back(pKF->mnId);

                //                cout << "R_pnp=\n" << pnp_R << "\n" ;
                //                cout << "t_pnp=\n" << pnp_t.transpose() << "\n";

                bMatch = true;
                mpMatchedKF = pKF;

                //                cv::Mat K = (cv::Mat_<double>(3, 3) << pSetting->fx, 0, pSetting->cx,
                //                             0, pSetting->fy, pSetting->cy,
                //                             0, 0, 1);
                //                cv::Mat D;
                //                std::vector<cv::Point3f> pts_3_set;
                //                std::vector<cv::Point2f> pts_2_set;
                //                cv::Mat rvec, tvec ;

                //                Eigen::Matrix3d cur_R = pKF->GetRotation();
                //                Eigen::Matrix3d cur_R_T = cur_R.transpose();
                //                Eigen::Vector3d cur_t = pKF->GetTranslation();
                //                for( int i = 0, sz = status.size() ; i < sz ; i++ )
                //                {
                //                    MapPoint* mp = mpMap->getMapPoint(ids[i]) ;
                //                    Eigen::Vector3d p = cur_R_T*(mp->pos-cur_t);

                //                    cv::Point3f pts_3(p(0), p(1), p(2));
                //                    pts_3_set.push_back(pts_3);

                //                    cv::Point2f pts_2( forw_pts[i].x, forw_pts[i].y ) ;
                //                    pts_2_set.push_back(pts_2);
                //                }

                //                int flags = cv::SOLVEPNP_ITERATIVE;
                //                //cv::Mat inliers;
                //                vector<uchar> inliers;
                //                int iterationsCount= 100;
                //                float reprojectionError = 3.0 ;
                //                double confidence = 0.99 ;

                //                cv::solvePnPRansac(pts_3_set, pts_2_set, K, D, rvec, tvec,
                //                                   false, iterationsCount, reprojectionError, confidence,
                //                                   inliers, flags);
                //                cv::Mat r;
                //                cv::Rodrigues(rvec, r);
                //                //cout << "r " << endl << r << endl;
                //                cv::cv2eigen(r, pnp_R);
                //                cv::cv2eigen(tvec, pnp_t);

                //                int num = 0 ;
                //                for( int i = 0 ; i < inliers.size(); i++ )
                //                {
                //                    if ( inliers[i] > 0 ){
                //                        num++ ;
                //                    }
                //                }

                //                cout << "after RANSAC, inliner = " << num << " \n" ;
                //                cout << "R_pnp=\n" << pnp_R << "\n" ;
                //                cout << "t_pnp=\n" << pnp_t.transpose() << "\n";

                break;
            }
        }
        //        nmatches = searchByBow(pKF, mpCurrentKF, 0.75, false, matches);

        //        if(nmatches<20)
        //        {
        //            vbDiscarded[i] = true;
        //            continue;
        //        }

        //        //nmatches = 20;
        //        nmatches = p3Dto2DRansac(mpCurrentKF, matches, pnp_R, pnp_t, false) ;
        //        if(nmatches<20)
        //        {
        //            vbDiscarded[i] = true;
        //            continue;
        //        }

        //        mvpLoopMapPoints.clear();
        //        set<KeyFrame*> vpLoopConnectedKFs = pKF->GetConnectedKeyFrames();
        //        vpLoopConnectedKFs.insert(pKF);

        //        for(set<KeyFrame*>::iterator vit=vpLoopConnectedKFs.begin();
        //            vit!=vpLoopConnectedKFs.end();
        //            vit++)
        //        {
        //            KeyFrame* pKF = *vit;
        //            for( int i = 0 ; i < pKF->N; i++ )
        //            {
        //                if ( pKF->pIDs[i] == 0 ){
        //                    continue ;
        //                }
        //                MapPoint* mp = mpMap->getMapPoint( pKF->pIDs[i] ) ;
        //                mvpLoopMapPoints.insert(mp) ;
        //            }
        //        }

        //        std::vector<MapPoint*> loopMPs(mvpLoopMapPoints.begin(), mvpLoopMapPoints.end() ) ;
        //        searchByProjection(mpCurrentKF, pnp_R, pnp_t, loopMPs, matches, pSetting->TH_HIGH, 100000000 ) ;
        //        //nmatches = 0 ;
        //        nmatches = p3Dto2DRansac(mpCurrentKF, matches, pnp_R, pnp_t, true) ;

        //        if(nmatches < 40)
        //        {
        //            vbDiscarded[i] = true;
        //            continue;
        //        }
        //        else
        //        {
        //            bMatch = true;
        //            mpMatchedKF = pKF;
        //            break;
        //        }
    }

    if ( bMatch ){
        mLastLoopKFid = mpCurrentKF->mnId;
    }

    return bMatch;
}

void LocalMapping::getBackGlobalOptimizationResult()
{
    //get back the pose result
    int ith = 0;
    int numOfKF = mpMap->mspKeyFrames.size();
    vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d>> R_new ;
    vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> t_new ;
    vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d>> R_old_2_new ;
    vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> t_old_2_new ;
    R_new.resize(numOfKF);
    t_new.resize(numOfKF);
    R_old_2_new.resize(numOfKF);
    t_old_2_new.resize(numOfKF);
    std::unordered_map<KeyFrame*, int>::iterator kfIter ;
    for( Map::KFMAPTYPE_ITER KFIter = mpMap->mspKeyFrames.begin();
         KFIter != mpMap->mspKeyFrames.end() ;
         KFIter++ )
    {
        if ( KFIter->first > fix_KF->mnId ){
            continue ;
        }
        KeyFrame* pKF = KFIter->second ;
        Eigen::Vector3d old_t = pKF->GetTranslation();
        Eigen::Matrix3d old_R = pKF->GetRotation();

        Eigen::Vector3d cur_t ;
        Eigen::Matrix3d cur_R ;

        kfIter = KF_index.find(pKF) ;
        if ( kfIter == KF_index.end() ){
            ROS_WARN("[bugs] in KF_index = %d", pKF->mnId ) ;
        }
        ith = kfIter->second ;

        cur_t << para_loop_Pose[ith][0], para_loop_Pose[ith][1], para_loop_Pose[ith][2];

        Eigen::Quaterniond cur_q ;
        cur_q.x() = para_loop_Pose[ith][3];
        cur_q.y() = para_loop_Pose[ith][4];
        cur_q.z() = para_loop_Pose[ith][5];
        cur_q.w() = para_loop_Pose[ith][6];
        cur_R = cur_q.normalized().toRotationMatrix();

        R_new[ith] = cur_R ;
        t_new[ith] = cur_t ;
        R_old_2_new[ith] = cur_R*old_R.transpose() ;
        t_old_2_new[ith] = -cur_R*old_R.transpose()*old_t+cur_t ;

        //ith++;
    }

    //update the features
    for( std::unordered_map<unsigned int, MapPoint*>::iterator pIter = mpMap->mspMapPoints.begin();
         pIter != mpMap->mspMapPoints.end() ; pIter++ )
    {
        MapPoint* mp = pIter->second ;
        if ( mp->mObservations.size() < 1 ){
            ROS_WARN("can not update map point = %d", pIter->first ) ;
            ROS_WARN(" mp->numOfStereoObservations = %d", mp->numOfStereoObservations ) ;
            ROS_WARN(" mp->numOfMonoObservations = %d", mp->numOfMonoObservations ) ;
            ROS_WARN(" mp->mObservations.size()  = %d", mp->mObservations.size() ) ;
        }
        KeyFrame* pKF = mp->mObservations.begin()->first;
        if ( pKF->mnId > fix_KF->mnId ){
            continue ;
        }
        if ( KF_index.find(pKF) == KF_index.end() ){
            ROS_WARN("bug in KF_index = %d", pKF->mnId ) ;
        }
        int index = KF_index[pKF] ;
        mp->pos = R_old_2_new[index]*mp->pos + t_old_2_new[index] ;
    }

    //set new pose
    //ith = 0;
    for( Map::KFMAPTYPE_ITER KFIter = mpMap->mspKeyFrames.begin();
         KFIter != mpMap->mspKeyFrames.end() ;
         KFIter++ )
    {
        if ( KFIter->first > fix_KF->mnId ){
            continue ;
        }
        KeyFrame* pKF = KFIter->second ;

        kfIter = KF_index.find(pKF) ;
        if ( kfIter == KF_index.end() ){
            ROS_WARN("[bugs] in KF_index = %d", pKF->mnId ) ;
        }
        ith = kfIter->second ;

        pKF->SetPose(R_new[ith], t_new[ith]);

        //ith++;
    }

    fix_KF->mspLoopEdges.insert(mpMatchedKF) ;
    mpMatchedKF->mspLoopEdges.insert(fix_KF) ;
}

void LocalMapping::GlobalOptimization_Loop()
{    
    vector<ceres::ResidualBlockId> residual_set;
    ceres::Solver::Summary summary;

    //double reprojectLoss = 5.991*SQ(1.0/pSetting->fx);
    //ceres::LossFunction *loss_function = new ceres::HuberLoss(reprojectLoss);
    //ceres::LossFunction *loss_function = new ceres::CauchyLoss(1.0);
    ceres::Solver::Options ceres_options;
    //ceres_options.max_solver_time_in_seconds = 0.003;
    ceres_options.num_threads = 8 ;
    //ceres_options.max_solver_time_in_seconds = 1.0;
    ceres_options.max_num_iterations = 8 ;
    ceres_options.function_tolerance = 1.0e-6;
    ceres_options.minimizer_type = ceres::TRUST_REGION;
    //ceres_options.linear_solver_type = ceres::DENSE_QR;
    ceres_options.linear_solver_type = ceres::SPARSE_SCHUR;
    ceres_options.trust_region_strategy_type = ceres::DOGLEG;
    //ceres_options.sparse_linear_algebra_library_type = ceres::SUITE_SPARSE;

    for( ; ros::ok() && !mbFinished ; )
    {
        std::unique_lock<std::mutex> lk(mutexGlobalOptimization);
        cvGlobalOptimization.wait(lk, [&]{
            return (onGlobalOptimization&&!doneGlobalOptimization) ||mbFinished ;});
        lk.unlock();

        if ( ros::ok() == false || mbFinished ){
            break ;
        }

        ceres::Solve(ceres_options, problemPoseGraph, &summary);
        ROS_WARN("[GLOBAL_OPT] Iterations:%d Time:%lf",
                 static_cast<int>(summary.iterations.size()),
                 summary.total_time_in_seconds*1000 );

        //release the fix block
        problemPoseGraph->SetParameterBlockVariable(para_loop_Pose[fix_KF_index]);

        //release the constraints
        problemPoseGraph->GetResidualBlocks(&residual_set);
        for (auto it : residual_set){
            problemPoseGraph->RemoveResidualBlock(it);
        }

        //lk.lock();
        doneGlobalOptimization = true ;
        //lk.unlock();
    }
    //puts("[---End of GlobalOptimization_Loop---]") ;
}

int LocalMapping::searchByProjection(KeyFrame* pKF,
                                     Eigen::Matrix3d& R_pnp,
                                     Eigen::Vector3d& t_pnp,
                                     std::vector<MapPoint*>& mp_Vector,
                                     std::vector<cv::Point2i> &matches,
                                     int descriptorTh,
                                     double distTh)
{
    matches.clear();
    Vector3d p3D_c ;

    //    mp_Vector.clear();
    //    for( int i = 0 ; i < pKF->N; i++ )
    //    {
    //        if ( pKF->pIDs[i] == 0 ){
    //            continue ;
    //        }
    //        mp_Vector.push_back( mpMap->mspMapPoints[pKF->pIDs[i]] );
    //    }
    //    R_pnp = pKF->GetRotation() ;
    //    t_pnp = pKF->GetTranslation() ;

    //    R_pnp.transposeInPlace() ;
    //    t_pnp = -R_pnp*t_pnp;
    for( MapPoint* mp:mp_Vector )
    {
        if ( mp->init == false ){
            continue ;
        }
        p3D_c = R_pnp*mp->pos + t_pnp ;

        float x = p3D_c(0)/p3D_c(2)*pSetting->fx + pSetting->cx ;
        float y = p3D_c(1)/p3D_c(2)*pSetting->fy + pSetting->cy ;
        //float x2 = (p3D_c(0)-pSetting->baseline)/p3D_c(2)*pSetting->fx + pSetting->cx ;
        int nGridPosX, nGridPosY;
        if ( pKF->PosInGrid( cv::Point2f(x, y), nGridPosX,nGridPosY ) )
        {
            int bestID = -1 ;
            int bestDist = pSetting->TH_HIGH ;
            for( int fID: pKF->mGrid[nGridPosX][nGridPosY] )
            {
                if ( pKF->mDescriptors.row(fID).rows != mp->mDescriptor.rows
                     || pKF->mDescriptors.row(fID).cols != mp->mDescriptor.cols )
                {
                    ROS_WARN("row0=%d row1=%d col0=%d col1=%d\n",
                             pKF->mDescriptors.row(fID).rows,
                             mp->mDescriptor.rows,
                             pKF->mDescriptors.row(fID).cols,
                             mp->mDescriptor.cols ) ;
                }
                int dist = DescriptorDistance(mp->mDescriptor,
                                              pKF->mDescriptors.row(fID));
                if ( dist > descriptorTh ){
                    continue ;
                }
                float errX1 = x - pKF->mvKeysUn[fID].x;
                float errY1 = y - pKF->mvKeysUn[fID].y;
                //if((errX1*errX1+errY1*errY1)>5.991)
                if((errX1*errX1+errY1*errY1)>distTh)
                {
                    //                  ROS_WARN("[mono] dist not satisfied, %f", errX1*errX1+errY1*errY1 ) ;
                    continue;
                }
                else
                {
                    if( dist > bestDist){
                        continue;
                    }
                    else{
                        bestDist = dist;
                        bestID = fID ;
                    }
                }
            }
            if ( bestID > -1 && mp->mnId < pKF->pIDs[bestID] )//merge two points
            {
                cv::Point2i tmp ;
                tmp.x = mp->mnId ;
                tmp.y = bestID ;

                matches.push_back(tmp);
            }
        }
    }
    return (int)matches.size();
}

void LocalMapping::poseGraphOptimization( KeyFrame* currentKF,
                                          KeyFrame* current_loopKF,
                                          const Eigen::Matrix3d& R_l_2_c,
                                          const Eigen::Vector3d& t_l_2_c)
{
    if ( mpMap->mspKeyFrames.size() >= MAX_KF ){
        return ;
    }

    loopKFset.insert(currentKF) ;
    loopKFset.insert(current_loopKF) ;

    //remove isolated KFs
    std::list<KeyFrame*> KF_to_erase ;
    for( Map::KFMAPTYPE_ITER KFIter = mpMap->mspKeyFrames.begin();
         KFIter != mpMap->mspKeyFrames.end() ;
         KFIter++ )
    {
        if ( KFIter->first > currentKF->mnId ){
            continue ;
        }
        KeyFrame* pKF = KFIter->second;
        if ( pKF->isolated || pKF->GetConnectedKeyFrames().size() < 1 ){
            KF_to_erase.push_back(pKF);
        }
    }
    for( KeyFrame* pKF: KF_to_erase )
    {
        mpMap->mpKeyFrameDatabase->erase(pKF);
        mpMap->EraseKeyFrame(pKF, true);
    }

    //init KF index, pose
    KF_index.clear();
    int numOfKF = 0;
    KeyFrame* KF_init = NULL ;
    for( Map::KFMAPTYPE_ITER KFIter = mpMap->mspKeyFrames.begin();
         KFIter != mpMap->mspKeyFrames.end() ;
         KFIter++ )
    {
        if ( KFIter->first > currentKF->mnId ){
            continue ;
        }
        KeyFrame* pKF = KFIter->second ;

        if ( NULL == KF_init ){
            KF_init = pKF;
        }

        //KF index
        KF_index[pKF] = numOfKF;

        //KF initial pose
        Eigen::Vector3d cur_t = pKF->GetTranslation();
        Eigen::Matrix3d cur_R = pKF->GetRotation();
        para_loop_Pose[numOfKF][0] = cur_t.x();
        para_loop_Pose[numOfKF][1] = cur_t.y();
        para_loop_Pose[numOfKF][2] = cur_t.z();
        Quaterniond q{cur_R};
        para_loop_Pose[numOfKF][3] = q.x();
        para_loop_Pose[numOfKF][4] = q.y();
        para_loop_Pose[numOfKF][5] = q.z();
        para_loop_Pose[numOfKF][6] = q.w();

        if ( numOfKF > totalInitKFinPoseGraph ){
            totalInitKFinPoseGraph = numOfKF ;
            problemPoseGraph->AddParameterBlock(para_loop_Pose[numOfKF],
                                                SIZE_POSE,
                                                new PoseLocalParameterization() );
        }
        numOfKF++;
    }

    //fix the origin
    if ( KF_index.find(currentKF) == KF_index.end() ){
        ROS_WARN("can not find currentKF in poseGraphOptimization") ;
        return;
    }
    fix_KF = currentKF ;
    fix_KF_index = KF_index[currentKF] ;
    //ROS_WARN("fix_KF_index=%d", fix_KF_index ) ;
    problemPoseGraph->SetParameterBlockConstant(para_loop_Pose[fix_KF_index]);

    //setup constraints
    for( Map::KFMAPTYPE_ITER KFIter = mpMap->mspKeyFrames.begin();
         KFIter != mpMap->mspKeyFrames.end() ;
         KFIter++ )
    {
        if ( KFIter->first > currentKF->mnId ){
            continue ;
        }
        KeyFrame* pKF = KFIter->second ;
        Eigen::Matrix3d pKF_R = pKF->GetRotation() ;
        Eigen::Vector3d pKF_t = pKF->GetTranslation() ;
        int pKF_index = KF_index[pKF] ;
        vector<KeyFrame*>neighKFs = pKF->GetBestCovisibilityKeyFrames(10);

        //loop edge
        for(KeyFrame* loopKF: pKF->mspLoopEdges )
        {
            if ( loopKF->mnId > pKF->mnId ){//loopKF->mnId < pKF->mnId
                continue ;
            }
            if ( KF_index.find(loopKF) == KF_index.end() ){
                ROS_WARN("can not find loopKF in poseGraphOptimization") ;
            }
            int loop_KF_index = KF_index[loopKF] ;
            Eigen::Matrix3d loop_KF_R = loopKF->GetRotation() ;
            Eigen::Vector3d loop_KF_t = loopKF->GetTranslation() ;

            Eigen::Matrix3d R_loopKF_2_pKF = pKF_R.transpose()*loop_KF_R ;
            Eigen::Quaterniond q_loopKF_2_pKF(R_loopKF_2_pKF) ;
            Eigen::Vector3d t_loopKF_2_pKF = pKF_R.transpose()*(loop_KF_t - pKF_t) ;

            problemPoseGraph->AddResidualBlock( new relativePoseFactor(q_loopKF_2_pKF, t_loopKF_2_pKF),
                                                NULL,
                                                para_loop_Pose[loop_KF_index],
                                                para_loop_Pose[pKF_index]) ;
        }

        //covibility edge
        int nmaxWeigth=0;
        KeyFrame* nKFmax=NULL;
        pKF->UpdateConnections();
        for(std::map<KeyFrame*, int>::iterator nIter = pKF->mConnectedKeyFrameWeights.begin();
            nIter != pKF->mConnectedKeyFrameWeights.end(); nIter++ )
        {
            KeyFrame* nKF = nIter->first ;
            int weight = nIter->second ;
            if ( nKF->mnId > pKF->mnId ){//nKF->mnId < pKF->mnId, order constraint
                continue ;
            }
            if ( weight > nmaxWeigth){
                nmaxWeigth = weight ;
                nKFmax = nKF;
            }
            if ( weight < pSetting->covisiblityTh ){//covisibility threshold
                continue ;
            }
            if ( KF_index.find(nKF) == KF_index.end() ){
                ROS_WARN("can not find nKF in poseGraphOptimization") ;
            }
            //printf("[nKF=%d pKF=%d] ", nKF->mnId, pKF->mnId ) ;
            int nKF_index = KF_index[nKF] ;
            Eigen::Matrix3d nKF_R = nKF->GetRotation() ;
            Eigen::Vector3d nKF_t = nKF->GetTranslation() ;

            Eigen::Matrix3d R_nKF_2_pKF = pKF_R.transpose()*nKF_R ;
            Eigen::Quaterniond q_nKF_2_pKF(R_nKF_2_pKF) ;
            Eigen::Vector3d t_nKF_2_pKF = pKF_R.transpose()*(nKF_t - pKF_t) ;

            problemPoseGraph->AddResidualBlock( new relativePoseFactor(q_nKF_2_pKF, t_nKF_2_pKF),
                                                NULL,
                                                para_loop_Pose[nKF_index],
                                                para_loop_Pose[pKF_index]) ;
        }
        if ( nmaxWeigth < pSetting->covisiblityTh )
        {

            if ( nmaxWeigth == 0 )
            {
                if ( pKF != KF_init ){
                    ROS_WARN("can not find treeEdge in poseGraphOptimization = %d", pKF->mnId) ;
                }
            }
            else{
                //printf("[nKFmax=%d pKF=%d] ", nKFmax->mnId, pKF->mnId ) ;
                KeyFrame* nKF = nKFmax ;
                if ( KF_index.find(nKF) == KF_index.end() ){
                    ROS_WARN("can not find nKFmax in poseGraphOptimization") ;
                }
                int nKF_index = KF_index[nKF] ;
                Eigen::Matrix3d nKF_R = nKF->GetRotation() ;
                Eigen::Vector3d nKF_t = nKF->GetTranslation() ;

                Eigen::Matrix3d R_nKF_2_pKF = pKF_R.transpose()*nKF_R ;
                Eigen::Quaterniond q_nKF_2_pKF(R_nKF_2_pKF) ;
                Eigen::Vector3d t_nKF_2_pKF = pKF_R.transpose()*(nKF_t - pKF_t) ;

                problemPoseGraph->AddResidualBlock( new relativePoseFactor(q_nKF_2_pKF, t_nKF_2_pKF),
                                                    NULL,
                                                    para_loop_Pose[nKF_index],
                                                    para_loop_Pose[pKF_index]) ;
            }
        }
    }

    //add the loop edge detected just now
    {
        KeyFrame* pKF = currentKF ;
        Eigen::Matrix3d pKF_R = pKF->GetRotation() ;
        Eigen::Vector3d pKF_t = pKF->GetTranslation() ;
        if ( KF_index.find(current_loopKF) == KF_index.end() ){
            ROS_WARN("can not find current_loopKF in poseGraphOptimization") ;
        }
        int loop_KF_index = KF_index[current_loopKF] ;


        Eigen::Matrix3d R_loopKF_2_pKF = R_l_2_c ;
        Eigen::Quaterniond q_loopKF_2_pKF(R_loopKF_2_pKF) ;
        Eigen::Vector3d t_loopKF_2_pKF = t_l_2_c ;

        problemPoseGraph->AddResidualBlock( new relativePoseFactor(q_loopKF_2_pKF, t_loopKF_2_pKF),
                                            NULL,
                                            para_loop_Pose[loop_KF_index],
                                            para_loop_Pose[fix_KF_index]) ;
    }

    onGlobalOptimization = true ;
    doneGlobalOptimization = false ;
    cvGlobalOptimization.notify_all();
}

} //namespace LT_SLAM
