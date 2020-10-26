#include "Map.h"

namespace LT_SLAM
{

Map::Map(System *pSys)
{
    pSystem = pSys;
    //Create KeyFrame Database
    mpKeyFrameDatabase = new KeyFrameDatabase();
    numOfKeyFrames = 0 ;

    numOfEraseKF = 0;
    numOfEraseMP = 0;

    clear();
}

Map::~Map()
{
    //    clear();
    //    if ( mpKeyFrameDatabase ){
    //        delete mpKeyFrameDatabase ;
    //    }
}

MapPoint* Map::getMapPoint(unsigned int pID)
{
    std::unordered_map<unsigned int, MapPoint*>::iterator pIter = mspMapPoints.find(pID) ;
    if ( pIter == mspMapPoints.end() ){
        ROS_WARN("can not get map point = %d", pID ) ;
        return NULL ;
    }
    else {
        return pIter->second ;
    }
}

void Map::AddKeyFrame(KeyFrame* pKF)
{
    numOfKeyFrames++ ;
    mspKeyFrames[pKF->mnId] = pKF ;
}

void Map::EraseKeyFrame(KeyFrame *pKF, bool eraseObservationFlag)
{
    //transfer the attach subvolumes into the parent keyframes.
    Eigen::Matrix3f currentR = pKF->GetRotation().cast<float>();
    Eigen::Vector3f currentT = pKF->GetTranslation().cast<float>();

    vector<KeyFrame*> tmp = pKF->GetBestCovisibilityKeyFrames(1) ;
    if ( tmp.size() > 0 )
    {
        KeyFrame* mpParent = tmp[0] ;
        Eigen::Matrix3f parentR = mpParent->GetRotation().cast<float>();
        Eigen::Vector3f parentT = mpParent->GetTranslation().cast<float>() ;

        for( std::list<chisel::ChunkPtr>::iterator iterChunks = pKF->attachedChunks.begin();
             iterChunks != pKF->attachedChunks.end(); iterChunks++)
        {
            chisel::ChunkPtr pChunk = (*iterChunks) ;
            pChunk->origin = parentR.transpose()*( currentR*pChunk->origin + currentT - parentT ) ;
            pChunk->corner0 = parentR.transpose()*( currentR*pChunk->corner0 + currentT - parentT );
            pChunk->corner1 = parentR.transpose()*( currentR*pChunk->corner1 + currentT - parentT );
            pChunk->corner2 = parentR.transpose()*( currentR*pChunk->corner2 + currentT - parentT );

            mpParent->attachedChunks.push_back(pChunk);
        }

        for( std::list<chisel::MeshPtr>::iterator iterMesh = pKF->attachedMeshes.begin() ;
             iterMesh != pKF->attachedMeshes.end(); iterMesh++)
        {
            chisel::MeshPtr pMesh = *iterMesh ;
            {//mesh
                int sz = pMesh->vertices.size();
                for( int i = 0 ; i < sz ; i++ ){
                    //transform into camra frame
                    pMesh->vertices[i] = parentR.transpose()*(
                                currentR*pMesh->vertices[i] + currentT - parentT ) ;
                }
            }
            {//grid
                int sz = pMesh->grids.size();
                for( int i = 0 ; i < sz ; i++ ){
                    //transform into camra frame
                    pMesh->grids[i] = parentR.transpose()*(
                                currentR*pMesh->grids[i] + currentT - parentT ) ;
                }
            }
            mpParent->attachedMeshes.push_back(pMesh);
        }
    }
    if ( eraseObservationFlag )
    {
        //erase observations
        //ROS_WARN("EraseKeyFrame =%d", pKF->mnId ) ;
        for( int i = 0; i < pKF->N; i++ )
        {
            if ( pKF->pIDs[i] == 0 ){
                continue ;
            }
            if ( mspMapPoints.find(pKF->pIDs[i]) == mspMapPoints.end() ){
                ROS_WARN("[EraseKeyFrame] can not find MP=%d", pKF->pIDs[i] ) ;
            }
            MapPoint* mp = getMapPoint(pKF->pIDs[i]) ;

            //ROS_WARN("eraseObservation %d->%d", mp->mnId, pKF->mnId ) ;
            eraseObservation(mp, pKF) ;
            //        mp->mObservations.erase(pKF) ;
            //        mp->updateState();
        }
        numOfKeyFrames-- ;
        mspKeyFrames.erase(pKF->mnId) ;
    }

    pKF->attachedChunks.clear();
    pKF->attachedMeshes.clear();
    pKF->depth.release();
    pKF->img.release();
    pKF->img_right.release();
    pKF->mDescriptors.release();

    delete pKF;
}

void Map::updateDescriptor(MapPoint *mp)
{
    if ( mp->mObservations.size() < 1 ){
        return ;
    }

    //1. ComputeDistinctiveDescriptors
    vector<cv::Mat> vDescriptors;
    vDescriptors.reserve(mp->mObservations.size());

    //unsigned int flag ;
    for(map<KeyFrame*,unsigned int>::iterator mit=mp->mObservations.begin(),
        mend=mp->mObservations.end();
        mit!= mend;
        mit++)
    {
        KeyFrame* pKF = mit->first;
        if ( pKF->mbBad ){
            continue ;
        }
        pKF->initConnectKeyFrames = false ;

        //flag = mit->second >> 16;
        unsigned int index = mit->second & 0xffff ;
        if ( pKF->mDescriptors.rows < 1 ){
            continue ;
        }
        //printf("pKF->mDescriptors.rows=%d index=%d\n", pKF->mDescriptors.rows, index ) ;
        vDescriptors.push_back(pKF->mDescriptors.row(index));
    }

    if(vDescriptors.empty()){
        return;
    }

    // Compute distances between them
    const size_t N = vDescriptors.size();
    float Distances[N][N];
    for(size_t i=0;i<N;i++)
    {
        Distances[i][i]=0;
        for(size_t j=i+1;j<N;j++)
        {
            int distij = DescriptorDistance(vDescriptors[i],vDescriptors[j]);
            Distances[i][j]=distij;
            Distances[j][i]=distij;
        }
    }

    // Take the descriptor with least median distance to the rest
    int BestMedian = INT_MAX;
    int BestIdx = 0;
    for(size_t i=0;i<N;i++)
    {
        vector<int> vDists(Distances[i],Distances[i]+N);
        sort(vDists.begin(),vDists.end());
        int median = vDists[0.5*(N-1)];

        if(median<BestMedian)
        {
            BestMedian = median;
            BestIdx = i;
        }
    }
    mp->mDescriptor = vDescriptors[BestIdx].clone();
}

void Map::updateMapPoint(MapPoint* mp)
{
    if ( mp->mObservations.size() < 1 ){
        return ;
    }
    //1. ComputeDistinctiveDescriptors
    updateDescriptor(mp) ;

    //2. update the position
    if ( mp->init == false )
    {
        bool tri_flag = false ;
        if ( mp->numOfStereoObservations > 0 )
        {
            for( std::map<KeyFrame*, unsigned int>::iterator obs = mp->mObservations.begin();
                 obs != mp->mObservations.end();
                 obs++ )
            {
                KeyFrame* pKF ;
                unsigned int index ;
                unsigned int flag ;
                Eigen::Vector3d f ;

                pKF = obs->first ;
                if ( pKF->mbBad ){
                    continue ;
                }
                flag = obs->second >> 16 ;
                index = obs->second & 0xffff ;

                //printf("flag=%d index=%d", flag, index ) ;

                if ( flag == 0 ){//mono
                    continue ;
                }
                else
                {//stereo
                    float disparity = pKF->mvKeysUn[index].x - pKF->mvKeysRightUn[index].x ;
                    if ( disparity < 3 ){
                        continue ;
                    }
                    float d = pSetting->bf/disparity ;
                    //        if ( d > pSetting->mThDepth ){
                    //            continue ;
                    //        }
                    f << (pKF->mvKeysUn[index].x-pSetting->cx)*d/pSetting->fx,
                            (pKF->mvKeysUn[index].y-pSetting->cy)*d/pSetting->fy,
                            d ;
                    mp->pos = pKF->GetRotation()*f + pKF->GetTranslation() ;

                    mp->init = true ;
                    tri_flag = true ;
                    break ;
                }
            }
        }
        if ( tri_flag == false )
        {
            if ( mp->numOfMonoObservations + mp->numOfMonoObservations < 3 )
            {
                mp->init = false ;
            }
            else
            {
                std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d>> R_b_2_w;
                std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> T_b_2_w;
                std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> observation;
                Eigen::Vector3d f ;
                std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> shift ;
                unsigned int index ;
                unsigned int flag ;
                KeyFrame* pKF ;

                for( std::map<KeyFrame*, unsigned int>::iterator obs = mp->mObservations.begin();
                     obs != mp->mObservations.end();
                     obs++ )
                {
                    pKF = obs->first ;
                    if ( pKF->mbBad ){
                        continue ;
                    }

                    flag = obs->second >> 16 ;
                    index = obs->second & 0xffff ;

                    //printf("flag=%d index=%d", flag, index ) ;

                    if ( flag == 0 ){//mono
                        f << (pKF->mvKeysUn[index].x-pSetting->cx)/pSetting->fx,
                                (pKF->mvKeysUn[index].y-pSetting->cy)/pSetting->fy,
                                1 ;
                        observation.push_back(f);
                        R_b_2_w.push_back( pKF->GetRotation() );
                        T_b_2_w.push_back( pKF->GetTranslation() );
                        shift.push_back( Eigen::Vector3d(0, 0, 0) ) ;
                    }
                    else{//stereo
                        f << (pKF->mvKeysUn[index].x-pSetting->cx)/pSetting->fx,
                                (pKF->mvKeysUn[index].y-pSetting->cy)/pSetting->fy,
                                1 ;
                        observation.push_back(f);
                        R_b_2_w.push_back( pKF->GetRotation() );
                        T_b_2_w.push_back( pKF->GetTranslation() );
                        shift.push_back( Eigen::Vector3d(0, 0, 0) ) ;

                        f << (pKF->mvKeysRightUn[index].x-pSetting->cx)/pSetting->fx,
                                (pKF->mvKeysRightUn[index].y-pSetting->cy)/pSetting->fy,
                                1 ;
                        observation.push_back(f);
                        R_b_2_w.push_back( pKF->GetRotation() );
                        T_b_2_w.push_back( pKF->GetTranslation() );
                        shift.push_back( Eigen::Vector3d(-pSetting->baseline, 0, 0) );
                    }
                }

                if ( (T_b_2_w[0]-T_b_2_w[T_b_2_w.size()-1]).norm() < 0.01 ){//not enough parallex
                    mp->init = false ;
                }
                else
                {
                    mp->init = mp->triangulate( R_b_2_w, T_b_2_w, observation, shift, mp->pos) ;
                    if ( mp->init )
                    {
                        // cout << mp->pos.transpose() << "\n" ;
                        //                        if ( fabs(mp->pos(0)) < 0.1 && fabs(mp->pos(1)) < 0.1 ){
                        //
                        //                        }
                        for( size_t i = 0 ; i < R_b_2_w.size() ; i++ )
                        {
                            //                cout << "pos =" << pos.transpose() << "\n" ;
                            //                cout << "R_b_2_w \n" << R_b_2_w[i] << "\n" ;
                            //                cout << "T_b_2_w =" << T_b_2_w[i].transpose() << "\n" ;

                            Eigen::Vector3d p = R_b_2_w[i].transpose()*(mp->pos - T_b_2_w[i]) ;
                            if ( fabs(p(2)) < 0.0001 ){
                                mp->init = false ;
                                break ;
                            }
                            //                p /= p(2) ;
                            //                cout << "p= " << p.transpose() << " \n" ;
                            //                printf("observation= %f %f\n", observation[i](0),
                            //                                   observation[i](1) ) ;

                            //                printf("diff= %f %f\n", (p(0)-observation[i](0))*pSetting->fx,
                            //                                   (p(1)-observation[i](1))*pSetting->fy ) ;
                        }
                    }
                }
            }
        }
    }

    //3. UpdateNormalAndDepth
    Eigen::Vector3d p3D ;
    Eigen::Vector3d normal ;
    normal.setZero();
    {
        p3D = mp->pos ;
        if ( mp->init == false ){
            mp->pos.setZero();
            mp->mNormalVector.setZero();
            return ;
        }
    }

    int n = 0;
    for(map<KeyFrame*,unsigned int>::iterator mit=mp->mObservations.begin(),
        mend=mp->mObservations.end();
        mit != mend;
        mit++)
    {
        KeyFrame* pKF = mit->first;
        if( pKF->mbBad ){
            continue ;
        }
        Eigen::Vector3d Owi = pKF->GetTranslation() ;
        Eigen::Vector3d normali = p3D - Owi;
        normali /= normali.norm() ;
        normal = normal + normali;
        n++;
    }
    mp->mNormalVector = normal/n;
}

void Map::addObservation(MapPoint* mp, KeyFrame* pKF,
                         unsigned int index, unsigned int flag)
{
    mp->mObservations[pKF] = (flag<<16) | index;
    if ( flag == 0 ){
        mp->numOfMonoObservations = mp->numOfMonoObservations + 1 ;
    }
    else {
        mp->numOfStereoObservations = mp->numOfStereoObservations + 1 ;
    }
    updateMapPoint(mp);
}

bool Map::eraseObservation(MapPoint* mp, KeyFrame* pKF)
{
    std::map<KeyFrame*, unsigned int>::iterator obs = mp->mObservations.find(pKF) ;
    if ( obs == mp->mObservations.end() ){
        ROS_WARN("can not find obs = %d->%d", mp->mnId, pKF->mnId ) ;
        return false ;
    }

    //update the covisibility information
    for( std::map<KeyFrame*, unsigned int>::iterator obs_KF = mp->mObservations.begin() ;
         obs_KF != mp->mObservations.end(); obs_KF++ ){
        obs_KF->first->initConnectKeyFrames = false ;
    }

    unsigned int flag = obs->second >> 16 ;
    unsigned int index = obs->second & 0xffff ;


    //map point to keyframe
    mp->mObservations.erase(obs) ;
    if ( flag == 0 ){//mono
        mp->numOfMonoObservations-- ;
    }
    else {//stereo
        mp->numOfStereoObservations-- ;
    }

    if ( mp->mObservations.size() != mp->numOfMonoObservations+mp->numOfStereoObservations ){
        ROS_WARN("[Error in map point erase]") ;
        ROS_WARN("flag=%d index=%d", flag, index ) ;
        ROS_WARN("mp->mObservations.size()=%d", mp->mObservations.size()) ;
        ROS_WARN("mp->numOfMonoObservations=%d", mp->numOfMonoObservations ) ;
        ROS_WARN("mp->numOfStereoObservations=%d", mp->numOfStereoObservations ) ;
    }

    //keyframe to map point
    pKF->pIDs[index] = 0 ;

    //keyframe to keyframe
    int nGridPosX, nGridPosY;
    if ( pKF->PosInGrid( pKF->mvKeysUn[index], nGridPosX,nGridPosY ) ){
        pKF->mGrid[nGridPosX][nGridPosY].remove(index);
    }

    if ( mp->mObservations.size() < 2 ){
        eraseMapPoint(mp) ;
    }

    //    if ( mp->numOfStereoObservations == 0 && mp->numOfMonoObservations < 2 )
    //    {//delete map point
    //        eraseMapPoint(mp) ;
    //    }
    //    else{
    //        //updateMapPoint(mp) ;
    //    }


    return true ;
}

bool Map::mergeMapPoint(unsigned int fromID, unsigned int toID)
{
    MapPoint* mp_from = getMapPoint(fromID) ;
    MapPoint* mp_to = getMapPoint(toID);
    std::map<KeyFrame*, unsigned int> observations = mp_from->mObservations;
    bool outlier_flag =false ;
    for(map<KeyFrame*,unsigned int>::iterator mit=observations.begin(),
        mend=observations.end();
        mit!=mend;
        mit++)
    {
        KeyFrame* pKF = mit->first;
        pKF->initConnectKeyFrames = false ;
        if ( mp_to->mObservations.find(pKF) != mp_to->mObservations.end() ){
            outlier_flag = true ;
            break ;
        }
    }
    if ( outlier_flag ){
        return false ;
    }

    mp_to->mObservations.insert(observations.begin(), observations.end()) ;
    for(map<KeyFrame*,unsigned int>::iterator mit=observations.begin(),
        mend=observations.end();
        mit!=mend;
        mit++)
    {
        KeyFrame* pKF = mit->first;

        //        if( !pKF->mbBad ){
        //unsigned int flag = obs.second >> 16 ;
        unsigned int index = mit->second & 0xffff ;
        pKF->pIDs[index] = toID ;
        //}
    }
    mspMapPoints.erase(fromID) ;
    delete mp_from ;

    updateMapPoint(mp_to);
    mergeMap[fromID] = toID ;
    return true ;

    //eraseMapPoint(mp_from);

    //    std::map<KeyFrame*, unsigned int> observations ;
    //    std::unordered_map<int, MapPoint*>::iterator pIterFrom;
    //    std::unordered_map<int, MapPoint*>::iterator pIterTo;
    //    {
    //        pIterFrom = mspMapPoints.find(fromID) ;
    //        if ( pIterFrom == mspMapPoints.end() ){
    //            ROS_WARN("can not find fromID = %d", fromID ) ;
    //            return ;
    //        }
    //        observations = pIterFrom->second->mObservations;

    //        pIterTo = mspMapPoints.find(toID) ;
    //        if ( pIterTo == mspMapPoints.end() ){

    //            ROS_WARN("can not find toID = %d", toID ) ;
    //            return ;
    //        }
    //        pIterTo->second->mObservations.insert( observations.begin(), observations.end() ) ;

    //        for(map<KeyFrame*,unsigned int>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
    //        {
    //            KeyFrame* pKF = mit->first;

    //            if(!pKF->mbBad){
    //                //unsigned int flag = obs.second >> 16 ;
    //                unsigned int index = mit->second & 0xffff ;
    //                pKF->pIDs[index] = toID ;
    //            }
    //        }
    //        mergeMap[fromID] = toID ;
    //        mspMapPoints.erase(pIterFrom) ;
    //    }

    //    updateMapPoint(toID) ;
    //    pIterTo->second->updateState();
}

void Map::clear()
{
    for(std::unordered_map<unsigned int, MapPoint*>::iterator sit=mspMapPoints.begin(),
        send=mspMapPoints.end();
        sit!=send;
        sit++ ){
        delete sit->second;
    }
    mspMapPoints.clear();

    for(Map::KFMAPTYPE::iterator sit=mspKeyFrames.begin(),
        send=mspKeyFrames.end();
        sit!=send;
        sit++ ){
        delete sit->second;
    }
    mspKeyFrames.clear();
}

MapPoint* Map::createMapPoint(unsigned int pID, const Eigen::Vector3d Pos, bool init )
{
    MapPoint* p = new MapPoint(pID, Pos, init) ;
    mspMapPoints[pID] = p ;
    return p;
}

void Map::eraseMapPoint(MapPoint* mp)
{
    std::map<KeyFrame*, unsigned int> observations = mp->mObservations ;
    mp->mObservations.clear();

    //printf("eraseMapPoint = %d->", mp->mnId ) ;
    for ( std::map<KeyFrame*, unsigned int>::iterator iter = observations.begin() ;
          iter != observations.end(); iter++)
    {
        //        unsigned int flag = iter->second >> 16 ;

        unsigned int index = iter->second & 0xffff ;
        //        if ( flag == 0 ){//mono
        //            mp->numOfMonoObservations-- ;
        //        }
        //        else {//stereo
        //            mp->numOfStereoObservations-- ;
        //        }
        //keyframe to map point
        iter->first->pIDs[index] = 0 ;
        iter->first->initConnectKeyFrames = false ;

        //printf("[%d,%d]", iter->first->mnId, index ) ;

        //keyframe to keyframe
        int nGridPosX, nGridPosY;
        if ( iter->first->PosInGrid( iter->first->mvKeysUn[index], nGridPosX,nGridPosY ) ){
            iter->first->mGrid[nGridPosX][nGridPosY].remove(index);
        }
    }
    //printf("\n");

    //printf("[before]mspMapPoints.sz = %d\n", mspMapPoints.size() ) ;
    mspMapPoints.erase(mp->mnId) ;
    //printf("[after]mspMapPoints.sz = %d\n", mspMapPoints.size() ) ;

    numOfEraseMP++ ;

    delete mp;
}

} //namespace LT_SLAM
