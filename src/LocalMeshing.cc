#include "LocalMeshing.h"

//#define BACKWARD_HAS_DW 1
//#include <backward.hpp>
//namespace backward
//{
//backward::SignalHandling sh;
//} // namespace backward

namespace LT_SLAM
{

inline void updateXYbound( double x, double y, int& lx, int& rx, int& ly, int& ry )
{
    if ( x < lx ){
        lx = x ;
    }
    if ( y < ly ){
        ly = y ;
    }
    if ( x > rx ){
        rx = x ;
    }
    if ( y > ry ){
        ry = y ;
    }
}

chisel::Vec3 LAMBERT(const chisel::Vec3& n, const chisel::Vec3& light)
{
    return fmax(n.dot(light), 0.0f) * chisel::Vec3(0.5, 0.5, 0.5);
}

inline void calculateDepthImage(cv::Mat disparity, cv::Mat& depthImage, float bf)
{
    int n = disparity.rows ;
    int m = disparity.cols ;

    depthImage.create(n, m, CV_32F);
    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < m; j++)
        {
            float d = disparity.at<float>(i, j);
            if ( d < 3.0 ) {
                depthImage.at<float>(i, j) = 0.0;
            }
            else {
                depthImage.at<float>(i, j) = bf / d;
            }
        }
    }
}

LocalMeshing::LocalMeshing( System* sys)
{
    pSystem = sys ;
    curImgTime = -1 ;

    mlpKeyFrameQueue.clear();

    chisel::Vec4 truncation(pSetting->truncationDistQuad,
                            pSetting->truncationDistLinear,
                            pSetting->truncationDistConst,
                            pSetting->truncationDistScale);

    chiselMap.reset(new chisel::Chisel(Eigen::Vector3i(pSetting->chunkSizeX,
                                                       pSetting->chunkSizeY,
                                                       pSetting->chunkSizeZ),
                                       pSetting->voxelResolution,
                                       pSetting->useColor)) ;
    chiselMap->chunkManager.updateMeshFlag = (pSetting->updateMeshFlag > 0);
    projectionIntegrator.SetCentroids(chiselMap->GetChunkManager().GetCentroids());

    projectionIntegrator.SetTruncator(chisel::TruncatorPtr(
                                          //new chisel::InverseTruncator(truncationDistScale, base_line, fx)));
                                          new chisel::QuadraticTruncator(truncation(0),
                                                                         truncation(1),
                                                                         truncation(2),
                                                                         truncation(3))));

    projectionIntegrator.SetWeighter(chisel::WeighterPtr(new chisel::ConstantWeighter(pSetting->weight)));
    projectionIntegrator.SetCarvingDist(pSetting->carvingDist);
    projectionIntegrator.SetCarvingEnabled(pSetting->useCarving>0);

    numChannels = 1;
    initColorImage = false ;
    SADWindowSize = 11 ;

    sgm2.init_disparity_method(7, 86);
}

LocalMeshing::~LocalMeshing()
{
    ;
}

void LocalMeshing::ProjectNeighboringKFsNew()
{
    KeyFrame *mpCurrentKF = curFrame.pKF ;
    Eigen::Matrix3f currentRGlobal = mpCurrentKF->GetRotation().cast<float>() ;
    Eigen::Vector3f currentTGlobal = mpCurrentKF->GetTranslation().cast<float>();

    int chunkSizeX = chiselMap->chunkManager.chunkSize(0) ;
    int chunkSizeY = chiselMap->chunkManager.chunkSize(1) ;
    int chunkSizeZ = chiselMap->chunkManager.chunkSize(2) ;
    float voxelResolutionMeters = chiselMap->chunkManager.voxelResolutionMeters ;
    float voxelResolutionMetersHalf = voxelResolutionMeters/2 ;
    int shiftXY = chunkSizeX*chunkSizeY;
    int shiftX = chunkSizeX;

    //ROS_WARN("neighboringKFs.size()=%d K=%d", curFrame.neighboringKFs.size(), pSetting->renderingK ) ;
    int renderNum = curFrame.neighboringKFs.size();
    for(int ith=0; ith < renderNum; ith++ )
    {
        KeyFrame* pKF = curFrame.neighboringKFs[ith] ;
        Eigen::Matrix3f currentR = pKF->GetRotation().cast<float>();
        Eigen::Vector3f currentT = pKF->GetTranslation().cast<float>();

        std::list<chisel::MeshPtr>::iterator iterMesh = pKF->attachedMeshes.begin() ;
        for( std::list<chisel::ChunkPtr>::iterator iterChunks = pKF->attachedChunks.begin();
             iterChunks != pKF->attachedChunks.end(); )
        {
            Eigen::Vector3f origin = currentR*((*iterChunks)->origin)+currentT ;
            Eigen::Vector3f corner0 = currentR*((*iterChunks)->corner0)+currentT ;
            Eigen::Vector3f corner1 = currentR*((*iterChunks)->corner1)+currentT ;
            Eigen::Vector3f corner2 = currentR*((*iterChunks)->corner2)+currentT ;

            float chunkCenterX = origin(0) ;
            float chunkCenterY = origin(1) ;
            float chunkCenterZ = origin(2) ;

            float newBound = pSetting->farPlaneDist ;
            //float newBound = 2*cubicBound ;
            if ( chunkCenterX < (currentTGlobal(0)-newBound) ||
                 chunkCenterX > (currentTGlobal(0)+newBound) ||
                 chunkCenterY < (currentTGlobal(1)-newBound) ||
                 chunkCenterY > (currentTGlobal(1)+newBound) ||
                 chunkCenterZ < (currentTGlobal(2)-newBound) ||
                 chunkCenterZ > (currentTGlobal(2)+newBound) )
            {
                iterChunks++ ;
                iterMesh++ ;
            }
            else
            {
                Eigen::Vector3f baseX = corner0-origin ;
                Eigen::Vector3f baseY = corner1-origin ;
                Eigen::Vector3f baseZ = corner2-origin ;
                for( int z = 0; z < chunkSizeZ; z++ )
                {
                    for( int y = 0; y < chunkSizeY; y++ )
                    {
                        for( int x = 0 ; x < chunkSizeX; x++ )
                        {
                            chisel::ColorVoxel color = (*iterChunks)->GetColorVoxelMutable(x, y, z) ;
                            chisel::DistVoxel dist = (*iterChunks)->GetDistVoxelMutable(x, y, z) ;

                            if ( dist.GetWeight() <= 1e-15 ){
                                continue ;
                            }

                            Eigen::Vector3f displacement
                                    = (x+0.5)/chunkSizeX*baseX +
                                    (y+0.5)/chunkSizeY*baseY +
                                    (z+0.5)/chunkSizeZ*baseZ ;
                            Eigen::Vector3f p = origin + displacement;

                            //std::cout << "p " << p.transpose() << "\n" ;

                            chisel::ChunkID chunkID;
                            chunkID(0) = floor(p(0)/(voxelResolutionMeters*chunkSizeX)) ;
                            chunkID(1) = floor(p(1)/(voxelResolutionMeters*chunkSizeY)) ;
                            chunkID(2) = floor(p(2)/(voxelResolutionMeters*chunkSizeZ)) ;

                            //std::cout << "chunkID " << chunkID.transpose() << "\n" ;

                            if (!chiselMap->chunkManager.HasChunk(chunkID)){
                                chiselMap->chunkManager.CreateChunk(chunkID);
                            }
                            chisel::ChunkPtr chunk = chiselMap->chunkManager.GetChunk(chunkID);
                            chiselMap->meshesToUpdate[chunkID] = true;

                            Eigen::Vector3f newChunkOrigin ;
                            newChunkOrigin << chunkID(0)*(voxelResolutionMeters*chunkSizeX),
                                    chunkID(1)*(voxelResolutionMeters*chunkSizeY),
                                    chunkID(2)*(voxelResolutionMeters*chunkSizeZ);
                            Eigen::Vector3f rest = p - newChunkOrigin;

                            rest /= voxelResolutionMeters;

                            short voxel_id_x = rest(0) ;
                            if ( voxel_id_x < 0 || voxel_id_x >= chunkSizeX ){
                                continue ;
                            }
                            short voxel_id_y = rest(1) ;
                            if ( voxel_id_y < 0 || voxel_id_y >= chunkSizeX ){
                                continue ;
                            }
                            short voxel_id_z = rest(2) ;
                            if ( voxel_id_z < 0 || voxel_id_z >= chunkSizeX ){
                                continue ;
                            }

                            chisel::VoxelID idVoxel = chunk->GetVoxelID(voxel_id_x, voxel_id_y, voxel_id_z) ;
                            //                            if ( idVoxel < 0 || idVoxel >= chunkSizeX*chunkSizeY*chunkSizeZ ){
                            //                                ROS_ERROR("idVoxel = %d x = %d y = %d z = %d", idVoxel, voxel_id_x, voxel_id_y, voxel_id_z ) ;
                            //                            }
                            //chisel::VoxelID idVoxel = voxel_id_z*shiftXY + voxel_id_y*shiftX + voxel_id_x ;
                            //printf( "voxel_id x=%d y=%d z=%d idVoxel=%d\n", voxel_id_x, voxel_id_y, voxel_id_z , idVoxel) ;

                            chisel::ColorVoxel& colorVoxel = chunk->GetColorVoxelMutable(idVoxel);
                            chisel::DistVoxel& distVoxel = chunk->GetDistVoxelMutable(idVoxel);

                            distVoxel.Integrate( dist.GetSDF(), dist.GetWeight() );
                            colorVoxel.Integrate( color.GetRed(), color.GetGreen(), color.GetBlue(), 1 );
                        }
                    }
                }

                //integrate tsdf
                //printf("chunkCenterX=%f chunkCenterY=%f chunkCenterZ=%f\n", chunkCenterX, chunkCenterY, chunkCenterZ ) ;
                //printf("currentTGlobal(0)=%f currentTGlobal(1)=%f currentTGlobal(2)=%f\n", currentTGlobal(0), currentTGlobal(1), currentTGlobal(2) );

                //delele the attache meshes & chunks
                iterChunks = pKF->attachedChunks.erase(iterChunks) ;
                iterMesh = pKF->attachedMeshes.erase(iterMesh) ;
            }
        }
    }
}

void LocalMeshing::extractMeshes(KeyFrame* mpCurrentKF )
{
    const chisel::MeshMap& meshMap = chiselMap->GetChunkManager().GetAllMeshes();

    if(meshMap.size() == 0){
        return;
    }

    Eigen::Matrix3f currentRGlobal = mpCurrentKF->GetRotation().cast<float>() ;
    Eigen::Vector3f currentTGlobal = mpCurrentKF->GetTranslation().cast<float>();

    std::list<chisel::ChunkID> extractList ;
    for (const std::pair<chisel::ChunkID, chisel::MeshPtr>& meshes : meshMap)
    {
        chisel::ChunkID chunkID = meshes.first ;
        if ( !chiselMap->chunkManager.HasChunk(chunkID) ){
            ROS_WARN("inconsistent chunks") ;
        }
        chisel::ChunkPtr chunk = chiselMap->chunkManager.GetChunk(chunkID);
        int chunkSizeX = chiselMap->chunkManager.chunkSize(0) ;
        int chunkSizeY = chiselMap->chunkManager.chunkSize(1) ;
        int chunkSizeZ = chiselMap->chunkManager.chunkSize(2) ;
        float voxelResolutionMeters = chiselMap->chunkManager.voxelResolutionMeters ;

        //float chunkCenterX = chunkID.x()*chunkSizeX*voxelResolutionMeters + chunkSizeX/2*voxelResolutionMeters  ;
        //float chunkCenterY = chunkID.y()*chunkSizeY*voxelResolutionMeters + chunkSizeY/2*voxelResolutionMeters  ;
        //float chunkCenterZ = chunkID.z()*chunkSizeZ*voxelResolutionMeters + chunkSizeZ/2*voxelResolutionMeters  ;

        Eigen::Vector3f origin = chunk->GetOrigin();
        float chunkCenterX = origin(0) ;
        float chunkCenterY = origin(1) ;
        float chunkCenterZ = origin(2) ;

        if ( chunkCenterX < (currentTGlobal(0)-pSetting->cubicBound) ||
             chunkCenterX > (currentTGlobal(0)+pSetting->cubicBound) ||
             chunkCenterY < (currentTGlobal(1)-pSetting->cubicBound) ||
             chunkCenterY > (currentTGlobal(1)+pSetting->cubicBound) ||
             chunkCenterZ < (currentTGlobal(2)-pSetting->cubicBound) ||
             chunkCenterZ > (currentTGlobal(2)+pSetting->cubicBound) )
        {
            extractList.push_back(chunkID);

            //extrach meshes and attach them into the current pose
            chisel::MeshPtr pMesh = meshes.second ;
            {//mesh
                int sz = pMesh->vertices.size();
                for( int i = 0 ; i < sz ; i++ ){
                    //transform into camra frame
                    pMesh->vertices[i] = currentRGlobal.transpose()*(pMesh->vertices[i]-currentTGlobal) ;
                }
            }
            {//grid
                int sz = pMesh->grids.size();
                for( int i = 0 ; i < sz ; i++ ){
                    //transform into camra frame
                    pMesh->grids[i] = currentRGlobal.transpose()*(pMesh->grids[i]-currentTGlobal) ;
                }
            }
            mpCurrentKF->attachedMeshes.push_back(pMesh);

            chunk->corner0 = chunk->origin + Eigen::Vector3f(chunkSizeX*voxelResolutionMeters, 0, 0);
            chunk->corner1 = chunk->origin + Eigen::Vector3f(0, chunkSizeY*voxelResolutionMeters, 0);
            chunk->corner2 = chunk->origin + Eigen::Vector3f(0, 0, chunkSizeZ*voxelResolutionMeters);
            chunk->origin = currentRGlobal.transpose()*(chunk->origin-currentTGlobal) ;
            chunk->corner0 = currentRGlobal.transpose()*(chunk->corner0-currentTGlobal) ;
            chunk->corner1 = currentRGlobal.transpose()*(chunk->corner1-currentTGlobal) ;
            chunk->corner2 = currentRGlobal.transpose()*(chunk->corner2-currentTGlobal) ;
            mpCurrentKF->attachedChunks.push_back(chunk);

        }
    }

    //remove chuncks ralated to meshes outside the bounding box
    for(chisel::ChunkID& id: extractList )
    {
        chiselMap->chunkManager.chunks.erase(id) ;
        chiselMap->chunkManager.allMeshes.erase(id);
    }

    //remove other chuncks outside the bounding box
    std::list<chisel::ChunkID> removeList ;
    for (const std::pair<chisel::ChunkID, chisel::ChunkPtr>& chunk : chiselMap->GetChunkManager().chunks)
    {
        chisel::ChunkPtr pChunk = chunk.second;
        Eigen::Vector3f origin = pChunk->GetOrigin();
        if ( origin(0) < (currentTGlobal(0)-pSetting->cubicBound) ||
             origin(0) > (currentTGlobal(0)+pSetting->cubicBound) ||
             origin(1) < (currentTGlobal(1)-pSetting->cubicBound) ||
             origin(1) > (currentTGlobal(1)+pSetting->cubicBound) ||
             origin(2) < (currentTGlobal(2)-pSetting->cubicBound) ||
             origin(2) > (currentTGlobal(2)+pSetting->cubicBound) )
        {
            removeList.push_back( chunk.first );
        }
    }
    //printf("start remove = %d\n", removeList.size());
    for( const chisel::ChunkID& id: removeList ){
        chiselMap->chunkManager.chunks.erase( id ) ;
    }
    //printf("end of remove\n") ;
}



void LocalMeshing::FillMarkerTopicWithMeshesLocal()
{
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = "base";
    marker.scale.x = 1;
    marker.scale.y = 1;
    marker.scale.z = 1;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1;
    marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
    marker.colors.clear();
    marker.points.clear();

    localChunkBoxes.clear();

    const chisel::MeshMap& meshMap = chiselMap->GetChunkManager().GetAllMeshes();
    if(meshMap.size() == 0){
        return;
    }

    chisel::Vec3 lightDir(0.8f, -0.2f, 0.7f);
    lightDir.normalize();
    chisel::Vec3 lightDir1(-0.5f, 0.2f, 0.2f);
    lightDir.normalize();
    const chisel::Vec3 ambient(0.2f, 0.2f, 0.2f);
    //int idx = 0;


    //render the current mesh
    for (const std::pair<chisel::ChunkID, chisel::MeshPtr>& meshes : meshMap)
    {
        const chisel::MeshPtr& mesh = meshes.second;
        for (size_t i = 0; i < mesh->grids.size(); i++)
        {
            const chisel::Vec3 &vec = mesh->grids[i];
            geometry_msgs::Point pt;
            pt.x = vec[0];
            pt.y = vec[1];
            pt.z = vec[2];
            localChunkBoxes.push_back(pt);
        }

        for (size_t i = 0; i < mesh->vertices.size(); i++)
        {
            const chisel::Vec3& p = mesh->vertices[i];

            geometry_msgs::Point pt;
            pt.x = p[0];
            pt.y = p[1];
            pt.z = p[2];
            marker.points.push_back(pt);

            if(mesh->HasColors())
            {
                const chisel::Vec3& meshCol = mesh->colors[i];
                std_msgs::ColorRGBA color;
                color.r = meshCol[0];
                color.g = meshCol[1];
                color.b = meshCol[2];
                color.a = 1.0;
                marker.colors.push_back(color);
            }
            else
            {
                if(mesh->HasNormals())
                {
                    const chisel::Vec3 normal = mesh->normals[i];
                    std_msgs::ColorRGBA color;
                    chisel::Vec3 lambert = LAMBERT(normal, lightDir) + LAMBERT(normal, lightDir1) + ambient;
                    color.r = fmin(lambert[0], 1.0);
                    color.g = fmin(lambert[1], 1.0);
                    color.b = fmin(lambert[2], 1.0);
                    color.a = 1.0;
                    marker.colors.push_back(color);
                }
                else
                {
                    std_msgs::ColorRGBA color;
                    color.r = p[0] * 0.25 + 0.5;
                    color.g = p[1] * 0.25 + 0.5;
                    color.b = p[2] * 0.25 + 0.5;
                    color.a = 1.0;
                    marker.colors.push_back(color);
                }
            }
        }
    }

    //printf("neighboringKFs.size()=%d K=%d\n", neighboringKFs.size(), renderingK ) ;
    int renderNum = curFrame.neighboringKFs.size();
    for(int ith=0; ith < renderNum; ith++ )
    {
        KeyFrame* pKF = curFrame.neighboringKFs[ith] ;
        Eigen::Matrix3f currentR = pKF->GetRotation().cast<float>() ;
        Eigen::Vector3f currentT = pKF->GetTranslation().cast<float>();

        for(chisel::MeshPtr pMesh: pKF->attachedMeshes )
        {
            for (size_t i = 0; i < pMesh->vertices.size(); i++)
            {
                const chisel::Vec3& vec = pMesh->vertices[i];
                chisel::Vec3 p = currentR*vec+currentT ;

                geometry_msgs::Point pt;
                pt.x = p[0];
                pt.y = p[1];
                pt.z = p[2];
                marker.points.push_back(pt);

                if(pMesh->HasColors())
                {
                    const chisel::Vec3& meshCol = pMesh->colors[i];
                    std_msgs::ColorRGBA color;
                    color.r = meshCol[0];
                    color.g = meshCol[1];
                    color.b = meshCol[2];
                    color.a = 1.0;
                    marker.colors.push_back(color);
                }
                else
                {
                    if(pMesh->HasNormals())
                    {
                        const chisel::Vec3 normal = pMesh->normals[i];
                        std_msgs::ColorRGBA color;
                        chisel::Vec3 lambert = LAMBERT(normal, lightDir) + LAMBERT(normal, lightDir1) + ambient;
                        color.r = fmin(lambert[0], 1.0);
                        color.g = fmin(lambert[1], 1.0);
                        color.b = fmin(lambert[2], 1.0);
                        color.a = 1.0;
                        marker.colors.push_back(color);
                    }
                    else
                    {
                        std_msgs::ColorRGBA color;
                        color.r = vec[0] * 0.25 + 0.5;
                        color.g = vec[1] * 0.25 + 0.5;
                        color.b = vec[2] * 0.25 + 0.5;
                        color.a = 1.0;
                        marker.colors.push_back(color);
                    }
                }
                //marker->indicies.push_back(idx);
                //idx++;
            }
        }

        for(chisel::MeshPtr pMesh: pKF->attachedMeshes )
        {
            for (size_t i = 0; i < pMesh->grids.size(); i++)
            {
                const chisel::Vec3& vec = pMesh->grids[i];
                chisel::Vec3 p = currentR*vec+currentT ;

                geometry_msgs::Point pt;
                pt.x = p[0];
                pt.y = p[1];
                pt.z = p[2];
                localChunkBoxes.push_back(pt);
            }
        }
    }
}

void LocalMeshing::Update()
{
    KeyFrame* mpCurrentKF;
    {
        unique_lock<mutex> lock(mMutexKeyFrameQueue);

        curFrame = mlpKeyFrameQueue.front();
        mlpKeyFrameQueue.pop_front();
        mpCurrentKF = curFrame.pKF ;
        // Avoid that a keyframe can be erased while it is being process by this thread
        // mpCurrentKF->SetNotErase();
    }

    double t = (double)cvGetTickCount() ;
    {
        TicToc tc_sgm ;

        cv::Mat mImGray = mpCurrentKF->img.clone();
        cv::Mat imGrayRight = mpCurrentKF->img_right.clone();
        cv::Mat disparity;
        for( int i = 0 ; i < pSetting->downSampleTimes; i++ )
        {
            cv::pyrDown(mImGray, mImGray, cv::Size(mImGray.cols/2, mImGray.rows/2) ) ;
            cv::pyrDown(imGrayRight, imGrayRight, cv::Size(imGrayRight.cols/2, imGrayRight.rows/2) ) ;
        }

//        cv::imshow("mImGray", mImGray ) ;
//        cv::imshow("imGrayRight", imGrayRight ) ;
//        cv::waitKey(1) ;
//        printf("%d %d %d %d\n", imGrayRight.rows, mImGray.rows, mImGray.cols, imGrayRight.cols ) ;

        int rows = mImGray.rows ;
        int cols = mImGray.cols ;
        float time = 0.0 ;
//        if ( ((rows%4) == 0) && ((cols%4) == 0) )
//        {
//            cv::Mat left = mImGray.clone() ;
//            cv::Mat right = imGrayRight.clone() ;

//            cv::Mat output = sgm2.compute_disparity_method(left, right, &time) ;

//            //cv::imshow("output", output*2 ) ;

//            output.convertTo(disparity, CV_32F );
//        }
//        else
//        {
//            cv::Mat left = mImGray.rowRange(0, rows-(rows%4)).colRange(0, cols-(cols%4)).clone() ;
//            cv::Mat right = imGrayRight.rowRange(0, rows-(rows%4)).colRange(0, cols-(cols%4)).clone();

//            cv::Mat output = sgm2.compute_disparity_method(left, right, &time) ;



//            output.convertTo(output, CV_32F ) ;
//            disparity.create(rows, cols, CV_32F );
//            disparity.setTo(0.0) ;
//            for( int i = rows-(rows%4)-1 ; i >= 0 ; i-- )
//            {
//                for( int j = cols-(cols%4)-1; j >= 0 ; j-- ){
//                    disparity.at<float>(i, j) = output.at<float>(i, j) ;
//                }
//            }
//        }


        TicToc tc_sgm_cpu ;
        static cv::Ptr<cv::StereoSGBM> sgbm_;
        sgbm_ = cv::StereoSGBM::create(0, 64, SADWindowSize) ;

        sgbm_->setPreFilterCap(63);
        sgbm_->setBlockSize(SADWindowSize > 0 ? SADWindowSize : 3);
        sgbm_->setP1(8*SADWindowSize*SADWindowSize);
        sgbm_->setP2(32*SADWindowSize*SADWindowSize) ;
        sgbm_->setMinDisparity(0);
        sgbm_->setNumDisparities(64);
        sgbm_->setUniquenessRatio(10);
        sgbm_->setSpeckleWindowSize(100);
        sgbm_->setSpeckleRange(2);
        sgbm_->setDisp12MaxDiff(1);
        sgbm_->setMode(cv::StereoSGBM::MODE_SGBM);
        sgbm_->compute(mImGray, imGrayRight, disparity ) ;
        disparity.convertTo(disparity, CV_32F );
        disparity /= 16.0 ;

        //set margin up and down
        for( int j = 0; j < disparity.cols; j++ )
        {
            for( int i = 0; i < pSetting->upMargin; i++ ){
                disparity.at<float>(i, j) = 0.0 ;
            }
            for( int i = disparity.rows - pSetting->downMargin; i < disparity.rows; i++ ){
                disparity.at<float>(i, j) = 0.0 ;
            }
        }

        //set margin left and right
        for( int i = 0; i < disparity.rows; i++ )
        {
            for( int j = 0; j < pSetting->leftMargin; j++ ){
                disparity.at<float>(i, j) = 0.0 ;
            }
            for( int j = disparity.cols-pSetting->rightMargin; j < disparity.cols; j++ ){
                disparity.at<float>(i, j) = 0.0 ;
            }
        }


        calculateDepthImage(disparity, mpCurrentKF->depth, pSetting->bfDepthMap );

//        ROS_WARN("SGM Time = %f", tc_sgm_cpu.toc() ) ;

        curDepth = mpCurrentKF->depth;

        cv::Mat disp_depth ;
        for( int i = 0 ; i < curDepth.rows; i++ )
        {
            for( int j=0; j < curDepth.cols ; j++ )
            {
                if ( curDepth.at<float>(i, j) < 0.1 ) {
                    continue ;
                }
                if ( curDepth.at<float>(i, j) > 20 ){
                    curDepth.at<float>(i, j) = 20 ;
                }
            }
        }
        cv::normalize(curDepth, disp_depth, 0, 255, CV_MINMAX, CV_8U);
        cv::imshow("Current depth", disp_depth) ;

        curImg = mImGray ;
    }

    ProjectNeighboringKFsNew() ;

    if ( initColorImage == false )
    {
        ColorImage.reset(new chisel::ColorImage<ColorData>(
                             curImg.cols, curImg.rows, numChannels));
        DepthImage.reset(new chisel::DepthImage<DepthData>(curDepth.cols, curDepth.rows));
        intrinsics.SetFx(pSetting->fxDepthMap);
        intrinsics.SetFy(pSetting->fyDepthMap);
        intrinsics.SetCx(pSetting->cxDepthMap);
        intrinsics.SetCy(pSetting->cyDepthMap);
        cameraModel.SetIntrinsics(intrinsics);
        cameraModel.SetWidth(curImg.cols);
        cameraModel.SetHeight(curImg.rows);
        cameraModel.SetFarPlane(pSetting->farPlaneDist);
        cameraModel.SetNearPlane(pSetting->nearPlaneDist);
        colorModel = depthModel = cameraModel;
    }
    //printf("curDepth.cols=%d curDepth.rows=%d\n", DepthImage->GetHeight(), DepthImage->GetWidth() ) ;

    //copy data
    {
        const DepthData* imageData = reinterpret_cast<const DepthData*>(curDepth.data);
        DepthData* depthImageData = DepthImage->GetMutableData();
        int totalPixels = curDepth.cols * curDepth.rows;
        for (int i = 0; i < totalPixels; i++){
            depthImageData[i] =  imageData[i];
        }

    }
    {
        const ColorData* imageData = reinterpret_cast<const ColorData*>(curImg.data);
        ColorData* colorImageData = ColorImage->GetMutableData();
        int totalPixels = curImg.cols * curImg.rows * numChannels;
        for (int i = 0; i < totalPixels; i++){
            colorImageData[i] =  imageData[i];
        }
    }


    //std::cout << "transform_inv\n" << transform_inv.translation() << "\n"  << transform_inv.linear() << "\n";
    chisel::Transform transform ;
    transform.translation() = mpCurrentKF->GetTranslation().cast<float>();
    transform.linear() = mpCurrentKF->GetRotation().cast<float>();


    if( pSetting->useColor ){
        //printf("CHISEL: Integrating depth scan and color image\n");
        chiselMap->IntegrateDepthScanColor<DepthData, ColorData>(
                    projectionIntegrator, DepthImage, transform,
                    depthModel, ColorImage,
                    transform, colorModel );
    }
    else{
        //printf("CHISEL: Integrating depth scan\n");
        chiselMap->IntegrateDepthScan<DepthData>(
                    projectionIntegrator, DepthImage,
                    transform, cameraModel);
    }
    ROS_WARN("Fusion KF TIME %lf", ((double)cvGetTickCount() - t) / (cvGetTickFrequency() * 1000));
    if ( debug_f.is_open() ){
        debug_f << "3 " << ((double)cvGetTickCount() - t) / (cvGetTickFrequency() * 1000) << "\n";
    }

    bool flag = false;
    {
        //ROS_WARN("update meshes") ;
        double t = (double)cvGetTickCount() ;
        flag = chiselMap->UpdateMeshes( pSetting->updateMeshInterval );
        if ( flag ){
            ROS_WARN("update Meshes TIME %lf", ((double)cvGetTickCount() - t) / (cvGetTickFrequency() * 1000));
        }
    }


    TicToc tc_render ;

    FillMarkerTopicWithMeshesLocal();
    ROS_WARN("marker.points=%d chunks=%d", marker.points.size(), localChunkBoxes.size() ) ;

    if ( debug_f.is_open() ){
        debug_f << "4 " << tc_render.toc() << "\n";
    }

    if ( flag ){
        extractMeshes( mpCurrentKF );
    }

    if ( pSetting->displayImmediateResult )
    {
        onView = true ;

        static cv::Mat cur_img_show ;
        cv::pyrDown(curImg, cur_img_show, cv::Size(curImg.cols/2, curImg.rows/2) ) ;
        cv::imshow("Current Keyframe", cur_img_show) ;

        static cv::Mat timeImg = cur_img_show.clone() ;
        timeImg.setTo( cv::Scalar(0,0,0) ) ;
        static int cntLoopImg = 0 ;

        if ( cntLoopImg > 0 ){
            cntLoopImg-- ;
        }
        {
            std::unique_lock<mutex> lock(mutexLoopImg) ;
            if ( loopImgFlag == true ){
                cv::imshow("Loop-Img", loopImg ) ;
                cntLoopImg = 10;
                //cv::moveWindow("Current Keyframe", 0, 700) ;
            }
            else {
                if ( cntLoopImg < 1 ){
                    cv::imshow("Loop-Img", timeImg ) ;
                }
            }
        }

        if ( pSetting->SHOW_TRACK == 0 )
        {
            cv::waitKey(pSetting->cvWaitTime) ;
        }
        onView = false ;
    }

    mpCurrentKF->depth.release();
    mpCurrentKF->img_right.release();

    //set viewing info
    setViewingInfo();

    //set the erase state of KFs
    {
        std::unique_lock<mutex> lock(pSystem->mpLocalMapper->mMutexNotErasedKFs) ;

        for( KeyFrame* pKF: curFrame.neighboringKFs )
        {
             std::multiset<KeyFrame*>::iterator kfIter = pSystem->mpLocalMapper->NotErasedKFs.find( pKF ) ;
             if ( kfIter == pSystem->mpLocalMapper->NotErasedKFs.end() ){
                 ROS_WARN("can not find KF, error in NotErasedKFs") ;
             }
             else{
                 pSystem->mpLocalMapper->NotErasedKFs.erase(kfIter) ;
             }
        }

        std::multiset<KeyFrame*>::iterator kfIter = pSystem->mpLocalMapper->NotErasedKFs.find( mpCurrentKF ) ;
        if ( kfIter == pSystem->mpLocalMapper->NotErasedKFs.end() ){
            ROS_WARN("can not find KF, error in NotErasedKFs") ;
        }
        else{
            pSystem->mpLocalMapper->NotErasedKFs.erase(kfIter) ;
        }
    }
}


void LocalMeshing::setViewingInfo()
{
    if ( pSetting->bDisplayViewer == 0 ){
        return ;
    }

    std::unique_lock<std::mutex> lk(pSystem->mpMapDrawer->mMutexLocalDenseMap) ;
    pSystem->mpMapDrawer->marker = marker ;
    pSystem->mpMapDrawer->localChunkBoxes = localChunkBoxes;

    pSystem->mpMapDrawer->neigborhoodKF_T.clear();
    pSystem->mpMapDrawer->neigborhoodKF_ids.clear();
    for( KeyFrame* pKF: curFrame.neighboringKFs )
    {
        pSystem->mpMapDrawer->neigborhoodKF_T.push_back(
                    pKF->GetTransformation().transpose()) ;
        pSystem->mpMapDrawer->neigborhoodKF_ids.insert(pKF->mnId) ;
    }
}

void LocalMeshing::Run()
{
    puts("start [Local Meshing]") ;
    mbFinished =false;

    while( ros::ok() )
    {
        // Check if there are keyframes in the queue
        if( !mlpKeyFrameQueue.empty() ){
            Update();
        }

        if( mbFinished ){
            break;
        }

        usleep(1000);
    }
    mbFinished = true ;
}

} //namespace LT_SLAM
