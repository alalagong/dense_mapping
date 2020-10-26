#include<chrono>
#include<ros/ros.h>
#include<rosbag/bag.h>
#include<rosbag/chunked_file.h>
#include<rosbag/view.h>
#include<rosbag/query.h>
#include"src/System.h"
#include <stdio.h>
#include <X11/Xlib.h>
#include <boost/filesystem.hpp>
#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<memory>
#include<functional>
#include<list>
#include<vector>
#include"src/libelas_omp/elas.h"

using namespace std;
using namespace LT_SLAM;

#define BACKWARD_HAS_DW 1
#include <backward.hpp>
namespace backward
{
backward::SignalHandling sh;
} // namespace backward

FColorMap colorMap = FColorMap(64) ;

int sceen_w = 1241 ;
int sceen_h = 376 ;

bool initViewer = false;

SGM2 sgm2 ;
visualization_msgs::Marker marker;
vector<geometry_msgs::Point> localChunkBoxes;

bool onView = false ;

chisel::ChiselPtr chiselMap;
chisel::ProjectionIntegrator projectionIntegrator;

int numChannels = 1 ;
int SADWindowSize = 13 ;
bool initColorImage = false ;
std::shared_ptr<chisel::ColorImage<ColorData> > ColorImage;
std::shared_ptr<chisel::DepthImage<DepthData> > DepthImage;
chisel::PinholeCamera cameraModel, colorModel, depthModel;
chisel::Intrinsics intrinsics;

void GetCurrentOpenGLCameraMatrix(Matrix3d R, Vector3d t, pangolin::OpenGlMatrix &M)
{
    M.m[0] = R(0,0);
    M.m[1] = R(1,0);
    M.m[2] = R(2,0);
    M.m[3]  = 0.0;

    M.m[4] = R(0,1);
    M.m[5] = R(1,1);
    M.m[6] = R(2,1);
    M.m[7]  = 0.0;

    M.m[8] = R(0,2);
    M.m[9] = R(1,2);
    M.m[10] = R(2,2);
    M.m[11]  = 0.0;

    M.m[12] = t(0);
    M.m[13] = t(1);
    M.m[14] = t(2);
    M.m[15]  = 1.0;
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

void readLidarData(string currFilenameBinary, cv::Mat& depth, int seq_id )
{
    // allocate 4 MB buffer (only ~130*4*4 KB are needed)
    int32_t num = 1000000;
    float lidar_data[1000000] ;

    // pointers
    float *px = lidar_data+0;
    float *py = lidar_data+1;
    float *pz = lidar_data+2;
    float *pr = lidar_data+3;

    // load point cloud
    FILE *stream;
    stream = std::fopen(currFilenameBinary.c_str(),"rb");
    num = std::fread(lidar_data,sizeof(float),num,stream)/4;
    std::vector<float> x(num) ;
    std::vector<float> y(num) ;
    std::vector<float> z(num) ;
    for (int32_t i=0; i<num; i++)
    {
        x[i] = *px ;
        y[i] = *py ;
        z[i] = *pz ;
        //point_cloud.points.push_back(tPoint(*px,*py,*pz,*pr));
        px+=4; py+=4; pz+=4; pr+=4;
    }
    fclose(stream);

    Eigen::MatrixXd Tr(4, 4);
    Eigen::Matrix3d R ;
    Eigen::Vector3d T ;


    if ( seq_id <= 2 )
    {
        //00-02
        Tr << 4.276802385584e-04, -9.999672484946e-01, -8.084491683471e-03, -1.198459927713e-02,
                -7.210626507497e-03, 8.081198471645e-03, -9.999413164504e-01, -5.403984729748e-02,
                9.999738645903e-01, 4.859485810390e-04, -7.206933692422e-03, -2.921968648686e-01,
                0, 0, 0, 1 ;
    }
    else if ( seq_id <= 3 )
    {
        //03
        Tr << 2.347736981471e-04, -9.999441545438e-01, -1.056347781105e-02, -2.796816941295e-03,
                1.044940741659e-02, 1.056535364138e-02, -9.998895741176e-01, -7.510879138296e-02,
                9.999453885620e-01, 1.243653783865e-04, 1.045130299567e-02, -2.721327964059e-01,
                0, 0, 0, 1 ;
    }
    else
    {
        //04-10
        Tr << -1.857739385241e-03, -9.999659513510e-01, -8.039975204516e-03, -4.784029760483e-03,
                -6.481465826011e-03, 8.051860151134e-03, -9.999466081774e-01, -7.337429464231e-02,
                9.999773098287e-01, -1.805528627661e-03, -6.496203536139e-03, -3.339968064433e-01,
                0, 0, 0, 1 ;
    }

    R = Tr.block(0, 0, 3, 3) ;
    T = Tr.block(0, 3, 3, 1) ;
    Eigen::Vector3d point, point2 ;
    depth = cv::Mat(sceen_h, sceen_w, CV_32F ) ;
    depth.setTo( 30.0 );

    int cnt = 0 ;
    double sum_depth = 0 ;
    for( int i = 0; i < num ; i++ )
    {
        point << x[i], y[i], z[i];
        point2 = R*point+T ;
        int u = point2(0)/point2(2)*pSetting->fxDepthMap+pSetting->cxDepthMap + 0.5 ;
        int v = point2(1)/point2(2)*pSetting->fyDepthMap+pSetting->cyDepthMap + 0.5 ;
        if ( u >= sceen_w || u < 0 || v >= sceen_h || v < 0 ){
            continue ;
        }
        if ( point2(2) < 0.1 ){
            continue ;
        }
        depth.at<float>(v, u) = point2(2);
        //printf("%f ", depth.at<float>(v, u) ) ;
        cnt++ ;
        sum_depth += point2(2) ;
    }
    //printf("\n average depth = %lf\n", sum_depth/cnt ) ;
}

struct keyFrameInfo
{
    int id ;
    float time ;
    double R[3][3] ;
    double t[3] ;
    //    double T[4][4] ;
};

inline float convert2realDepth( float z_far, float z_near, float z_buffer )
{
    float up = 2*z_far*z_near ;
    float down = (z_far+z_near-(z_far-z_near)*(2*z_buffer-1) ) ;
    return up/down ;
}


#define EVALUATE_K 10
#define DIV 10
#define COMPARE_THRESHOLD 15

void FillMarkerTopicWithMeshesLocal()
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
        }
    }


    //    localVoxels.clear();
    //    geometry_msgs::Point pt;

    //    const std::vector<chisel::Vec3, Eigen::aligned_allocator<chisel::Vec3> >&
    //            centroids = chiselMap->GetChunkManager().GetCentroids();
    //    for (const std::pair<chisel::ChunkID, chisel::ChunkPtr>& chunk : chiselMap->GetChunkManager().chunks)
    //    {
    //        chisel::ChunkPtr pChunk = chunk.second;
    //        for( int id = 0, sz = pSetting->chunkSizeX*pSetting->chunkSizeY*pSetting->chunkSizeZ ;
    //             id < sz; id++ )
    //        {
    //            chisel::DistVoxel& distVoxel = pChunk->GetDistVoxelMutable(id);
    //            if ( distVoxel.GetWeight() > 1.0e-5 )
    //            {
    //                const chisel::Vec3& centroid = centroids[id] + pChunk->GetOrigin();

    //                pt.x = centroid[0];
    //                pt.y = centroid[1];
    //                pt.z = centroid[2];
    //                localVoxels.push_back(pt);
    //            }
    //        }
    //    }
    //printf("localVoxels.sz = %d\n", localVoxels.size() ) ;
}

void extractMeshes(Vector3d t_k_2_0)
{
    const chisel::MeshMap& meshMap = chiselMap->GetChunkManager().GetAllMeshes();

    if( meshMap.size() == 0){
        return;
    }

    Eigen::Vector3f currentTGlobal = t_k_2_0.cast<float>();


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

void updateModel(cv::Mat& mImGray, cv::Mat& imGrayRight,
                 cv::Mat& curDepth,
                 Matrix3d R_k_2_0, Vector3d t_k_2_0 )
{

    //printf("%d %d\n", left.rows, left.cols ) ;

    cv::Mat curImg = mImGray ;

    if ( initColorImage == false )
    {
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
                                              //                                              new chisel::InverseTruncator(pSetting->truncationDistScale, pSetting->baseline, pSetting->fx)));
                                              new chisel::QuadraticTruncator(truncation(0),
                                                                             truncation(1),
                                                                             truncation(2),
                                                                             truncation(3))));

        projectionIntegrator.SetWeighter(chisel::WeighterPtr(new chisel::ConstantWeighter(pSetting->weight)));
        projectionIntegrator.SetCarvingDist(pSetting->carvingDist);
        projectionIntegrator.SetCarvingEnabled(pSetting->useCarving>0);

        ColorImage.reset(new chisel::ColorImage<ColorData>(
                             curImg.cols, curImg.rows, numChannels));
        DepthImage.reset(new chisel::DepthImage<DepthData>(curImg.cols, curImg.rows));

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

        sgm2.init_disparity_method(10, 100);
        initColorImage = true;
    }

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
    transform.translation() = t_k_2_0.cast<float>();
    transform.linear() = R_k_2_0.cast<float>();

    double t = (double)cvGetTickCount() ;
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

    bool flag = false;
    {
        //ROS_WARN("update meshes") ;
        double t = (double)cvGetTickCount() ;
        flag = chiselMap->UpdateMeshes( pSetting->updateMeshInterval );
        if ( flag ){
            ROS_WARN("update Meshes TIME %lf", ((double)cvGetTickCount() - t) / (cvGetTickFrequency() * 1000));
        }
    }

    FillMarkerTopicWithMeshesLocal();
    ROS_WARN("marker.points=%d chunks=%d", marker.points.size(), localChunkBoxes.size() ) ;

    extractMeshes( t_k_2_0 );
}

pangolin::OpenGlMatrix getModelViewMatrix( Eigen::Matrix3f& currRot, Eigen::Vector3f& currPose )
{
    Eigen::Quaternionf currQuat(currRot);
    Eigen::Vector3f forwardVector(0, 0, 1);
    Eigen::Vector3f upVector(0, -1, 0);

    Eigen::Vector3f forward = (currQuat * forwardVector).normalized();
    Eigen::Vector3f up = (currQuat * upVector).normalized();

    Eigen::Vector3f eye(currPose(0), currPose(1), currPose(2));
    Eigen::Vector3f at = eye + forward;

    Eigen::Vector3f z = (eye - at).normalized();  // Forward
    //Eigen::Vector3f z = (forward).normalized();  // Forward
    Eigen::Vector3f x = up.cross(z).normalized(); // Right
    Eigen::Vector3f y = z.cross(x);

    Eigen::Matrix4d m;
    m << x(0),  x(1),  x(2),  -(x.dot(eye)),
            y(0),  y(1),  y(2),  -(y.dot(eye)),
            z(0),  z(1),  z(2),  -(z.dot(eye)),
            0,     0,     0,              1;

    pangolin::OpenGlMatrix mv;
    memcpy(&mv.m[0], m.data(), sizeof(Eigen::Matrix4d));

    return mv;
}

void DrawMeshesLocal()
{
    if ( marker.points.empty() ){
        return ;
    }

    glLineWidth(pSetting->mMeshLineWidth);
    glShadeModel(GL_SMOOTH);
    //glColor4f(0.8f,0.8f,0.0f, 0.1f);
    glBegin(GL_TRIANGLES);

    for(size_t i=0, sz = marker.points.size(); i<sz; i++)
    {
        glColor4f(marker.colors[i].r, marker.colors[i].g, marker.colors[i].b, 1.0f);
        glVertex3f(marker.points[i].x, marker.points[i].y, marker.points[i].z);
        //        color = computePubPointAndColor(tris[i].p1.y-y_base) ;
        //        glColor4f(color[0]/255.0, color[1]/255.0, color[2]/255.0, 1.0f);
        //        glVertex3f(tris[i].p1.x, tris[i].p1.y, tris[i].p1.z);

        //        color = computePubPointAndColor(tris[i].p2.y-y_base) ;
        //        glColor4f(color[0]/255.0, color[1]/255.0, color[2]/255.0, 1.0f);
        //        glVertex3f(tris[i].p2.x, tris[i].p2.y, tris[i].p2.z);
    }

    glEnd();
}

inline void drawFace(geometry_msgs::Point pt[8], int i, int j, int m, int n )
{
    glVertex3f(pt[i].x, pt[i].y, pt[i].z);
    glVertex3f(pt[j].x, pt[j].y, pt[j].z);
    glVertex3f(pt[m].x, pt[m].y, pt[m].z);
    glVertex3f(pt[n].x, pt[n].y, pt[n].z);
}

void DrawChunkBoxes( float chunkSizeX, float chunkSizeY, float chunkSizeZ, float chunkResolution, float currentHeight)
{
    //    chunkSizeX /= 2 ;
    //    chunkSizeY /= 2 ;
    //    chunkSizeZ /= 2 ;
    //    chunkSizeX *= chunkResolution ;
    //    chunkSizeY *= chunkResolution ;
    //    chunkSizeZ *= chunkResolution ;

    glBegin(GL_QUADS);
    for(const geometry_msgs::Point& p: localChunkBoxes )
    {
        geometry_msgs::Point pt[8] ;
        int ith = 0 ;
        for( int i=-1; i <= 1; i += 2 )
        {
            for( int j=-1; j <= 1; j += 2 )
            {
                for( int k=-1; k <= 1; k += 2 )
                {
                    //                    pt[ith].x = p.x + i*chunkSizeX ;
                    //                    pt[ith].y = p.y + j*chunkSizeY ;
                    //                    pt[ith].z = p.z + k*chunkSizeZ ;
                    pt[ith].x = p.x + i*chunkResolution ;
                    pt[ith].y = p.y + j*chunkResolution ;
                    pt[ith].z = p.z + k*chunkResolution ;
                    ith++ ;
                }
            }
        }
        float d = p.y - currentHeight ;
        //d += 4 ;
        if ( d < -10 ){
            d = -10 ;
        }
        if ( d > 10 ){
            d = 10 ;
        }
        int i0 = (d+10)/20*62 ;
        cv::Vec3b c = colorMap.at(i0) ;
        glColor3f(c.val[0]/255.0, c.val[1]/255.0, c.val[2]/255.0);

        drawFace(pt, 0, 1, 3, 2);
        drawFace(pt, 0, 2, 6, 4);
        drawFace(pt, 1, 3, 7, 5);
        drawFace(pt, 5, 7, 6, 4);
        drawFace(pt, 3, 2, 6, 7);
        drawFace(pt, 0, 1, 5, 4);
    }
    glEnd();
}

void test(Matrix3d R, Vector3d t, cv::Mat& depthImg )
{
    float z_far = 50 ;
    float z_near = 0.01 ;

    if ( initViewer == false )
    {
        pangolin::CreateWindowAndBind("Map Viewer", sceen_w, sceen_h);


        // 3D Mouse handler requires depth testing to be enabled
        glEnable(GL_DEPTH_TEST);
        // Issue specific OpenGl we might need
        glEnable (GL_BLEND);
        glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        initViewer = true ;
    }

    static pangolin::OpenGlRenderState s_cam = pangolin::OpenGlRenderState(
                pangolin::ProjectionMatrix(sceen_w, sceen_h, pSetting->fxDepthMap,
                                           pSetting->fyDepthMap,
                                           pSetting->cxDepthMap,
                                           pSetting->cyDepthMap,
                                           z_near,
                                           z_far)
                );
    // Add named OpenGL viewport to window and provide 3D Handler
    static pangolin::View& d_cam = pangolin::CreateDisplay()
            //.SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -(float)sceen_w/sceen_h)
            .SetBounds(0.0, 1.0, 0, 1.0 )
            .SetHandler(new pangolin::Handler3D(s_cam));


    // Define Camera Render Object (for view / scene browsing)
    //    pangolin::OpenGlMatrix Twc;
    //    Twc.SetIdentity();


    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    Matrix3f in_R = R.cast<float>();
    Vector3f in_t = t.cast<float>();
    pangolin::OpenGlMatrix mv = getModelViewMatrix(in_R, in_t) ;

    //    cout << mv << "\n" ;
    //    mv.SetIdentity();
    s_cam.SetModelViewMatrix(mv);
    d_cam.Activate(s_cam);

    glClearColor(0.9f,0.9f,0.9f,1.0f);

    DrawMeshesLocal();
    //DrawChunkBoxes(pSetting->chunkSizeX,pSetting->chunkSizeY,pSetting->chunkSizeZ, pSetting->voxelResolution, in_t(1) );

    pangolin::FinishFrame();

    GLfloat* data = new GLfloat[sceen_w*sceen_h] ;
    glReadPixels(0, 0, sceen_w, sceen_h, GL_DEPTH_COMPONENT, GL_FLOAT, data) ;

    depthImg = cv::Mat(sceen_h, sceen_w, CV_32F ) ;

    for( int i = sceen_h-1; i >= 0 ; i--)
    {
        for( int j = 0; j < sceen_w; j++ ){
            depthImg.at<float>(sceen_h-1-i, j) = convert2realDepth(z_far, z_near, data[i*sceen_w+j]) ;
        }
    }
}


void display_depth(cv::Mat depth, string name, int threshold, bool enlarge=false)
{
    cv::Mat disp_depth = depth.clone() ;
    for( int i = 0 ; i < disp_depth.rows; i++ )
    {
        for( int j=0; j < disp_depth.cols ; j++ )
        {
            if ( depth.at<float>(i, j) < 0.1 ) {
                continue ;
            }
            if ( depth.at<float>(i, j) > threshold ){
                disp_depth.at<float>(i, j) = threshold ;
                continue ;
            }
            if ( enlarge )
            {
                int x , y ;
                for( int dx = -2; dx <= 2 ; dx++ )
                {
                    for( int dy = -2; dy <= 2 ; dy++ )
                    {
                        x = i+dx ;
                        y = j+dy ;
                        if ( x< 0 || x >= disp_depth.rows || y < 0 || y >= disp_depth.cols ){
                            continue ;
                        }
                        disp_depth.at<float>(x, y) = depth.at<float>(i, j) ;
                    }
                }
            }
        }
    }

    disp_depth = disp_depth/threshold ;
    disp_depth = disp_depth*255 ;
    disp_depth.convertTo(disp_depth, CV_8U);

    //cv::normalize(disp_depth, disp_depth, 0, 255, CV_MINMAX, CV_8U);
    cv::applyColorMap(disp_depth, disp_depth, cv::COLORMAP_RAINBOW) ;
    for( int i = 0 ; i < disp_depth.rows; i++ )
    {
        for( int j=0; j < disp_depth.cols ; j++ )
        {
            if ( disp_depth.at<cv::Vec3b>(i, j)[0] == 0 &&
                 disp_depth.at<cv::Vec3b>(i, j)[1] == 0 &&
                 disp_depth.at<cv::Vec3b>(i, j)[2] == 255 ){
                disp_depth.at<cv::Vec3b>(i, j)[0] = 0 ;
                disp_depth.at<cv::Vec3b>(i, j)[1] = 0 ;
                disp_depth.at<cv::Vec3b>(i, j)[2] = 0 ;
            }
        }
    }


    cv::imshow(name, disp_depth ) ;
}

int compare_depth(cv::Mat gt_depth, cv::Mat test_depth, int* depth_diff, int* depth_safe, int* outlier )
{
    int height = test_depth.rows;
    int width = test_depth.cols;
    int sum = 0 ;
    for( int i = height-1; i >= 0 ; i--)
    {
        for( int j = 0; j < width; j++ )
        {
            if ( gt_depth.at<float>(i, j) < 0.1 ){
                continue ;
            }

            if ( test_depth.at<float>(i, j) < 0.1 ){
                continue ;
            }

            if ( gt_depth.at<float>(i, j) > COMPARE_THRESHOLD ){
                continue ;
            }

            if ( test_depth.at<float>(i, j) > COMPARE_THRESHOLD ){
                continue ;
            }

            sum++ ;
            for ( int k = 1 ; k <= EVALUATE_K; k++ )
            {

                if ( fabs(test_depth.at<float>(i, j)-gt_depth.at<float>(i, j)) < float(k)/DIV+0.05 ){
                    depth_diff[k-1]++ ;
                }
                if ( test_depth.at<float>(i, j) - float(k)/DIV < gt_depth.at<float>(i, j)+0.05 ){
                    depth_safe[k-1]++;
                }
                if ( test_depth.at<float>(i, j) + (float)k < gt_depth.at<float>(i, j) ){
                    outlier[k-1]++ ;
                }
            }

        }
    }

    return sum ;
}

void setR_t(Matrix3d& R_k_2_0, Vector3d& t_k_2_0, keyFrameInfo& tmp)
{
    for( int i = 0 ; i < 3; i++ )
    {
        t_k_2_0(i) = tmp.t[i] ;
        for( int j=0; j < 3 ; j++ ){
            R_k_2_0(i, j) = tmp.R[i][j] ;
        }
    }
}

void initRawDepthMap(cv::Mat& mImGray, cv::Mat& imGrayRight, cv::Mat& depth)
{
    for( int i = 0 ; i < pSetting->downSampleTimes; i++ )
    {
        cv::pyrDown(mImGray, mImGray, cv::Size(mImGray.cols/2, mImGray.rows/2) ) ;
        cv::pyrDown(imGrayRight, imGrayRight, cv::Size(imGrayRight.cols/2, imGrayRight.rows/2) ) ;
    }

    cv::Mat curImg = mImGray;

    int rows = curImg.rows ;
    int cols = curImg.cols ;
    float time = 0.0 ;
    cv::Mat disparity;
    if ( ((rows%4) == 0) && ((cols%4) == 0) )
    {
        cv::Mat left = mImGray.clone() ;
        cv::Mat right = imGrayRight.clone() ;
        cv::Mat output = sgm2.compute_disparity_method(left, right, &time) ;

        if ( false )
        {
            //BilateralFilter
            cv::Ptr<cv::cuda::DisparityBilateralFilter> bFilter =
                    cv::cuda::createDisparityBilateralFilter(128, 24, 5);
            static cv::cuda::GpuMat d_disp(output.size(), CV_8U);
            static cv::cuda::GpuMat im_gpu(left.size(), CV_8U);
            static cv::cuda::GpuMat refine_disp(left.size(), CV_8U);
            d_disp.upload(output);
            im_gpu.upload(left);
            bFilter->apply(d_disp, im_gpu, refine_disp );
            refine_disp.download(output);
        }

        output.convertTo(disparity, CV_32F );
    }
    else
    {
        cv::Mat left = mImGray.rowRange(0, rows-(rows%4)).colRange(0, cols-(cols%4)).clone() ;
        cv::Mat right = imGrayRight.rowRange(0, rows-(rows%4)).colRange(0, cols-(cols%4)).clone();
        cv::Mat output = sgm2.compute_disparity_method(left, right, &time) ;

        if ( false )
        {
            cv::Ptr<cv::cuda::DisparityBilateralFilter> bFilter =
                    cv::cuda::createDisparityBilateralFilter(128, 24, 5);
            static cv::cuda::GpuMat d_disp(output.size(), CV_8U);
            static cv::cuda::GpuMat im_gpu(left.size(), CV_8U);
            static cv::cuda::GpuMat refine_disp(left.size(), CV_8U);
            d_disp.upload(output);
            im_gpu.upload(left);
            bFilter->apply(d_disp, im_gpu, refine_disp );
            refine_disp.download(output);
        }

        output.convertTo(output, CV_32F ) ;

        disparity.create(rows, cols, CV_32F );
        disparity.setTo(0.0) ;
        for( int i = rows-(rows%4)-1 ; i >= 0 ; i-- )
        {
            for( int j = cols-(cols%4)-1; j >= 0 ; j-- ){
                disparity.at<float>(i, j) = output.at<float>(i, j) ;
            }
        }
    }





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

    calculateDepthImage(disparity, depth, pSetting->bfDepthMap );
    //ROS_WARN("SGM Time = %f", tc_sgm.toc() ) ;
}

int main(int argc, char **argv)
{
    bool enableELAS = false ;

    XInitThreads();
    ros::init(argc, argv, "lt_slam");

    ros::NodeHandle nh("~") ;
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Warn);

    string packagePath = ros::package::getPath("lt_slam");
    string posePath = "/home/ygling2008/" ;
    string dictPath = packagePath + "//Vocabulary//ORBvoc.bin" ;
    string configPath;
    size_t scan_num ;
    int seq_id = 5;

    if ( argc < 2 ){
        configPath = packagePath + "/config/KITTI05.yaml";
        posePath = posePath + "KITTI05.txt" ;
    }
    else {
        configPath = packagePath + "/config/KITTI" + argv[1] + ".yaml";
        posePath = posePath + "KITTI" + argv[1] + ".txt" ;

        if ( argv[1][1] == '1' || argv[1][1] == '2' || argv[1][1] == '9' ){
            scan_num = 12 ;
        }
        else {
            scan_num = 13 ;
        }

        seq_id = (argv[1][0] - '0')*10 + argv[1][1] - '0' ;
    }

    ROS_WARN("seq_id=%d", seq_id) ;
    if ( seq_id <= 2 )
    {
        //00-02
        sceen_w = 1241 ;
        sceen_h = 376 ;
    }
    else if ( seq_id <= 3 )
    {
        //03
        sceen_w = 1242 ;
        sceen_h = 375 ;
    }
    else
    {
        //04-10
        sceen_w = 1226 ;
        sceen_h = 370 ;
    }


    pSetting = new Setting(configPath) ;
    for( int i=0; i < pSetting->downSampleTimes; i++ )
    {
        sceen_w = sceen_w/2 ;
        sceen_h = sceen_h/2 ;
    }

    //printf("w=%d h=%d\n", sceen_w, sceen_h ) ;

    //    pSetting->bMeshing = 1 ;
    //    pSetting->downSampleTimes = 0 ;

    ROS_WARN("scan_num=%d", scan_num ) ;
    ROS_WARN("posePath=%s", posePath.c_str() ) ;

    cv::FileStorage fSettings(configPath, cv::FileStorage::READ);
    string lidarPath = fSettings["lidarPath"] ;
    string bagPath = fSettings["bagPath"];
    string gtPoseFile = fSettings["gtPoseFile"] ;
    fSettings.release();

    keyFrameInfo tmp ;
    vector<keyFrameInfo> KFList, KFList_GT ;
    FILE* fp ;

    fp = std::fopen(posePath.c_str(), "r");
    if(!fp) {
        printf("File opening failed");
        return EXIT_FAILURE;
    }
    for(  int k = -1 ; ros::ok(); )
    {
        if ( scan_num == 12 )
        {
            k++ ;
            int tt = fscanf(fp, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
                            &tmp.R[0][0], &tmp.R[0][1], &tmp.R[0][2], &tmp.t[0],
                    &tmp.R[1][0], &tmp.R[1][1], &tmp.R[1][2], &tmp.t[1],
                    &tmp.R[2][0], &tmp.R[2][1], &tmp.R[2][2], &tmp.t[2] ) ;
            if ( tt != scan_num )
            {
                printf("tt = %d\n", tt ) ;
                break ;
            }
        }
        else
        {
            int tt = fscanf(fp, "%d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf", &k,
                            &tmp.R[0][0], &tmp.R[0][1], &tmp.R[0][2], &tmp.t[0],
                    &tmp.R[1][0], &tmp.R[1][1], &tmp.R[1][2], &tmp.t[1],
                    &tmp.R[2][0], &tmp.R[2][1], &tmp.R[2][2], &tmp.t[2] ) ;

            if ( tt != scan_num )
            {
                printf("tt = %d\n", tt ) ;
                break ;
            }
        }
        tmp.id = k ;
        KFList.push_back(tmp);
    }
    std::fclose(fp) ;
    ROS_WARN("KFList.size() =%d", KFList.size() ) ;

    fp = std::fopen(gtPoseFile.c_str(), "r");
    if(!fp) {
        printf("File opening failed");
        return EXIT_FAILURE;
    }
    scan_num = 12 ;
    for(int k = -1 ; ros::ok(); )
    {
        k++ ;
        int tt = fscanf(fp, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
                        &tmp.R[0][0], &tmp.R[0][1], &tmp.R[0][2], &tmp.t[0],
                &tmp.R[1][0], &tmp.R[1][1], &tmp.R[1][2], &tmp.t[1],
                &tmp.R[2][0], &tmp.R[2][1], &tmp.R[2][2], &tmp.t[2] ) ;
        if ( tt != scan_num )
        {
            printf("tt = %d\n", tt ) ;
            break ;
        }
        tmp.id = k ;
        KFList_GT.push_back(tmp);
    }
    std::fclose(fp) ;
    ROS_WARN("KFList_GT.size() =%d", KFList_GT.size() ) ;

    //    int total_diff[EVALUATE_K] ;
    //    int total_diff_safe[EVALUATE_K] ;
    //    int total_depth[EVALUATE_K] ;
    //    int total_depth_safe[EVALUATE_K] ;
    //    memset(total_diff, 0, sizeof(total_diff) ) ;
    //    memset(total_diff_safe, 0, sizeof(total_diff_safe) ) ;
    //    memset(total_depth, 0, sizeof(total_depth) ) ;
    //    memset(total_depth_safe, 0, sizeof(total_depth_safe) ) ;

    //    int sceen_w = 1226 ;
    //    int sceen_h = 370 ;

    int totalImg = KFList_GT.size() ;
    char filePath[1024] ;
    cv::Mat depth_lidar ;
    cv::Mat imLeft, imRight;
    cv::Mat depth ;
    cv::Mat depth_render ;
    cv::Mat depth_REMODE ;

    //depth origin
    int origin_total_sum = 0;
    int origin_s_diff[EVALUATE_K] ;
    int origin_s_safe[EVALUATE_K] ;
    int origin_s_outlier[EVALUATE_K] ;
    int origin_depth_diff[EVALUATE_K] ;
    int origin_depth_safe[EVALUATE_K] ;
    int origin_outlier[EVALUATE_K] ;
    memset(origin_s_diff, 0, sizeof(origin_s_diff) ) ;
    memset(origin_s_safe, 0, sizeof(origin_s_safe) ) ;
    memset(origin_s_outlier, 0, sizeof(origin_s_outlier) ) ;

    //depth fusion
    int fusion_total_sum = 0;
    int fusion_s_diff[EVALUATE_K] ;
    int fusion_s_safe[EVALUATE_K] ;
    int fusion_s_outlier[EVALUATE_K] ;
    int fusion_depth_diff[EVALUATE_K] ;
    int fusion_depth_safe[EVALUATE_K] ;
    int fusion_outlier[EVALUATE_K] ;
    memset(fusion_s_diff, 0, sizeof(fusion_s_diff) ) ;
    memset(fusion_s_safe, 0, sizeof(fusion_s_safe) ) ;
    memset(fusion_s_outlier, 0, sizeof(fusion_s_outlier) ) ;

    //depth REMODE
    int REMODE_total_sum = 0;
    int REMODE_s_diff[EVALUATE_K] ;
    int REMODE_s_safe[EVALUATE_K] ;
    int REMODE_s_outlier[EVALUATE_K] ;
    int REMODE_depth_diff[EVALUATE_K] ;
    int REMODE_depth_safe[EVALUATE_K] ;
    int REMODE_outlier[EVALUATE_K] ;
    memset(REMODE_s_diff, 0, sizeof(REMODE_s_diff) ) ;
    memset(REMODE_s_safe, 0, sizeof(REMODE_s_safe) ) ;
    memset(REMODE_s_outlier, 0, sizeof(REMODE_s_outlier) ) ;

    Matrix3d last_R_k_2_0 ;
    Vector3d last_t_k_2_0 ;
    Matrix3d R_k_2_0 ;
    Vector3d t_k_2_0 ;
    Matrix3d detla_R ;
    Vector3d detla_t ;
    cv::Mat last_depth ;

    const int32_t dims[3] = {sceen_w,sceen_h,sceen_w}; // bytes per line = width

    cv::Mat D1(sceen_h, sceen_w, CV_32F ) ;
    cv::Mat D2(sceen_h, sceen_w, CV_32F ) ;

    Elas::parameters param;
    param.postprocess_only_left = false;
    Elas elas(param);

    for( int ith = 0; ith < totalImg && ros::ok(); ith++ )
    {
        tmp = KFList_GT[ith] ;

        printf("ith=%d\n", ith ) ;

        sprintf(filePath, "%s//image_0//%06d.png",  bagPath.c_str(), tmp.id ) ;
        imLeft = cv::imread(filePath, CV_LOAD_IMAGE_GRAYSCALE );
        sprintf(filePath, "%s//image_1//%06d.png",  bagPath.c_str(), tmp.id ) ;
        imRight = cv::imread(filePath, CV_LOAD_IMAGE_GRAYSCALE );

        //printf("n=%d m=%d\n", imLeft.cols, imLeft.rows ) ;

        char filename[256] ;
        sprintf(filename, "%06d.bin", tmp.id ) ;
        readLidarData(lidarPath+filename, depth_lidar, seq_id) ;

        setR_t(R_k_2_0, t_k_2_0, tmp );

        initRawDepthMap(imLeft, imRight, depth);


        //printf("%d %d\n", imLeft.rows, imLeft.cols ) ;

        //elas depth map
        if ( enableELAS )
        {
            elas.process(imLeft.data, imRight.data, (float*)D1.data, (float*)D2.data, dims);
            calculateDepthImage(D1, depth_REMODE, pSetting->bfDepthMap );
        }


        if ( ith > 1 )
        {
            //            detla_R ;
            //            detla_t ;
        }

        updateModel(imLeft, imRight, depth, R_k_2_0, t_k_2_0);

        //        pangolin::OpenGlMatrix Twc ;
        //        GetCurrentOpenGLCameraMatrix(R_k_2_0, t_k_2_0, Twc) ;
        //        d_cam.Activate(s_cam);

        test(R_k_2_0, t_k_2_0, depth_render) ;

        {
            //cv::imshow("imLeft", imLeft) ;
            display_depth(depth_render, "depth_render", pSetting->farPlaneDist ) ;
            //display_depth(depth, "depth", pSetting->farPlaneDist ) ;
            if ( enableELAS ){
                display_depth(depth_REMODE, "depth_REMODE", pSetting->farPlaneDist ) ;
            }
            display_depth(depth_lidar, "depth_lidar", pSetting->farPlaneDist, true ) ;


            cv::imshow("Current Keyframe", imLeft) ;
        }

        char key = cv::waitKey(10);
        if ( key == 's' ){
            for( ; ; )
            {
                usleep(1000);
                key = cv::waitKey(1);
                if ( key == 's' ){
                    break ;
                }
            }
        }

        memset(origin_depth_diff, 0, sizeof(origin_depth_diff) ) ;
        memset(origin_depth_safe, 0, sizeof(origin_depth_safe) ) ;
        memset(origin_outlier, 0, sizeof(origin_outlier) ) ;
        memset(fusion_depth_diff, 0, sizeof(fusion_depth_diff) ) ;
        memset(fusion_depth_safe, 0, sizeof(fusion_depth_safe) ) ;
        memset(fusion_outlier, 0, sizeof(fusion_outlier) ) ;
        memset(REMODE_depth_diff, 0, sizeof(REMODE_depth_diff) ) ;
        memset(REMODE_depth_safe, 0, sizeof(REMODE_depth_safe) ) ;
        memset(REMODE_outlier, 0, sizeof(REMODE_outlier) ) ;

        origin_total_sum +=
                compare_depth(depth_lidar, depth, origin_depth_diff, origin_depth_safe, origin_outlier) ;

        fusion_total_sum +=
                compare_depth(depth_lidar, depth_render, fusion_depth_diff, fusion_depth_safe, fusion_outlier) ;

        REMODE_total_sum +=
                compare_depth(depth_lidar, depth_REMODE, REMODE_depth_diff, REMODE_depth_safe, REMODE_outlier) ;

        //puts("orginal disparity ratio") ;
        for( int i = 0 ; i < EVALUATE_K ; i++ ){
            //printf("%f ", (float)s_diff[i]/sum ) ;
            fusion_s_diff[i] += fusion_depth_diff[i] ;
            fusion_s_safe[i] += fusion_depth_safe[i] ;
            fusion_s_outlier[i] += fusion_outlier[i] ;
            origin_s_diff[i] += origin_depth_diff[i] ;
            origin_s_safe[i] += origin_depth_safe[i] ;
            origin_s_outlier[i] += origin_outlier[i] ;
            REMODE_s_diff[i] += REMODE_depth_diff[i] ;
            REMODE_s_safe[i] += REMODE_depth_safe[i] ;
            REMODE_s_outlier[i] += REMODE_outlier[i] ;
        }


        cv::Mat diff_Img(imLeft.rows, imLeft.cols, CV_8UC3) ;
        cv::Mat err_Img(imLeft.rows, imLeft.cols, CV_8UC3) ;
        err_Img.setTo( cv::Vec3b(128, 128, 128) ) ;

        int length = sceen_w/4.0*3 ;
        for( int i = 0; i < 40; i++ )
        {

            for( int j = 0; j < length; j++ ){
                err_Img.at<cv::Vec3b>(i, j) = colorMap.at(  float(j)/length*63 ) ;
            }

            for( int j = length; j < sceen_w; j++ ){
                err_Img.at<cv::Vec3b>(i, j) = colorMap.at(63) ;
            }
        }
        cv::line(err_Img, cv::Point2f(5, 50), cv::Point2f(5, 45), cv::Scalar(255, 255, 255), 2, 8);
        cv::putText(err_Img, "0.0", cv::Point2f(1, 70), cv::FONT_HERSHEY_SIMPLEX,
                    0.5, cv::Scalar(255, 255, 255), 2 );

        cv::line(err_Img, cv::Point2f(length, 50), cv::Point2f(length, 45), cv::Scalar(255, 255, 255), 2, 8);
        cv::putText(err_Img, "1.0", cv::Point2f(length-5, 70), cv::FONT_HERSHEY_SIMPLEX,
                    0.5, cv::Scalar(255, 255, 255), 2 );

        cv::line(err_Img, cv::Point2f(sceen_w-5, 50), cv::Point2f(sceen_w-5, 45), cv::Scalar(255, 255, 255), 2, 8);
        cv::putText(err_Img, "> 1.0", cv::Point2f((sceen_w+length)/2, 70), cv::FONT_HERSHEY_SIMPLEX,
                    0.5, cv::Scalar(255, 255, 255), 2 );

        cv::line(err_Img, cv::Point2f(5, 50), cv::Point2f(sceen_w-5, 50), cv::Scalar(255, 255, 255), 2, 8);

        for( int i = sceen_h-1; i >= 0 ; i--)
        {
            for( int j = 0; j < sceen_w; j++ )
            {
                diff_Img.at<cv::Vec3b>(i, j)[0] = 128 ;
                diff_Img.at<cv::Vec3b>(i, j)[1] = 128 ;
                diff_Img.at<cv::Vec3b>(i, j)[2] = 128 ;
                if ( depth_lidar.at<float>(i, j) < 0.1 ){
                    continue ;
                }
                if ( depth_render.at<float>(i, j) < 0.1 ){
                    continue ;
                }
                if ( depth_lidar.at<float>(i, j) > COMPARE_THRESHOLD ){
                    continue ;
                }
                if ( depth_render.at<float>(i, j) > COMPARE_THRESHOLD ){
                    continue ;
                }

                for( int ii = i-2; ii <= i+2 ; ii++ )
                {
                    for( int jj=j-2; jj <= j+2; jj++ )
                    {
                        if ( ii <= 0 || ii >= sceen_h-1 || jj <= 0 || jj >= sceen_w-1 ){
                            continue ;
                        }
                        diff_Img.at<cv::Vec3b>(ii, jj)[0] = 0 ;
                        diff_Img.at<cv::Vec3b>(ii, jj)[1] = 0 ;
                        diff_Img.at<cv::Vec3b>(ii, jj)[2] = 255 ;
                        err_Img.at<cv::Vec3b>(ii, jj) = colorMap.at(63) ;
                    }
                }

                if ( depth_render.at<float>(i, j) < depth_lidar.at<float>(i, j)+1.0 )
                {
                    for( int ii = i-2; ii <= i+2 ; ii++ )
                    {
                        for( int jj=j-2; jj <= j+2; jj++ )
                        {
                            if ( ii <= 0 || ii >= sceen_h-1 || jj <= 0 || jj >= sceen_w-1 ){
                                continue ;
                            }
                            diff_Img.at<cv::Vec3b>(ii, jj)[0] = 0 ;
                            diff_Img.at<cv::Vec3b>(ii, jj)[1] = 255 ;
                            diff_Img.at<cv::Vec3b>(ii, jj)[2] = 0 ;
                        }
                    }
                }

                for ( int k = 1 ; k <= EVALUATE_K; k++ )
                {
                    if ( fabs(depth_render.at<float>(i, j)-depth_lidar.at<float>(i, j)) < float(k)/DIV+0.1 )
                    {
                        for( int ii = i-2; ii <= i+2 ; ii++ )
                        {
                            for( int jj=j-2; jj <= j+2; jj++ )
                            {
                                if ( ii <= 0 || ii >= sceen_h-1 || jj <= 0 || jj >= sceen_w-1 ){
                                    continue ;
                                }
                                err_Img.at<cv::Vec3b>(ii, jj) = colorMap.at( (60.0/EVALUATE_K)*k );
                            }
                        }
                        break ;
                    }
                }
            }
        }
        char writeFileName[256] ;

        sprintf(writeFileName, "/home/ygling2008/graph_tmp/0_%04d.png", ith ) ;
        cv::imwrite(writeFileName, diff_Img ) ;
        cv::imshow("diff_Image", diff_Img ) ;

        sprintf(writeFileName, "/home/ygling2008/graph_tmp/1_%04d.png", ith ) ;
        cv::imwrite(writeFileName, err_Img ) ;
        cv::imshow("err_Img", err_Img) ;



        last_R_k_2_0 = R_k_2_0 ;
        last_t_k_2_0 = t_k_2_0 ;
        last_depth = depth ;
    }

    char result_path[256] ;
    sprintf(result_path, "//home//ygling2008//MapResults//%02d.txt", seq_id ) ;

    ofstream f ;
    f.open(result_path);


    puts("final fusion depth ratio") ;
    for( int i = 0 ; i < EVALUATE_K ; i++ ){
        printf("%f ", (float)fusion_s_diff[i]/fusion_total_sum ) ;
        f << setprecision(6) << (float)fusion_s_diff[i]/fusion_total_sum << " " ;
    }
    printf("\n") ;
    f << "\n" ;

    puts("final fusion safe ratio") ;
    for( int i = 0 ; i < EVALUATE_K ; i++ ){
        printf("%f ", (float)fusion_s_safe[i]/fusion_total_sum ) ;
        f << setprecision(6) << (float)fusion_s_safe[i]/fusion_total_sum << " " ;
    }
    printf("\n") ;
    f << "\n" ;

    puts("final fusion outlier ratio") ;
    for( int i = 0 ; i < EVALUATE_K ; i++ ){
        printf("%f ", (float)fusion_s_outlier[i]/fusion_total_sum ) ;
        f << setprecision(6) << (float)fusion_s_outlier[i]/fusion_total_sum << " " ;
    }
    printf("\n") ;
    f << "\n" ;



    puts("final origin depth ratio") ;
    for( int i = 0 ; i < EVALUATE_K ; i++ ){
        printf("%f ", (float)origin_s_diff[i]/origin_total_sum ) ;
        f << setprecision(6) <<  (float)origin_s_diff[i]/origin_total_sum << " " ;
    }
    printf("\n") ;
    f << "\n" ;

    puts("final origin safe ratio") ;
    for( int i = 0 ; i < EVALUATE_K ; i++ ){
        printf("%f ", (float)origin_s_safe[i]/origin_total_sum ) ;
        f << setprecision(6) << (float)origin_s_safe[i]/origin_total_sum  << " " ;
    }
    f << "\n" ;

    puts("final origin outlier ratio") ;
    for( int i = 0 ; i < EVALUATE_K ; i++ ){
        printf("%f ", (float)origin_s_outlier[i]/origin_total_sum ) ;
        f << setprecision(6) << (float)origin_s_outlier[i]/origin_total_sum << " " ;
    }
    printf("\n") ;
    f << "\n" ;



    puts("final REMODE depth ratio") ;
    for( int i = 0 ; i < EVALUATE_K ; i++ ){
        printf("%f ", (float)REMODE_s_diff[i]/REMODE_total_sum ) ;
        f << setprecision(6) <<  (float)REMODE_s_diff[i]/REMODE_total_sum << " " ;
    }
    printf("\n") ;
    f << "\n" ;

    puts("final REMODE safe ratio") ;
    for( int i = 0 ; i < EVALUATE_K ; i++ ){
        printf("%f ", (float)REMODE_s_safe[i]/REMODE_total_sum ) ;
        f << setprecision(6) << (float)REMODE_s_safe[i]/REMODE_total_sum  << " " ;
    }
    printf("\n") ;
    f << "\n" ;

    puts("final REMODE outlier ratio") ;
    for( int i = 0 ; i < EVALUATE_K ; i++ ){
        printf("%f ", (float)REMODE_s_outlier[i]/REMODE_total_sum ) ;
        f << setprecision(6) << (float)REMODE_s_outlier[i]/REMODE_total_sum << " " ;
    }
    printf("\n") ;
    f << "\n" ;


    f.close();

    //std::getchar() ;

    return 0;
}
