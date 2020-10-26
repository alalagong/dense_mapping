#include "MapDrawer.h"

namespace LT_SLAM
{

FColorMap colorMap = FColorMap(64) ;

cv::Vec3b computePubPointAndColor(float d)
{
    d += 4 ;
    if ( d < 0 ){
        d = 0 ;
    }
    if ( d > 6 ){
        d = 6 ;
    }
    int i0 = d/6*62 ;
    return colorMap.at(i0) ;
}


MapDrawer::MapDrawer(System *sys){
    initPose = false ;
    pSystem = sys;
}

MapDrawer::~MapDrawer(){
    ;
}

inline void drawFace(geometry_msgs::Point pt[8], int i, int j, int m, int n )
{
    glVertex3f(pt[i].x, pt[i].y, pt[i].z);
    glVertex3f(pt[j].x, pt[j].y, pt[j].z);
    glVertex3f(pt[m].x, pt[m].y, pt[m].z);

    glVertex3f(pt[j].x, pt[j].y, pt[j].z);
    glVertex3f(pt[m].x, pt[m].y, pt[m].z);
    glVertex3f(pt[n].x, pt[n].y, pt[n].z);
}

void MapDrawer::DrawCube( float x, float y, float z, float cubicBound)
{
    float half = cubicBound ;
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
                pt[ith].x = x + i*half ;
                pt[ith].y = y + j*half ;
                pt[ith].z = z + k*half ;
                ith++ ;
            }
        }
    }
    glLineWidth(pSetting->mKeyFrameLineWidth*2);
    glColor3f(1.0f,114.0f/255,55.0f/255);
    glBegin(GL_LINES);

    int i = 0 ;
    int j = 1 ;
    glVertex3f(pt[i].x, pt[i].y, pt[i].z);
    glVertex3f(pt[j].x, pt[j].y, pt[j].z);

    i = 0 ;
    j = 2 ;
    glVertex3f(pt[i].x, pt[i].y, pt[i].z);
    glVertex3f(pt[j].x, pt[j].y, pt[j].z);

    i = 1 ;
    j = 3 ;
    glVertex3f(pt[i].x, pt[i].y, pt[i].z);
    glVertex3f(pt[j].x, pt[j].y, pt[j].z);

    i = 2 ;
    j = 3 ;
    glVertex3f(pt[i].x, pt[i].y, pt[i].z);
    glVertex3f(pt[j].x, pt[j].y, pt[j].z);

    i = 1 ;
    j = 5 ;
    glVertex3f(pt[i].x, pt[i].y, pt[i].z);
    glVertex3f(pt[j].x, pt[j].y, pt[j].z);

    i = 3 ;
    j = 7 ;
    glVertex3f(pt[i].x, pt[i].y, pt[i].z);
    glVertex3f(pt[j].x, pt[j].y, pt[j].z);

    i = 2 ;
    j = 6 ;
    glVertex3f(pt[i].x, pt[i].y, pt[i].z);
    glVertex3f(pt[j].x, pt[j].y, pt[j].z);

    i = 0 ;
    j = 4 ;
    glVertex3f(pt[i].x, pt[i].y, pt[i].z);
    glVertex3f(pt[j].x, pt[j].y, pt[j].z);

    i = 5 ;
    j = 7 ;
    glVertex3f(pt[i].x, pt[i].y, pt[i].z);
    glVertex3f(pt[j].x, pt[j].y, pt[j].z);

    i = 4 ;
    j = 5 ;
    glVertex3f(pt[i].x, pt[i].y, pt[i].z);
    glVertex3f(pt[j].x, pt[j].y, pt[j].z);

    i = 6 ;
    j = 7 ;
    glVertex3f(pt[i].x, pt[i].y, pt[i].z);
    glVertex3f(pt[j].x, pt[j].y, pt[j].z);

    i = 4 ;
    j = 6 ;
    glVertex3f(pt[i].x, pt[i].y, pt[i].z);
    glVertex3f(pt[j].x, pt[j].y, pt[j].z);

    glEnd();
}

inline void drawFaceQuad(geometry_msgs::Point pt[8], int i, int j, int m, int n )
{
    glVertex3f(pt[i].x, pt[i].y, pt[i].z);
    glVertex3f(pt[j].x, pt[j].y, pt[j].z);
    glVertex3f(pt[m].x, pt[m].y, pt[m].z);
    glVertex3f(pt[n].x, pt[n].y, pt[n].z);
}

void MapDrawer::DrawChunkBoxesDist(float chunkResolution, Eigen::Vector3f t, double threshold)
{
    unique_lock<mutex> lock(mMutexLocalDenseMap);

    //double t = (double)cvGetTickCount() ;
    glShadeModel(GL_SMOOTH);
    //glBegin(GL_TRIANGLES);
    glBegin(GL_QUADS);
    //ROS_WARN("localChunkBoxes.sz = %u", localChunkBoxes.size() ) ;
    Eigen::Vector3f pos ;
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
                    pt[ith].x = p.x + i*chunkResolution/2 ;
                    pt[ith].y = p.y + j*chunkResolution/2 ;
                    pt[ith].z = p.z + k*chunkResolution/2 ;
                    ith++ ;
                }
            }
        }
        pos << p.x, p.y, p.z ;
        float d = (pos-t).norm() ;
        if ( d > threshold ){
            d = threshold ;
        }
        int i0 = d/threshold*62 ;
        cv::Vec3b c = colorMap.at(i0) ;
        glColor3f(c.val[0]/255.0, c.val[1]/255.0, c.val[2]/255.0);

        drawFaceQuad(pt, 0, 1, 3, 2);
        drawFaceQuad(pt, 0, 2, 6, 4);
        drawFaceQuad(pt, 1, 3, 7, 5);
        drawFaceQuad(pt, 5, 7, 6, 4);
        drawFaceQuad(pt, 3, 2, 6, 7);
        drawFaceQuad(pt, 0, 1, 5, 4);
    }
    glEnd();
}

void MapDrawer::DrawChunkBoxes(float chunkResolution, float currentHeight, bool flag=false, double threshold=5)
{
    unique_lock<mutex> lock(mMutexLocalDenseMap);

    //double t = (double)cvGetTickCount() ;
    glShadeModel(GL_SMOOTH);
    //glBegin(GL_TRIANGLES);
    glBegin(GL_QUADS);
    //ROS_WARN("localChunkBoxes.sz = %u", localChunkBoxes.size() ) ;
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
                    pt[ith].x = p.x + i*chunkResolution/2 ;
                    pt[ith].y = p.y + j*chunkResolution/2 ;
                    pt[ith].z = p.z + k*chunkResolution/2 ;
                    ith++ ;
                }
            }
        }
        float d ;
        if ( flag ){
            d = p.z - currentHeight ;
        }
        else {
            d = p.y - currentHeight ;
        }
        //d += 4 ;
        if ( d < -threshold ){
            d = -threshold ;
        }
        if ( d > threshold ){
            d = threshold ;
        }
        int i0 = (d+threshold)/(2*threshold)*62 ;
        cv::Vec3b c = colorMap.at(i0) ;
        glColor3f(c.val[0]/255.0, c.val[1]/255.0, c.val[2]/255.0);

        drawFaceQuad(pt, 0, 1, 3, 2);
        drawFaceQuad(pt, 0, 2, 6, 4);
        drawFaceQuad(pt, 1, 3, 7, 5);
        drawFaceQuad(pt, 5, 7, 6, 4);
        drawFaceQuad(pt, 3, 2, 6, 7);
        drawFaceQuad(pt, 0, 1, 5, 4);
    }
    glEnd();

    //ROS_WARN("DrawChunkBoxes TIME %lf", ((double)cvGetTickCount() - t) / (cvGetTickFrequency() * 1000));
}

void MapDrawer::DrawKeyFramesLocal(float scale)
{
    std::unique_lock<std::mutex> lk(mMutexPose);
    if ( neigborhoodKF_T.empty() ){
        return ;
    }
    const float &w = pSetting->mCameraSize*scale;
    const float h = w*0.75;
    const float z = w*0.6;

    //printf("vpKFs=%d\n", vpKFs.size() ) ;
    for(size_t i=0; i<neigborhoodKF_T.size(); i++)
    {
        cv::Mat Twc ;
        cv::eigen2cv(neigborhoodKF_T[i], Twc) ;
        //std::cout << Twc << "\n" ;

        glPushMatrix();

        glMultMatrixd(Twc.ptr<GLdouble>(0));

        glLineWidth(pSetting->mKeyFrameLineWidth*2);
        glColor3f(1.0f,0.0f,1.0f);
        glBegin(GL_LINES);
        glVertex3f(0,0,0);
        glVertex3f(w,h,z);
        glVertex3f(0,0,0);
        glVertex3f(w,-h,z);
        glVertex3f(0,0,0);
        glVertex3f(-w,-h,z);
        glVertex3f(0,0,0);
        glVertex3f(-w,h,z);

        glVertex3f(w,h,z);
        glVertex3f(w,-h,z);

        glVertex3f(-w,h,z);
        glVertex3f(-w,-h,z);

        glVertex3f(-w,h,z);
        glVertex3f(w,h,z);

        glVertex3f(-w,-h,z);
        glVertex3f(w,-h,z);
        glEnd();

        glPopMatrix();
    }
}

void MapDrawer::DrawMeshesLocal()
{
    unique_lock<mutex> lock(mMutexLocalDenseMap);
    if ( marker.points.empty() ){
        return ;
    }

    glLineWidth(pSetting->mMeshLineWidth);
    glShadeModel(GL_SMOOTH);
    //glColor4f(0.8f,0.8f,0.0f, 0.1f);
    glBegin(GL_TRIANGLES);
//double t = (double)cvGetTickCount() ;
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
//    ROS_WARN("DrawChunkBoxes TIME %lf", ((double)cvGetTickCount() - t) / (cvGetTickFrequency() * 1000));
}

void MapDrawer::DrawReferencePoints()
{
    glPointSize(pSetting->mPointSize);
    glBegin(GL_POINTS);
    unique_lock<mutex> lock(mMutexLocalMap);


    glColor3f(1.0,0.0,0.0);
    for( cv::Point3f& p: localMapPoints ){
        glVertex3f(p.x, p.y, p.z);
    }

    glColor3f(0.0, 0.0,0.0);
    for( cv::Point3f& p: allMapPoints ){
        glVertex3f(p.x, p.y, p.z);
    }

    glEnd();
}

void MapDrawer::DrawMapPoints()
{
//    const vector<MapPoint*> &vpMPs = pSystem->mpMap->GetAllMapPoints();
//    const vector<MapPoint*> &vpRefMPs = pSystem->mpMap->GetReferenceMapPoints();

//    set<MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

//    if(vpMPs.empty())
//        return;

//    glPointSize(mPointSize);
//    glBegin(GL_POINTS);
//    glColor3f(0.0,0.0,0.0);

//    for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
//    {
//        if(vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i]))
//            continue;
//        cv::Mat pos = vpMPs[i]->GetWorldPos();
//        glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
//    }
//    glEnd();

//    glPointSize(mPointSize);
//    glBegin(GL_POINTS);
//    glColor3f(1.0,0.0,0.0);

//    for(set<MapPoint*>::iterator sit=spRefMPs.begin(), send=spRefMPs.end(); sit!=send; sit++)
//    {
//        if((*sit)->isBad())
//            continue;
//        cv::Mat pos = (*sit)->GetWorldPos();
//        glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));

//    }

//    glEnd();
}

void MapDrawer::DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph, float scale)
{
    std::unique_lock<std::mutex> lk(mMutexPose);
    if ( NonneigborhoodKF_T.empty() ){
        return ;
    }
    const float &w = pSetting->mCameraSize*scale;
    const float h = w*0.75;
    const float z = w*0.6;

    //printf("vpKFs=%d\n", vpKFs.size() ) ;
    for(size_t i=0; i<NonneigborhoodKF_T.size(); i++)
    {
        if ( neigborhoodKF_ids.find(NonneigborhoodKF_ids[i]) !=
             neigborhoodKF_ids.end() ){
            continue ;
        }
        cv::Mat Twc ;
        cv::eigen2cv(NonneigborhoodKF_T[i], Twc) ;
        //std::cout << Twc << "\n" ;

        glPushMatrix();

        glMultMatrixd(Twc.ptr<GLdouble>(0));

        glLineWidth(pSetting->mKeyFrameLineWidth*2);
        glColor3f(0.0f,0.0f,1.0f);
        glBegin(GL_LINES);
        glVertex3f(0,0,0);
        glVertex3f(w,h,z);
        glVertex3f(0,0,0);
        glVertex3f(w,-h,z);
        glVertex3f(0,0,0);
        glVertex3f(-w,-h,z);
        glVertex3f(0,0,0);
        glVertex3f(-w,h,z);

        glVertex3f(w,h,z);
        glVertex3f(w,-h,z);

        glVertex3f(-w,h,z);
        glVertex3f(-w,-h,z);

        glVertex3f(-w,h,z);
        glVertex3f(w,h,z);

        glVertex3f(-w,-h,z);
        glVertex3f(w,-h,z);
        glEnd();

        glPopMatrix();
    }
}

void MapDrawer::DrawCurrentCamera(pangolin::OpenGlMatrix& Twc, float scale)
{
//    printf("pSetting->mCameraSize = %f\n", pSetting->mCameraSize ) ;
//    printf("pSetting->mCameraLineWidth = %f\n", pSetting->mCameraLineWidth ) ;

    const float &w = pSetting->mCameraSize*scale;
    const float h = w*0.75;
    const float z = w*0.6;

    glPushMatrix();

#ifdef HAVE_GLES
    glMultMatrixf(Twc.m);
#else
    glMultMatrixd(Twc.m);
#endif

    glLineWidth(pSetting->mCameraLineWidth);
    glColor3f(1.0f,1.0f,0.0f);
    glBegin(GL_LINES);
    glVertex3f(0,0,0);
    glVertex3f(w,h,z);
    glVertex3f(0,0,0);
    glVertex3f(w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,h,z);

    glVertex3f(w,h,z);
    glVertex3f(w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(-w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(w,h,z);

    glVertex3f(-w,-h,z);
    glVertex3f(w,-h,z);
    glEnd();

    glPopMatrix();
}

void MapDrawer::SetCurrentCameraPose(const cv::Mat &Tcw)
{
    unique_lock<mutex> lock(mMutexCamera);
//    mCameraPose = Tcw.clone();
}

void MapDrawer::SetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M, const cv::Mat& mCameraPose )
{
    if(!mCameraPose.empty())
    {
        cv::Mat Rwc(3,3,CV_32F);
        cv::Mat twc(3,1,CV_32F);
        {
            unique_lock<mutex> lock(mMutexCamera);
            Rwc = mCameraPose.rowRange(0,3).colRange(0,3).t();
            twc = -Rwc*mCameraPose.rowRange(0,3).col(3);
        }

        M.m[0] = Rwc.at<float>(0,0);
        M.m[1] = Rwc.at<float>(1,0);
        M.m[2] = Rwc.at<float>(2,0);
        M.m[3]  = 0.0;

        M.m[4] = Rwc.at<float>(0,1);
        M.m[5] = Rwc.at<float>(1,1);
        M.m[6] = Rwc.at<float>(2,1);
        M.m[7]  = 0.0;

        M.m[8] = Rwc.at<float>(0,2);
        M.m[9] = Rwc.at<float>(1,2);
        M.m[10] = Rwc.at<float>(2,2);
        M.m[11]  = 0.0;

        M.m[12] = twc.at<float>(0);
        M.m[13] = twc.at<float>(1);
        M.m[14] = twc.at<float>(2);
        M.m[15]  = 1.0;
    }
    else
        M.SetIdentity();
}


void MapDrawer::GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M)
{
    std::unique_lock<std::mutex> lk(mMutexPose);
    if ( initPose == false ){
        M.SetIdentity();
        return ;
    }

//    if ( pSystem->mpTracker->mvpLocalKeyFrames.size() < 1 ){
//        M.SetIdentity();
//        return ;
//    }
    Eigen::Matrix3d Rwc = cur_R ;
    Eigen::Vector3d twc = cur_t ;
//    Eigen::Matrix3d Rwc = pSystem->mpTracker->cur_R.transpose() ;
//    Eigen::Vector3d twc = -pSystem->mpTracker->cur_R.transpose()*pSystem->mpTracker->cur_t ;

    M.m[0] = Rwc(0,0);
    M.m[1] = Rwc(1,0);
    M.m[2] = Rwc(2,0);
    M.m[3] = 0.0;

    M.m[4] = Rwc(0,1);
    M.m[5] = Rwc(1,1);
    M.m[6] = Rwc(2,1);
    M.m[7] = 0.0;

    M.m[8] = Rwc(0,2);
    M.m[9] = Rwc(1,2);
    M.m[10] = Rwc(2,2);
    M.m[11]  = 0.0;

    M.m[12] = twc(0);
    M.m[13] = twc(1);
    M.m[14] = twc(2);
    M.m[15]  = 1.0;
}

void MapDrawer::GetPose(Matrix3f &R, Vector3f &t)
{
    std::unique_lock<std::mutex> lk(mMutexPose);
    if ( initPose == false )
    {
        R.setIdentity();
        t.setZero();
    }
    else
    {
        R = cur_R.cast<float>() ;
        t = cur_t.cast<float>() ;
    }
}

} //namespace LT_SLAM
