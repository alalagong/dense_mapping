#include "Viewer.h"
using namespace std;

namespace LT_SLAM
{

Viewer::Viewer(System* pSystem):
    pSystem(pSystem), mbFinishRequested(false), mbFinished(true), mbStopped(false), mbStopRequested(false)
{
    ;
}


Viewer::~Viewer()
{
    ;
}

inline float convert2realDepth( float z_far, float z_near, float z_buffer )
{
    float up = 2*z_far*z_near ;
    float down = (z_far+z_near-(z_far-z_near)*(2*z_buffer-1) ) ;
    return up/down ;
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

void Viewer::test()
{
    //    mbFinished = false;

    //    int sceen_w = 1226 ;
    //    int sceen_h = 370 ;
    //    float z_far = 100 ;
    //    float z_near = 0.1 ;

    //    pangolin::CreateWindowAndBind("Map Viewer", sceen_w, sceen_h);

    //    // 3D Mouse handler requires depth testing to be enabled
    //    glEnable(GL_DEPTH_TEST);
    //    // Issue specific OpenGl we might need
    //    glEnable (GL_BLEND);
    //    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    //    // Define Camera Render Object (for view / scene browsing)
    //    mViewpointF = 707.0912 ;
    //    pangolin::OpenGlMatrix Twc;
    //    Twc.SetIdentity();

    //    pangolin::OpenGlRenderState s_cam(
    //                pangolin::ProjectionMatrix(sceen_w, sceen_h, pSetting->mViewpointF,
    //                                           pSetting->mViewpointF,
    //                                           pSetting->mViewpointX,
    //                                           pSetting->mViewpointCy,
    //                                           z_near,
    //                                           z_far)
    //                );
    ////    pangolin::OpenGlMatrix rotated_view = pangolin::ModelViewLookAtRUB(0, 0, 0, 0, 0, 1, 0, -1, 0 ) ;
    ////    pangolin::OpenGlMatrix rotated_view_inv = rotated_view.Inverse();
    ////    cout << "rotated_view\n" ;
    ////    cout << rotated_view << "\n" ;
    //    //s_cam.SetModelViewMatrix(rotated_view) ;

    //    // Add named OpenGL viewport to window and provide 3D Handler
    //    pangolin::View& d_cam = pangolin::CreateDisplay()
    //            //.SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -(float)sceen_w/sceen_h)
    //            .SetBounds(0.0, 1.0, 0, 1.0 )
    //            .SetHandler(new pangolin::Handler3D(s_cam));



    //    KeyFrame* renderKF = NULL ;
    //    while ( ros::ok() && !pangolin::ShouldQuit() )
    //    {
    //        while ( renderKF == mpMapDrawer->mpLocalMesher->mpWorkingKF )
    //        {
    //            while(isStopped() && ros::ok() && !CheckFinish() )
    //            {
    //                usleep(3000);
    //            }

    //            if(CheckFinish())
    //                break;
    //        }
    //        renderKF = mpMapDrawer->mpLocalMesher->mpWorkingKF ;

    //        double t = cvGetTickCount() ;
    //        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    //        cv::Mat T = renderKF->GetPoseInverse();

    //        Eigen::Matrix4f currPose ;
    //        for( int i = 0; i < 4; i++ ){
    //            for( int j = 0 ; j < 4; j++ ){
    //                currPose(i, j) = T.at<float>(i, j) ;
    //            }
    //        }
    //        pangolin::OpenGlMatrix mv = getModelViewMatrix( currPose ) ;

    //        s_cam.SetModelViewMatrix(mv);
    //        d_cam.Activate(s_cam);

    //        glClearColor(0.9f,0.9f,0.9f,1.0f);

    //        mpMapDrawer->DrawMeshesLocal();

    //        pangolin::FinishFrame();

    //        GLfloat* data = new GLfloat[sceen_w*sceen_h] ;
    //        glReadPixels(0, 0, sceen_w, sceen_h, GL_DEPTH_COMPONENT, GL_FLOAT, data) ;

    //        cv::Mat depthImg(sceen_h, sceen_w, CV_32F ) ;
    //        cv::Mat disparity(sceen_h, sceen_w, CV_32F ) ;
    //        cv::Mat depth16(sceen_h, sceen_w, CV_16U ) ;
    //        //cv::Mat depthImg8 ;
    //        for( int i = sceen_h-1; i >= 0 ; i--)
    //        {
    //            for( int j = 0; j < sceen_w; j++ ){
    //                depthImg.at<float>(sceen_h-1-i, j) = convert2realDepth(z_far, z_near, data[i*sceen_w+j]) ;
    //                if ( depthImg.at<float>(sceen_h-1-i, j) > z_far-10 ){
    //                    depthImg.at<float>(sceen_h-1-i, j) = 0 ;
    //                    disparity.at<float>(sceen_h-1-i, j) = -1.0 ;
    //                }
    //                else if ( depthImg.at<float>(sceen_h-1-i, j) < 0.1 ){
    //                    disparity.at<float>(sceen_h-1-i, j) = -1.0 ;
    //                }
    //                else{
    //                    disparity.at<float>(sceen_h-1-i, j) = base_line/depthImg.at<float>(sceen_h-1-i, j) ;
    //                }

    //                depth16.at<unsigned short>(sceen_h-1-i, j) = depthImg.at<float>(sceen_h-1-i, j)*65535/30 ;
    //                //                if ( depthImg.at<float>(sceen_h-1-i, j) > 20 ){
    //                //                    depthImg.at<float>(sceen_h-1-i, j) = 20 ;
    //                //                }
    //            }
    //        }
    //        char filename[256] ;
    //        sprintf(filename, "/home/ygling2008/graph_data/%06d.png", int(renderKF->mTimeStamp+0.1) ) ;
    //        cv::imwrite(filename, depth16) ;

    //        if(Stop())
    //        {
    //            while(isStopped())
    //            {
    //                usleep(3000);
    //            }
    //        }

    //        if(CheckFinish())
    //            break;
    //        double renderTime = ((double)cvGetTickCount() - t) / (cvGetTickFrequency() * 1000) ;
    //        if ( renderTime < 150 ){
    //            usleep((150-renderTime)*1000) ;
    //        }
    //    }

    //    SetFinish();
}

void Viewer::Run()
{
    puts("start viewer") ;

    mbFinished = false;
    //int sceen_w = 1226 ;
    //int sceen_h = 370*2 ;
    int sceen_w = 1024 ;
    int sceen_h = 768 ;
    float z_far = 2000.0 ;
    float z_near = 1 ;

    pangolin::CreateWindowAndBind("Map Viewer", sceen_w, sceen_h);

    pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(175));
    pangolin::Var<bool> menuFollowCamera("menu.Follow Camera", pSetting->followPose>0, true);
    pangolin::Var<bool> menuShowPoints("menu.Show Points",true,true);
    pangolin::Var<bool> menuShowALLKF("menu.Show KeyFrames",true,true);
    pangolin::Var<bool> menuShowLocalKF("menu.Show LocalKeyFrames",true,true);
    pangolin::Var<bool> menuShowGraph("menu.Show Graph",true,true);
    pangolin::Var<bool> menuShowLocalMesh("menu.Show Mesh",true,true);
    pangolin::Var<bool> menuShowLocalChunks("menu.Show LocalChunks",false,true);
    pangolin::Var<bool> menuResetFPV("menu.Reset FPV",false,true);
    pangolin::Var<bool> menuResetTPV("menu.Reset TPV",false,true);
    pangolin::Var<bool> menuShowCube("menu.Show Cube", false,true);


    if ( pSetting->bMeshing ){
        menuShowLocalMesh = true ;
        menuShowLocalChunks = false ;
    }
    else{
        menuShowLocalMesh = false ;
        menuShowLocalChunks = true ;
    }

    // 3D Mouse handler requires depth testing to be enabled
    glEnable(GL_DEPTH_TEST);

    // Issue specific OpenGl we might need
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // Define Camera Render Object (for view / scene browsing)
    //    pangolin::OpenGlRenderState s_cam(
    //                pangolin::ProjectionMatrix(sceen_w, sceen_h, pSetting->mViewpointF, pSetting->mViewpointF,
    //                                           sceen_w/2, sceen_h/2, z_near, z_far),
    //                pangolin::ModelViewLookAt( pSetting->mViewpointX, pSetting->mViewpointY, pSetting->mViewpointZ,
    //                                          0,0,0,0.0,-1.0, 0.0)
    //                );
    pangolin::OpenGlRenderState s_cam(
                pangolin::ProjectionMatrix(sceen_w, sceen_h, pSetting->mViewpointF, pSetting->mViewpointF,
                                           sceen_w/2, sceen_h/2, z_near, z_far)
                //                pangolin::ModelViewLookAt( pSetting->mViewpointX, pSetting->mViewpointTopDownZ,-0.1,
                //                                          0,0,0,0.0,-1.0, 0.0)
                );

    //    printf("pSetting->mViewpointX = %lf\n", pSetting->mViewpointX ) ;
    //    printf("pSetting->mViewpointY = %lf\n", pSetting->mViewpointY ) ;
    //    printf("pSetting->mViewpointZ = %lf\n", pSetting->mViewpointZ ) ;
    //    printf("pSetting->mViewpointF = %lf\n", pSetting->mViewpointF ) ;

    pangolin::OpenGlRenderState s_cam2(
                pangolin::ProjectionMatrix(sceen_w, sceen_h,1000,1000,
                                           sceen_w/2, sceen_h/2, z_near, z_far),
                pangolin::ModelViewLookAt( pSetting->mViewpointX, pSetting->mViewpointTopDownZ,-0.1,
                                           0,0,0,0.0,-1.0, 0.0)
                );



    pangolin::View& d_cam2 = pangolin::Display("Third Person View")
            .SetAspect((float)sceen_w/sceen_h)
            .SetBounds(0.0, 1.0, 0, 1.0)
            .SetHandler(new pangolin::Handler3D(s_cam2));


    pangolin::OpenGlMatrix Twc;
    Twc.SetIdentity();
    pangolin::OpenGlMatrix Twc2;
    Twc2.SetIdentity();

    //cv::namedWindow("ORB-SLAM2: Current Frame");

    bool bFollow = true;
    bool bLocalizationMode = false;
    float currentPoseX ;
    float currentPoseY ;
    float currentPoseZ ;
    Eigen::Matrix3f cur_R ;
    Eigen::Vector3f cur_t ;
    while( !pangolin::ShouldQuit() && ros::ok() )
    {
        double t = cvGetTickCount() ;
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        pSystem->mpMapDrawer->GetPose(cur_R, cur_t);
        pSystem->mpMapDrawer->GetCurrentOpenGLCameraMatrix(Twc);

        currentPoseX = Twc.m[12] ;
        currentPoseY = Twc.m[13] ;
        currentPoseZ = Twc.m[14] ;


        if ( menuResetTPV ){
            s_cam2.SetModelViewMatrix(
                        pangolin::ModelViewLookAt( pSetting->mViewpointX, pSetting->mViewpointTopDownZ,-0.1,
                                                   0,0,0,0.0,-1.0, 0.0)
                        );
        }

        //cur_t(2) -= 10 ;
        pangolin::OpenGlMatrix mv = getModelViewMatrix(cur_R, cur_t) ;


        d_cam2.Activate(s_cam2);
        glClearColor(128.0/255.0f, 128/255.0f, 128/255.0f, 1.0f);
        pSystem->mpMapDrawer->DrawCurrentCamera(Twc, 0.25);
        if ( menuShowPoints ){
            pSystem->mpMapDrawer->DrawReferencePoints();
        }
        if( menuShowLocalKF ){
            pSystem->mpMapDrawer->DrawKeyFramesLocal(0.25);
        }
        if( menuShowALLKF || menuShowGraph){
            pSystem->mpMapDrawer->DrawKeyFrames(menuShowALLKF,menuShowGraph, 0.25);
        }
        if ( menuShowCube ){
            pSystem->mpMapDrawer->DrawCube( Twc.m[12], Twc.m[13], Twc.m[14], pSetting->cubicBound );
        }
        if ( menuShowLocalMesh ){
            pSystem->mpMapDrawer->DrawMeshesLocal();
        }
        if ( menuShowLocalChunks ){
            pSystem->mpMapDrawer->DrawChunkBoxes(pSetting->voxelResolution, currentPoseY, false, 5);
        }

        pangolin::FinishFrame();


        if(Stop())
        {
            while(isStopped())
            {
                usleep(3000);
            }
        }

        if(CheckFinish())
            break;

        double renderTime = ((double)cvGetTickCount() - t) / (cvGetTickFrequency() * 1000) ;
        if ( renderTime < 100 ){
            usleep((100-renderTime)*1000) ;
        }
    }

    SetFinish();
}

void Viewer::Run_Multi_Move()
{
    puts("start viewer") ;

    mbFinished = false;
    //int sceen_w = 1226 ;
    //int sceen_h = 370*2 ;
    int sceen_w = 1500 ;
    int sceen_h = 900 ;
    float z_far = 1000.0 ;
    float z_near = 0.1 ;

    pangolin::CreateWindowAndBind("Map Viewer", sceen_w+160, sceen_h);

    pangolin::CreatePanel("menu").SetBounds(0.0,1.0, 0.0, pangolin::Attach::Pix(175));
    pangolin::Var<bool> menuFollowCamera("menu.Follow Camera", pSetting->followPose>0, true);
    pangolin::Var<bool> menuShowPoints("menu.Show Points", false,true);
    pangolin::Var<bool> menuShowALLKF("menu.Show KeyFrames",true,true);
    pangolin::Var<bool> menuShowLocalKF("menu.Show LocalKeyFrames",true,true);
    pangolin::Var<bool> menuShowGraph("menu.Show Graph",true,true);
    pangolin::Var<bool> menuShowLocalMesh("menu.Show Mesh",true,true);
    pangolin::Var<bool> menuShowLocalChunks("menu.Show LocalChunks",false,true);
    pangolin::Var<bool> menuShowCube("menu.Show Cube", false,true);
    pangolin::Var<bool> menuShowTopDown("menu.Show TopDownView", false, true ) ;


    //    if ( pSetting->updateMeshFlag ){
    //        menuShowLocalMesh = true ;
    //        menuShowLocalChunks = false ;
    //    }
    //    else{
    //        menuShowLocalMesh = false ;
    //        menuShowLocalChunks = true ;
    //    }

    // 3D Mouse handler requires depth testing to be enabled
    glEnable(GL_DEPTH_TEST);

    // Issue specific OpenGl we might need
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
                pangolin::ProjectionMatrix(sceen_w, sceen_h, pSetting->mViewpointF, pSetting->mViewpointF,
                                           sceen_w/2, sceen_h/2, z_near, z_far)
                ,pangolin::ModelViewLookAt( pSetting->mViewpointX, pSetting->mViewpointY, pSetting->mViewpointZ,
                                            0,0,0,0.0,-1.0, 0.0)
                );
    s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt( pSetting->mViewpointX, pSetting->mViewpointTopDownZ,-0.1,
                                                        0,0,0,0.0,-1.0, 0.0));

    // Add named OpenGL viewport to window and provide 3D Handler
    pangolin::View& d_cam = pangolin::Display("Reconstructed View")
            .SetAspect((float)sceen_w/sceen_h)
            .SetBounds(0.0, 1.0, 0, 1.0)
            .SetHandler(new pangolin::Handler3D(s_cam));


    pangolin::OpenGlMatrix Twc;
    Twc.SetIdentity();
    pangolin::OpenGlMatrix TwcIdentity;
    TwcIdentity.SetIdentity();

    bool bFollow = true;
    float currentPoseX ;
    float currentPoseY ;
    float currentPoseZ ;
    Eigen::Matrix3f cur_R ;
    Eigen::Vector3f cur_t ;
    bool initFlag = true ;
    while( !pangolin::ShouldQuit() && ros::ok() )
    {
        double t = cvGetTickCount() ;
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glClearColor(130.0/255.0f, 130/255.0f, 130/255.0f, 1.0f);

        pSystem->mpMapDrawer->GetPose(cur_R, cur_t);
        pSystem->mpMapDrawer->GetCurrentOpenGLCameraMatrix(Twc);

        currentPoseX = Twc.m[12] ;
        currentPoseY = Twc.m[13] ;
        currentPoseZ = Twc.m[14] ;


        if( menuFollowCamera && bFollow )
        {
            s_cam.Follow(Twc);
        }
        else if(menuFollowCamera && !bFollow)
        {
            if ( menuShowTopDown ){
                s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt( pSetting->mViewpointX, pSetting->mViewpointTopDownZ,-0.1,
                                                                    0,0,0,0.0,-1.0, 0.0));
            }
            else
            {
                s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt( pSetting->mViewpointX, pSetting->mViewpointY, pSetting->mViewpointZ,
                                                                    0,0,0,0.0,-1.0, 0.0));
                s_cam.Follow(Twc);
            }

            bFollow = true;
        }
        else if(!menuFollowCamera && bFollow)
        {
            bFollow = false;
        }
        d_cam.Activate(s_cam);

        pSystem->mpMapDrawer->DrawCurrentCamera(Twc, 1.0);
        if ( menuShowLocalMesh ){
            pSystem->mpMapDrawer->DrawMeshesLocal();
        }
        if ( menuShowLocalChunks ){
            pSystem->mpMapDrawer->DrawChunkBoxes(pSetting->voxelResolution, currentPoseZ, true, 5);
        }
        if ( menuShowPoints ){
            pSystem->mpMapDrawer->DrawReferencePoints();
        }
        if( menuShowLocalKF ){
            pSystem->mpMapDrawer->DrawKeyFramesLocal(1.0);
        }
        if( menuShowALLKF || menuShowGraph){
            pSystem->mpMapDrawer->DrawKeyFrames(menuShowALLKF,menuShowGraph, 1.0);
        }
        if ( menuShowCube ){
            pSystem->mpMapDrawer->DrawCube( Twc.m[12], Twc.m[13], Twc.m[14], pSetting->cubicBound );
        }

        pangolin::FinishFrame();

        if(Stop())
        {
            while(isStopped())
            {
                usleep(3000);
            }
        }

        if(CheckFinish())
            break;

        double renderTime = ((double)cvGetTickCount() - t) / (cvGetTickFrequency() * 1000) ;
        if ( renderTime < 100 ){
            usleep((100-renderTime)*1000) ;
        }
    }

    SetFinish();
}

void Viewer::Run_Multi()
{
    puts("start viewer") ;

    mbFinished = false;
    //int sceen_w = 1226 ;
    //int sceen_h = 370*2 ;
    int sceen_w = 800 ;
    int sceen_h = 650 ;
    float z_far = 2000.0 ;
    float z_near = 1.0 ;

    pangolin::CreateWindowAndBind("Map Viewer", sceen_w*2+160, sceen_h);

    pangolin::CreatePanel("menu").SetBounds(0.0,1.0, pangolin::Attach::Pix(sceen_w*2),pangolin::Attach::Pix(sceen_w*2+160));
    pangolin::Var<bool> menuFollowCamera("menu.Follow Camera", pSetting->followPose>0, true);
    pangolin::Var<bool> menuShowPoints("menu.Show Points",true,true);
    pangolin::Var<bool> menuShowALLKF("menu.Show KeyFrames",true,true);
    pangolin::Var<bool> menuShowLocalKF("menu.Show LocalKeyFrames",true,true);
    pangolin::Var<bool> menuShowGraph("menu.Show Graph",true,true);
    pangolin::Var<bool> menuShowLocalMesh("menu.Show Mesh",true,true);
    pangolin::Var<bool> menuShowLocalChunks("menu.Show LocalChunks",true,true);
    pangolin::Var<bool> menuResetFPV("menu.Reset FPV",false,true);
    pangolin::Var<bool> menuResetTPV("menu.Reset TPV",false,true);
    pangolin::Var<bool> menuShowCube("menu.Show Cube", true,true);


    if ( pSetting->updateMeshFlag ){
        menuShowLocalMesh = true ;
        menuShowLocalChunks = false ;
    }
    else{
        menuShowLocalMesh = false ;
        menuShowLocalChunks = true ;
    }

    // 3D Mouse handler requires depth testing to be enabled
    glEnable(GL_DEPTH_TEST);

    // Issue specific OpenGl we might need
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);


    pangolin::OpenGlRenderState s_cam(
                pangolin::ProjectionMatrix(sceen_w, sceen_h, pSetting->mViewpointF, pSetting->mViewpointF,
                                           sceen_w/2, sceen_h/2, z_near, z_far)
                ,pangolin::ModelViewLookAt( pSetting->mViewpointX, pSetting->mViewpointY, pSetting->mViewpointZ,
                                            0,0,0,0.0,-1.0, 0.0)
                );

    pangolin::OpenGlRenderState s_cam2(
                pangolin::ProjectionMatrix(sceen_w, sceen_h,1000,1000,
                                           sceen_w/2, sceen_h/2, z_near, z_far),
                pangolin::ModelViewLookAt( pSetting->mViewpointX, pSetting->mViewpointTopDownZ,-0.1,
                                           0,0,0,0.0,-1.0, 0.0)
                );


    // Add named OpenGL viewport to window and provide 3D Handler
    pangolin::View& d_cam = pangolin::Display("Third Person View")
            .SetAspect((float)sceen_w/sceen_h)
            .SetBounds(0.0, 1.0, 0, 1.0)
            .SetHandler(new pangolin::Handler3D(s_cam));

    pangolin::View& d_cam2 = pangolin::Display("Top-Down View")
            .SetAspect((float)sceen_w/sceen_h)
            .SetBounds(0.0, 1.0, 0, 1.0)
            .SetHandler(new pangolin::Handler3D(s_cam2));

    pangolin::Display("multi")
            .SetBounds(0.0, 1.0, 0, pangolin::Attach::Pix(sceen_w*2))
            .SetLayout(pangolin::LayoutEqualHorizontal)
            .AddDisplay(d_cam)
            .AddDisplay(d_cam2);

    pangolin::OpenGlMatrix Twc;
    Twc.SetIdentity();
    pangolin::OpenGlMatrix Twc2;
    Twc2.SetIdentity();

    //cv::namedWindow("ORB-SLAM2: Current Frame");

    bool bFollow = true;
    bool bLocalizationMode = false;
    float currentPoseX ;
    float currentPoseY ;
    float currentPoseZ ;
    Eigen::Matrix3f cur_R ;
    Eigen::Vector3f cur_t ;
    while( !pangolin::ShouldQuit() && ros::ok() )
    {
        double t = cvGetTickCount() ;
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glClearColor(130.0/255.0f, 130/255.0f, 130/255.0f, 1.0f);

        pSystem->mpMapDrawer->GetPose(cur_R, cur_t);
        pSystem->mpMapDrawer->GetCurrentOpenGLCameraMatrix(Twc);

        currentPoseX = Twc.m[12] ;
        currentPoseY = Twc.m[13] ;
        currentPoseZ = Twc.m[14] ;

        if ( menuResetFPV ){
            s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt( pSetting->mViewpointX, pSetting->mViewpointY, pSetting->mViewpointZ,
                                                                0,0,0,0.0,-1.0, 0.0));
        }
        if ( menuResetTPV ){
            s_cam2.SetModelViewMatrix(pangolin::ModelViewLookAt( pSetting->mViewpointX, pSetting->mViewpointTopDownZ,-0.1,
                                                                0,0,0,0.0,-1.0, 0.0));
        }
        if ( menuFollowCamera ){
            s_cam2.Follow(Twc);
        }

        //cur_t(2) -= 10 ;
        pangolin::OpenGlMatrix mv = getModelViewMatrix(cur_R, cur_t) ;
        //cout << mv << "\n" ;
        //cout << "cur_R " << cur_R << "\n" << "cur_t =" << cur_t.transpose() << "\n" ;
        s_cam.SetModelViewMatrix(mv);
        d_cam.Activate(s_cam);
        pSystem->mpMapDrawer->DrawChunkBoxesDist(pSetting->voxelResolution*1.5, cur_t, 10);
//        GLfloat* data = new GLfloat[sceen_w*sceen_h] ;
//        glReadPixels(0, 0, sceen_w, sceen_h, GL_DEPTH_COMPONENT, GL_FLOAT, data) ;
        //pSystem->mpMapDrawer->DrawCurrentCamera(Twc, 1.0);


        d_cam2.Activate(s_cam2);
        //pSystem->mpMapDrawer->DrawCurrentCamera(Twc, 1.0);
        if ( menuShowLocalChunks ){
            pSystem->mpMapDrawer->DrawChunkBoxes(pSetting->voxelResolution*1.5, currentPoseY, false, 5);
        }
        if( menuShowLocalKF ){
            pSystem->mpMapDrawer->DrawKeyFramesLocal(1.0);
        }
        if( menuShowALLKF || menuShowGraph){
            pSystem->mpMapDrawer->DrawKeyFrames(menuShowALLKF,menuShowGraph, 1.0);
        }
        if ( menuShowCube ){
            pSystem->mpMapDrawer->DrawCube( Twc.m[12], Twc.m[13], Twc.m[14], pSetting->cubicBound );
        }
        pangolin::FinishFrame();


        if(Stop())
        {
            while(isStopped())
            {
                usleep(3000);
            }
        }

        if(CheckFinish())
            break;

        double renderTime = ((double)cvGetTickCount() - t) / (cvGetTickFrequency() * 1000) ;
        if ( renderTime < 100 ){
            usleep((100-renderTime)*1000) ;
        }
    }

    SetFinish();
}

void Viewer::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool Viewer::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void Viewer::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;
}

bool Viewer::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

void Viewer::RequestStop()
{
    unique_lock<mutex> lock(mMutexStop);
    if(!mbStopped)
        mbStopRequested = true;
}

bool Viewer::isStopped()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopped;
}

bool Viewer::Stop()
{
    unique_lock<mutex> lock(mMutexStop);
    unique_lock<mutex> lock2(mMutexFinish);

    if(mbFinishRequested)
        return false;
    else if(mbStopRequested)
    {
        mbStopped = true;
        mbStopRequested = false;
        return true;
    }

    return false;

}

void Viewer::Release()
{
    unique_lock<mutex> lock(mMutexStop);
    mbStopped = false;
}

}
