#include "Tracking.h"
using namespace std;
using namespace cv;

namespace LT_SLAM
{

unsigned int Tracking::featureNextID = 1;
unsigned int Tracking::frameNextID = 0;

struct greaterThanPtr :
        public std::binary_function<const float *, const float *, bool>
{
    bool operator () (const float * a, const float * b) const
    // Ensure a fully deterministic result of the sort
    { return (*a > *b) ? true : (*a < *b) ? false : (a > b); }
};

void goodFeaturesToTrack_extended( cv::Mat _image, vector<cv::Point2f>& _corners,
                                   int maxCorners, vector<cv::Point2f>& _corners_extra,
                                   int maxCorner_extra,
                                   double qualityLevel, double minDistance,
                                   cv::Mat _mask, int blockSize=3,
                                   bool useHarrisDetector=false, double harrisK=0.04 )
{

    Mat image = _image, eig, tmp;
    if (image.empty())
    {
        _corners.clear();
        return;
    }

    if( useHarrisDetector )
        cornerHarris( image, eig, blockSize, 3, harrisK );
    else
        cornerMinEigenVal( image, eig, blockSize, 3 );

    double maxVal = 0;
    minMaxLoc( eig, 0, &maxVal, 0, 0, _mask );
    threshold( eig, eig, maxVal*qualityLevel, 0, THRESH_TOZERO );
    dilate( eig, tmp, Mat());

    Size imgsize = image.size();
    std::vector<const float*> tmpCorners;

    // collect list of pointers to features - put them into temporary image
    Mat mask = _mask;
    for( int y = 1; y < imgsize.height - 1; y++ )
    {
        const float* eig_data = (const float*)eig.ptr(y);
        const float* tmp_data = (const float*)tmp.ptr(y);
        const uchar* mask_data = mask.data ? mask.ptr(y) : 0;

        for( int x = 1; x < imgsize.width - 1; x++ )
        {
            float val = eig_data[x];
            if( val != 0 && val == tmp_data[x] && (!mask_data || mask_data[x]) )
                tmpCorners.push_back(eig_data + x);
        }
    }

    std::vector<Point2f> corners;
    size_t i, j, total = tmpCorners.size(), ncorners = 0;

    if (total == 0)
    {
        _corners.clear();
        _corners_extra.clear();
        return;
    }

    std::sort( tmpCorners.begin(), tmpCorners.end(), greaterThanPtr() );
    vector<bool> vst(tmpCorners.size());

    if (minDistance >= 1)
    {
        // Partition the image into larger grids
        int w = image.cols;
        int h = image.rows;

        const int cell_size = cvRound(minDistance);
        const int grid_width = (w + cell_size - 1) / cell_size;
        const int grid_height = (h + cell_size - 1) / cell_size;

        std::vector<std::vector<Point2f> > grid(grid_width*grid_height);

        minDistance *= minDistance;

        for( i = 0; i < total; i++ )
        {
            vst[i] = false ;

            int ofs = (int)((const uchar*)tmpCorners[i] - eig.ptr());
            int y = (int)(ofs / eig.step);
            int x = (int)((ofs - y*eig.step)/sizeof(float));

            bool good = true;

            int x_cell = x / cell_size;
            int y_cell = y / cell_size;

            int x1 = x_cell - 1;
            int y1 = y_cell - 1;
            int x2 = x_cell + 1;
            int y2 = y_cell + 1;

            // boundary check
            x1 = std::max(0, x1);
            y1 = std::max(0, y1);
            x2 = std::min(grid_width-1, x2);
            y2 = std::min(grid_height-1, y2);

            for( int yy = y1; yy <= y2; yy++ )
            {
                for( int xx = x1; xx <= x2; xx++ )
                {
                    std::vector <Point2f> &m = grid[yy*grid_width + xx];

                    if( m.size() )
                    {
                        for(j = 0; j < m.size(); j++)
                        {
                            float dx = x - m[j].x;
                            float dy = y - m[j].y;

                            if( dx*dx + dy*dy < minDistance )
                            {
                                good = false;
                                goto break_out2;
                            }
                        }
                    }
                }
            }

break_out2:

            if (good)
            {
                grid[y_cell*grid_width + x_cell].push_back(Point2f((float)x, (float)y));

                corners.push_back(Point2f((float)x, (float)y));
                ++ncorners;

                vst[i] = true ;

                if( maxCorners > 0 && (int)ncorners == maxCorners )
                    break;
            }
        }
    }
    else
    {
        for( i = 0; i < total; i++ )
        {
            int ofs = (int)((const uchar*)tmpCorners[i] - eig.ptr());
            int y = (int)(ofs / eig.step);
            int x = (int)((ofs - y*eig.step)/sizeof(float));

            corners.push_back(Point2f((float)x, (float)y));
            ++ncorners;
            if( maxCorners > 0 && (int)ncorners == maxCorners )
                break;
        }
    }

    _corners = corners;
    _corners_extra.clear();
    int corners_needed = maxCorner_extra - maxCorners ;
    for( i = 0; i < total; i++ )
    {
        if ( vst[i] ){
            continue ;
        }

        int ofs = (int)((const uchar*)tmpCorners[i] - eig.ptr());
        int y = (int)(ofs / eig.step);
        int x = (int)((ofs - y*eig.step)/sizeof(float));

        _corners_extra.push_back(Point2f((float)x, (float)y));

        if ( _corners_extra.size() >= corners_needed ){
            break ;
        }
    }
}

Tracking::Tracking()
{
    trackCnt = 0;
    stereoMatchCnt = 0;
    pre_frame_id = 0 ;
    extra_pts_num = 200 ;

    trackingFrameQueue.clear();
    clahe = cv::createCLAHE();
}

void Tracking::GrabImageStereo(const cv::Mat &imRectLeft,
                               const cv::Mat &imRectRight,
                               cv::Mat& depth,
                               const MYTIMESTAMP &timestamp)
{
    mImGray = imRectLeft;
    mImGrayRight = imRectRight;
    if ( depth.rows > 0 ){
        imDepth = depth.clone();
    }

    //    if(mImGray.channels()==3)
    //    {
    //        if(pSetting->mbRGB)
    //        {
    //            cv::cvtColor(mImGray,mImGray,CV_RGB2GRAY);
    //            cv::cvtColor(mImGrayRight,mImGrayRight,CV_RGB2GRAY);
    //        }
    //        else
    //        {
    //            cv::cvtColor(mImGray,mImGray,CV_BGR2GRAY);
    //            cv::cvtColor(mImGrayRight,mImGrayRight,CV_BGR2GRAY);
    //        }
    //    }
    //    else if(mImGray.channels()==4)
    //    {
    //        if(pSetting->mbRGB)
    //        {
    //            cv::cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
    //            cv::cvtColor(mImGrayRight,mImGrayRight,CV_RGBA2GRAY);
    //        }
    //        else
    //        {
    //            cv::cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
    //            cv::cvtColor(mImGrayRight,mImGrayRight,CV_BGRA2GRAY);
    //        }
    //    }

    cur_time = timestamp ;
    Track();
}


bool Tracking::inBorder(const cv::Point2f &pt)
{
    int img_x = cvRound(pt.x);
    int img_y = cvRound(pt.y);
    return BORDER_SIZE <= img_x
            && img_x < pSetting->width - BORDER_SIZE
            && BORDER_SIZE <= img_y
            && img_y < pSetting->height - BORDER_SIZE;
}

void Tracking::setMask(int ROW, int COL)
{
    mask = cv::Mat(ROW, COL, CV_8UC1, cv::Scalar(255));
    cv::rectangle(mask, cv::Point2f(0, 0), cv::Point2f(COL-1, BORDER_SIZE-1), 0, -1 ) ;
    cv::rectangle(mask, cv::Point2f(0, 0), cv::Point2f(BORDER_SIZE-1, ROW-1), 0, -1 ) ;
    cv::rectangle(mask, cv::Point2f(0, ROW-BORDER_SIZE), cv::Point2f(COL-1, ROW-1), 0, -1 ) ;
    cv::rectangle(mask, cv::Point2f(COL-BORDER_SIZE, 0), cv::Point2f(COL-1, ROW-1), 0, -1 ) ;

    //    for( cv::Point2f& p: forw_pts ){
    //        cv::circle(mask, p, pSetting->MIN_DIST, 0, -1);
    //    }
    // prefer to keep features that are tracked for long time
    vector<pair<int, pair<cv::Point2f, int>>> cnt_pts_id;

    for (unsigned int i = 0; i < forw_pts.size(); i++)
        cnt_pts_id.push_back(make_pair(track_cnt[i], make_pair(forw_pts[i], ids[i])));

    sort(cnt_pts_id.begin(), cnt_pts_id.end(), [](const pair<int, pair<cv::Point2f, int>> &a, const pair<int, pair<cv::Point2f, int>> &b)
    {
        return a.first > b.first;
    });

    forw_pts.clear();
    ids.clear();
    track_cnt.clear();

    for (auto &it : cnt_pts_id)
    {
        if (mask.at<uchar>(it.second.first) == 255)
        {
            forw_pts.push_back(it.second.first);
            ids.push_back(it.second.second);
            track_cnt.push_back(it.first);
            cv::circle(mask, it.second.first, pSetting->MIN_DIST, 0, -1);
        }
    }
}

void Tracking::rejectWithF()
{
    if (forw_pts.size() >= 8)
    {
        ROS_DEBUG("FM ransac begins");
        TicToc t_f;
        //        vector<cv::Point2f> un_prev_pts(cur_pts.size()), un_forw_pts(forw_pts.size());
        //        for (unsigned int i = 0; i < cur_pts.size(); i++)
        //        {
        //            Eigen::Vector3d tmp_p;
        //            m_camera->liftProjective(Eigen::Vector2d(prev_pts[i].x, prev_pts[i].y), tmp_p);
        //            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
        //            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
        //            un_prev_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());

        //            m_camera->liftProjective(Eigen::Vector2d(forw_pts[i].x, forw_pts[i].y), tmp_p);
        //            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
        //            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
        //            un_forw_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());
        //        }
        vector<cv::Point2f>& un_prev_pts = prev_pts;
        vector<cv::Point2f>& un_forw_pts = forw_pts;
        vector<uchar> status;
        cv::findFundamentalMat(un_prev_pts, un_forw_pts, cv::FM_RANSAC, pSetting->F_THRESHOLD, 0.99, status);
        int size_a = prev_pts.size();
        reduceVector(prev_pts, status);
        reduceVector(cur_pts, status);
        reduceVector(forw_pts, status);
        reduceVector(ids, status);
        reduceVector(track_cnt, status);
        ROS_DEBUG("FM ransac: %d -> %d: %f", size_a, (int)forw_pts.size(), 1.0 * forw_pts.size() / size_a);
        ROS_DEBUG("FM ransac costs: %fms", t_f.toc());
    }
}

int Tracking::TrackFeatures(cv::Mat& mImGray, cv::Mat& imGrayRight)
{
    cv::Mat img, img_right;
    TicToc t_total;

    frameNextID++ ;
    if (pSetting->equalized)
    {
        TicToc t_c;
        clahe->apply(mImGray, img);
        //clahe->apply(imGrayRight, img_right);
        ROS_DEBUG("CLAHE costs: %fms", t_c.toc());
    }
    else{
        img = mImGray.clone();
    }

    if (forw_img.empty()){
        prev_img = cur_img = forw_img = img;
    }
    else{
        forw_img = img;
    }

    forw_pts.clear();

    if (cur_pts.size() > 0)
    {
        TicToc t_o;
        vector<uchar> status;
        vector<float> err;
        cv::calcOpticalFlowPyrLK(cur_img, forw_img,
                                 cur_pts, forw_pts,
                                 status, err,
                                 cv::Size(21, 21), 3,
                                 TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.01),
                                 0, 1e-4 );

        //        cv::imshow("cur_img", cur_img ) ;
        //        cv::imshow("forw_img", forw_img ) ;



        int sumSucceeded = 0 ;
        for (int i = 0; i < int(forw_pts.size()); i++)
        {
            sumSucceeded += (status[i] > 0 ) ;
            if (status[i] && !inBorder(forw_pts[i])){
                status[i] = 0;
            }
        }

        //printf("sumSucceeded=%d\n", sumSucceeded ) ;

        if ( pSetting->equalized == false && sumSucceeded < 20 )
        {
            int sumSucceeded_new = 0 ;
            cv::Mat cur_img_equalized ;
            cv::Mat forw_img_equalized ;
            vector<uchar> status_equalized;
            vector<float> err_equalized;
            vector<cv::Point2f> forw_pts_equalized;

            clahe->apply(cur_img, cur_img_equalized);
            clahe->apply(forw_img, forw_img_equalized);
            cv::calcOpticalFlowPyrLK(cur_img_equalized,
                                     forw_img_equalized,
                                     cur_pts,
                                     forw_pts_equalized,
                                     status_equalized,
                                     err_equalized,
                                     cv::Size(21, 21), 3,
                                     TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.01),
                                     0, 1e-4 );

            for (int i = 0; i < int(forw_pts_equalized.size()); i++)
            {
                sumSucceeded_new += (status_equalized[i] > 0 ) ;
                if (status_equalized[i] && !inBorder(forw_pts_equalized[i])){
                    status_equalized[i] = 0;
                }
            }
            ROS_WARN("[tracker] sumSucceeded=%d sumSucceeded_new=%d", sumSucceeded, sumSucceeded_new ) ;

            if ( sumSucceeded_new > sumSucceeded*2 ){
                status = status_equalized ;
                forw_pts = forw_pts_equalized;
            }
        }

        reduceVector(prev_pts, status);
        reduceVector(cur_pts, status);
        reduceVector(forw_pts, status);
        reduceVector(ids, status);
        reduceVector(track_cnt, status);
        ROS_DEBUG("temporal optical flow costs: %fms", t_o.toc());

        //        if ( forw_pts.size() <= 8 ){
        //            return 0;
        //        }
    }

    bool newReferenceFrameFlag = true ;
    double parallax_sum = 0 ;
    //newReferenceFrameFlag = ( (frameNextID%pSetting->FREQ) == 1 ) ;
    if ( prev_pts.size() < 30 ){
        newReferenceFrameFlag = true ;
    }
    else{
        for( size_t i = 0, sz = prev_pts.size(); i< sz; i++ ){
            parallax_sum += sqrt(SQ(prev_pts[i].x - forw_pts[i].x) + SQ(prev_pts[i].y - forw_pts[i].y)) ;
        }
        if ( parallax_sum/(double)prev_pts.size() > pSetting->MIN_PARALLAX ){
            newReferenceFrameFlag = true ;
        }
        else {
            newReferenceFrameFlag = false ;
        }
    }
    newReferenceFrameFlag |= (frameNextID - pre_frame_id ) > pSetting->FREQ ;

    //printf("parallax_aver=%lf newReferenceFrameFlag=%d\n", parallax_sum/(double)prev_pts.size(), newReferenceFrameFlag ) ;

    if ( newReferenceFrameFlag )
    {
        rejectWithF();
        //increase sucessful track number
        for (int &n : track_cnt){
            n++;
        }

        ROS_DEBUG("set mask begins");
        TicToc t_m;
        setMask(img.rows, img.cols);
        ROS_DEBUG("set mask costs %fms", t_m.toc());

        ROS_DEBUG("detect feature begins");
        TicToc t_t;
        int n_max_cnt = pSetting->maxFeatures - static_cast<int>(forw_pts.size());
        if (n_max_cnt > 0)
        {

            if(mask.empty())
                cout << "mask is empty " << endl;
            if (mask.type() != CV_8UC1)
                cout << "mask type wrong " << endl;
            if (mask.size() != forw_img.size())
                cout << "wrong size " << endl;
            goodFeaturesToTrack_extended(forw_img, n_pts, n_max_cnt, extra_pts, extra_pts_num, 0.1, pSetting->MIN_DIST, mask, 3, false);
        }
        else{
            n_pts.clear();
        }
        ROS_DEBUG("detect feature costs: %fms", t_t.toc());

        ROS_DEBUG("add feature begins");
        TicToc t_a;
        for (cv::Point2f &p : n_pts)
        {
            forw_pts.push_back(p);
            ids.push_back(featureNextID);
            featureNextID++;
            track_cnt.push_back(1);
        }
        ROS_DEBUG("selectFeature costs: %fms", t_a.toc());

        prev_img = forw_img;
        prev_pts = forw_pts;
        pre_frame_id = frameNextID ;
    }
    cur_img = forw_img;
    cur_pts = forw_pts;

    //check stereo
    trackCnt++ ;
    if (trackCnt > pSetting->skipFrames && cur_pts.size() > 0)
    {
        trackCnt = 0;
        r_status.clear();
        r_err.clear();

        if (pSetting->equalized) {
            TicToc t_c;
            clahe->apply(imGrayRight, img_right);
            ROS_DEBUG("CLAHE costs: %fms", t_c.toc());
        }
        else{
            img_right = imGrayRight.clone();
        }

        TicToc t_o;
        cv::calcOpticalFlowPyrLK(cur_img, img_right,
                                 cur_pts, cur_pts_right,
                                 r_status, r_err,
                                 cv::Size(21, 21), 5,
                                 TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.01),
                                 0, 1e-4 );
        ROS_DEBUG("spatial optical flow costs: %fms", t_o.toc());
        vector<cv::Point2f> ll, rr;
        vector<int> idx;
        for (unsigned int i = 0; i < r_status.size(); i++)
        {
            //            if (!inBorder(cur_pts_right[i])){
            //                r_status[i] = 0;
            //            }

            if (r_status[i])
            {
                idx.push_back(i);

                ll.push_back(cur_pts[i]) ;
                rr.push_back(cur_pts_right[i]);
                //                Eigen::Vector3d tmp_p;
                //                trackerData[0].m_camera->liftProjective(Eigen::Vector2d(trackerData[0].cur_pts[i].x, trackerData[0].cur_pts[i].y), tmp_p);
                //                tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
                //                tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
                //                ll.push_back(cv::Point2f(tmp_p.x(), tmp_p.y()));

                //                trackerData[1].m_camera->liftProjective(Eigen::Vector2d(trackerData[1].cur_pts[i].x, trackerData[1].cur_pts[i].y), tmp_p);
                //                tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
                //                tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
                //                rr.push_back(cv::Point2f(tmp_p.x(), tmp_p.y()));
            }
        }
        if (ll.size() >= 8)
        {
            vector<uchar> status;
            TicToc t_f;
            //cv::findFundamentalMat(ll, rr, cv::FM_RANSAC, 1.0, 0.5, status);
            cv::findFundamentalMat(ll, rr, cv::FM_RANSAC, pSetting->F_THRESHOLD, 0.99, status);
            ROS_DEBUG("find f cost: %f", t_f.toc());
            stereoMatchCnt = 0 ;
            for (unsigned int i = 0; i < status.size(); i++)
            {
                if (status[i] == 0){
                    r_status[idx[i]] = 0;
                }
                stereoMatchCnt += r_status[idx[i]];
            }
        }
        if ( debug_f.is_open() ){
            debug_f << "0 " << t_total.toc() << "\n";
        }
        //save tracking data
        {
            std::unique_lock<std::mutex> lk(trackingFrameQueueMutex) ;
            trackingFrameQueue.push_back( TrackingFrame(r_status,
                                                        mImGray,
                                                        imGrayRight,
                                                        cur_pts,
                                                        cur_pts_right,
                                                        ids,
                                                        (int)cur_pts.size(),
                                                        frameNextID,
                                                        cur_time
                                                        )
                                          );
            //puts("[push tracking data]");
        }

        if ( pSetting->SHOW_TRACK )
        {
            cv::Mat stereo_img(img.rows, img.cols*2, CV_8UC3 ) ;
            cv::Mat tmp ;
            cv::cvtColor(img, stereo_img.colRange(0, img.cols), CV_GRAY2RGB) ;
            cv::cvtColor(img_right, stereo_img.colRange(img.cols, img.cols*2), CV_GRAY2RGB) ;

            for(size_t i = 0 ; i < cur_pts.size() ; i++ )
            {
                double len = std::min(1.0, 1.0 * track_cnt[i] / 20);
                cv::circle(stereo_img, cur_pts[i],
                           2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
                if ( r_status[i] )
                {
                    cv::circle(stereo_img, cur_pts_right[i] + cv::Point2f(img.cols, 0),
                               2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
                }
            }

            //            for(size_t i = 0 ; i < extra_pts.size() ; i++ )
            //            {
            //                cv::circle(stereo_img, extra_pts[i],
            //                           2, cv::Scalar(20, 200, 20), 2);
            //            }

            cv::imshow("stereo_img", stereo_img ) ;
            //cv::imshow("mask", mask );
            char key = cv::waitKey(pSetting->cvWaitTime) ;
            if ( key == 's' )
            {
                for( ; ; )
                {
                    usleep(1000);
                    key = cv::waitKey(1);
                    if ( key == 's' ){
                        break ;
                    }
                }
            }
        }

        return 2 ;
    }
    else{
        return 1;
    }
}

void Tracking::ComputeImageBounds(const cv::Mat &imLeft, bool calibrated)
{
    pSetting->width = imLeft.cols ;
    pSetting->height = imLeft.rows ;

    //printf("pSetting->width=%d pSetting->height=%d\n",pSetting->width, pSetting->height ) ;

    if(calibrated == false)
    {
        cv::Mat mat(4,2,CV_32F);
        mat.at<float>(0,0)=0.0; mat.at<float>(0,1)=0.0;
        mat.at<float>(1,0)=imLeft.cols; mat.at<float>(1,1)=0.0;
        mat.at<float>(2,0)=0.0; mat.at<float>(2,1)=imLeft.rows;
        mat.at<float>(3,0)=imLeft.cols; mat.at<float>(3,1)=imLeft.rows;

        // Undistort corners
        mat=mat.reshape(2);
        cv::undistortPoints(mat,mat, pSetting->K_l, pSetting->D_l,cv::Mat(), pSetting->P_l);
        mat=mat.reshape(1);

        pSetting->mnMinX = min(mat.at<float>(0,0),mat.at<float>(2,0));
        pSetting->mnMaxX = max(mat.at<float>(1,0),mat.at<float>(3,0));
        pSetting->mnMinY = min(mat.at<float>(0,1),mat.at<float>(1,1));
        pSetting->mnMaxY = max(mat.at<float>(2,1),mat.at<float>(3,1));

    }
    else
    {
        pSetting->mnMinX = 0.0f;
        pSetting->mnMaxX = imLeft.cols;
        pSetting->mnMinY = 0.0f;
        pSetting->mnMaxY = imLeft.rows;
    }

    pSetting->mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/(pSetting->mnMaxX - pSetting->mnMinX);
    pSetting->mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/(pSetting->mnMaxY - pSetting->mnMinY);
}

void Tracking::Track()
{
    TicToc t_localMap;
    if ( pSetting->mbInitialComputations == false )
    {
        //printf("%d %d %d %d\n", mImGray.cols, mImGray.rows, mImGrayRight.cols, mImGrayRight.rows) ;

        ComputeImageBounds(mImGray, true) ;
        pSetting->mbInitialComputations = true ;
    }
    trackFeaturesFlag = TrackFeatures(mImGray, mImGrayRight);
    ROS_DEBUG("Tracking costs: %fms", t_localMap.toc() );
}

} //namespace LT_SLAM
