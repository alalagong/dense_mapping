#ifndef UTIL_H
#define UTIL_H
#include <string>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <iostream>
#include <string>
#include <memory>
#include <functional>
#include <list>
#include <vector>
#include <fstream>
#include <algorithm>
#include <vector>
#include <list>
#include <set>
#include <map>
#include <unordered_map>
#include <math.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Sparse>
#include <eigen3/Eigen/StdVector>


#include "opencv2/opencv.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/eigen.hpp"

#include <ceres/ceres.h>
#include <ceres/local_parameterization.h>
#include <ceres/rotation.h>

#include "ceres_factor/pose_local_parameterization.h"
#include "ceres_factor/projection_factor.h"
#include "ceres_factor/pnp_factor.h"
#include "ceres_factor/relative_pose_factor.h"

#include <pangolin/pangolin.h>

#include "../Vocabulary/DBoW2/BowVector.h"
#include "../Vocabulary/DBoW2/FORB.h"
#include "../Vocabulary/DBoW2/TemplatedVocabulary.h"

//#include "sgm/libsgm.h"
#include "sgm2/disparity_method.h"

#include "FColorMap.h"

#include "tic_toc.h"

#include "ORBextractor.h"

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <pcl_ros/point_cloud.h>
#include <visualization_msgs/Marker.h>
#include <image_transport/image_transport.h>
#include <ros/package.h>
#include <sensor_msgs/fill_image.h>
#include <sensor_msgs/image_encodings.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Point.h>


namespace LT_SLAM
{

inline float SQ(float a){
    return a*a;
}

template <typename T>
inline void reduceVector(vector<T> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

//inline void reduceVector(vector<unsigned int> &v, vector<uchar> status)
//{
//    int j = 0;
//    for (int i = 0; i < int(v.size()); i++)
//        if (status[i])
//            v[j++] = v[i];
//    v.resize(j);
//}

//inline void reduceVector(vector<int> &v, vector<uchar> status)
//{
//    int j = 0;
//    for (int i = 0; i < int(v.size()); i++)
//        if (status[i])
//            v[j++] = v[i];
//    v.resize(j);
//}

inline bool CheckDistEpipolarLine(const Eigen::Vector3d& kp1,
                                  const Eigen::Vector3d& kp2,
                                  const Eigen::Matrix3d& F12)
{
    // Epipolar line in second image l = x1'F12 = [a b c]
    Eigen::Vector3d kp = F12.transpose()*kp1 ;

//    const float a = kp1.pt.x*F12.at<float>(0,0)+kp1.pt.y*F12.at<float>(1,0)+F12.at<float>(2,0);
//    const float b = kp1.pt.x*F12.at<float>(0,1)+kp1.pt.y*F12.at<float>(1,1)+F12.at<float>(2,1);
//    const float c = kp1.pt.x*F12.at<float>(0,2)+kp1.pt.y*F12.at<float>(1,2)+F12.at<float>(2,2);

    //const float num = a*kp2.pt.x+b*kp2.pt.y+c;
    //const float den = a*a+b*b;
    const float num = kp.dot(kp2);
    const float den = SQ(kp(0))+SQ(kp(1));

    if(den==0){
        return false;
    }

    const float dsqr = num*num/den;
    return dsqr<3.84;
}

inline int DescriptorDistance(const cv::Mat &a, const cv::Mat &b)
{
    const int *pa = a.ptr<int32_t>();
    const int *pb = b.ptr<int32_t>();

    int dist=0;

    for(int i=0; i<8; i++, pa++, pb++)
    {
        unsigned  int v = *pa ^ *pb;
        v = v - ((v >> 1) & 0x55555555);
        v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
        dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
    }

    return dist;
}

inline Eigen::Matrix3d MakeMatrix( Eigen::Vector3d& X, Eigen::Vector3d& Y )
{
    // make sure that we actually have two unique vectors.
    assert( X != Y );

    Eigen::Matrix3d R;
    Eigen::Vector3d b0 = X ;
    b0.normalize() ;
    Eigen::Vector3d b2 = X.cross(Y) ;
    b2.normalize() ;
    Eigen::Vector3d b1 = b2.cross(X) ;
    b1.normalize() ;

    R.col(0) = b0;
    R.col(1) = b1;
    R.col(2) = b2;

    return R.transpose();
}

class EDGELINK
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    int pKF ;
    Eigen::Matrix3d R ;
    Eigen::Vector3d t ;

    EDGELINK(){
        ;
    }
    ~EDGELINK(){
        ;
    }
};

struct MYTIMESTAMP
{
    unsigned int sec ;
    unsigned int nsec ;
    bool operator < (const MYTIMESTAMP& a )const
    {
        if ( sec != a.sec ){
            return sec < a.sec ;
        }
        else {
            return nsec < a.nsec ;
        }
    }
    bool operator > (const MYTIMESTAMP& a )const
    {
        if ( sec != a.sec ){
            return sec > a.sec ;
        }
        else {
            return nsec > a.nsec ;
        }
    }
    bool operator == (const MYTIMESTAMP& a )const{
        return (sec == a.sec) && (nsec == a.nsec) ;
    }
    double toSec()
    {
        return double(sec)+double(nsec)/(1000000000.0) ;
    }
};

class Utility
{
  public:
    template <typename Derived>
    static Eigen::Quaternion<typename Derived::Scalar> deltaQ(const Eigen::MatrixBase<Derived> &theta)
    {
        typedef typename Derived::Scalar Scalar_t;

        Eigen::Quaternion<Scalar_t> dq;
        Eigen::Matrix<Scalar_t, 3, 1> half_theta = theta;
        half_theta /= static_cast<Scalar_t>(2.0);
        dq.w() = static_cast<Scalar_t>(1.0);
        dq.x() = half_theta.x();
        dq.y() = half_theta.y();
        dq.z() = half_theta.z();
        return dq;
    }

    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 3, 3> skewSymmetric(const Eigen::MatrixBase<Derived> &q)
    {
        Eigen::Matrix<typename Derived::Scalar, 3, 3> ans;
        ans << typename Derived::Scalar(0), -q(2), q(1),
            q(2), typename Derived::Scalar(0), -q(0),
            -q(1), q(0), typename Derived::Scalar(0);
        return ans;
    }

    template <typename Derived>
    static Eigen::Quaternion<typename Derived::Scalar> positify(const Eigen::QuaternionBase<Derived> &q)
    {
        //printf("a: %f %f %f %f", q.w(), q.x(), q.y(), q.z());
        //Eigen::Quaternion<typename Derived::Scalar> p(-q.w(), -q.x(), -q.y(), -q.z());
        //printf("b: %f %f %f %f", p.w(), p.x(), p.y(), p.z());
        //return q.template w() >= (typename Derived::Scalar)(0.0) ? q : Eigen::Quaternion<typename Derived::Scalar>(-q.w(), -q.x(), -q.y(), -q.z());
        return q;
    }

    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 4, 4> Qleft(const Eigen::QuaternionBase<Derived> &q)
    {
        Eigen::Quaternion<typename Derived::Scalar> qq = positify(q);
        Eigen::Matrix<typename Derived::Scalar, 4, 4> ans;
        ans(0, 0) = qq.w(), ans.template block<1, 3>(0, 1) = -qq.vec().transpose();
        ans.template block<3, 1>(1, 0) = qq.vec(), ans.template block<3, 3>(1, 1) = qq.w() * Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity() + skewSymmetric(qq.vec());
        return ans;
    }

    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 4, 4> Qright(const Eigen::QuaternionBase<Derived> &p)
    {
        Eigen::Quaternion<typename Derived::Scalar> pp = positify(p);
        Eigen::Matrix<typename Derived::Scalar, 4, 4> ans;
        ans(0, 0) = pp.w(), ans.template block<1, 3>(0, 1) = -pp.vec().transpose();
        ans.template block<3, 1>(1, 0) = pp.vec(), ans.template block<3, 3>(1, 1) = pp.w() * Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity() - skewSymmetric(pp.vec());
        return ans;
    }

    static Eigen::Vector3d R2ypr(const Eigen::Matrix3d &R)
    {
        Eigen::Vector3d n = R.col(0);
        Eigen::Vector3d o = R.col(1);
        Eigen::Vector3d a = R.col(2);

        Eigen::Vector3d ypr(3);
        double y = atan2(n(1), n(0));
        double p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
        double r = atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));
        ypr(0) = y;
        ypr(1) = p;
        ypr(2) = r;

        return ypr / M_PI * 180.0;
    }

    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 3, 3> ypr2R(const Eigen::MatrixBase<Derived> &ypr)
    {
        typedef typename Derived::Scalar Scalar_t;

        Scalar_t y = ypr(0) / 180.0 * M_PI;
        Scalar_t p = ypr(1) / 180.0 * M_PI;
        Scalar_t r = ypr(2) / 180.0 * M_PI;

        Eigen::Matrix<Scalar_t, 3, 3> Rz;
        Rz << cos(y), -sin(y), 0,
            sin(y), cos(y), 0,
            0, 0, 1;

        Eigen::Matrix<Scalar_t, 3, 3> Ry;
        Ry << cos(p), 0., sin(p),
            0., 1., 0.,
            -sin(p), 0., cos(p);

        Eigen::Matrix<Scalar_t, 3, 3> Rx;
        Rx << 1., 0., 0.,
            0., cos(r), -sin(r),
            0., sin(r), cos(r);

        return Rz * Ry * Rx;
    }

    static Eigen::Matrix3d g2R(const Eigen::Vector3d &g)
    {
        Eigen::Matrix3d R0;
        Eigen::Vector3d ng1 = g.normalized();
        Eigen::Vector3d ng2{0, 0, 1.0};
        R0 = Eigen::Quaterniond::FromTwoVectors(ng1, ng2).toRotationMatrix();
        double yaw = R2ypr(R0).x();
        R0 = ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
        // R0 = Utility::ypr2R(Eigen::Vector3d{-90, 0, 0}) * R0;
        return R0;
    }

    template <size_t N>
    struct uint_
    {
    };

    template <size_t N, typename Lambda, typename IterT>
    void unroller(const Lambda &f, const IterT &iter, uint_<N>)
    {
        unroller(f, iter, uint_<N - 1>());
        f(iter + N);
    }

    template <typename Lambda, typename IterT>
    void unroller(const Lambda &f, const IterT &iter, uint_<0>)
    {
        f(iter);
    }
};


} //namespace LT_SLAM


#endif // UTIL_H

