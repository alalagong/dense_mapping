#include "MapPoint.h"


namespace LT_SLAM
{

MapPoint::MapPoint(unsigned int pID, Eigen::Vector3d Pos, bool flag)
{
    mnId = pID ;
    pos = Pos ;
    init = flag ;
    mObservations.clear();
    mNormalVector.setZero();
    mDescriptor.release();
    numOfMonoObservations = 0 ;
    numOfStereoObservations = 0;
}

MapPoint::MapPoint(MapPoint* p)
{
    pos = p->pos;
    mNormalVector = p->mNormalVector;
    mnId = p->mnId;
    mObservations = p->mObservations;
    mDescriptor = p->mDescriptor.clone();
    init = p->init;
    numOfMonoObservations = p->numOfMonoObservations ;
    numOfStereoObservations = p->numOfStereoObservations ;
}

MapPoint::~MapPoint()
{
    ;
}

bool MapPoint::triangulate(const vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d>>& R_b_2_w,
                        const vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>& T_b_2_w,
                        const vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>& observation,
                        const vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>& shift,
                        Eigen::Vector3d& result_p )
{
    Eigen::Matrix3d ATA ;
    Eigen::Vector3d ATb ;
    ATA.setZero() ;
    ATb.setZero() ;
    Eigen::Matrix<double, 2, 3> A, left ;
    Eigen::Vector2d b ;
    for( int i = 0, sz = observation.size(); i < sz ; i++ )
    {
        left << -1.0, 0.0, observation[i](0), 0.0, -1.0, observation[i](1) ;
        A = left*R_b_2_w[i].transpose() ;
        b = left*(-R_b_2_w[i].transpose()*T_b_2_w[i]+shift[i] ) ;
        ATA = ATA + A.transpose()*A ;
        ATb = ATb + A.transpose()*b ;
    }
    Eigen::SelfAdjointEigenSolver< Eigen::Matrix3d > saes(ATA);
    Eigen::Vector3d eigenvalues = saes.eigenvalues();
    const double smallest = (eigenvalues[0]);
    //const double largest = (eigenvalues[2]);
    //cout << "eigenvalues : " << eigenvalues.transpose() << "\n" ;
    if(smallest < 0.000003 ){
        // this means, it has a non-observable depth
        result_p.setZero() ;
        return false ;
    } else {
        // OK, well constrained
        result_p = ATA.ldlt().solve(-ATb) ;
        if ( fabs(result_p(2)) < 0.0000001 ){
            return false ;
        }
        if ( std::isnan(result_p(0)) ||  std::isnan(result_p(1)) || std::isnan(result_p(2)) ){
            return false ;
        }
        //return sqrt(smallest)/sqrt(largest);
        return true;
    }
}

} //namespace LT_SLAM
