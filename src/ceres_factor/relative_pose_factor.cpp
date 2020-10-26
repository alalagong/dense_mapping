#include "relative_pose_factor.h"


relativePoseFactor::relativePoseFactor(Eigen::Quaterniond& detla_q_i_2_j, Eigen::Vector3d& detla_t_i_2_j ):
    detla_q_i_2_j(detla_q_i_2_j), detla_t_i_2_j(detla_t_i_2_j)
{
    ;
}

bool relativePoseFactor::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{
//    par_pose[0]= cur_t(0);
//    par_pose[1]= cur_t(1);
//    par_pose[2]= cur_t(2);
//    par_pose[3]= cur_q.x();
//    par_pose[4]= cur_q.y();
//    par_pose[5]= cur_q.z();
//    par_pose[6]= cur_q.w();
    Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Quaterniond Qi;
    Qi.x() = parameters[0][3];
    Qi.y() = parameters[0][4];
    Qi.z() = parameters[0][5];
    Qi.w() = parameters[0][6];
    Eigen::Matrix3d Ri = Qi.toRotationMatrix();
    Eigen::Matrix3d Ri_T = Ri.transpose();

    Eigen::Vector3d Pj(parameters[1][0], parameters[1][1], parameters[1][2]);
    Eigen::Quaterniond Qj;
    Qj.x() = parameters[1][3];
    Qj.y() = parameters[1][4];
    Qj.z() = parameters[1][5];
    Qj.w() = parameters[1][6];
    Eigen::Matrix3d Rj = Qj.toRotationMatrix();
    Eigen::Matrix3d Rj_T = Rj.transpose();

    Eigen::Map<Eigen::Vector3d> residual_t(residuals);
    Eigen::Map<Eigen::Vector3d> residual_q(residuals+3);

    residual_t = Rj_T*(Pi-Pj)-detla_t_i_2_j ;
    residual_q = 2.0*(detla_q_i_2_j*Qi.inverse()*Qj).vec();

    //printf("%lf %lf %lf %lf %lf %lf\n", residuals[0], residuals[1], residuals[2], residuals[3],residuals[4],residuals[5]) ;

    if (jacobians)
    {
        if (jacobians[0])
        {
            Eigen::Map<Eigen::Matrix<double, 6, 7, Eigen::RowMajor>> jacobian_pose_i(jacobians[0]);
            jacobian_pose_i.setZero();
            //jacobian_pose_i.block<7, 1>(0, 6).setConstant(1.0);

            jacobian_pose_i.block<3, 3>(0, 0) = Rj_T;
            jacobian_pose_i.block<3, 3>(3, 3) = -(Qleft(Qj.inverse() * Qi) * Qright(detla_q_i_2_j.inverse())).bottomRightCorner<3, 3>();
        }

        if (jacobians[1])
        {
            Eigen::Map<Eigen::Matrix<double, 6, 7, Eigen::RowMajor>> jacobian_pose_j(jacobians[1]);
            jacobian_pose_j.setZero();
            //jacobian_pose_j.block<7, 1>(0, 6).setConstant(1.0);

            jacobian_pose_j.block<3, 3>(0, 0) = -Rj_T;
            jacobian_pose_j.block<3, 3>(0, 3) = skewSymmetric( Rj_T*(Pi-Pj) );
            jacobian_pose_j.block<3, 3>(3, 3) = Qleft(detla_q_i_2_j * Qi.inverse() * Qj).bottomRightCorner<3, 3>();
        }
    }

    return true;
}

void relativePoseFactor::check(double **parameters)
{
    ;
}

