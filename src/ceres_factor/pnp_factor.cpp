#include "pnp_factor.h"


PnPFactor::PnPFactor(const double x, const double y, const double shift, const Eigen::Vector3d& p):
    x(x), y(y), shift(shift), f_w(p)
{
    ;
}

bool PnPFactor::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
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
    Eigen::Vector3d f_i = Ri_T*(f_w-Pi) + Eigen::Vector3d(shift, 0, 0) ;
    Eigen::Map<Eigen::Vector2d> residual(residuals);

    bool invalid ;
    if ( f_i(2) < 0.000001 ){
        residual << 0.0, 0.0 ;
        invalid = true ;
    }
    else {
        residual << f_i(0)/f_i(2)-x, f_i(1)/f_i(2)-y;
        invalid = false ;
    }

    Eigen::Matrix<double, 2, 3> reduce(2, 3);

    reduce << 1. / f_i(2), 0.0, -f_i(0) / (f_i(2) * f_i(2)),
            0.0, 1. / f_i(2), -f_i(1) / (f_i(2) * f_i(2));
    if (jacobians)
    {
        if (jacobians[0])
        {
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_pose_j(jacobians[0]);

            if ( invalid ){
                jacobian_pose_j.setZero();
            }
            else
            {
                Eigen::Matrix<double, 3, 6> jaco_j;
                jaco_j.leftCols<3>() = -Ri_T;
                jaco_j.rightCols<3>() = skewSymmetric(f_i - Eigen::Vector3d(shift, 0, 0));

                jacobian_pose_j.leftCols<6>() = reduce * jaco_j;
                jacobian_pose_j.rightCols<1>().setZero();
            }
        }
    }

    return true;
}

void PnPFactor::check(double **parameters)
{
    //    double *res = new double[15];
    //    double **jaco = new double *[4];
    //    jaco[0] = new double[2 * 7];
    //    jaco[1] = new double[2 * 7];
    //    jaco[2] = new double[2 * 7];
    //    jaco[3] = new double[2 * 1];
    //    Evaluate(parameters, res, jaco);
    //    puts("check begins");

    //    puts("my");

    //    std::cout << Eigen::Map<Eigen::Matrix<double, 2, 1>>(res).transpose() << std::endl
    //              << std::endl;
    //    std::cout << Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>>(jaco[0]) << std::endl
    //              << std::endl;
    //    std::cout << Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>>(jaco[1]) << std::endl
    //              << std::endl;
    //    std::cout << Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>>(jaco[2]) << std::endl
    //              << std::endl;
    //    std::cout << Eigen::Map<Eigen::Vector2d>(jaco[3]) << std::endl
    //              << std::endl;

    //    Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
    //    Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

    //    Eigen::Vector3d Pj(parameters[1][0], parameters[1][1], parameters[1][2]);
    //    Eigen::Quaterniond Qj(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);

    //    Eigen::Vector3d tic(parameters[2][0], parameters[2][1], parameters[2][2]);
    //    Eigen::Quaterniond qic(parameters[2][6], parameters[2][3], parameters[2][4], parameters[2][5]);
    //    double inv_dep_i = parameters[3][0];

    //    Eigen::Vector3d pts_camera_i = pts_i / inv_dep_i;
    //    Eigen::Vector3d pts_imu_i = qic * pts_camera_i + tic;
    //    Eigen::Vector3d pts_w = Qi * pts_imu_i + Pi;
    //    Eigen::Vector3d pts_imu_j = Qj.inverse() * (pts_w - Pj);
    //    Eigen::Vector3d pts_camera_j = qic.inverse() * (pts_imu_j - tic);

    //    double dep_j = pts_camera_j.z();

    //    Eigen::Vector2d residual;
    //    residual = (pts_camera_j / dep_j).head<2>() - pts_j.head<2>();

    //    residual = sqrt_info * residual;

    //    puts("num");
    //    std::cout << residual.transpose() << std::endl;

    //    const double eps = 1e-6;
    //    Eigen::Matrix<double, 2, 19> num_jacobian;
    //    for (int k = 0; k < 19; k++)
    //    {
    //        Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
    //        Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

    //        Eigen::Vector3d Pj(parameters[1][0], parameters[1][1], parameters[1][2]);
    //        Eigen::Quaterniond Qj(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);

    //        Eigen::Vector3d tic(parameters[2][0], parameters[2][1], parameters[2][2]);
    //        Eigen::Quaterniond qic(parameters[2][6], parameters[2][3], parameters[2][4], parameters[2][5]);
    //        double inv_dep_i = parameters[3][0];

    //        int a = k / 3, b = k % 3;
    //        Eigen::Vector3d delta = Eigen::Vector3d(b == 0, b == 1, b == 2) * eps;

    //        if (a == 0)
    //            Pi += delta;
    //        else if (a == 1)
    //            Qi = Qi * Utility::deltaQ(delta);
    //        else if (a == 2)
    //            Pj += delta;
    //        else if (a == 3)
    //            Qj = Qj * Utility::deltaQ(delta);
    //        else if (a == 4)
    //            tic += delta;
    //        else if (a == 5)
    //            qic = qic * Utility::deltaQ(delta);
    //        else if (a == 6)
    //            inv_dep_i += delta.x();

    //        Eigen::Vector3d pts_camera_i = pts_i / inv_dep_i;
    //        Eigen::Vector3d pts_imu_i = qic * pts_camera_i + tic;
    //        Eigen::Vector3d pts_w = Qi * pts_imu_i + Pi;
    //        Eigen::Vector3d pts_imu_j = Qj.inverse() * (pts_w - Pj);
    //        Eigen::Vector3d pts_camera_j = qic.inverse() * (pts_imu_j - tic);

    //        double dep_j = pts_camera_j.z();

    //        Eigen::Vector2d tmp_residual;
    //        tmp_residual = (pts_camera_j / dep_j).head<2>() - pts_j.head<2>();
    //        tmp_residual = sqrt_info * tmp_residual;

    //        num_jacobian.col(k) = (tmp_residual - residual) / eps;
    //    }
    //    std::cout << num_jacobian << std::endl;
}

