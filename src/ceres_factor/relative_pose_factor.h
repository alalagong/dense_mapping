#pragma once

#include "ceres_util.h"

class relativePoseFactor : public ceres::SizedCostFunction<6, 7, 7>
{
public:
    relativePoseFactor(Eigen::Quaterniond& detla_q_i_2_j, Eigen::Vector3d& detla_t_i_2_j );
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
    void check(double **parameters);

    Eigen::Quaterniond detla_q_i_2_j ;
    Eigen::Vector3d detla_t_i_2_j ;
};
