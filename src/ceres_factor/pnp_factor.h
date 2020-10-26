#pragma once

#include "ceres_util.h"

class PnPFactor : public ceres::SizedCostFunction<2, 7>
{
public:
    PnPFactor(const double x, const double y, const double shift, const Eigen::Vector3d& p);
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
    void check(double **parameters);

    double x, y, shift;
    Eigen::Vector3d f_w ;
};
