#pragma once

#include "ceres_util.h"
#include <iostream>

class ProjectionFactor : public ceres::SizedCostFunction<2, 7, 3>
{
public:
    ProjectionFactor(const double x, const double y, const double shift);
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
    void check(double **parameters);

    double x, y, shift;
};
