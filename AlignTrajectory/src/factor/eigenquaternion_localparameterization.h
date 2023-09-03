/***
 * @Author:
 * @Date: 2023-07-18 19:06:13
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2023-07-18 19:06:13
 * @FilePath: /hint_map/include/hint_map/factor/eigenquaternion_localparameterization.h
 * @Description:
 */
#pragma once

#include <ceres/ceres.h>
#include <ceres/manifold.h>
#include <Eigen/Eigen>


class EigenQuaternionLocalParameterization : public ceres::LocalParameterization{

    virtual bool Plus(const double *x, const double *delta, double *x_plus_delta) const;
    virtual bool ComputeJacobian(const double *x, double *jacobian) const;
    virtual int GlobalSize() const { return 4; };
    virtual int LocalSize() const { return 3; };

};


class PoseParameterization : public ceres::LocalParameterization {
public:
    // 四元数定义顺序为, x, y, z, w

    bool Plus(const double *x, const double *delta, double *x_plus_delta) const override;

    bool ComputeJacobian(const double *x, double *jacobian) const override {
        Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> j(jacobian);
        j.topRows<6>().setIdentity();
        j.bottomRows<1>().setZero();

        return true;
    }

    int GlobalSize() const override {
        return 7;
    }

    int LocalSize() const override {
        return 6;
    }
};