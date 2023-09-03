/*** 
 * @Author: 
 * @Date: 2023-07-18 19:05:58
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2023-07-18 19:05:59
 * @FilePath: /hint_map/src/factor/eigenquaternion_localparameterization.cpp
 * @Description: 
 */
#include "eigenquaternion_localparameterization.h"
#include "math_util.hpp"

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

bool EigenQuaternionLocalParameterization::Plus(const double *x, const double *delta, double *x_plus_delta) const
{
    Eigen::Map<const Eigen::Quaterniond> _q(x );

    Eigen::Quaterniond dq = deltaQ(Eigen::Map<const Eigen::Vector3d>(delta ));

    Eigen::Map<Eigen::Quaterniond> q(x_plus_delta );
    q = (_q * dq).normalized();
    return true;
}
bool EigenQuaternionLocalParameterization::ComputeJacobian(const double *x, double *jacobian) const
{
    Eigen::Map<Eigen::Matrix<double, 4, 3, Eigen::RowMajor>> j(jacobian);
    j.topRows<3>().setIdentity();
    j.bottomRows<1>().setZero();

    return true;
}


bool PoseParameterization::Plus(const double *x, const double *delta, double *x_plus_delta) const{
Eigen::Map<const Eigen::Vector3d> _p(x);
Eigen::Map<const Eigen::Quaterniond> _q(x + 3);

Eigen::Map<const Eigen::Vector3d> dp(delta);

Eigen::Quaterniond dq = deltaQ(Eigen::Map<const Eigen::Vector3d>(delta + 3));

Eigen::Map<Eigen::Vector3d> p(x_plus_delta);
Eigen::Map<Eigen::Quaterniond> q(x_plus_delta + 3);

p = _p + dp;
q = (_q * dq).normalized();

return true;
}