//
// Created by cjq on 23-7-28.
//

#pragma once

#include <ceres/ceres.h>

#include <utility>
#include "pose.h"

class RotationFactor: public ceres::SizedCostFunction<3,4> {
public:
    RotationFactor(Pose* pose_op, Pose* pose_vins): pose_op_(pose_op), pose_vins_(pose_vins){}
    bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const override;
private:
    Pose* pose_op_;
    Pose* pose_vins_;
};


class PoseFactor: public ceres::SizedCostFunction<6,7> {
public:
    PoseFactor(Pose* pose_op, Pose* pose_vins): pose_op_(pose_op), pose_vins_(pose_vins){}
    bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const override;
private:
    Pose* pose_op_;
    Pose* pose_vins_;
};


class RelativePoseFactor: public ceres::SizedCostFunction<6,7,7> {
public:
    explicit RelativePoseFactor(Pose pose_e): pose_e_(std::move(pose_e)){
        info_matrix_ = Eigen::Matrix<double,6,6>::Identity();
        info_matrix_(0,0) = 100.;
        info_matrix_(1,1) = 100.;
        info_matrix_(2,2) = 100.;
        info_matrix_(3,3) = 10000.;
        info_matrix_(4,4) = 10000.;
        info_matrix_(5,5) = 10000.;
    }
    bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const override;

private:
    Pose pose_e_;
    Eigen::Matrix<double,6,6> info_matrix_;
};


// The estimated measurement is:
//      t_ab = [ p_ab ]  = [ R(q_a)^T * (p_b - p_a) ]
//             [ q_ab ]    [ q_a^{-1] * q_b         ]
//
class PoseGraph3dErrorTerm {
public:
    PoseGraph3dErrorTerm( Pose t_ab_measured, Eigen::Matrix<double, 6, 6> sqrt_information)
            : t_ab_measured_(std::move(t_ab_measured)), sqrt_information_(std::move(sqrt_information)) {}

    template <typename T>
    bool operator()(const T* const T_a_ptr, const T* const T_b_ptr, T* residuals_ptr) const {
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> p_a(T_a_ptr);
        Eigen::Map<const Eigen::Quaternion<T>> q_a(T_a_ptr+3);

        Eigen::Map<const Eigen::Matrix<T, 3, 1>> p_b(T_b_ptr);
        Eigen::Map<const Eigen::Quaternion<T>> q_b(T_b_ptr+3);

        // Compute the relative transformation between the two frames.
        Eigen::Quaternion<T> q_a_inverse = q_a.conjugate();
        Eigen::Quaternion<T> q_ab_estimated = q_a_inverse * q_b;

        // Represent the displacement between the two frames in the A frame.
        Eigen::Matrix<T, 3, 1> p_ab_estimated = q_a_inverse * (p_b - p_a);

        // Compute the error between the two orientation estimates.
        Eigen::Quaternion<T> delta_q = t_ab_measured_.q().cast<T>() * q_ab_estimated.conjugate();

        // Compute the residuals.
        // [ position         ]   [ delta_p          ]
        // [ orientation (3x1)] = [ 2 * delta_q(0:2) ]
        Eigen::Map<Eigen::Matrix<T, 6, 1>> residuals(residuals_ptr);
        residuals.template block<3, 1>(0, 0) = p_ab_estimated - t_ab_measured_.t().cast<T>();
        residuals.template block<3, 1>(3, 0) = T(2.0) * delta_q.vec();

        // Scale the residuals by the measurement uncertainty.
        residuals.applyOnTheLeft(sqrt_information_.template cast<T>());

        return true;
    }

    static ceres::CostFunction* Create(Pose t_ab_measured,
                                       const Eigen::Matrix<double, 6, 6>& sqrt_information) {
        return new ceres::AutoDiffCostFunction<PoseGraph3dErrorTerm, 6, 7, 7>(
                new PoseGraph3dErrorTerm(std::move(t_ab_measured), sqrt_information));
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    // The measurement for the position of B relative to A in the A frame.
     Pose t_ab_measured_;
    // The square root of the measurement information matrix.
    const Eigen::Matrix<double, 6, 6> sqrt_information_;
};
