//
// Created by cjq on 23-7-28.
//

#include "pose_factor.h"
#include <cmath>
#include <sophus/so3.hpp>


Mat3d ComputeJr_inv(const Vec3d& phi){
    double theta = phi.norm();
    Vec3d omega = phi / theta;
    double cot_theta_2 = 1./std::tan(theta / 2);
    Mat3d Jr = theta / 2. * cot_theta_2 * Mat3d::Identity() +
               (1 - theta / 2.*cot_theta_2)*omega * omega.transpose()
               + theta / 2. * Sophus::SO3d::hat(omega);
    return Jr;
}


bool RotationFactor::Evaluate(const double *const *parameters, double *residuals, double **jacobians) const {
    Eigen::Quaterniond Qe(parameters[0][3], parameters[0][0], parameters[0][1], parameters[0][2]);

    Quaterniond q_error = pose_op_->q().inverse() * Qe * pose_vins_->q();

    Eigen::Map<Eigen::Vector3d> residual(residuals);
    residual = Sophus::SO3d(q_error).log();

    if (jacobians)
    {
        Eigen::Map<Eigen::Matrix<double, 3, 4, Eigen::RowMajor>> jacobian_pose_map(jacobians[0]);
        jacobian_pose_map.leftCols<3>() = ComputeJr_inv(residual);
        jacobian_pose_map.rightCols<1>().setZero();
    }
    return true;
}



bool PoseFactor::Evaluate(const double *const *parameters, double *residuals, double **jacobians) const {
    Eigen::Vector3d t_e(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Quaterniond q_e(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

    Quaterniond q_a_inv = pose_op_->q().inverse();
    Vec3d t_a = pose_op_->t();
    Quaterniond q_b = pose_vins_->q();
    Vec3d t_b = pose_vins_->t();

    ///计算误差
    Vec3d error_t = q_a_inv * q_e * t_b + q_a_inv * t_e - q_a_inv*t_a;
    Quaterniond error_q = q_a_inv * q_e * q_b;
    Vec3d error_so3 = Sophus::SO3d(error_q).log();

    Eigen::Map<Vec6d> residual(residuals);
    residual.topRows(3) = error_t;
    residual.bottomRows(3) = error_so3;

    if (jacobians)
    {
        Eigen::Map<Eigen::Matrix<double, 6, 7, Eigen::RowMajor>> jacobian_pose_map(jacobians[0]);

        //derror_t /dt
        jacobian_pose_map.block<3,3>(0,0) = q_a_inv.toRotationMatrix();
        //derror_t /dq
        jacobian_pose_map.block<3,3>(0,3) =
                -q_a_inv.toRotationMatrix() * q_e.toRotationMatrix() * Sophus::SO3d::hat(t_b);
        //derror_q /dt
        jacobian_pose_map.block<3,3>(3,0) = Mat3d::Zero();
        //derror_q /dq
        jacobian_pose_map.block<3,3>(3,3) = ComputeJr_inv(error_so3);

        jacobian_pose_map.rightCols<1>().setZero();
    }

    return true;
}


//误差: e = T_a^-1  *  T_e  * T_b，其中 T_a是后一帧(k+1)， T_e是相对变换， T_b是前一帧(k)
bool RelativePoseFactor::Evaluate(const double *const *parameters, double *residuals, double **jacobians) const {
    Eigen::Vector3d t_a(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Quaterniond q_a(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);
    Eigen::Vector3d t_b(parameters[1][0], parameters[1][1], parameters[1][2]);
    Eigen::Quaterniond q_b(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);

    Quaterniond q_a_inv = q_a.inverse();
    Quaterniond q_e = pose_e_.q();
    Vec3d t_e = pose_e_.t();

    ///计算误差
    Vec3d error_t = q_a_inv * q_e * t_b + q_a_inv * t_e - q_a_inv*t_a;
    Quaterniond error_q = q_a_inv * q_e * q_b;
    Vec3d error_so3 = Sophus::SO3d(error_q).log();

    Eigen::Map<Vec6d> residual(residuals);
    residual.topRows(3) = error_t ;
    residual.bottomRows(3) = error_so3 ;

    residual = info_matrix_ * residual;

    if (jacobians)
    {
        if(jacobians[0]){
            Eigen::Map<Eigen::Matrix<double, 6, 7, Eigen::RowMajor>> jacobian_pose_map(jacobians[0]);

            //derror_t /dta
            jacobian_pose_map.block<3,3>(0,0) = -q_a_inv.toRotationMatrix();
            ///TODO 重点检查公式
            //derror_t /dqa
            jacobian_pose_map.block<3,3>(0,3) =
                    Sophus::SO3d::hat(q_a_inv.toRotationMatrix() * (q_e * t_b + t_e - t_a)) ;

            //derror_q /dta
            jacobian_pose_map.block<3,3>(3,0) = Mat3d::Zero();
            //derror_q /dqa
            jacobian_pose_map.block<3,3>(3,3) =
                    - ComputeJr_inv(error_so3) * (q_a_inv * q_e * q_b).toRotationMatrix().transpose() ;

            jacobian_pose_map.rightCols<1>().setZero();

            jacobian_pose_map.block<6,6>(0,0) = info_matrix_ * jacobian_pose_map.block<6,6>(0,0);
        }

        if(jacobians[1]){
            Eigen::Map<Eigen::Matrix<double, 6, 7, Eigen::RowMajor>> jacobian_pose_map(jacobians[1]);

            //derror_t /dtb
            jacobian_pose_map.block<3,3>(0,0) = (q_a_inv * q_e).toRotationMatrix() ;
            //derror_t /dqb
            jacobian_pose_map.block<3,3>(0,3) = Mat3d::Zero();

            //derror_q /dta
            jacobian_pose_map.block<3,3>(3,0) = Mat3d::Zero();
            //derror_q /dqb
            jacobian_pose_map.block<3,3>(3,3) = ComputeJr_inv(error_so3)  ;

            jacobian_pose_map.rightCols<1>().setZero();

            jacobian_pose_map.block<6,6>(0,0) = info_matrix_ * jacobian_pose_map.block<6,6>(0,0);
        }


    }

    return true;
}