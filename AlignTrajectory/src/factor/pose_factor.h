//
// Created by cjq on 23-7-28.
//

#ifndef ALIGNTRAJECTORY_POSE_FACTOR_H
#define ALIGNTRAJECTORY_POSE_FACTOR_H

#include <ceres/ceres.h>
#include "pose.h"

class RotationFactor: public ceres::SizedCostFunction<3,4> {
public:
    RotationFactor(Pose* pose_op, Pose* pose_vins): pose_op_(pose_op), pose_vins_(pose_vins){
    }

    bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const override;


private:
    Pose* pose_op_;
    Pose* pose_vins_;
};


class PoseFactor: public ceres::SizedCostFunction<6,7> {
public:
    PoseFactor(Pose* pose_op, Pose* pose_vins): pose_op_(pose_op), pose_vins_(pose_vins){
    }

    bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const override;


private:
    Pose* pose_op_;
    Pose* pose_vins_;
};


#endif //ALIGNTRAJECTORY_POSE_FACTOR_H
