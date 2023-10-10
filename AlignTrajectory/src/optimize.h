//
// Created by cjq on 23-7-31.
//

#ifndef ALIGNTRAJECTORY_OPTIMIZE_H
#define ALIGNTRAJECTORY_OPTIMIZE_H

#include <ceres/ceres.h>

#include "pose.h"


namespace Optimize{

    Quaterniond OptimizeDeltaR(const vector<PoseStamped>& pose_ceres_op,const vector<PoseStamped>& pose_ceres_vins);

    Pose OptimizeDeltaT(const vector<PoseStamped>& pose_ceres_a, const vector<PoseStamped>& pose_ceres_b);

    vector<PoseStamped> OptimizeWithRelativePose(const vector<PoseStamped>& poses_vins,
                                                 const PoseStamped& pose_start,
                                                 const PoseStamped& pose_end,
                                                 const PoseStamped& pose_start_vins,
                                                 const PoseStamped& pose_end_vins,
                                                 const PoseStamped& pose_ai
    );

    vector<PoseStamped> OptimizeWithRelativePose2(const vector<PoseStamped>& poses_vins,
                                                  const vector<PoseStamped>& poses_gt,
                                                  const PoseStamped& T_begin,
                                                  const PoseStamped& T_end,
                                                  const PoseStamped& T_vins_begin,
                                                  const PoseStamped& T_vins_end,
                                                  const PoseStamped& pose_ai
    );

    vector<PoseStamped> OptimizeWithRelativePoseInv(const vector<PoseStamped>& poses_vins_inv,
                                                    const vector<PoseStamped>& poses_gt_inv,
                                                    const PoseStamped& T_begin_inv,
                                                    const PoseStamped& T_end_inv,
                                                    const PoseStamped& T_vins_begin_inv,
                                                    const PoseStamped& T_vins_end_inv,
                                                    const PoseStamped& pose_ai);

    ceres::Solver::Summary SolveCeresProblem(ceres::Problem &problem);

}



#endif //ALIGNTRAJECTORY_OPTIMIZE_H
