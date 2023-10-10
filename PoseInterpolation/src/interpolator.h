//
// Created by cjq on 23-5-11.
//

#ifndef POSEINTERPOLATION_INTERPOLATOR_H
#define POSEINTERPOLATION_INTERPOLATOR_H

#include "def.h"
#include "tum_pose.h"

vector<TumPose> PoseInterpolate(const vector<TumPose>& key_poses,const vector<double> &sample_times);



#endif //POSEINTERPOLATION_INTERPOLATOR_H
