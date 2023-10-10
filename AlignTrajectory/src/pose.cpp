//
// Created by cjq on 23-8-1.
//
#include "pose.h"


bool Pose::VerifyRotationMatrix(const Eigen::Matrix3d& R){
    double threshold = 0.0001;
    ///判断行列式是否为1
    if(std::abs(R.determinant()) - 1 > threshold){
        return false;
    }
    ///判断是否正交
    Eigen::Matrix3d I_or_not = R.transpose() * R;
    for(int i=0;i<3;++i){
        for(int j=0;j<3;++j){
            if(i==j && std::abs(I_or_not(i,j) - 1) > threshold){
                return false;
            }
            else{
                if(std::abs(I_or_not(i,j) - 0) > threshold)
                    return false;
            }
        }
    }
    return true;
}


