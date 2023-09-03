//
// Created by cjq on 23-7-25.
//
#include <iostream>
#include <filesystem>

#include <eigen3/Eigen/Eigen>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <spdlog/spdlog.h>

#include "../utils.h"

namespace fs=std::filesystem;

using namespace std;


DEFINE_string(est_pose_path, "", "est_pose_path");

int main(int argc, char *argv[]) {
    google::ParseCommandLineFlags(&argc, &argv, true);
    CHECK(!FLAGS_est_pose_path.empty())<<"please set -est_pose_path";

    if(!fs::exists(FLAGS_est_pose_path)){
        SPDLOG_ERROR("est_pose_path:{} not exists!",FLAGS_est_pose_path);
        return -1;
    }

    auto est_poses = ReadPosesFromTxt(FLAGS_est_pose_path);

    Eigen::AngleAxisd angle_ax_x(-0.5*M_PI, Eigen::Vector3d::UnitX());
    Eigen::Matrix3d rotation_m = angle_ax_x.toRotationMatrix();
    Eigen::Quaterniond rotation_q(rotation_m);

    for(auto& est:est_poses){
        est.q = rotation_q * est.q;
        est.t = rotation_q * est.t;
    }

    WritePosesToTxt("output_poses_rotated.txt",est_poses);

    return 0;
}
