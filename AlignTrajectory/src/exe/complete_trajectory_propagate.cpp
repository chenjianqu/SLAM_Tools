//
// Created by cjq on 23-7-28.
//
#include <filesystem>
#include <fstream>
#include <gflags/gflags.h>
#include <sophus/so3.hpp>
#include <ceres/ceres.h>
#include <calib_manager/calib_manager.h>
#include <spdlog/spdlog.h>
#include "utils.h"

namespace fs=std::filesystem;


std::optional<PoseStamped> findCorrPose(const vector<PoseStamped>& poses,const PoseStamped& target_pose){
    //lower_bound返回的是[begin, end)区间中第一个使cmp(element, value)为false的数。
    //查找第一个大于等于value的数
    auto pose_corr_it = std::lower_bound(poses.begin(),poses.end(),target_pose,
                                         [](const auto& element,const auto& value){
                                             return element.time_stamp() < value.time_stamp();
                                         });
    if(pose_corr_it == poses.end()){
        SPDLOG_ERROR("Can not find stamp:{} in opt_poses",target_pose.time_stamp());
        return {};
    }
    return {*pose_corr_it};
}


DEFINE_string(vins_pose_path, "", "vins_pose_path");
DEFINE_string(op_pose_path, "", "op_pose_path");
DEFINE_string(out_pose_path, "./output_pose.txt", "out_pose_path");
DEFINE_string(calib_date, "20230608", "calib_date");
DEFINE_string(calib_vehicle, "HS5-001", "calib_vehicle");

int main(int argc, char *argv[]) {
    google::ParseCommandLineFlags(&argc, &argv, true);

    CHECK(!FLAGS_vins_pose_path.empty()) << "please set -vins_pose_path";
    CHECK(!FLAGS_op_pose_path.empty()) << "please set -gt_pose_path";
    CHECK(!FLAGS_out_pose_path.empty()) << "please set -out_pose_path";
    CHECK(!FLAGS_calib_date.empty()) << "please set -calib_date";
    CHECK(!FLAGS_calib_vehicle.empty()) << "please set -calib_vehicle";

    if (!fs::exists(FLAGS_vins_pose_path)) {
        SPDLOG_ERROR("vins_pose_path:{} not exists!", FLAGS_vins_pose_path);
        return -1;
    }
    if (!fs::exists(FLAGS_op_pose_path)) {
        SPDLOG_ERROR("gt_pose_path:{} not exists!", FLAGS_op_pose_path);
        return -1;
    }

    fs::path output_dir = fs::path(FLAGS_out_pose_path).parent_path();
    if(!fs::exists(output_dir)){
        fs::create_directories(output_dir);
        SPDLOG_INFO("Create dir:{}",output_dir.string());
    }
    auto vins_poses_imu = ReadPosesFromTxt(FLAGS_vins_pose_path);
    auto op_poses = ReadPosesFromTxt(FLAGS_op_pose_path);
    vector<PoseStamped> output_poses;
    output_poses.reserve(vins_poses_imu.size());

    ///读取外参
    calib_manager::CalibManager calib(FLAGS_calib_vehicle,std::stoi(FLAGS_calib_date));
    Eigen::Matrix4d T_lidar2_imu2 = calib.GetSensorGraph()["lidar-1"]["imu-2"];

    ///将VINS位姿转到lidar位姿
    vector<PoseStamped> vins_poses_lidar;
    vins_poses_lidar.reserve(vins_poses_imu.size());
    for(auto &pose:vins_poses_imu){
        vins_poses_lidar.emplace_back(pose.leftMul(T_lidar2_imu2));
    }


    //用vins的0.3比例的pose来计算delta量
    double ratio_estimate = 0.3;
    int size_estimate = 0;
    int size_poses = vins_poses_lidar.size();
    int ref_vins_index = 100;

    PoseStamped ref_pose_vins = vins_poses_lidar[ref_vins_index];
    PoseStamped ref_pose_inv = ref_pose_vins.Inverse();
    PoseStamped ref_pose_op = *findCorrPose(op_poses,ref_pose_vins);
    output_poses.push_back(ref_pose_op);

    cout<<"ref time:"<<ref_pose_vins.time_stamp()<<endl;
    for (int i=ref_vins_index+1;i<size_poses;++i){
        PoseStamped delta_pose = vins_poses_lidar[i].leftMul(&ref_pose_inv);
        PoseStamped propa_pose = delta_pose.leftMul(&ref_pose_op);
        output_poses.push_back(propa_pose);
    }



    ///将VINS时间戳开始前的OP位姿放进输出的数组
    vector<PoseStamped> poses_all;
    poses_all.reserve(op_poses.size());
    double begin_time = ref_pose_vins.time_stamp();
    double end_time = std::numeric_limits<double>::max();
    for(auto &pose:op_poses){
        if(pose.time_stamp() < begin_time){
            poses_all.push_back(pose);
        }
    }
    poses_all.insert(poses_all.end(),output_poses.begin(),output_poses.end());

    WritePosesToTxt(FLAGS_out_pose_path,poses_all);

    return 0;
}