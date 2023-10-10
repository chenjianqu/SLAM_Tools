//
// Created by cjq on 23-7-28.
//
#include <filesystem>
#include <fstream>
#include <optional>
#include <gflags/gflags.h>
#include <sophus/so3.hpp>
#include <ceres/ceres.h>
#include <ceres/manifold.h>
#include <spdlog/spdlog.h>
#include <spdlog/fmt/fmt.h>
#include "utils.h"
#include "factor/local_parameterization.h"
#include "factor/pose_factor.h"

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


Pose OptimizeDeltaT(const vector<PoseStamped>& pose_ceres_a, const vector<PoseStamped>& pose_ceres_b){
    ceres::Problem::Options problem_options;
    ceres::Problem problem(problem_options);

    Pose pose_ans = Pose::Identity();

    Vec7d T_para;
    T_para.topRows(3) = pose_ans.t();
    T_para.bottomRows(4) = pose_ans.q().coeffs();
    problem.AddParameterBlock(T_para.data(),7,new PoseLocalParameterization());

    int size_pose = pose_ceres_a.size();
    for(int i=0;i<size_pose;++i){
        auto* factor = new PoseFactor((Pose *) &pose_ceres_a[i], (Pose *)&pose_ceres_b[i]);
        problem.AddResidualBlock(factor, nullptr, T_para.data());
    }

    TicToc t_solver;
    ceres::Solver::Options options;
    options.num_threads = 1;
    options.linear_solver_type = ceres::DENSE_QR;
    options.max_num_iterations = 100;
    options.check_gradients = false;
    options.gradient_check_relative_precision = 1e-4;
    options.eta = 1e-1;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    cout<<summary.BriefReport()<<endl;

    pose_ans.t() = T_para.topRows(3);
    pose_ans.q().coeffs() = T_para.bottomRows(4);

    return pose_ans;
}

DEFINE_string(poses_b_path, "", "poses_b_path");
DEFINE_string(poses_a_path, "", "poses_a_path");

/// T_a = DeltaT * T_b

int main(int argc, char *argv[]) {
    google::ParseCommandLineFlags(&argc, &argv, true);

    CHECK(!FLAGS_poses_b_path.empty()) << "please set -poses_b_path";
    CHECK(!FLAGS_poses_a_path.empty()) << "please set -poses_a_path";

    if (!fs::exists(FLAGS_poses_b_path)) {
        SPDLOG_ERROR("vins_pose_path:{} not exists!", FLAGS_poses_b_path);
        return -1;
    }
    if (!fs::exists(FLAGS_poses_a_path)) {
        SPDLOG_ERROR("gt_pose_path:{} not exists!", FLAGS_poses_a_path);
        return -1;
    }

    auto poses_raw_b = ReadPosesFromTxt(FLAGS_poses_b_path);
    auto poses_raw_a = ReadPosesFromTxt(FLAGS_poses_a_path);

    int size_estimate = 0;
    vector<PoseStamped> pose_ceres_a;
    vector<PoseStamped> pose_ceres_b;

    for(int i=0; i < poses_raw_b.size(); ++i){
        const PoseStamped& pose_b = poses_raw_b[i];
        auto pose_a = findCorrPose(poses_raw_a, pose_b);
        if(!pose_a)
            continue;
        size_estimate++;
        pose_ceres_b.push_back(pose_b);
        pose_ceres_a.push_back(*pose_a);
    }

    // T_a = DeltaT * T_b
    Pose delta_T = OptimizeDeltaT(pose_ceres_a, pose_ceres_b);
    cout<<"Delta T:"<<delta_T.DebugString()<<endl;

    ///Compute error
    for(int i=0; i < pose_ceres_a.size(); ++i){
        auto predict_pose_a = pose_ceres_b[i].leftMul(&delta_T);
        Vec3d error_t = predict_pose_a.t() - pose_ceres_a[i].t();
        cout << fmt::format("{} : {}", predict_pose_a.time_stamp(), VecToStr(error_t)) << endl;
    }


    WriteOnePoseToTxt("output_pose.txt",&delta_T);

    return 0;
}