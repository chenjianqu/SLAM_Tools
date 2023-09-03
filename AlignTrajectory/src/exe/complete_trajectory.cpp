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
#include "factor/eigenquaternion_localparameterization.h"
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


Quaterniond OptimizeDeltaR(const vector<PoseStamped>& pose_ceres_op,const vector<PoseStamped>& pose_ceres_vins){
    ceres::Problem::Options problem_options;
    ceres::Problem problem(problem_options);

    Eigen::Quaterniond q_parameter = Quaterniond::Identity();
    auto* q_para = new EigenQuaternionLocalParameterization();
    problem.AddParameterBlock(q_parameter.coeffs().data(),4,q_para);

    int size_pose = pose_ceres_op.size();
    for(int i=0;i<size_pose;++i){
        auto* factor = new RotationFactor((Pose *) &pose_ceres_op[i], (Pose *)&pose_ceres_vins[i]);
        problem.AddResidualBlock(factor, nullptr, q_parameter.coeffs().data());
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

    return q_parameter;
}


Pose OptimizeDeltaT(const vector<PoseStamped>& pose_ceres_op,const vector<PoseStamped>& pose_ceres_vins){
    ceres::Problem::Options problem_options;
    ceres::Problem problem(problem_options);

    Pose pose_ans = Pose::Identity();

    Vec7d T_para;
    T_para.topRows(3) = pose_ans.t();
    T_para.bottomRows(4) = pose_ans.q().coeffs();
    problem.AddParameterBlock(T_para.data(),7,new PoseParameterization());

    int size_pose = pose_ceres_op.size();
    for(int i=0;i<size_pose;++i){
        auto* factor = new PoseFactor((Pose *) &pose_ceres_op[i], (Pose *)&pose_ceres_vins[i]);
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
    Eigen::Matrix4d T_lidar2_imu2 = Eigen::Matrix4d::Identity();

    ///将VINS位姿转到lidar位姿
    vector<PoseStamped> vins_poses_lidar;
    vins_poses_lidar.reserve(vins_poses_imu.size());
    for(auto &pose:vins_poses_imu){
        vins_poses_lidar.emplace_back(pose.leftMul(T_lidar2_imu2));
    }

    ///计算delta量
    Sophus::Vector3d delta_so3 = Sophus::Vector3d::Ones();
    Sophus::Vector3d delta_t = Sophus::Vector3d::Ones();

    //用vins的0.3比例的pose来计算delta量
    double ratio_estimate = 0.3;
    int size_estimate = 0;
    int size_poses = vins_poses_lidar.size();

    vector<PoseStamped> pose_ceres_op;
    vector<PoseStamped> pose_ceres_vins;

    for(int i=0;i<vins_poses_lidar.size();++i){
        ///用前30%和后30%来估计delta
        if( i >= ratio_estimate*size_poses &&
            i < (size_poses - ratio_estimate*size_poses) )
            continue;

        const PoseStamped& vins_pose = vins_poses_lidar[i];
        auto op_pose = findCorrPose(op_poses,vins_pose);
        if(!op_pose)
            continue;
        //cout<<fmt::format("op pose:{}",op_pose.DebugString())<<endl;
        //cout<<fmt::format("vins_lidar pose:{}",vins_pose.DebugString())<<endl;
        delta_so3 += Sophus::SO3d(op_pose->q()).log() - Sophus::SO3d(vins_pose.q()).log();
        size_estimate++;
        output_poses.push_back(*op_pose);

        pose_ceres_vins.push_back(vins_pose);
        pose_ceres_op.push_back(*op_pose);
    }
    delta_so3 /= static_cast<double>(size_estimate);

    ////Ceres求Delta R
    Pose delta_T4 = OptimizeDeltaT(pose_ceres_op,pose_ceres_vins);
    cout<<delta_T4.DebugString()<<endl;

    //Quaterniond delta_q = OptimizeDeltaR(pose_ceres_op,pose_ceres_vins);
    Quaterniond delta_q = delta_T4.q();

    Sophus::SO3d delta_R(delta_q);

    Vec3d delta_euler = delta_q.toRotationMatrix().eulerAngles(2,1,0)*R2D;
    cout<<fmt::format("优化出来的deltaR:{}", VecToStr(delta_euler))<<endl;


    vector<PoseStamped> output_poses_1;
    output_poses_1.reserve(vins_poses_imu.size());
    int ref_vins_index = 100;
    PoseStamped ref_pose_vins = vins_poses_lidar[ref_vins_index];
    PoseStamped ref_pose_inv = ref_pose_vins.Inverse();
    PoseStamped ref_pose_op = *findCorrPose(op_poses,ref_pose_vins);
    output_poses_1.push_back(ref_pose_op);

    cout<<"ref time:"<<ref_pose_vins.time_stamp()<<endl;
    for (int i=ref_vins_index+1;i<size_poses;++i){
        PoseStamped delta_pose = vins_poses_lidar[i].leftMul(&ref_pose_inv);
        delta_pose.q() = delta_q * delta_pose.q();
        delta_pose.t() = delta_q * delta_pose.t();
        PoseStamped propa_pose = delta_pose.leftMul(&ref_pose_op);
        //对比
        auto op_pose = findCorrPose(op_poses,propa_pose);
        cout<<fmt::format("             op pose:{}",op_pose->DebugString())<<endl;
        cout<<fmt::format("vins_compensate_pose:{}",propa_pose.DebugString())<<endl;

        output_poses_1.push_back(propa_pose);
    }

    ///将VINS时间戳开始前的OP位姿放进输出的数组
    vector<PoseStamped> poses_all_1;
    poses_all_1.reserve(op_poses.size());
    double begin_time_1 = ref_pose_vins.time_stamp();
    for(auto &pose:op_poses){
        if(pose.time_stamp() < begin_time_1){
            poses_all_1.push_back(pose);
        }
    }
    poses_all_1.insert(poses_all_1.end(),output_poses_1.begin(),output_poses_1.end());

    WritePosesToTxt(FLAGS_out_pose_path+"_test",poses_all_1);


    /*for(int i=0;i<vins_poses_lidar.size();++i){
        if( i >= ratio_estimate*size_poses &&
            i < (size_poses - ratio_estimate*size_poses) )
            continue;
        const PoseStamped& vins_pose = vins_poses_lidar[i];
        auto op_pose = findCorrPose(op_poses,vins_pose);
        if(!op_pose)
            continue;
        delta_t += op_pose->t() - delta_R*vins_pose.t();
    }
    delta_t /= static_cast<double>(size_estimate);

    cout<<fmt::format("estimate_delta_size:{} predict_size:{}",
                      size_estimate,vins_poses_lidar.size()-size_estimate)<<endl;
    cout<<"delta_t:"<<delta_t.transpose()<<endl;


    ///计算补偿后的VINS位姿
    for(int i=0;i<vins_poses_lidar.size();++i){
        if( i >= ratio_estimate*size_poses &&
            i < (size_poses - ratio_estimate*size_poses) )
        {
            const PoseStamped& vins_pose = vins_poses_lidar[i];
            auto op_pose = findCorrPose(op_poses,vins_pose);
            if(!op_pose)
                continue;
            PoseStamped vins_compensate_pose = vins_pose.leftMul(delta_R.unit_quaternion(),delta_t);
            output_poses.push_back(vins_compensate_pose);
            cout<<fmt::format("             op pose:{}",op_pose->DebugString())<<endl;
            cout<<fmt::format("vins_compensate_pose:{}",vins_compensate_pose.DebugString())<<endl;
        }
    }

    ///将VINS时间戳开始前的OP位姿放进输出的数组
    vector<PoseStamped> poses_all;
    poses_all.reserve(op_poses.size());
    double begin_time = vins_poses_lidar[0].time_stamp();
    double end_time = vins_poses_lidar[size_poses-1].time_stamp();
    for(auto &pose:op_poses){
        if(pose.time_stamp() < begin_time){
            poses_all.push_back(pose);
        }
    }
    poses_all.insert(poses_all.end(),output_poses.begin(),output_poses.end());
    for(auto &pose:op_poses){
        if(pose.time_stamp() > end_time){
            poses_all.push_back(pose);
        }
    }
    WritePosesToTxt(FLAGS_out_pose_path,poses_all);*/

    return 0;
}