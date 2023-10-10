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
#include <calib_manager/calib_manager.h>
#include "utils.h"
#include "optimize.h"

namespace fs=std::filesystem;



void SeparateRectification(const vector<PoseStamped>& pose_ceres_opt,
                           const vector<PoseStamped>& pose_ceres_vins,
                           const vector<PoseStamped>& vins_poses_lidar,
                           const vector<PoseStamped>& opt_poses_lidar);

void JointRectification(const vector<PoseStamped>& pose_ceres_opt,
                        const vector<PoseStamped>& pose_ceres_vins,
                        const vector<PoseStamped>& vins_poses_lidar,
                        const vector<PoseStamped>& opt_poses_lidar);

void SeparateRectiWithOffset(const vector<PoseStamped>& pose_ceres_opt,
                             const vector<PoseStamped>& pose_ceres_vins,
                             const vector<PoseStamped>& vins_poses_lidar,
                             const vector<PoseStamped>& opt_poses_lidar);

void SeparateRectiWithT(const vector<PoseStamped>& pose_ceres_opt,
                        const vector<PoseStamped>& pose_ceres_vins,
                        const vector<PoseStamped>& vins_poses_lidar,
                        const vector<PoseStamped>& opt_poses_lidar);

void SeparateRectiTwoStep(const vector<PoseStamped>& pose_ceres_opt,
                          const vector<PoseStamped>& pose_ceres_vins,
                          const vector<PoseStamped>& vins_poses_lidar,
                          const vector<PoseStamped>& opt_poses_lidar);

void SeparateRectiFullTrajectory(const vector<PoseStamped>& vins_poses_lidar,
                                 const vector<PoseStamped>& opt_poses_lidar);

void OptimizeTargetTrajectory(const vector<PoseStamped>& vins_poses_lidar,
                              const vector<PoseStamped>& opt_poses_lidar);


std::optional<PoseStamped> findCorrPose(const vector<PoseStamped>& poses,const PoseStamped& target_pose);

void OutputPoses(const string& save_path,
                 const vector<PoseStamped>& corrected_poses_lidar,
                 double end_estimate_time);

vector<PoseStamped> ComputeLocalPoses(const vector<PoseStamped> &poses);


DEFINE_string(vins_pose_path, "", "vins_pose_path");
DEFINE_string(op_pose_path, "", "op_pose_path");
DEFINE_string(out_pose_path, "./output", "out_pose_path");
DEFINE_string(calib_date, "20230608", "calib_date");
DEFINE_string(calib_vehicle, "HS5-001", "calib_vehicle");
DEFINE_int32(estimate_frame_num,720,"使用图像的前estimate_frame_num帧来估计Delta");
DEFINE_int32(rectify_frame_num,240,"使用图像的前estimate_frame_num帧来估计Delta");

DEFINE_int32(start_index,100,"开始补全的帧号（对于VINS的轨迹）");
DEFINE_int32(completion_len,100,"补全的长度（对于VINS的轨迹）");


Vec3d opt_utm_ref;
vector<PoseStamped> opt_poses_all;

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
    auto vins_poses_input_imu = ReadPosesFromTxt(FLAGS_vins_pose_path);
    opt_poses_all = ReadPosesFromTxt(FLAGS_op_pose_path);

    vector<PoseStamped> output_poses;
    output_poses.reserve(vins_poses_input_imu.size());

    ///读取外参
    calib_manager::CalibManager calib(FLAGS_calib_vehicle,std::stoi(FLAGS_calib_date));
    Eigen::Matrix4d T_lidar2_imu2= calib.GetSensorGraph()["lidar-1"]["imu-2"]; //T_bl = T^b_l
    Pose Pose_lidar2_imu2(T_lidar2_imu2);

    ///将VINS位姿转到lidar位姿 ,对应的OPT位姿
    vector<PoseStamped> vins_poses_lidar;
    vector<PoseStamped> opt_poses_lidar;
    opt_poses_lidar.reserve(vins_poses_lidar.size());
    vins_poses_lidar.reserve(vins_poses_input_imu.size());
    for(auto &pose:vins_poses_input_imu){
        auto opt_pose = findCorrPose(opt_poses_all, pose);
        if(!opt_pose)
            continue;
        //vins_poses_lidar.emplace_back(pose.leftMul(T_lidar2_imu2));
        vins_poses_lidar.emplace_back(pose * Pose_lidar2_imu2);
        opt_poses_lidar.push_back(*opt_pose);
    }
    ///为了数值稳定性,两个轨迹在同一原点
    opt_utm_ref = opt_poses_lidar[0].t() - vins_poses_lidar[0].t();
    for(auto& pose:opt_poses_lidar){
        pose.t() = pose.t() - opt_utm_ref;
    }

    WritePosesToTxt(FLAGS_out_pose_path+"_lidar_vins_poses.txt",vins_poses_lidar);
    WritePosesToTxt(FLAGS_out_pose_path+"_lidar_opt_poses.txt",opt_poses_lidar);

    ///计算每一帧相对第一帧的变换
    vector<PoseStamped> vins_poses_local = ComputeLocalPoses(vins_poses_lidar);
    vector<PoseStamped> opt_poses_local = ComputeLocalPoses(opt_poses_lidar);

    WritePosesToTxt(FLAGS_out_pose_path+"_local_vins_poses.txt",vins_poses_local);
    WritePosesToTxt(FLAGS_out_pose_path+"_local_opt_poses.txt",opt_poses_local);


    vector<PoseStamped> pose_ceres_opt;
    vector<PoseStamped> pose_ceres_vins;

    for(int i=0;i<vins_poses_lidar.size();++i){
        const PoseStamped& vins_pose = vins_poses_lidar[i];
        const PoseStamped& opt_pose = opt_poses_lidar[i];
        pose_ceres_vins.push_back(vins_pose);
        pose_ceres_opt.push_back(opt_pose);
        //cout<<VecToStr(Eigen::Vector3d(vins_pose.t() - opt_pose.t()))<<endl;
    }

    ///保存原始轨迹
/*    double end_rectify_time = vins_poses_lidar[FLAGS_estimate_frame_num+FLAGS_rectify_frame_num].time_stamp();
    vector<PoseStamped> poses_raw;
    poses_raw.reserve(opt_poses_all.size());
    for(auto& pose:opt_poses_all){
        poses_raw.push_back(pose);
    }
    WritePosesToTxt(FLAGS_out_pose_path+"_raw_poses.txt",poses_raw);*/


    //JointRectification(pose_ceres_opt,pose_ceres_vins,
    //                   vins_poses_lidar,opt_poses_lidar);

    //SeparateRectification(pose_ceres_opt, pose_ceres_vins,
    //                   vins_poses_lidar, opt_poses_lidar);

    //SeparateRectiWithOffset(pose_ceres_opt, pose_ceres_vins,
    //                        vins_poses_lidar, opt_poses_lidar);

    //SeparateRectiWithT(pose_ceres_opt, pose_ceres_vins,
    //                        vins_poses_lidar, opt_poses_lidar);

    //SeparateRectiFullTrajectory(vins_poses_lidar, opt_poses_lidar);

    //SeparateRectiTwoStep(pose_ceres_opt, pose_ceres_vins,
    //                   vins_poses_lidar, opt_poses_lidar);

    OptimizeTargetTrajectory(vins_poses_lidar,opt_poses_lidar);

    return 0;
}

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


/**
 * 第一种方式，优化一个T
 * @param pose_ceres_opt
 * @param pose_ceres_vins
 * @param opt_poses_all
 * @param vins_poses_lidar
 * @param opt_poses_lidar
 * @param end_estimate_time
 * @param size_estimate
 */
/*
void JointRectification(const vector<PoseStamped>& pose_ceres_opt,
                        const vector<PoseStamped>& pose_ceres_vins,
                        const vector<PoseStamped>& vins_poses_lidar,
                        const vector<PoseStamped>& opt_poses_lidar
){
    Pose deltaT = Optimize::OptimizeDeltaT(pose_ceres_opt, pose_ceres_vins);
    SPDLOG_INFO("delta pose:{}",deltaT.DebugString());

    vector<PoseStamped> poses_corrected;
    poses_corrected.reserve(vins_poses_lidar.size());
    for(int i=0;i<vins_poses_lidar.size();++i){
        const PoseStamped& vins_pose = vins_poses_lidar[i];
        const PoseStamped& opt_pose = opt_poses_lidar[i];
        PoseStamped corrected_pose = vins_pose.leftMul(&deltaT);
        //cout << VecToStr(Eigen::Vector3d(corrected_pose.t() - opt_pose.t())) << endl;
        poses_corrected.push_back(corrected_pose);
    }

    WritePosesToTxt(FLAGS_out_pose_path+"_T_corrected_local.txt", poses_corrected);

    double end_estimate_time = vins_poses_lidar[FLAGS_estimate_frame_num].time_stamp();
    OutputPoses(FLAGS_out_pose_path+"_T_corrected.txt",
                poses_corrected,end_estimate_time);
}
*/



/**
 * 第二种方法, 先求R,再平移
 * @param pose_ceres_opt
 * @param pose_ceres_vins
 * @param opt_poses_all
 * @param vins_poses_lidar
 * @param opt_poses_lidar
 * @param end_estimate_time
 * @param size_estimate
 */
/*
void SeparateRectification(const vector<PoseStamped>& pose_ceres_opt,
                           const vector<PoseStamped>& pose_ceres_vins,
                           const vector<PoseStamped>& vins_poses_lidar,
                           const vector<PoseStamped>& opt_poses_lidar
                        ){

    Quaterniond deltaR = Optimize::OptimizeDeltaR(pose_ceres_opt, pose_ceres_vins);
    SPDLOG_INFO("deltaR:{}", VecToStr(Vec3d(deltaR.toRotationMatrix().eulerAngles(2,1,0)*R2D)));

    ///矫正 R
    vector<PoseStamped> corrected_poses;
    corrected_poses.reserve(opt_poses_all.size());
    Vec3d delta_t_avg = Vec3d::Zero();
    for(int i=0;i<vins_poses_lidar.size();++i) {
        const PoseStamped &vins_pose = vins_poses_lidar[i];
        const PoseStamped &opt_pose = opt_poses_lidar[i];

        PoseStamped corrected_pose = vins_pose;
        corrected_pose.q() = deltaR * vins_pose.q();
        //corrected_pose.t() = deltaR * vins_pose.t();
        corrected_poses.push_back(corrected_pose);

        Vec3d error_t = opt_pose.t()-corrected_pose.t();
        delta_t_avg += error_t;
    }
    delta_t_avg /= static_cast<double>(vins_poses_lidar.size());
    SPDLOG_INFO("delta_t_avg:{}", VecToStr(delta_t_avg));

    ///矫正t
    for(int i=0; i < corrected_poses.size(); ++i) {
         PoseStamped &corrected_pose = corrected_poses[i];
        const PoseStamped &opt_pose = opt_poses_lidar[i];
        corrected_pose.t() += delta_t_avg;

        Vec3d error_t = opt_pose.t()-corrected_pose.t();
        Vec3d error_r = opt_pose.euler() - corrected_pose.euler();

        //cout << fmt::format("{} | {}",VecToStr(error_t),
        //                    VecToStr(error_r)) << endl;
    }

    CHECK_EQ(corrected_poses.size(), opt_poses_lidar.size());

    ///进行输出
    double end_estimate_time = vins_poses_lidar[FLAGS_estimate_frame_num].time_stamp();
    OutputPoses(FLAGS_out_pose_path+"_R_corrected.txt",corrected_poses,end_estimate_time);
}

*/

/*

void SeparateRectiWithOffset(const vector<PoseStamped>& pose_ceres_opt,
                             const vector<PoseStamped>& pose_ceres_vins,
                             const vector<PoseStamped>& vins_poses_lidar,
                             const vector<PoseStamped>& opt_poses_lidar
){

    Quaterniond deltaR = Optimize::OptimizeDeltaR(pose_ceres_opt, pose_ceres_vins);
    SPDLOG_INFO("deltaR:{}", VecToStr(Vec3d(deltaR.toRotationMatrix().eulerAngles(2,1,0)*R2D)));

    ///矫正 R
    vector<PoseStamped> corrected_poses_lidar;
    corrected_poses_lidar.reserve(opt_poses_all.size());
    for(int i=0;i<vins_poses_lidar.size();++i) {
        const PoseStamped &vins_pose = vins_poses_lidar[i];
        const PoseStamped &opt_pose = opt_poses_lidar[i];

        PoseStamped corrected_pose = vins_pose;
        corrected_pose.q() = deltaR * vins_pose.q();
        corrected_pose.t() = deltaR * vins_pose.t();
        corrected_poses_lidar.push_back(corrected_pose);
    }

    CHECK_EQ(corrected_poses_lidar.size(),opt_poses_lidar.size());

    ///矫正 t
    Vec3d t_offset = - corrected_poses_lidar[FLAGS_estimate_frame_num].t()
            +opt_poses_lidar[FLAGS_estimate_frame_num].t();

    for(int i=0;i<corrected_poses_lidar.size();++i) {
        corrected_poses_lidar[i].t() += t_offset;
    }

    double end_estimate_time = vins_poses_lidar[FLAGS_estimate_frame_num].time_stamp();
    OutputPoses(FLAGS_out_pose_path+"_Roffset_correct.txt",corrected_poses_lidar,end_estimate_time);
}

*/



/*

void SeparateRectiWithT(const vector<PoseStamped>& pose_ceres_opt,
                             const vector<PoseStamped>& pose_ceres_vins,
                             const vector<PoseStamped>& vins_poses_lidar,
                             const vector<PoseStamped>& opt_poses_lidar
){

    Quaterniond deltaR = Optimize::OptimizeDeltaR(pose_ceres_opt, pose_ceres_vins);
    SPDLOG_INFO("deltaR:{}", VecToStr(Vec3d(deltaR.toRotationMatrix().eulerAngles(2,1,0)*R2D)));

    Pose deltaT = Optimize::OptimizeDeltaT(pose_ceres_opt, pose_ceres_vins);
    SPDLOG_INFO("delta pose:{}",deltaT.DebugString());


    ///矫正 R
    vector<PoseStamped> corrected_poses_lidar;
    corrected_poses_lidar.reserve(opt_poses_all.size());
    for(int i=0;i<vins_poses_lidar.size();++i) {
        const PoseStamped &vins_pose = vins_poses_lidar[i];
        const PoseStamped &opt_pose = opt_poses_lidar[i];

        PoseStamped corrected_pose = vins_pose;
        corrected_pose.q() = deltaR * vins_pose.q();
        //corrected_pose.t() = deltaR * vins_pose.t();
        corrected_poses_lidar.push_back(corrected_pose);
    }

    CHECK_EQ(corrected_poses_lidar.size(),opt_poses_lidar.size());

    ///矫正 t
    Vec3d t_offset = deltaT.t();

    for(int i=0;i<corrected_poses_lidar.size();++i) {
        corrected_poses_lidar[i].t() += t_offset;
    }

    double end_estimate_time = vins_poses_lidar[FLAGS_estimate_frame_num].time_stamp();
    OutputPoses(FLAGS_out_pose_path+"_RT_correct.txt",corrected_poses_lidar,end_estimate_time);
}


*/


/*

void SeparateRectiTwoStep(const vector<PoseStamped>& pose_ceres_opt,
                        const vector<PoseStamped>& pose_ceres_vins,
                        const vector<PoseStamped>& vins_poses_lidar,
                        const vector<PoseStamped>& opt_poses_lidar
){
    Pose deltaT = Optimize::OptimizeDeltaT(pose_ceres_opt, pose_ceres_vins);
    SPDLOG_INFO("delta pose:{}",deltaT.DebugString());

    ///矫正 T
    vector<PoseStamped> pose_ceres_vins_correct;
    pose_ceres_vins_correct.reserve(pose_ceres_vins.size());
    for(const auto & vins_pose : pose_ceres_vins) {
        pose_ceres_vins_correct.push_back(vins_pose.leftMul(deltaT));
    }

    Quaterniond deltaR = Optimize::OptimizeDeltaR(pose_ceres_opt, pose_ceres_vins_correct);
    SPDLOG_INFO("deltaR:{}", VecToStr(Vec3d(deltaR.toRotationMatrix().eulerAngles(2,1,0)*R2D)));

    vector<PoseStamped> corrected_poses_lidar;
    corrected_poses_lidar.reserve(opt_poses_all.size());
    vector<PoseStamped> corrected_poses_lidar_1;
    corrected_poses_lidar_1.reserve(opt_poses_all.size());

    double end_rectify_time = vins_poses_lidar[FLAGS_estimate_frame_num+FLAGS_rectify_frame_num].time_stamp();

    for(int i=0;i<vins_poses_lidar.size();++i) {
        const PoseStamped &vins_pose = vins_poses_lidar[i];
        if(vins_pose.time_stamp()>end_rectify_time){
            break;
        }
        PoseStamped corrected_pose = vins_pose.leftMul(deltaT);///DeltaT
        corrected_poses_lidar_1.push_back(corrected_pose);

        corrected_pose.q() = deltaR * corrected_pose.q();///DeltaR
        corrected_poses_lidar.push_back(corrected_pose);
    }

    //CHECK_EQ(corrected_poses_lidar.size(),opt_poses_lidar.size());

    ///进行输出
    double end_estimate_time = vins_poses_lidar[FLAGS_estimate_frame_num].time_stamp();
    OutputPoses(FLAGS_out_pose_path+"_twoStep_correct.txt",corrected_poses_lidar,end_estimate_time);
    OutputPoses(FLAGS_out_pose_path+"_twoStep1_correct.txt",corrected_poses_lidar_1,end_estimate_time);
}

*/



void OutputPoses(const string& save_path,
                 const vector<PoseStamped>& corrected_poses_lidar,
                 double end_estimate_time){

    vector<PoseStamped> poses_output;
    poses_output.reserve(opt_poses_all.size());
    for(auto &pose:opt_poses_all){
        if(pose.time_stamp() <= end_estimate_time){
            poses_output.push_back(pose);
        }
    }

    for(int i=0;i<corrected_poses_lidar.size();++i){
        if(corrected_poses_lidar[i].time_stamp() <= end_estimate_time)
            continue;
        PoseStamped corrected_pose = corrected_poses_lidar[i];
        corrected_pose.t() += opt_utm_ref;
        poses_output.push_back(corrected_pose);
    }

    cout<<fmt::format("start correct time:{:.6f}",end_estimate_time)<<endl;

    WritePosesToTxt(save_path,poses_output);
}


/**
 * 将轨迹补全问题建模为优化问题
 * @param vins_poses_lidar
 * @param opt_poses_lidar
 */
void OptimizeTargetTrajectory(const vector<PoseStamped>& vins_poses_lidar,
                              const vector<PoseStamped>& opt_poses_lidar){

    CHECK_GE(FLAGS_start_index,1);
    CHECK_GE(FLAGS_completion_len,1);

    int end_index = FLAGS_start_index + FLAGS_completion_len;
    CHECK_LT(FLAGS_start_index,vins_poses_lidar.size());
    CHECK_LT(end_index,vins_poses_lidar.size());

    vector<PoseStamped> poses_vins(vins_poses_lidar.begin()+FLAGS_start_index,vins_poses_lidar.begin()+end_index);
    vector<PoseStamped> poses_gt(opt_poses_lidar.begin()+FLAGS_start_index,opt_poses_lidar.begin()+end_index);

    const PoseStamped& T_begin = opt_poses_lidar[FLAGS_start_index - 1];
    const PoseStamped& T_end = opt_poses_lidar[end_index];

    const PoseStamped& T_vins_begin = vins_poses_lidar[FLAGS_start_index - 1];
    const PoseStamped& T_vins_end = vins_poses_lidar[end_index];
    PoseStamped pose_ai = PoseStamped::Identity();

    SPDLOG_INFO("T_begin time:{:.6f}", T_begin.time_stamp());
    SPDLOG_INFO("T_end time:{:.6f}", T_end.time_stamp());

#if 0
    ///估计两个轨迹的一个外参
    int est_ex_num = 100;
    int estimate_start_index = std::max(0,start_index-est_ex_num);
    int estimate_end_index = std::min(start_index,estimate_start_index + est_ex_num);

    vector<PoseStamped> pose_ceres_opt;
    vector<PoseStamped> pose_ceres_vins;
    for(int i=estimate_start_index;i<=estimate_end_index;++i){
        const PoseStamped& vins_pose = vins_poses_lidar[i];
        const PoseStamped& opt_pose = opt_poses_lidar[i];
        pose_ceres_vins.push_back(vins_pose);
        pose_ceres_opt.push_back(opt_pose);
    }

    Quaterniond deltaR = Optimize::OptimizeDeltaR(pose_ceres_opt,pose_ceres_vins);

    ///外参矫正
    cout<<"deltaR:"<<deltaR.toRotationMatrix().eulerAngles(2,1,0)<<endl;
    for(auto& pose:poses_vins){
        pose.t() = deltaR*pose.t();
        pose.q() = deltaR*pose.q();
    }
#endif


#if 1
    vector<PoseStamped> outputs = Optimize::OptimizeWithRelativePose(
            poses_vins,T_begin, T_end,
            T_vins_begin,T_vins_end,pose_ai);
#else
    vector<PoseStamped> outputs = Optimize::OptimizeWithRelativePose2(
            poses_vins, poses_gt, T_begin,T_end,
            T_vins_begin, T_vins_end, pose_ai);
#endif

    ///下面写入轨迹
    vector<PoseStamped> poses_output;
    poses_output.reserve(opt_poses_all.size());
    for(auto &pose:opt_poses_all){
        if(pose.time_stamp() <= T_begin.time_stamp()){
            poses_output.push_back(pose);
        }
    }
    for(auto pose:outputs){
        pose.t() += opt_utm_ref;
        poses_output.push_back(pose);
    }
    vector<PoseStamped> poses_output_raw;
    poses_output_raw.reserve(opt_poses_all.size());
    for(auto &pose:opt_poses_all){
        if(pose.time_stamp() >= T_end.time_stamp()){
            poses_output.push_back(pose);
        }
        poses_output_raw.push_back(pose);
    }
    WritePosesToTxt(FLAGS_out_pose_path+"_optimize_result.txt",poses_output);
    WritePosesToTxtComma(FLAGS_out_pose_path+"_optimize_result_comma.txt",poses_output);
    WritePosesToTxt(FLAGS_out_pose_path+"_optimize_raw.txt",poses_output_raw);


    ///下面写入局部轨迹
    double margin_second = 1.;
    poses_output.clear();
    poses_output.reserve(opt_poses_lidar.size());
    for(auto &pose:opt_poses_lidar){
        if(pose.time_stamp() <= T_begin.time_stamp() &&
           pose.time_stamp() > T_begin.time_stamp() - margin_second){
            poses_output.push_back(pose);
        }
    }
    poses_output.insert(poses_output.end(),outputs.begin(),outputs.end());

    poses_output_raw.clear();
    poses_output_raw.reserve(opt_poses_lidar.size());
    for(auto &pose:opt_poses_lidar){
        if(pose.time_stamp() >= T_end.time_stamp() &&
           pose.time_stamp() < T_end.time_stamp() + margin_second){
            poses_output.push_back(pose);
        }
        if(pose.time_stamp() >= T_begin.time_stamp() - margin_second &&
           pose.time_stamp() < T_end.time_stamp() + margin_second){
            poses_output_raw.push_back(pose);
        }
    }

    WritePosesToTxt(FLAGS_out_pose_path+"_optimize_local_result.txt",poses_output);
    WritePosesToTxt(FLAGS_out_pose_path+"_optimize_local_raw.txt",poses_output_raw);
}


/**
 * 将轨迹变换到相对第一帧
 * @param poses
 * @return
 */
vector<PoseStamped> ComputeLocalPoses(const vector<PoseStamped> &poses){
    const PoseStamped& ref_pose = poses[0];
    PoseStamped ref_inv = ref_pose.Inverse();

    vector<PoseStamped> poses_local(poses.size());
    poses_local[0] = PoseStamped::Identity(poses[0].time_stamp());

    for(int i=1;i<poses.size();++i){
        poses_local[i] = ref_inv * poses[i];
        poses_local[i].time_stamp() = poses[i].time_stamp();
    }

    return poses_local;
}

