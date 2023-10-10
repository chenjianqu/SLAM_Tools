//
// Created by cjq on 23-7-31.
//

#include "optimize.h"

#include <sophus/so3.hpp>
#include <ceres/ceres.h>
#include <ceres/loss_function.h>
#include <ceres/manifold.h>

#include "factor/local_parameterization.h"
#include "factor/pose_factor.h"

namespace Optimize{\


Quaterniond OptimizeDeltaR(const vector<PoseStamped>& pose_ceres_op,const vector<PoseStamped>& pose_ceres_vins){
    ceres::Problem::Options problem_options;
    ceres::Problem problem(problem_options);

    Eigen::Quaterniond q_parameter = pose_ceres_op[0].q() * pose_ceres_vins[0].q().inverse();
    auto* q_para = new QuaternionLocalParameterization();
    problem.AddParameterBlock(q_parameter.coeffs().data(),4,q_para);

    int size_pose = pose_ceres_op.size();
    for(int i=0;i<size_pose;++i){
        auto* factor = new RotationFactor((Pose *) &pose_ceres_op[i], (Pose *)&pose_ceres_vins[i]);
        problem.AddResidualBlock(factor, nullptr, q_parameter.coeffs().data());
    }

    SolveCeresProblem(problem);

    return q_parameter;
}



Pose OptimizeDeltaT(const vector<PoseStamped>& pose_ceres_a, const vector<PoseStamped>& pose_ceres_b){
    ceres::Problem::Options problem_options;
    ceres::Problem problem(problem_options);

    ///设置初值
    Pose pose_ans = pose_ceres_b[0].leftMul(pose_ceres_a[0].Inverse());

    Vec7d T_para = pose_ans.MakeVec7d();
    problem.AddParameterBlock(T_para.data(),7,new PoseLocalParameterization());

    int size_pose = pose_ceres_a.size();
    for(int i=0;i<size_pose;++i){
        auto* factor = new PoseFactor((Pose *) &pose_ceres_a[i], (Pose *)&pose_ceres_b[i]);
        problem.AddResidualBlock(factor, nullptr, T_para.data());
    }

    SolveCeresProblem(problem);

    pose_ans.t() = T_para.topRows(3);
    pose_ans.q().coeffs() = T_para.bottomRows(4);

    return pose_ans;
}


vector<PoseStamped> OptimizeWithRelativePose(const vector<PoseStamped>& poses_vins,
                                             const PoseStamped& pose_start,
                                             const PoseStamped& pose_end,
                                             const PoseStamped& pose_start_vins,
                                             const PoseStamped& pose_end_vins,
                                             const PoseStamped& pose_ai
){
    CHECK_GE(poses_vins.size(),2);
    CHECK_LT(pose_start.time_stamp(),poses_vins[0].time_stamp());
    CHECK_GT(pose_end.time_stamp(),poses_vins[poses_vins.size()-1].time_stamp());

    cout<<"pose_ai:"<<pose_ai.DebugString()<<endl;

    int vins_size = poses_vins.size();

    ceres::Problem::Options problem_options;
    ceres::Problem problem(problem_options);

    ceres::LossFunction *loss_function;
    loss_function = new ceres::HuberLoss(1.);

    ///固定前后两frame
    Vec7d pose_start_para = pose_start.MakeVec7d();
    Vec7d pose_end_para = pose_end.MakeVec7d();
    problem.AddParameterBlock(pose_start_para.data(),7,new PoseLocalParameterization());
    problem.AddParameterBlock(pose_end_para.data(),7,new PoseLocalParameterization());

    problem.SetParameterBlockConstant(pose_start_para.data());
    problem.SetParameterBlockConstant(pose_end_para.data());

    vector<PoseStamped> poses_before;
    vector<Vec7d> poses_vins_para(vins_size);
    for(int i=0;i<vins_size;++i){///添加中间的节点
        PoseStamped pose = pose_ai * poses_vins[i];
        poses_before.push_back(pose);
        ///初值很重要
        //poses_vins_para[i] = pose.MakeVec7d();
        poses_vins_para[i].topRows(3) = Vec3d::Zero();
        poses_vins_para[i].bottomRows(4) = Quaterniond::Identity().coeffs();
        problem.AddParameterBlock(poses_vins_para[i].data(),7,new PoseLocalParameterization());
    }

    //误差: e = T_a^-1  *  T_e  * T_b，其中 T_a是后一帧(k+1)， T_e是相对变换， T_b是前一帧(k)

    ///添加约束
    PoseStamped relative_pose = pose_ai * poses_vins[0] *(pose_ai * pose_start_vins).Inverse();

    cout<<"actual 0:"<<poses_vins[0].DebugString()<<endl;
    cout<<"predict 0:"<<(relative_pose * pose_start).DebugString()<<endl;

    auto* before_factor = new RelativePoseFactor(static_cast<Pose>(relative_pose));
    problem.AddResidualBlock(before_factor, nullptr,
                             poses_vins_para[0].data(),
                             pose_start_para.data());

    relative_pose = pose_ai * pose_end_vins *(pose_ai * poses_vins[vins_size-1]).Inverse();
    auto* after_factor = new RelativePoseFactor(static_cast<Pose>(relative_pose));
    problem.AddResidualBlock(after_factor, nullptr, pose_end_para.data(),
                             poses_vins_para[poses_vins.size()-1].data());

    for(int i=0;i<vins_size-1;++i){
        relative_pose = pose_ai * poses_vins[i+1] *(pose_ai * poses_vins[i]).Inverse();
        auto* factor = new RelativePoseFactor(static_cast<Pose>(relative_pose));
        problem.AddResidualBlock(factor, loss_function,
                                 poses_vins_para[i+1].data(),poses_vins_para[i].data());
    }


    SolveCeresProblem(problem);

    vector<PoseStamped> outputs(poses_vins.size());
    for(int i=0;i<poses_vins.size();++i){
        outputs[i].time_stamp() = poses_vins[i].time_stamp();
        outputs[i].SetVec7d(poses_vins_para[i]);
        //cout<<fmt::format("{} | {} -> {}",i,poses_before[i].DebugString(),
        //                  outputs[i].DebugString())<<endl;
    }

    return outputs;
}




vector<PoseStamped> OptimizeWithRelativePose2(const vector<PoseStamped>& poses_vins,
                                              const vector<PoseStamped>& poses_gt,
                                              const PoseStamped& T_begin,
                                              const PoseStamped& T_end,
                                              const PoseStamped& T_vins_begin,
                                              const PoseStamped& T_vins_end,
                                              const PoseStamped& pose_ai){
    CHECK_GE(poses_vins.size(),2);
    CHECK_LT(T_begin.time_stamp(), poses_vins[0].time_stamp());
    CHECK_GT(T_end.time_stamp(), poses_vins[poses_vins.size() - 1].time_stamp());

    cout<<"pose_ai:"<<pose_ai.DebugString()<<endl;

    int vins_size = poses_vins.size();

    PoseStamped T_begin_inv = T_begin.Inverse();
    PoseStamped T_end_inv = T_end.Inverse();
    PoseStamped T_vins_begin_inv = T_vins_begin.Inverse();
    PoseStamped T_vins_end_inv = T_vins_end.Inverse();

    vector<PoseStamped> poses_vins_inv(vins_size);
    for(int i=0;i<vins_size;++i){
        poses_vins_inv[i] = poses_vins[i].Inverse();
    }

    vector<PoseStamped> poses_gt_inv(poses_gt.size());
    for(int i=0;i<poses_gt.size();++i){
        poses_gt_inv[i] = poses_gt[i].Inverse();
    }

    return OptimizeWithRelativePoseInv(poses_vins_inv,poses_gt_inv,T_begin_inv,T_end_inv,
                                       T_vins_begin_inv,T_vins_end_inv,pose_ai);
}


vector<PoseStamped> OptimizeWithRelativePoseInv(const vector<PoseStamped>& poses_vins_inv,
                                                const vector<PoseStamped>& poses_gt_inv,
                                                const PoseStamped& T_begin_inv,
                                                const PoseStamped& T_end_inv,
                                                const PoseStamped& T_vins_begin_inv,
                                                const PoseStamped& T_vins_end_inv,
                                                const PoseStamped& pose_ai){

    int vins_size = poses_vins_inv.size();

    ceres::Problem::Options problem_options;
    ceres::Problem problem(problem_options);

    ///优化变量,并设置初值
    ceres::LossFunction *loss_function = nullptr;
    //loss_function = new ceres::HuberLoss(1.);

    ///固定前后两frame
    Vec7d T_begin_para = T_begin_inv.MakeVec7d();
    Vec7d T_end_para = T_end_inv.MakeVec7d();
    problem.AddParameterBlock(T_begin_para.data(), 7, new PoseLocalParameterization());
    problem.AddParameterBlock(T_end_para.data(), 7, new PoseLocalParameterization());

    problem.SetParameterBlockConstant(T_begin_para.data());
    problem.SetParameterBlockConstant(T_end_para.data());

    vector<Vec7d> poses_vins_para(vins_size);
    for(int i=0;i<vins_size;++i){///添加中间的节点
        PoseStamped pose = pose_ai * poses_vins_inv[i];
        //poses_vins_para[i] = pose.MakeVec7d();
        poses_vins_para[i].topRows(3) = Vec3d::Zero();
        poses_vins_para[i].bottomRows(4) = Quaterniond::Identity().coeffs();
        problem.AddParameterBlock(poses_vins_para[i].data(),7,new PoseLocalParameterization());
    }

    ///添加约束
    Eigen::Matrix<double, 6, 6> sqrt_information = Eigen::Matrix<double,6,6>::Identity();
    sqrt_information(0,0) = 100.;
    sqrt_information(1,1) = 100.;
    sqrt_information(2,2) = 100.;
    sqrt_information(3,3) = 10000.;
    sqrt_information(4,4) = 10000.;
    sqrt_information(5,5) = 10000.;

    PoseStamped relative_pose = T_vins_begin_inv.Inverse() * poses_vins_inv[0];
    ceres::CostFunction* cost_function=
            PoseGraph3dErrorTerm::Create(static_cast<Pose>(relative_pose), sqrt_information);
    //第一个参数是第k帧，第二个参数是第k+1帧
    problem.AddResidualBlock(cost_function, loss_function, T_begin_para.data(), poses_vins_para[0].data());

    cout<<"actual 0:"<<poses_gt_inv[0].Inverse().DebugString()<<endl;
    cout << "predict 0:" << (T_begin_inv * relative_pose).Inverse().DebugString() << endl;

    relative_pose = poses_vins_inv[vins_size-1].Inverse() * T_vins_end_inv;
    cost_function = PoseGraph3dErrorTerm::Create(static_cast<Pose>(relative_pose), sqrt_information);
    problem.AddResidualBlock(cost_function, loss_function, poses_vins_para[vins_size-1].data(), T_end_para.data());

    for(int i=0;i<vins_size-1;++i){
        relative_pose = poses_vins_inv[i].Inverse() * poses_vins_inv[i+1];
        cost_function = PoseGraph3dErrorTerm::Create(static_cast<Pose>(relative_pose), sqrt_information);
        problem.AddResidualBlock(cost_function, loss_function,
                                 poses_vins_para[i].data(),poses_vins_para[i+1].data());
    }


    SolveCeresProblem(problem);

    vector<PoseStamped> outputs(vins_size);
    for(int i=0;i<vins_size;++i){
        outputs[i].time_stamp() = poses_vins_inv[i].time_stamp();
        outputs[i].SetVec7d(poses_vins_para[i]);

        outputs[i] = outputs[i].Inverse();

        //cout<<fmt::format("{} | {}",i,outputs[i].DebugString())<<endl;
    }

//        relative_pose = pose_ai * outputs[0] *(pose_ai * T_begin).Inverse();
//        cout<<fmt::format("opt {:.3f}-{:.3f} relative:{}",
//                          T_begin.time_stamp(),outputs[0].time_stamp(),relative_pose.DebugString())<<endl;

    return outputs;
}


ceres::Solver::Summary SolveCeresProblem(ceres::Problem &problem){
    TicToc t_solver;
    ceres::Solver::Options options;
    options.num_threads = 10;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.max_num_iterations = 20000;
    options.check_gradients = false;
    options.parameter_tolerance = 1e-12;
    options.function_tolerance = 1e-8;
    options.eta = 1e-1;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    cout<<summary.BriefReport()<<endl;
    SPDLOG_INFO("ceres used time: {:.2f} s",t_solver.Toc());

    return summary;
}


}