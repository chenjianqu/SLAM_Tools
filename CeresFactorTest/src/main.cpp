#include <iostream>
#include <random>

#include <ceres/ceres.h>
#include <pcl/visualization/cloud_viewer.h>

#include "def.h"
#include "box_factor.h"

using PointT=pcl::PointXYZRGB;

vector<Vec3d> GeneratePoints(int n,Vec3d &center,double sigma){
    vector<Vec3d> points;
    points.reserve(n);

    std::default_random_engine random_engine;
    std::normal_distribution<double> normal_dist(0,sigma);

    for (int i = 0; i < n; ++i) {
        points.emplace_back(center.x()+normal_dist(random_engine),
                            center.y()+normal_dist(random_engine),
                            center.z()+normal_dist(random_engine));
    }

    return points;
}





int main() {
    std::cout << "Hello, World!" << std::endl;
    int n=100;
    Vec3d init_P(0.5,0.01,0.01);
    Vec3d init_dims(0.8,0.8,0.8);

    Vec3d center(2,2,0);
    vector<Vec3d> points = GeneratePoints( n,center,0.1);

    pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
    for(int i=0;i<n;++i){
        PointT p(255,0,0);
        p.x = points[i].x();
        p.y = points[i].y();
        p.z = points[i].z();
        cloud->points.emplace_back(p);
    }


    TicToc tt,t_all;

    ceres::Problem problem;
    ceres::LossFunction *loss_function = new ceres::HuberLoss(1.0);

    Mat3d rotation;
    rotation.setIdentity();
    Eigen::Quaterniond q(rotation);

    double dims[3]={init_dims.x(),init_dims.y(),init_dims.z()};
    double pose_para[7]={init_P.x(),init_P.y(),init_P.z(), q.x() ,q.y(),q.z(),q.w()};

    ///初始点
    PointT p_raw(0,0,255);
    p_raw.x = pose_para[0];
    p_raw.y = pose_para[1];
    p_raw.z = pose_para[2];
    cloud->points.push_back(p_raw);

    ///添加残差块

    ///三角化点落在包围框内
    /*for(int i=0;i<points.size();++i){
        problem.AddResidualBlock(new BoxEncloseStereoPointFactor(points[i]),
                                 loss_function,
                                 pose_para,dims);
    }*/


    //约束点

    PointT p_constraint(0,255,0);
    p_constraint.x = center.x();
    p_constraint.y = center.y();
    p_constraint.z = center.z();
    cloud->points.push_back(p_constraint);
    ///某个点的误差
    /*problem.AddResidualBlock(new BoxPoseNormFactor(Eigen::Vector3d(p_constraint.x,p_constraint.y,p_constraint.z),
                                                   Eigen::Matrix3d::Identity(),Eigen::Vector3d::Zero(),
                                                   Eigen::Matrix3d::Identity(),Eigen::Vector3d::Zero()),
                             loss_function,pose_para);*/

    problem.AddResidualBlock(new BoxPositionFactor(Eigen::Vector3d(p_constraint.x,p_constraint.y,p_constraint.z),
                                                   Eigen::Matrix3d::Identity(),Eigen::Vector3d::Zero(),
                                                   Eigen::Matrix3d::Identity(),Eigen::Vector3d::Zero()),
                             loss_function,pose_para);


    ///设置ceres选项
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    //options.num_threads = 2;
    options.trust_region_strategy_type = ceres::DOGLEG;
    options.max_num_iterations = 20;
    //options.use_explicit_schur_complement = true;
    //options.minimizer_progress_to_stdout = true;
    //options.use_nonmonotonic_steps = true;
    options.max_solver_time_in_seconds = 0.08;

    ///求解
    TicToc t_solver;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    fmt::print("InstanceManager::Optimization 优化完成 Iterations: {}\n", summary.iterations.size());
    fmt::print("InstanceManager::Optimization | Solve:{} ms \n",tt.TocThenTic());
    fmt::print("InstanceManager::Optimization all:{} ms \n",t_all.Toc());

    fmt::print("初始:P:({}) target::({})\n", VecToStr(init_P), VecToStr(center));
    fmt::print("优化结果:P:({},{},{}) dim:({},{},{})\n",pose_para[0],pose_para[1],pose_para[2],
               dims[0],dims[1],dims[2]);

    ///绘制结果
    PointT p(0,0,255);
    p.x = pose_para[0];
    p.y = pose_para[1];
    p.z = pose_para[2];
    cloud->points.push_back(p);


    pcl::visualization::CloudViewer viewer ("test");
    viewer.showCloud(cloud);
    while (!viewer.wasStopped()){ };


    return 0;
}
