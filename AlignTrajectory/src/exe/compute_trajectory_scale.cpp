//
// Created by cjq on 23-7-29.
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


/**
 * 位姿插值
 * @param key_poses 关键帧的位姿
 * @param sample_times 要插值的时间点
 * @return 插值后的位姿
 */
vector<Pose> PoseInterpolate(const vector<Pose>& key_poses,const vector<double> &sample_times){
    vector<Pose> outputs;

    int k1=0,k2=1;

    for(double t:sample_times){
        if(k2>=key_poses.size()){
            break;
        }

        //位姿t在k1关键帧的左边,使用k1,k2插值到左边
        if(t < key_poses[k1].time_stamp){
            ///TODO
            auto msg = fmt::format("t < k1 and k2, t:{:.3f},k1:{:.3f}",t,key_poses[k1].time_stamp);
            //throw std::runtime_error();
            std::cerr<<msg<<endl;
            continue;
        }
            //位姿t在k2关键帧的右边，此时应向右查找关键帧
        else if(t > key_poses[k2].time_stamp){
            if(k2 >= key_poses.size()){//如果无法向右查找，则使用当前的两个kp进行插值
                ///TODO
                auto msg = fmt::format("t > k1 and k2, t:{:.3f},k2:{:.3f}",t,key_poses[k2].time_stamp);
                //throw std::runtime_error();
                std::cerr<<msg<<endl;
                continue;
            }
            else{ //向右查找
                while(k2<key_poses.size() && t > key_poses[k2].time_stamp){
                    k1++;
                    k2++;
                }
            }
        }

        ///时刻刚好相等的情况下
        if(t == key_poses[k1].time_stamp){
            outputs.push_back(key_poses[k1]);
        }
        else if(t == key_poses[k2].time_stamp){
            outputs.push_back(key_poses[k2]);
            k1=k2;
            k2++;
        }
            //位姿t在k1,k2的中间
        else if(t > key_poses[k1].time_stamp && t<key_poses[k2].time_stamp){
            double percent = (t - key_poses[k1].time_stamp) /(key_poses[k2].time_stamp - key_poses[k1].time_stamp);
            Eigen::Vector3d P_t = key_poses[k1].t + percent * (key_poses[k2].t - key_poses[k1].t);

            Eigen::Quaterniond Q_t = key_poses[k1].q.slerp(percent,key_poses[k2].q);//球面插值
            Q_t.normalize();

            Pose p;
            p.time_stamp = t;
            p.t = P_t;
            p.q = Q_t;
            outputs.emplace_back(p);
        }
    }

    return outputs;
}



DEFINE_string(est_pose_path, "", "est_pose_path");
DEFINE_string(gt_pose_path, "", "gt_pose_path");

int main(int argc, char *argv[]) {
    google::ParseCommandLineFlags(&argc, &argv, true);
    CHECK(!FLAGS_est_pose_path.empty())<<"please set -est_pose_path";
    CHECK(!FLAGS_gt_pose_path.empty())<<"please set -gt_pose_path";

    if(!fs::exists(FLAGS_est_pose_path)){
        SPDLOG_ERROR("est_pose_path:{} not exists!",FLAGS_est_pose_path);
        return -1;
    }
    if(!fs::exists(FLAGS_gt_pose_path)){
        SPDLOG_ERROR("gt_pose_path:{} not exists!",FLAGS_gt_pose_path);
        return -1;
    }

    auto est_poses = ReadPosesFromTxt(FLAGS_est_pose_path);
    auto gt_poses = ReadPosesFromTxt(FLAGS_gt_pose_path);


    ///位姿插值
    vector<double> est_time_vec;
    est_time_vec.reserve(est_poses.size());
    for (auto &est : est_poses){
        est_time_vec.push_back(est.time_stamp);
    }
    std::vector<Pose> gt_poses_interp = PoseInterpolate(gt_poses,est_time_vec);

    //est.t * scale = gt.t

    double est_dist = 0;
    Eigen::Vector3d last_t;
    bool is_init= false;
    for(auto &est:est_poses){
        if(!is_init){
            is_init = true;
            last_t = est.t;
            continue;
        }
        Eigen::Vector3d vec = est.t - last_t;
        last_t = est.t;
        est_dist += vec.norm();
    }
    double gt_dist = 0;
    is_init= false;
    for(auto &gt:gt_poses_interp){
        if(!is_init){
            is_init = true;
            last_t = gt.t;
            continue;
        }
        Eigen::Vector3d vec = gt.t - last_t;
        last_t = gt.t;
        gt_dist += vec.norm();
    }

    double scale = gt_dist / est_dist;

    cout<<"scale:"<<scale<<endl;

    for(auto &est:est_poses){
        est.t = est.t * scale;
    }

    ///旋转位姿
    Eigen::AngleAxisd angle_ax_x(-0.5*M_PI, Eigen::Vector3d::UnitX());
    Eigen::Matrix3d rotation_m = angle_ax_x.toRotationMatrix();
    Eigen::Quaterniond rotation_q(rotation_m);

    for(auto& est:est_poses){
        est.q = rotation_q * est.q;
        est.t = rotation_q * est.t;
    }

    WritePosesToTxt("output_poses.txt",est_poses);

    return 0;
}
