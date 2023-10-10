//
// Created by cjq on 23-5-11.
//

#include "interpolator.h"

/**
 * 位姿插值
 * @param key_poses 关键帧的位姿
 * @param sample_times 要插值的时间点
 * @return 插值后的位姿
 */
vector<TumPose> PoseInterpolate(const vector<TumPose>& key_poses,const vector<double> &sample_times){
    vector<TumPose> outputs;

    int k1=0,k2=1;

    for(double t:sample_times){
        if(k2>=key_poses.size()){
            break;
        }

        //位姿t在k1关键帧的左边,使用k1,k2插值到左边
        if(t < key_poses[k1].getTime()){
            ///TODO
            auto msg = fmt::format("t < k1 and k2, t:{:.3f},k1:{:.3f}",t,key_poses[k1].getTime());
            //throw std::runtime_error();
            std::cerr<<msg<<endl;
            continue;
        }
            //位姿t在k2关键帧的右边，此时应向右查找关键帧
        else if(t > key_poses[k2].getTime()){
            if(k2 >= key_poses.size()){//如果无法向右查找，则使用当前的两个kp进行插值
                ///TODO
                auto msg = fmt::format("t > k1 and k2, t:{:.3f},k2:{:.3f}",t,key_poses[k2].getTime());
                //throw std::runtime_error();
                std::cerr<<msg<<endl;
                continue;
            }
            else{ //向右查找
                while(k2<key_poses.size() && t > key_poses[k2].getTime()){
                    k1++;
                    k2++;
                }
            }
        }

        ///时刻刚好相等的情况下
        if(t == key_poses[k1].getTime()){
            outputs.push_back(key_poses[k1]);
        }
        else if(t == key_poses[k2].getTime()){
            outputs.push_back(key_poses[k2]);
            k1=k2;
            k2++;
        }
            //位姿t在k1,k2的中间
        else if(t > key_poses[k1].getTime() && t<key_poses[k2].getTime()){
            double percent = (t - key_poses[k1].getTime()) /(key_poses[k2].getTime() - key_poses[k1].getTime());
            Vec3d P_t = key_poses[k1].getT() + percent * (key_poses[k2].getT() - key_poses[k1].getT());

            Quatd Q_t = key_poses[k1].getQ().slerp(percent,key_poses[k2].getQ());//球面插值
            Q_t.normalize();

            outputs.emplace_back(t,P_t,Q_t);
        }
    }

    return outputs;
}


