#include <iostream>
#include <fstream>

#include <spdlog/formatter.h>

#include "def.h"
#include "file_utils.h"
#include "tum_pose.h"

vector<TumPose> PoseInterpolate(const vector<TumPose>& key_poses,const vector<double> &sample_times);

int main(int argc,char **argv) {

    if(argc!=3){
        cerr<<"Usage: ./PoseInterpolation ${odom_txt_path} ${image_dir}"<<endl;
        return -1;
    }

    string odom_txt_path=argv[1];
    string image_dir=argv[2];

    if(!IsExists(odom_txt_path)){
        cerr<<"wrong odom_txt_path, that is "<<odom_txt_path<<endl;
        return -1;
    }
    if(!IsExists(image_dir)){
        cerr<<"wrong image_dir, that is "<<image_dir<<endl;
        return -1;
    }

    vector<string> img_names;
    GetDirectoryFileNames(image_dir, img_names);

    vector<TumPose> poses = TumPose::ReadOdomTxt(odom_txt_path);

    ///手动预处理
    //去掉第一个poses
    poses.erase(poses.begin());

    cout<<"img_names size:"<<img_names.size()<<endl;
    cout<<"poses size:"<<poses.size()<<endl;

    if(img_names.size()<=2 || poses.size()<=2){
        cerr<<"the number of files is too small"<<endl;
        return -1;
    }

    vector<double> samples;
    for(const string& name:img_names){
        samples.push_back(std::stod(StringStem(name)));
    }

    ///根据时间将位姿进行排序
    std::sort(samples.begin(),samples.end());
    TumPose::SortByTime(poses);

    vector<TumPose> poses_interpolation = PoseInterpolate(poses,samples);

    TumPose::WriteOdomTxt("poses_interpolation.txt",poses_interpolation);

    return 0;
}


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
            throw std::runtime_error(fmt::format("t < k1 and k2, t:{:.3f},k1:{:.3f}",
                                                 t,key_poses[k1].getTime()));
        }
        //位姿t在k2关键帧的右边，此时应向右查找关键帧
        else if(t > key_poses[k2].getTime()){
            if(k2 >= key_poses.size()){//如果无法向右查找，则使用当前的两个kp进行插值
                ///TODO
                throw std::runtime_error(fmt::format("t > k1 and k2, t:{:.3f},k2:{:.3f}",
                                                     t,key_poses[k2].getTime()));
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

