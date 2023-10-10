#include <iostream>
#include <fstream>

#include <spdlog/formatter.h>

#include "def.h"
#include "file_utils.h"
#include "tum_pose.h"
#include "interpolator.h"


int main(int argc,char **argv) {

    if(argc!=3){
        cerr<<"Usage: ./PoseInterpolation ${odom_txt_path} ${image_dir}"<<endl;
        return -1;
    }

    fs::path odom_txt_path=argv[1];
    fs::path image_dir=argv[2];

    if(!fs::exists(odom_txt_path)){
        cerr<<"wrong odom_txt_path, that is "<<odom_txt_path<<endl;
        return -1;
    }
    if(!fs::is_directory(image_dir)){
        cerr<<"wrong image_dir, that is "<<image_dir<<endl;
        return -1;
    }

    vector<string> img_names;
    GetAllImageNames(image_dir, img_names);

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
        samples.push_back(std::stod(fs::path(name).stem().string()));
    }

    ///根据时间将位姿进行排序
    std::sort(samples.begin(),samples.end());
    TumPose::SortByTime(poses);

    vector<TumPose> poses_interpolation = PoseInterpolate(poses,samples);

    TumPose::WriteOdomTxt("poses_interpolation.txt",poses_interpolation);

    return 0;
}


