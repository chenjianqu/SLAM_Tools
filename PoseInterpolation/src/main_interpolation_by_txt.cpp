#include <iostream>
#include <fstream>

#include <spdlog/formatter.h>

#include "def.h"
#include "file_utils.h"
#include "tum_pose.h"
#include "interpolator.h"


int main(int argc,char **argv) {

    if(argc!=3){
        cerr<<"Usage: ./interpolation_by_txt ${odom_txt_path} ${img_time_txt}"<<endl;
        return -1;
    }

    fs::path odom_txt_path=argv[1];
    fs::path img_time_txt=argv[2];

    if(!fs::exists(odom_txt_path)){
        cerr<<"wrong odom_txt_path, that is "<<odom_txt_path<<endl;
        return -1;
    }
    if(!fs::exists(img_time_txt)){
        cerr<<"wrong img_time_txt, that is "<<img_time_txt<<endl;
        return -1;
    }

    vector<TumPose> poses = TumPose::ReadOdomTxt(odom_txt_path);

    auto img_times = TumPose::ReadTimeStampTxt(img_time_txt);

    ///根据时间将位姿进行排序
    TumPose::SortByTime(poses);

    vector<TumPose> poses_interpolation = PoseInterpolate(poses,img_times);

    TumPose::WriteOdomTxt("poses_interpolation.txt",poses_interpolation);

    return 0;
}


