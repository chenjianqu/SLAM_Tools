//
// Created by cjq on 23-4-11.
//

#include "tum_pose.h"


vector<TumPose> TumPose::ReadOdomTxt(const string &txt_path)
{
    std::ifstream infile(txt_path, std::ios::in);
    if (!infile.is_open()){
        cout << "ReadOdomTxt() 读取文件失败" << endl;
        return {};
    }

    vector<TumPose> poses;

    string line;
    while (getline(infile,line)){
        vector<string> tokens;
        split(line,tokens," ");
        if(tokens.size()!=8){
            return poses;
        }

        poses.emplace_back(
                std::stod(tokens[0]),//time
                std::stod(tokens[1]),//tx
                std::stod(tokens[2]),//ty
                std::stod(tokens[3]),//tz
                std::stod(tokens[4]),//qx
                std::stod(tokens[5]),//qy
                std::stod(tokens[6]),//qz
                std::stod(tokens[7])//qw
        );
    }

    return poses;
}


void TumPose::WriteOdomTxt(const string &txt_path,const vector<TumPose> &poses){
    std::ofstream fout(txt_path, std::ios::out);
    fout.setf(std::ios::fixed, std::ios::floatfield);

    for(auto &pose:poses){
        fout<<pose.getTime()<<" "
        <<pose.getT().x()<<" "
        <<pose.getT().y()<<" "
        <<pose.getT().z()<<" "
        <<pose.getQ().x()<<" "
        <<pose.getQ().y()<<" "
        <<pose.getQ().z()<<" "
        <<pose.getQ().w()<<std::endl;
    }

    fout.close();
}

/**
 * 根据时间对位姿数组进行排序
 * @param poses
 */
void TumPose::SortByTime(vector<TumPose> &poses) {
    std::sort(poses.begin(),poses.end(),[](TumPose& x,TumPose& y){
        return x.getTime() < y.getTime();
    });
}






