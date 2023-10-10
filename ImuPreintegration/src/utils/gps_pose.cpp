//
// Created by cjq on 23-4-23.
//

#include "gps_pose.h"

std::map<double,GpsPose> GpsPose::ReadGpsPoseFromTxt(const string& path){
    ///Read GPS Record
    std::ifstream gps_file(path,std::ios::in);
    if(!gps_file.is_open()){
        cerr<<"Open gps_file:"<<path<<" failed"<<endl;
        return {};
    }

    std::map<double,GpsPose> poses;
    ///每一行的格式：
    /*
    time_str<<" "
    <<gps_msg_ptr->latitude<<" "
    <<gps_msg_ptr->longitude<<" "
    <<gps_msg_ptr->altitude<<" "
    <<(int)gps_msg_ptr->position_covariance_type<<" "
    <<gps_msg_ptr->position_covariance[0]<<" "
    <<gps_msg_ptr->position_covariance[1]<<" "
    <<gps_msg_ptr->position_covariance[2]<<" "
    <<gps_msg_ptr->position_covariance[3]<<" "
    <<gps_msg_ptr->position_covariance[4]<<" "
    <<gps_msg_ptr->position_covariance[5]<<" "
    <<gps_msg_ptr->position_covariance[6]<<" "
    <<gps_msg_ptr->position_covariance[7]<<" "
    <<gps_msg_ptr->position_covariance[8]<<std::endl;
     */
    string line;
    while(std::getline(gps_file,line)){
        vector<string> tokens;
        split(line,tokens);
        if(tokens.size()!=14){
            break;
        }
        GpsPose pose;
        pose.stamp = std::stod(tokens[0]);
        pose.longitude = std::stod(tokens[1]);
        pose.latitude = std::stod(tokens[2]);
        pose.altitude = std::stod(tokens[3]);
        pose.covariance_type = std::stoi(tokens[4]);
        pose.cov<<std::stod(tokens[5]),std::stod(tokens[6]),std::stod(tokens[7]),
                std::stod(tokens[8]),std::stod(tokens[9]),std::stod(tokens[10]),
                std::stod(tokens[11]),std::stod(tokens[12]),std::stod(tokens[13]);
        poses.insert({pose.stamp,pose});
    }
    gps_file.close();

    return std::move(poses);
}




/// https://blog.csdn.net/xfxf996/article/details/81541867
///   <summary>
/// 计算两点GPS坐标的距离
double GpsPose::Distance(const GpsPose& pose1,const GpsPose& pose2){
    double  jl_jd  =   102834.74258026089786013677476285 ;
    double  jl_wd  =   111712.69150641055729984301412873 ;
    double  b  =  std::abs((pose1.longitude  -  pose2.longitude)  *  jl_jd);
    double  a  =  std::abs((pose1.latitude  -  pose2.latitude)  *  jl_wd);
    double c = std::abs(pose1.altitude - pose2.altitude);
    return  std::sqrt((a*a + b*b + c*c));
}

