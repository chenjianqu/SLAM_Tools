//
// Created by cjq on 23-7-25.
//

#include "utils.h"

namespace fs=std::filesystem;


std::vector<PoseStamped> ReadPosesFromTxt(const std::string& txt_path){
    if(!fs::exists(txt_path)){
        return {};
    }

    std::vector<PoseStamped> output;
    std::ifstream infile(txt_path);

    std::string line;
    while(std::getline(infile,line)){
        vector<double> token = StringLineToVector(line);
        PoseStamped pose;
        pose.time_stamp() = token[0];
        pose.t().x() = token[1];
        pose.t().y() = token[2];
        pose.t().z() = token[3];
        pose.q().x() = token[4];
        pose.q().y() = token[5];
        pose.q().z() = token[6];
        pose.q().w() = token[7];
        output.push_back(pose);
    }
    return output;
}



void WritePosesToTxt(const std::string& txt_path,const std::vector<PoseStamped> &poses){
    std::ofstream fout(txt_path);
    for(auto &pose:poses){
        fout<<fmt::format(
                "{} {} {} {} {} {} {} {}",
                pose.time_stamp(),pose.t().x(),pose.t().y(),pose.t().z(),
                pose.q().x(),pose.q().y(),pose.q().z(),pose.q().w()) <<std::endl;
    }
    fout.close();
}


void WritePosesToTxtComma(const std::string& txt_path,const std::vector<PoseStamped> &poses){
    std::ofstream fout(txt_path);
    for(auto &pose:poses){
        fout<<fmt::format(
                "{},{},{},{},{},{},{},{}",
                pose.time_stamp(),pose.t().x(),pose.t().y(),pose.t().z(),
                pose.q().x(),pose.q().y(),pose.q().z(),pose.q().w()) <<std::endl;
    }
    fout.close();
}


void WriteOnePoseToTxt(const std::string& txt_path,const Pose *pose){
    std::ofstream fout(txt_path);
    fout<<fmt::format(
            "{} {} {} {} {} {} {} {}",
            0,pose->t().x(),pose->t().y(),pose->t().z(),
            pose->q().x(),pose->q().y(),pose->q().z(),pose->q().w()) <<std::endl;
    fout.close();

}


vector<double> StringLineToVector(const string& line){
    vector<string> tokens;
    if(line.find(',') != string::npos){
        split(line,tokens,",");
    }
    else{
        split(line,tokens," ");
    }

    vector<double> vec_data(tokens.size());
    for(int i=0;i<tokens.size();++i){
        try {
            vec_data[i] = std::stod(tokens[i]);
        }
        catch (std::exception &e){
            SPDLOG_ERROR("StringLineToVector() raise error,token:{},error:\n{}",tokens[i],e.what());
        }
    }
    return vec_data;
}
