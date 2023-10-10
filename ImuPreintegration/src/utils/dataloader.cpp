/*******************************************************
 * Copyright (C) 2022, Chen Jianqu, Shanghai University
 *
 * This file is part of dynamic_vins.
 * Github:https://github.com/chenjianqu/dynamic_vins
 *
 * Licensed under the MIT License;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "dataloader.h"
#include "file_utils.h"

using namespace std::chrono_literals;




Dataloader::Dataloader(const fs::path &left_images_dir_){
    is_stereo=false;
    left_images_dir = left_images_dir_;

    GetAllImagePaths(left_images_dir, left_paths_vector);

    std::sort(left_paths_vector.begin(), left_paths_vector.end());

    //获取索引的最大值和最小值
    //vector<string> names;
    //GetAllImageNames(io_para::kImageDatasetLeft,names);
    //std::sort(names.begin(),names.end());
    //kStartIndex = stoi(fs::path(names[0]).stem().string());
    //kEndIndex = stoi(fs::path(*names.rbegin()).stem().string());
    //Debugv("Dataloader() kStartIndex:{}  kEndIndex:{}", kStartIndex,kEndIndex);

    //index=kStartIndex;
    index=0;
    time=0.;
}


Dataloader::Dataloader(const fs::path &left_images_dir_,const fs::path &right_images_dir_){
    is_stereo=true;
    left_images_dir = left_images_dir_;
    right_images_dir = right_images_dir_;

    GetAllImagePaths(left_images_dir, left_paths_vector);
    GetAllImagePaths(right_images_dir, right_paths_vector);

    if(left_paths_vector.size() != right_paths_vector.size()){
        cerr << fmt::format("left images number:{},right image number:{}, not equal!",
                            left_paths_vector.size(), right_paths_vector.size()) << endl;
        std::terminate();
    }

    std::sort(left_paths_vector.begin(), left_paths_vector.end());
    std::sort(right_paths_vector.begin(), right_paths_vector.end());

    //获取索引的最大值和最小值
    //vector<string> names;
    //GetAllImageNames(io_para::kImageDatasetLeft,names);
    //std::sort(names.begin(),names.end());
    //kStartIndex = stoi(fs::path(names[0]).stem().string());
    //kEndIndex = stoi(fs::path(*names.rbegin()).stem().string());
    //Debugv("Dataloader() kStartIndex:{}  kEndIndex:{}", kStartIndex,kEndIndex);

    //index=kStartIndex;
    index=0;
    time=0.;
}



std::tuple<cv::Mat,cv::Mat> Dataloader::LoadStereoImages(){
    if(index >= left_paths_vector.size()){
        return {};
    }
    cout << left_paths_vector[index] << endl;

    cv::Mat color0  = cv::imread(left_paths_vector[index], -1);
    cv::Mat color1 = cv::imread(right_paths_vector[index], -1);
    seq_id = fs::path(left_paths_vector[index]).filename().stem();

    index++;
    return {color0,color1};
}



cv::Mat Dataloader::LoadMonoImages(){
    if(index >= left_paths_vector.size()){
        return {};
    }
    cout << left_paths_vector[index] << endl;

    cv::Mat color0  = cv::imread(left_paths_vector[index], -1);
    seq_id = fs::path(left_paths_vector[index]).filename().stem();
    index++;
    return color0;
}

