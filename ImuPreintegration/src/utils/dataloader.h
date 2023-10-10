/*******************************************************
 * Copyright (C) 2022, Chen Jianqu, Shanghai University
 *
 * This file is part of dynamic_vins.
 * Github:https://github.com/chenjianqu/dynamic_vins
 *
 * Licensed under the MIT License;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#ifndef DYNAMIC_VINS_CALLBACK_H
#define DYNAMIC_VINS_CALLBACK_H

#include <memory>
#include <queue>
#include <filesystem>


#include "def.h"

namespace fs=std::filesystem;



/**
 * 图像读取类
 */
class Dataloader{
public:
    using Ptr = std::shared_ptr<Dataloader>;
    Dataloader(const fs::path &left_images_dir,const fs::path &right_images_dir);
    Dataloader(const fs::path &left_images_dir);


    std::tuple<cv::Mat,cv::Mat> LoadStereoImages();

    cv::Mat LoadMonoImages();

    int get_index() const {return index;}
    string get_seq_id() const {return seq_id;}

private:
    bool is_stereo{false};

    vector<string> left_paths_vector;
    vector<string> right_paths_vector;

    int index{0};
    double time{0};
    string seq_id{};

    fs::path left_images_dir,right_images_dir;
};


#endif //DYNAMIC_VINS_CALLBACK_H
