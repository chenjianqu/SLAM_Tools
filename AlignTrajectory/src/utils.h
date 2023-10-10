//
// Created by cjq on 23-7-25.
//

#ifndef ALIGNTRAJECTORY_UTILS_H
#define ALIGNTRAJECTORY_UTILS_H

#include <iostream>
#include <filesystem>
#include <fstream>
#include <regex>

#include <eigen3/Eigen/Eigen>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <spdlog/spdlog.h>

#include "def.h"
#include "pose.h"

using Eigen::Quaterniond;
using Eigen::Matrix3d;


std::vector<PoseStamped> ReadPosesFromTxt(const std::string& txt_path);

void WritePosesToTxt(const std::string& txt_path,const std::vector<PoseStamped> &poses);

void WritePosesToTxtComma(const std::string& txt_path,const std::vector<PoseStamped> &poses);


void WriteOnePoseToTxt(const std::string& txt_path,const Pose *pose);

vector<double> StringLineToVector(const string& line);




#endif //ALIGNTRAJECTORY_UTILS_H
