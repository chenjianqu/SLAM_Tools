/*******************************************************
 * Copyright (C) 2022, Chen Jianqu, Shanghai University
 *
 * This file is part of dynamic_vins.
 *
 * Licensed under the MIT License;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#ifndef DYNAMIC_VINS_PARAMETER_H
#define DYNAMIC_VINS_PARAMETER_H

#include <vector>
#include <fstream>
#include <map>
#include <iostream>
#include <exception>
#include <tuple>

#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <spdlog/spdlog.h>



using std::cout;
using std::endl;
using std::cerr;
using std::string;
using std::pair;
using std::vector;
using std::tuple;

using namespace std::chrono_literals;
namespace fs=std::filesystem;

template <typename EigenType>
using EigenContainer = std::vector< EigenType ,Eigen::aligned_allocator<EigenType>>;

using Vec2d = Eigen::Vector2d;
using Vec3d = Eigen::Vector3d;
using Vec7d = Eigen::Matrix<double, 7, 1>;
using Mat2d = Eigen::Matrix2d;
using Mat3d = Eigen::Matrix3d;
using Mat4d = Eigen::Matrix4d;
using Mat23d = Eigen::Matrix<double, 2, 3>;
using Mat24d = Eigen::Matrix<double, 2, 4>;
using Mat34d = Eigen::Matrix<double, 3, 4>;
using Mat35d = Eigen::Matrix<double, 3, 5>;
using Mat36d = Eigen::Matrix<double, 3, 6>;
using Mat37d = Eigen::Matrix<double, 3, 7>;
using Quatd = Eigen::Quaterniond;

using VecVector3d = EigenContainer<Eigen::Vector3d>;
using VecMatrix3d = EigenContainer<Eigen::Matrix3d>;


inline std::shared_ptr<spdlog::logger> sg_logger;


class Config {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using Ptr=std::shared_ptr<Config>;

    explicit Config(const std::string &file_name);

    inline static double kInitDepth;
    inline static double kMinParallax;
    inline static double ACC_N, ACC_W;
    inline static double GYR_N, GYR_W;

    inline static std::vector<Eigen::Matrix3d> RIC;
    inline static std::vector<Eigen::Vector3d> TIC;

    inline static Eigen::Vector3d G{0.0, 0.0, 9.8};

    inline static double BIAS_ACC_THRESHOLD,BIAS_GYR_THRESHOLD;
    inline static double kMaxSolverTime;
    inline static int KNumIter;
    inline static int ESTIMATE_EXTRINSIC;
    inline static int is_estimate_td;
    inline static int ROLLING_SHUTTER;
    inline static std::string kExCalibResultPath;
    inline static std::string kVinsResultPath;
    inline static std::string kOutputFolder;
    inline static std::string kImuTopic;
    inline static int kRow, kCol;
    inline static double TD;
    inline static int kCamNum;
    inline static int is_stereo;
    inline static int is_use_imu;
    inline static std::map<int, Eigen::Vector3d> pts_gt;
    inline static std::string kImage0Topic, kImage1Topic,kImage0SegTopic,kImage1SegTopic;
    inline static std::string FISHEYE_MASK;
    inline static std::vector<std::string> kCamPath;
    inline static int kMaxCnt; //每帧图像上的最多检测的特征数量
    inline static int kMaxDynamicCnt;
    inline static int kMinDist; //检测特征点时的最小距离
    inline static int kMinDynamicDist; //检测特征点时的最小距离
    inline static double kFThreshold;
    inline static int kShowTrack;
    inline static int kFlowBack; //是否反向计算光流，判断之前光流跟踪的特征点的质量

    inline static std::unordered_map<unsigned int,int> ViodeKeyToIndex;
    inline static std::set<int> ViodeDynamicIndex;

    inline static std::string kDetectorOnnxPath;
    inline static std::string kDetectorSerializePath;

    inline static int kInputHeight,kInputWidth,kInputChannel=3;

    inline static bool is_input_seg;

    inline static std::vector<std::string> CocoLabelVector;

    inline static std::string kEstimatorLogPath;
    inline static std::string kEstimatorLogLevel;
    inline static std::string kEstimatorLogFlush;
    inline static std::string kFeatureTrackerLogPath;
    inline static std::string kFeatureTrackerLogLevel;
    inline static std::string kFeatureTrackerLogFlush;
    inline static std::string kSegmentorLogPath;
    inline static std::string kSegmentorLogLevel;
    inline static std::string kSegmentorLogFlush;

    inline static int kVisualInstDuration;

    inline static std::string kExtractorModelPath;

    inline static std::string kRaftFnetOnnxPath;
    inline static std::string kRaftFnetTensorrtPath;
    inline static std::string kRaftCnetOnnxPath;
    inline static std::string kRaftCnetTensorrtPath;
    inline static std::string kRaftUpdateOnnxPath;
    inline static std::string kRaftUpdateTensorrtPath;

    inline static int kSoloNmsPre;
    inline static int kSoloMaxPerImg;
    inline static std::string kSoloNmsKernel;
    inline static float kSoloNmsSigma;
    inline static float kSoloScoreThr;
    inline static float kSoloMaskThr;
    inline static float kSoloUpdateThr;

    inline static int kTrackingMaxAge;
    inline static int kTrackingNInit;

    inline static std::string kBasicDir;

    inline static std::atomic_bool ok{true};
};

using cfg = Config;



#endif

