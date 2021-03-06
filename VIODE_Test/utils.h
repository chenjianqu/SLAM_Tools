/*******************************************************
 * Copyright (C) 2022, Chen Jianqu, Shanghai University
 *
 * This file is part of dynamic_vins.
 *
 * Licensed under the MIT License;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#ifndef DYNAMIC_VINS_UTILS_H
#define DYNAMIC_VINS_UTILS_H

#include <string>
#include <vector>
#include <chrono>

#include <spdlog/logger.h>
#include <opencv2/opencv.hpp>

#include "parameters.h"



class TicToc{
public:
    TicToc(){
        tic();
    }
    void tic(){
        start = std::chrono::system_clock::now();
    }
    double toc(){
        end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        return elapsed_seconds.count() * 1000;
    }
    double toc_then_tic(){
        auto t=toc();
        tic();
        return t;
    }
    void toc_print_tic(const char* str){
        cout<<str<<":"<<toc()<<" ms"<<endl;
        tic();
    }
private:
    std::chrono::time_point<std::chrono::system_clock> start, end;
};

struct Track {
    int id;
    cv::Rect2f box;
};


struct ImageInfo{
    int origin_h,origin_w;
    ///图像的裁切信息
    int rect_x, rect_y, rect_w, rect_h;
};



inline cv::Point2f operator*(const cv::Point2f &lp,const cv::Point2f &rp){
    return {lp.x * rp.x,lp.y * rp.y};
}

template<typename MatrixType>
inline std::string EigenToStr(const MatrixType &m){
    std::string text;
    for(int i=0;i<m.rows();++i){
        for(int j=0;j<m.cols();++j)
            text+=fmt::format("{:.2f} ",m(i,j));
        if(m.rows()>1) text+="\n";
    }
    return text;
}

template<typename T>
inline std::string VecToStr(const Eigen::Matrix<T,3,1> &vec){
    return EigenToStr(vec.transpose());
}

template<typename T>
inline std::string QuaternionToStr(const Eigen::Quaternion<T> &q){
    return fmt::format("x:{:.2f} y:{:.2f} z:{:.2f} w:{:.2f}",q.x(),q.y(),q.z(),q.w());
}


void DrawText(cv::Mat &img, const std::string &str, const cv::Scalar &color, const cv::Point& pos, float scale= 1.f, int thickness= 1, bool reverse = false);

void DrawBbox(cv::Mat &img, const cv::Rect2f& bbox, const std::string &label = "", const cv::Scalar &color = {0, 0, 0});


float CalBoxIoU(const cv::Point2f &box1_minPt, const cv::Point2f &box1_maxPt,
                const cv::Point2f &box2_minPt, const cv::Point2f &box2_maxPt);

float CalBoxIoU(const cv::Rect2f &bb_test, const cv::Rect2f &bb_gt);

cv::Scalar color_map(int64_t n);



template <typename Arg1, typename... Args>
inline void DebugS(const char* fmt, const Arg1 &arg1, const Args&... args){ sg_logger->log(spdlog::level::debug, fmt, arg1, args...);}
template<typename T>
inline void DebugS(const T& msg){sg_logger->log(spdlog::level::debug, msg); }
template <typename Arg1, typename... Args>
inline void InfoS(const char* fmt, const Arg1 &arg1, const Args&... args){sg_logger->log(spdlog::level::info, fmt, arg1, args...);}
template<typename T>
inline void InfoS(const T& msg){sg_logger->log(spdlog::level::info, msg);}
template <typename Arg1, typename... Args>
inline void WarnS(const char* fmt, const Arg1 &arg1, const Args&... args){sg_logger->log(spdlog::level::warn, fmt, arg1, args...);}
template<typename T>
inline void WarnS(const T& msg){sg_logger->log(spdlog::level::warn, msg);}
template <typename Arg1, typename... Args>
inline void CriticalS(const char* fmt, const Arg1 &arg1, const Args&... args){sg_logger->log(spdlog::level::critical, fmt, arg1, args...);}
template<typename T>
inline void CriticalS(const T& msg){sg_logger->log(spdlog::level::critical, msg);}



#endif //DYNAMIC_VINS_UTILS_H
