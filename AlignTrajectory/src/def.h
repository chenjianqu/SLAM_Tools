/*******************************************************
 * Copyright (C) 2022, Chen Jianqu, Shanghai University
 *
 * This file is part of dynamic_vins.
 * Github:https://github.com/chenjianqu/dynamic_vins
 *
 * Licensed under the MIT License;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#ifndef DYNAMIC_VINS_DEF_H
#define DYNAMIC_VINS_DEF_H

#include <string>
#include <vector>
#include <chrono>
#include <iostream>
#include <regex>
#include <filesystem>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <spdlog/logger.h>
#include <opencv2/opencv.hpp>


using namespace std::chrono_literals;

using std::cout;
using std::endl;
using std::cerr;
using std::string;
using std::pair;
using std::vector;
using std::tuple;
using std::map;


template <typename EigenType>
using EigenContainer = std::vector< EigenType ,Eigen::aligned_allocator<EigenType>>;

using Vec2d = Eigen::Vector2d;
using Vec3d = Eigen::Vector3d;
using Vec4d = Eigen::Vector4d;
using Vec5d = Eigen::Matrix<double, 5, 1>;
using Vec6d = Eigen::Matrix<double, 6, 1>;
using Vec7d = Eigen::Matrix<double, 7, 1>;
using Mat2d = Eigen::Matrix2d;
using Mat3d = Eigen::Matrix3d;
using Mat4d = Eigen::Matrix4d;
using Mat23d = Eigen::Matrix<double, 2, 3>;
using Mat24d = Eigen::Matrix<double, 2, 4>;
using Mat28d = Eigen::Matrix<double, 2, 8>;
using Mat34d = Eigen::Matrix<double, 3, 4>;
using Mat35d = Eigen::Matrix<double, 3, 5>;
using Mat36d = Eigen::Matrix<double, 3, 6>;
using Mat37d = Eigen::Matrix<double, 3, 7>;
using Mat38d = Eigen::Matrix<double, 3, 8>;
using Quatd = Eigen::Quaterniond;
using Eigen::Quaterniond;

using VecVector3d = EigenContainer<Eigen::Vector3d>;
using VecMatrix3d = EigenContainer<Eigen::Matrix3d>;


class TicToc{
public:
    TicToc(){
        Tic();
    }
    void Tic(){
        start_ = std::chrono::system_clock::now();
    }
    double Toc(){
        end_ = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end_ - start_;
        return elapsed_seconds.count() * 1000;
    }
    double TocThenTic(){
        auto t= Toc();
        Tic();
        return t;
    }
    void TocPrintTic(const char* str){
        std::cout << str << ":" << Toc() << " ms" <<std::endl;
        Tic();
    }
private:
    std::chrono::time_point<std::chrono::system_clock> start_, end_;
};



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

template<typename MatrixType>
inline std::string EigenFloatToStr(const MatrixType &m){
    std::string text;
    for(int i=0;i<m.rows();++i){
        for(int j=0;j<m.cols();++j)
            text+=fmt::format("{:.6f} ",m(i,j));
        if(m.rows()>1) text+="\n";
    }
    return text;
}

template<typename T>
inline std::string VecToStr(const Eigen::Matrix<T,3,1> &vec){
    return EigenToStr(vec.transpose());
}

template<typename T>
inline std::string VecToStr(const Eigen::Matrix<T,2,1> &vec){
    return EigenToStr(vec.transpose());
}

template<typename T>
inline std::string VecFloatToStr(const Eigen::Matrix<T,3,1> &vec){
    return EigenFloatToStr(vec.transpose());
}

template<typename T>
inline std::string VecFloatToStr(const Eigen::Matrix<T,2,1> &vec){
    return EigenFloatToStr(vec.transpose());
}

template<typename T>
inline std::string QuaternionToStr(const Eigen::Quaternion<T> &q){
    return fmt::format("x:{:.2f} y:{:.2f} z:{:.2f} w:{:.2f}",q.x(),q.y(),q.z(),q.w());
}

template<typename T>
inline std::string QuaternionFloatToStr(const Eigen::Quaternion<T> &q){
    return fmt::format("x:{:.6f} y:{:.6f} z:{:.6f} w:{:.6f}",q.x(),q.y(),q.z(),q.w());
}

template<typename T>
inline std::string ss_to_str(const T &e){
    std::stringstream ss;
    ss<<e;
    return ss.str();
}


inline std::string PadNumber(int number,int name_width){
    std::stringstream ss;
    ss<<std::setw(name_width)<<std::setfill('0')<<number;
    string target_name;
    ss >> target_name;
    return target_name;
}

/**
 * 将浮点数转换为字符串,并保留decimals_len位小数
 * @param num
 * @param decimals_len
 * @return
 */
inline std::string DoubleToStr(double num,int decimals_len){
    std::stringstream ss;
    ss<<std::fixed<<std::setprecision(decimals_len)<<num;
    return ss.str();
}


template<typename T>
inline std::string StdVectorToStr(const std::vector<T> &vec){
    string s;
    for(auto &e:vec){
        s += std::to_string(e) + " ";
    }
    return s;
}

template<typename T>
inline std::string CArrayToStr(const T* vec,size_t size){
    string s;
    for(int i=0;i<size;++i){
        s += std::to_string(vec[i]) + " ";
    }
    return s;
}


inline std::string DimsToStr(cv::Size list){
    return "[" + std::to_string(list.height) + ", " + std::to_string(list.width) + "]";
}


inline cv::Point2f operator*(const cv::Point2f &lp,const cv::Point2f &rp){
    return {lp.x * rp.x,lp.y * rp.y};
}

inline void split(const std::string& source, std::vector<std::string>& tokens, const std::string& delimiters = " ") {
    std::regex re(delimiters);
    std::copy(std::sregex_token_iterator(source.begin(), source.end(),re,-1),
              std::sregex_token_iterator(),
              std::back_inserter(tokens));
}

inline std::string FileNameStem(const std::string &path){
    size_t lc= path.find_last_of('/');
    if(lc!=std::string::npos){
        std::string file_name = path.substr(lc+1);
        size_t dot_end = file_name.find_last_of('.');
        return file_name.substr(0,dot_end);
    }
    else{
        size_t dot_end = path.find_last_of('.');
        return path.substr(0,dot_end);
    }
}





#endif //DYNAMIC_VINS_DEF_H
