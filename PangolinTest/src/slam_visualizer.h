//
// Created by cjq on 23-9-3.
//

#ifndef PANGOLINTEST_SLAM_VISUALIZER_H
#define PANGOLINTEST_SLAM_VISUALIZER_H

#include <iostream>
#include <iomanip>
#include <Eigen/Dense>
#include <pangolin/pangolin.h>
#include <opencv2/opencv.hpp>
#include <queue>
#include <unistd.h>
#include <thread>
#include <string>



// 创建一个VectorXd用于pangolin::Var 输出数据
// 根据pangolin文档，自定义类型需要重载输入输出流操作符
struct VecXd
{
    Eigen::VectorXd vec_ = Eigen::Vector3d::Zero();
};

// 使用 inline 休息避免头文件中的非模板、非成员重复包含
inline std::ostream &operator<<(std::ostream &out, const VecXd &r)
{
    int N = r.vec_.size();
    out.setf(std::ios::fixed);
    out << "="
        << " [";
    for (int i = 0; i < N - 1; i++)
    {
        out << std::setprecision(2) << r.vec_(i) << ", ";
    }
    out << r.vec_(N - 1) << "]";
    return out;
}

inline std::istream &operator>>(std::istream &in, VecXd &r)
{
    return in;
}

// ==================== slam_visualizer.h =============
class SlamVisualizer
{
public:
    SlamVisualizer(int width = 752, int height = 480) : WIN_WIDTH_(width), WIN_HEIGHT_(height) {}
    ~SlamVisualizer() {}

    void initDraw();

    void activeAllView();

    void drawCubeTest();

    void drawCamWithPose(Eigen::Vector3d &pos, Eigen::Quaterniond &quat);

    void drawTraj(std::vector<Eigen::Vector3d> &traj);
    /**
     * @brief 画一个简单的相机模型
     * @param scale：缩放尺寸，默认为1
     */
    void drawCam(const float scale = 1.);

    void drawCoordinate();

    void displayImg(cv::Mat &originImg, cv::Mat &trackImg);

    void displayData(Eigen::Vector3d &pos, Eigen::Quaterniond &quat);

    void registerUICallback();

private:
    pangolin::OpenGlRenderState s_cam_;
    pangolin::View d_cam_, d_img_, d_track_;
    pangolin::GlTexture imageTexture_, trackTexture_;
    pangolin::DataLog pose_log_;

    // 存储ui面板的控件对象
    std::vector<pangolin::Var<bool>> ui_set_;
    // 存储data面板的控件对象
    std::vector<pangolin::Var<VecXd>> data_set_;
    // 是否显示相机
    bool camera_visible_ = true;
    // 是否显示轨迹
    bool traj_visible_ = true;
    // 是否显示参考坐标系
    bool coordinate_visible_ = true;
    // 是否显示图像
    bool img_visible_ = true;
    // 窗口尺寸
    int WIN_WIDTH_;
    int WIN_HEIGHT_;
};



#endif //PANGOLINTEST_SLAM_VISUALIZER_H
