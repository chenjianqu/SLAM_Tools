//
// Created by cjq on 23-7-28.
//

#pragma once

#include <spdlog/spdlog.h>
#include "def.h"

constexpr double D2R = (M_PI / 180.0);
constexpr double R2D = (180.0 / M_PI);



class Pose {
public:
    Pose()=default;
    Pose(const Quaterniond& q,const Eigen::Vector3d& t):q_(q),t_(t){}
    Pose(const Eigen::Matrix3d& r,const Eigen::Vector3d& t):q_(r),t_(t){}

    explicit Pose(const Eigen::Matrix4d& T4x4){
        Eigen::Matrix3d R = T4x4.topLeftCorner<3,3>();
        if(!VerifyRotationMatrix(R)){
            SPDLOG_ERROR("When construct Pose(T4x4),the rotation matrix invalid!"
                         "\n R3x3:\n{}",EigenFloatToStr(R));
        }
        q_ = R;
        t_ = T4x4.block<3,1>(0,3);
    }

    Pose(const Pose& pose){
        q_ = pose.q_;
        t_ = pose.t_;
    }

    Pose(Pose&& pose) noexcept {
        q_ = std::move(pose.q_);
        t_ = std::move(pose.t_);
    }

    Pose& operator= (const Pose& pose){
        q_ = pose.q_;
        t_ = pose.t_;
        return *this;
    }

    virtual ~Pose()= default;

    /**
     * 判断3x3矩阵是否为旋转矩阵
     * @param R
     * @return
     */
    static bool VerifyRotationMatrix(const Eigen::Matrix3d& R);

    static Pose Identity(){
        return {Eigen::Matrix3d::Identity(),Eigen::Vector3d::Zero()};
    }

    [[nodiscard]] Pose Inverse() const{
        return {q_.inverse(), -(q_.inverse()*t_)};
    }

    [[nodiscard]] Eigen::Matrix4d MakeMatrix4d() const {
        Eigen::Matrix4d M = Eigen::Matrix4d ::Zero();
        M(3, 3) = 1;
        M.block<3, 3>(0, 0) = q_.toRotationMatrix();
        M.block<3, 1>(0, 3) = t_;
        return M;
    }

    [[nodiscard]] Vec7d MakeVec7d() const {
        Vec7d ans;
        ans.topRows(3) = t();
        ans.bottomRows(4) = q().coeffs();
        return ans;
    }

    void SetVec7d(const Vec7d& vec){
        t_ = vec.topRows(3);
        q_.coeffs() = vec.bottomRows(4);//x y z w
        q_.normalize();
    }

    [[nodiscard]] const Quaterniond & q() const{
        return q_;
    }
    Quaterniond& q(){
        return q_;
    }
    [[nodiscard]] const Eigen::Vector3d & t() const{
        return t_;
    }
    Eigen::Vector3d& t(){
        return t_;
    }

    [[nodiscard]] Eigen::Matrix3d R() const{
        return q().toRotationMatrix();
    }

    [[nodiscard]] Eigen::Vector3d euler() const{
        return R().eulerAngles(2,1,0);
    }

    [[nodiscard]] virtual std::string DebugString() const{
        Eigen::Vector3d euler = this->euler() * R2D;
        return fmt::format("t:{:.3f} {:.3f} {:.3f} euler:{:.3f} {:.3f} {:.3f}",
                           t_.x(),t_.y(),t_.z(),
                           euler.x(),euler.y(),euler.z());
    }

private:
    Quaterniond q_;
    Eigen::Vector3d t_;
} ;



class PoseStamped : public Pose{
public:
    PoseStamped() = default;
    PoseStamped(double time_stamp,const Quaterniond& q,const Vec3d &t):Pose(q,t),time_stamp_(time_stamp){}
    PoseStamped(double time_stamp,const Pose& pose):Pose(pose),time_stamp_(time_stamp){}
    explicit PoseStamped(double time_stamp,const Eigen::Matrix4d& T4x4):Pose(T4x4),time_stamp_(time_stamp){}

    ~PoseStamped() override = default;

    static PoseStamped Identity(){
        return {0.,Eigen::Quaterniond ::Identity(),Eigen::Vector3d::Zero()};
    }

    static PoseStamped Identity(double time_stamp){
        return {time_stamp,Eigen::Quaterniond ::Identity(),Eigen::Vector3d::Zero()};
    }

    /**
     * 输入一个 T_left, 自身为T,计算 T_out = T_left * T
     * @param left_q
     * @param left_t
     * @return
     */
    [[nodiscard]] PoseStamped leftMul(const Quaterniond& left_q,const Vec3d &left_t) const{
        return {time_stamp_,left_q * q(), left_q * t() + left_t};
    }

    [[nodiscard]] PoseStamped leftMul(const Eigen::Matrix4d& left_T) const{
        Quaterniond left_q(left_T.topLeftCorner<3,3>());
        Vec3d left_t = left_T.block<3,1>(0,3);
        return {time_stamp_, left_q * q(), left_q * t() + left_t};
    }

    [[nodiscard]] PoseStamped leftMul(const Pose* left_T) const{
        return {time_stamp_,left_T->q() * q(), left_T->q() * t() + left_T->t()};
    }
    [[nodiscard]] PoseStamped leftMul(const Pose& left_T) const{
        return {time_stamp_,left_T.q() * q(), left_T.q() * t() + left_T.t()};
    }
    [[nodiscard]] PoseStamped leftMul(const PoseStamped& left_T) const{
        return {time_stamp_,left_T.q() * q(), left_T.q() * t() + left_T.t()};
    }

    [[nodiscard]] PoseStamped operator*(const Pose& other) const{
        return {time_stamp_,q() * other.q(), q() * other.t() + t()};
    }

    [[nodiscard]] PoseStamped operator*(const PoseStamped& other) const{
        return {time_stamp_,q() * other.q(), q() * other.t() + t()};
    }

    [[nodiscard]] PoseStamped Inverse() const{
        return {time_stamp_,Pose::Inverse()};
    }

    [[nodiscard]] std::string DebugString() const override  {
        return fmt::format("time_stamp:{} {}",time_stamp_,Pose::DebugString());
    }

    [[nodiscard]] double time_stamp() const{
        return time_stamp_;
    }

    double& time_stamp(){
        return time_stamp_;
    }

private:
    double time_stamp_{};
};

