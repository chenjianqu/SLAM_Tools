//
// Created by cjq on 23-4-11.
//

#ifndef POSEINTERPOLATION_TUM_POSE_H
#define POSEINTERPOLATION_TUM_POSE_H

#include <Eigen/Eigen>
#include "def.h"

class TumPose {
public:
    TumPose(double time,double tx,double ty,double tz,double qx,double qy,double qz,double qw):
    time_(time),t_{tx,ty,tz},q_{qw,qx,qy,qz}{}

    TumPose(double time,const Vec3d &t,const Quatd &q):
    time_(time),t_{t},q_{q}{}

    static vector<TumPose> ReadOdomTxt(const fs::path &txt_path);

    static vector<double> ReadTimeStampTxt(const fs::path &txt_path);


    static void WriteOdomTxt(const fs::path &txt_path,const vector<TumPose> &poses);

    static void SortByTime(vector<TumPose> &poses);

    [[nodiscard]] double getTime() const {
        return time_;
    }

    void setTime(double time) {
        time_ = time;
    }

    [[nodiscard]] const Vec3d &getT() const {
        return t_;
    }

    void setT(const Vec3d &t) {
        t_ = t;
    }

    [[nodiscard]] const Quatd &getQ() const {
        return q_;
    }

    void setQ(const Quatd &q) {
        q_ = q;
    }

private:
    double time_;
    Vec3d t_;
    Quatd q_;
};


#endif //POSEINTERPOLATION_TUM_POSE_H
