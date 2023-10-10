//
// Created by cjq on 23-8-29.
//

#ifndef SLAMVIEWER_VIEWER_TYPES_H
#define SLAMVIEWER_VIEWER_TYPES_H

#include <sophus/so3.hpp>
#include <sophus/se3.hpp>

class MapPoint {
public:
    void SetWorldPos(const Vec3d &Pos){point_w = Pos;}
    const Vec3d& GetWorldPos()const {return point_w;}

    bool isBad()const {return is_bad_;}

private:
    bool is_bad_{false};
    Vec3d point_w;
};



class KeyFrame
{
public:
    long unsigned int mnId;


    // Pose functions
    void SetPose(const Sophus::SE3f &Tcw);
    void SetVelocity(const Eigen::Vector3f &Vw_);

    Sophus::SE3f GetPose();

    Sophus::SE3f GetPoseInverse();
    Eigen::Vector3f GetCameraCenter();

    Eigen::Vector3f GetImuPosition();
    Eigen::Matrix3f GetImuRotation();
    Sophus::SE3f GetImuPose();
    Eigen::Matrix3f GetRotation();
    Eigen::Vector3f GetTranslation();
    Eigen::Vector3f GetVelocity();
    bool isVelocitySet();

    KeyFrame* GetParent();

    std::vector<KeyFrame*> GetCovisiblesByWeight(const int &w);

    std::set<KeyFrame*> GetLoopEdges();

    KeyFrame* mPrevKF;
    KeyFrame* mNextKF;

private:

};


class Map{
public:
    std::vector<MapPoint*> GetAllMapPoints();
    std::vector<MapPoint*> GetReferenceMapPoints();
    std::vector<KeyFrame*> GetAllKeyFrames();


    // DEBUG: show KFs which are used in LBA
    std::set<long unsigned int> msOptKFs;
    std::set<long unsigned int> msFixedKFs;
};

class Tracking{
public:
// Tracking states
enum eTrackingState{
    SYSTEM_NOT_READY=-1,
    NO_IMAGES_YET=0,
    NOT_INITIALIZED=1,
    OK=2,
    RECENTLY_LOST=3,
    LOST=4,
    OK_KLT=5
};
};


#endif //SLAMVIEWER_VIEWER_TYPES_H
