//
// Created by cjq on 23-4-23.
//

#ifndef COLMAPUTILS_GPS_POSE_H

#include "def.h"

struct GpsPose{
    static std::map<double,GpsPose> ReadGpsPoseFromTxt(const string& path);

    static double Distance(const GpsPose& pose1,const GpsPose& pose2);

    double stamp;
    double longitude;//经度
    double latitude;//纬度
    double altitude;//高度

    int covariance_type;
    Mat3d cov;//协方差
};


#define COLMAPUTILS_GPS_POSE_H

#endif //COLMAPUTILS_GPS_POSE_H
