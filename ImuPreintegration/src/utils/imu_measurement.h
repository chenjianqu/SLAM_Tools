//
// Created by cjq on 23-5-4.
//

#ifndef IMUPREINTEGRATION_IMU_MEASUREMENT_H
#define IMUPREINTEGRATION_IMU_MEASUREMENT_H


class ImuMeasurement{
public:
    double time;
    double angular_x;
    double angular_y;
    double angular_z;
    double acc_x;
    double acc_y;
    double acc_z;
};


#endif //IMUPREINTEGRATION_IMU_MEASUREMENT_H
