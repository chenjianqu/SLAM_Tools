//
// Created by cjq on 23-5-4.
//

#ifndef IMUPREINTEGRATION_IMUPROCESSOR_H
#define IMUPREINTEGRATION_IMUPROCESSOR_H

#include "utils/def.h"
#include "integration_base.h"

class ImuProcessor {
public:
    ImuProcessor() = default;
    ImuProcessor(const fs::path &config_file);


    void ProcessIMU(double t, double dt, const Eigen::Vector3d &linear_acceleration, const Eigen::Vector3d &angular_velocity,
                    double curr_time);

private:
    bool first_imu{false};

    Eigen::Vector3d acc_0, gyr_0;

    std::unordered_map<double,IntegrationBase*> pre_integrations;

    int frame_count = 0;

    Vector3d Ps[(WINDOW_SIZE + 1)];
    Vector3d Vs[(WINDOW_SIZE + 1)];
    Matrix3d Rs[(WINDOW_SIZE + 1)];
    Vector3d Bas[(WINDOW_SIZE + 1)];
    Vector3d Bgs[(WINDOW_SIZE + 1)];
    double td;

    Vector3d g;
};


#endif //IMUPREINTEGRATION_IMUPROCESSOR_H
