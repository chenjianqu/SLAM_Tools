//
// Created by cjq on 23-5-4.
//

#include "imu_processor.h"
#include "utility.h"
#include "integration_base.h"



ImuProcessor::ImuProcessor(const fs::path &config_file) {
    Parameters::readParameters(config_file);

    td = TD;
    g = G;

    for (int i = 0; i < WINDOW_SIZE + 1; i++)
    {
        Rs[i].setIdentity();
        Ps[i].setZero();
        Vs[i].setZero();
        Bas[i].setZero();
        Bgs[i].setZero();
        delete pre_integrations[i];
        pre_integrations[i] = nullptr;
    }

}



void ImuProcessor::ProcessIMU(double t, double dt, const Eigen::Vector3d &linear_acceleration,
                              const Eigen::Vector3d &angular_velocity,double curr_time){
    if (!first_imu){
        first_imu = true;
        acc_0 = linear_acceleration;
        gyr_0 = angular_velocity;
    }

    cout<<fmt::format("dt:{} d:({},{},{})  gyr:({},{},{})",dt,linear_acceleration.x(),
                      linear_acceleration.y(),linear_acceleration.z(),angular_velocity.x(),
                      angular_velocity.y(),angular_velocity.z())<<endl;

    if(pre_integrations.count(curr_time)==0){
        pre_integrations.insert({curr_time,
                                 new IntegrationBase(acc_0, gyr_0, Bas[frame_count], Bgs[frame_count])});
    }

    if (pre_integrations.size()>1){
        pre_integrations[curr_time]->push_back(dt, linear_acceleration, angular_velocity);
        cout<<fmt::format("curr_time:{} p:({},{},{})",curr_time,
                          pre_integrations[curr_time]->delta_p.x(),
                          pre_integrations[curr_time]->delta_p.y(),
                          pre_integrations[curr_time]->delta_p.z()
                          )<<endl;
    }

    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;

}



