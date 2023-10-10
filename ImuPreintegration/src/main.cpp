#include <iostream>
#include "utils/def.h"
#include "utils/file_utils.h"
#include "utils/imu_measurement.h"
#include "imu_processor.h"



int main() {

    ImuProcessor imu_processor("/home/cjq/CLionProjects/vins_fusion_ws/src/VINS-Fusion/config/euroc/euroc_stereo_config.yaml");



    fs::path rgb_dir("/media/cjq/新加卷/datasets/COLMAP/sfm_imu_city/city_2022-10-17-15-41-09/sfm/1600_1988/images");

    vector<string> img_names;
    GetAllImageNames(rgb_dir, img_names);

    std::sort(img_names.begin(),img_names.end());
    std::vector<ImuMeasurement> imu_data;

    std::ifstream infile("/media/cjq/新加卷/datasets/COLMAP/sfm_imu_city/city_2022-10-17-15-41-09/sfm/1600_1988/imu_reduce.txt",
                         std::ios::in);
    if(!infile.is_open()){
        cerr<<"open imu txt file failed!"<<endl;
        return -1;
    }

    string line;
    while(std::getline(infile,line)){
        vector<string> tokens;
        split(line,tokens);
        if(tokens.size()==7){
            ImuMeasurement imu_d{};
            imu_d.time = std::stod(tokens[0]);
            imu_d.angular_x = std::stod(tokens[1]);
            imu_d.angular_y = std::stod(tokens[2]);
            imu_d.angular_z = std::stod(tokens[3]);
            imu_d.acc_x = std::stod(tokens[4]);
            imu_d.acc_y = std::stod(tokens[5]);
            imu_d.acc_z = std::stod(tokens[6]);
            imu_data.push_back(imu_d);
        }
    }

    double prev_frame_time = -1;

    int imu_idx=0;
    for(int i=0;i<img_names.size();++i){
        string& img_name = img_names[i];
        double curr_frame_time = std::stod(fs::path(img_name).stem().string());


        std::vector<pair<double, Eigen::Vector3d>> accBuf;
        std::vector<pair<double, Eigen::Vector3d>> gyrBuf;
        while(imu_idx < imu_data.size() && imu_data[imu_idx].time < curr_frame_time){
            Eigen::Vector3d acc(imu_data[imu_idx].acc_x, imu_data[imu_idx].acc_y, imu_data[imu_idx].acc_z);
            Eigen::Vector3d gyr(imu_data[imu_idx].angular_x, imu_data[imu_idx].angular_y, imu_data[imu_idx].angular_z);
            accBuf.emplace_back(imu_data[imu_idx].time, acc);
            gyrBuf.emplace_back(imu_data[imu_idx].time, gyr);
            imu_idx++;
        }

        ///进行预积分
        cout<<fmt::format("acc_buf size:{}",accBuf.size())<<endl;
        for(size_t j = 0; j < accBuf.size(); j++)
        {
            double dt;
            if(j == 0)
                dt = accBuf[j].first - prev_frame_time;
            else if (j == accBuf.size() - 1)
                dt = curr_frame_time - accBuf[j - 1].first;
            else
                dt = accBuf[j].first - accBuf[j - 1].first;

            cout<<"j:"<<j<<endl;

            imu_processor.ProcessIMU(accBuf[j].first, dt, accBuf[j].second, gyrBuf[j].second,
                                     curr_frame_time);
        }

        prev_frame_time = curr_frame_time;
    }




    return 0;
}
