//
// Created by cjq on 23-7-27.
//

#ifndef COLMAP_CONFIGREADER_H
#define COLMAP_CONFIGREADER_H

#include <string>
#include <vector>
#include <filesystem>
#include <spdlog/spdlog.h>
#include <yaml-cpp/yaml.h>
#include <Eigen/Eigen>


class ConfigReader{
public:
    explicit ConfigReader(const std::string& config_path):config_path_(config_path){
        if(!std::filesystem::exists(config_path)){
            SPDLOG_ERROR("config_path {} not exists!",config_path);
            std::terminate();
        }
        try {
            config = YAML::LoadFile(config_path);
        } catch (YAML::Exception &exception) {
            SPDLOG_ERROR("Failed to open configuration file:{}",config_path);
            std::terminate();
        }
    }

    template<class T> T get(const std::string& key){
        if(!config[key].IsDefined()){
            SPDLOG_ERROR("key:{} not exists in the file:{}",key,config_path_);
            std::terminate();
        }
        try {
            T value = config[key].as<T>();
            return value;
        }
        catch (std::exception& e){
            SPDLOG_ERROR("try to get key:{} from {} failed! \n what:{}",key,config_path_,e.what());
        }
        return {};
    }

    template<class T> T get(const std::string& key1,const std::string& key2){
        if(!config[key1].IsDefined()){
            SPDLOG_ERROR("key:{} not exists in the file:{}",key1,config_path_);
            std::terminate();
        }
        if(!config[key1][key2].IsDefined()){
            SPDLOG_ERROR("key:{}-{} not exists in the file:{}",key1,key2,config_path_);
            std::terminate();
        }
        try {
            T value = config[key1][key2].as<T>();
            return value;
        }
        catch (std::exception& e){
            SPDLOG_ERROR("try to get key:{}-{} from {} failed! \n what:{}",key1,key2,config_path_,e.what());
        }
        return {};
    }

    Eigen::Vector3d getVector3d(const std::string& key){
        auto vec_data = get<std::vector<double>>(key);
        return Eigen::Vector3d {vec_data.data()};
    }

    Eigen::Vector3d getVector3d(const std::string& key1,const std::string& key2){
        auto vec_data = get<std::vector<double>>(key1,key2);
        return Eigen::Vector3d {vec_data.data()};
    }

    bool exists(const std::string& key){
        return config[key].IsDefined();
    }

    bool exists(const std::string& key1,const std::string& key2){
        if(!config[key1].IsDefined())
            return false;
        if(!config[key1][key2].IsDefined())
            return false;
        return true;
    }

    YAML::Node& node(){return config;}

private:
    YAML::Node config;
    std::string config_path_;
};


#endif //COLMAP_CONFIGREADER_H
