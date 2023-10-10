//
// Created by cjq on 23-8-26.
//

#include <queue>
#include <thread>
#include <filesystem>
#include <fstream>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <spdlog/fmt/fmt.h>
#include <spdlog/spdlog.h>

#include "utils/config_reader.h"

using namespace std;
namespace fs=std::filesystem;


DEFINE_string(yaml_path, "", "yaml_path");
DEFINE_string(key, "", "key");
DEFINE_string(string_value, "", "string_value");
DEFINE_int32(int_value, 0, "int_value");
DEFINE_double(double_value, 0, "double_value");
DEFINE_bool(bool_value, false, "bool_value");


int main(int argc, char **argv) {
    google::ParseCommandLineFlags(&argc, &argv, true);

    if(FLAGS_yaml_path.empty()){
        SPDLOG_ERROR("please set -yaml_path");
        return -1;
    }
    if(FLAGS_key.empty()){
        SPDLOG_ERROR("please set -key");
        return -1;
    }

    ConfigReader reader(FLAGS_yaml_path);

    google::CommandLineFlagInfo info;
    if(GetCommandLineFlagInfo("string_value" ,&info) && !info.is_default) {
        reader.node()[FLAGS_key] = FLAGS_string_value;
    }
    if(GetCommandLineFlagInfo("int_value" ,&info) && !info.is_default) {
        reader.node()[FLAGS_key] = FLAGS_int_value;
    }
    if(GetCommandLineFlagInfo("double_value" ,&info) && !info.is_default) {
        reader.node()[FLAGS_key] = FLAGS_double_value;
    }
    if(GetCommandLineFlagInfo("bool_value" ,&info) && !info.is_default) {
        reader.node()[FLAGS_key] = FLAGS_bool_value;
    }

    std::ofstream fout(FLAGS_yaml_path);
    fout << reader.node();
    fout.close();

    return 0;
}