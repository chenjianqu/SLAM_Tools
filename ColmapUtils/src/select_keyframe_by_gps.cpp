//
// Created by cjq on 23-4-23.
//

#include "utils/def.h"
#include "utils/gps_pose.h"
#include "utils/file_utils.h"


int main(int argc,char** argv) {

    if(argc!=4){
        cerr<<"Usage:./select_keyframe_by_gps ${img_dir} ${gps_record_path} ${save_dir}"<<endl;
        return -1;
    }

    fs::path img_dir(argv[1]);
    fs::path gps_record_path(argv[2]);
    fs::path save_dir(argv[3]);

    if(!fs::exists(img_dir)){
        cerr<<img_dir<<" is not exist"<<endl;
        return -1;
    }
    else if(!fs::is_directory(img_dir)){
        cerr<<img_dir<<" is not a dir"<<endl;
        return -1;
    }
    if(!fs::exists(gps_record_path)){
        cerr<<gps_record_path<<" is not exist"<<endl;
        return -1;
    }
    if(!fs::exists(save_dir)){
        cerr<<save_dir<<" is not exist"<<endl;
        return -1;
    }
    else if(!fs::is_directory(save_dir)){
        cerr<<save_dir<<" is not a dir"<<endl;
        return -1;
    }

    std::map<double,GpsPose> poses=GpsPose::ReadGpsPoseFromTxt(gps_record_path);

    vector<fs::path> img_names = GetDirectoryFileNames(img_dir);
    vector<fs::path> kf_names;

    auto name_to_stamp = [](const string &name){
        string s = name;
        while (s.find(".png") < s.length()) {
            s.erase(s.find(".png"), s.size());
        }
        return std::stod(s);
    };

    auto it_last= poses.end();

    for(const auto &name:img_names){
        double img_stamp = name_to_stamp(name);//文件名可能时.png或.png.png
        auto near = poses.lower_bound(img_stamp);
        if(near==poses.end()){ //若容器内所有节点都比 img_stamp 小，即img_stamp是最后一个时间戳，则返回值：map.end()
            cerr<<fmt::format("所有的GPS位姿时间戳均小于当前图像时间戳:{:.6f}",img_stamp)<<endl;
            kf_names.push_back(name);
            it_last = near;
        }
        else if(near==poses.begin()){
            cerr<<fmt::format("所有的GPS位姿时间戳均大于等于当前图像时间戳:{:.6f}",img_stamp)<<endl;
            kf_names.push_back(name);
            it_last = near;
        }
        else{
            if(it_last == poses.end()){
                it_last = near;
            }
            else{
                const GpsPose &pose_curr = near->second;
                const GpsPose &pose_last = it_last->second;
                double delta = GpsPose::Distance(pose_curr,pose_last);

                if(delta>0.5)//关键帧的两个图像帧至少需要间隔0.5m
                {
                    kf_names.push_back(name);
                    it_last = near;
                }
                cout<<delta<<endl;
            }
        }
    }

    for(fs::path &name:kf_names){
        CopyFile(img_dir/name,save_dir/name);
        cout<<save_dir/name<<endl;
    }

    return 0;
}



