//
// Created by cjq on 23-4-18.
//

#include <iostream>
#include <filesystem>
#include <fstream>


#include "def.h"


using namespace std;
namespace fs=std::filesystem;

/*
 * 读取COLMAP输出的位姿，格式为：
IMAGE_ID, QW, QX, QY, QZ, TX, TY, TZ, CAMERA_ID, NAME
POINTS2D[] as (X, Y, POINT3D_ID)
转换为TUM格式:
time tx ty tz qx qy qz qw

 * 如：
1 0.99992858260802309 0.0014179597844599932 -0.0110791358827215 0.0042510965234694937 -0.25764907856375746 0.061655082901598483 6.561897614517453 1 1665979474.077000.png
1194.1837158203125 28.054967880249023 -1 1197.033203125 27.84843635559082 -1 1197.033203125 27.84843635559082 -1 ...
 */

int main(int argc,char** argv) {
    if(argc!=2){
        cerr<<"Usage:./ReadColmapToTum ${pose_file}"<<endl;
        return -1;
    }

    fs::path pose_path(argv[1]);
    if(!fs::exists(pose_path)){
        cerr<<pose_path<<" does not exist"<<endl;
        return -1;
    }

    ifstream fin_pose(pose_path,std::ios::in);
    if(!fin_pose.is_open()){
        cerr<<"open "<<pose_path<<" failed"<<endl;
        return -1;
    }


    string line;

    ///省略前4行
    constexpr int kDiscardLineNum=4;
    for(int i=0;i<kDiscardLineNum;++i){
        std::getline(fin_pose,line);
    }

    ///注意COLMAP是无序的，这里需要转换为有序的TUM结果,通过MAP
    std::map<string,string> map_lines;

    int line_idx=-1;
    while(std::getline(fin_pose,line)){
        line_idx++;
        if(line_idx%2==1){ //跳过奇数行
            continue;
        }
        vector<string> tokens;
        split(line,tokens);

        //IMAGE_ID, QW, QX, QY, QZ, TX, TY, TZ, CAMERA_ID, NAME
        fs::path img_name = tokens[9];
        string time_stamp = img_name.stem();

        ///注意，COLMAP默认的保存的位姿是 T_cw，也就是将点从世界坐标系变换到相机坐标系下。这与TUM的位姿表示T_wc是相反的
        ///https://blog.csdn.net/weixin_44120025/article/details/124604229
        ///因此需要将COLMAP位姿做逆变换
        Quatd q_cw(std::stod(tokens[1]),std::stod(tokens[2]),std::stod(tokens[3]),std::stod(tokens[4]));
        Quatd q_wc = q_cw.inverse();
        Vec3d t_cw(std::stod(tokens[5]),std::stod(tokens[6]),std::stod(tokens[7]));
        Vec3d t_wc = - (q_wc * t_cw);

        string write_line = fmt::format("{} {:.10f} {:.10f} {:.10f} {:.10f} {:.10f} {:.10f} {:.10f}",
                                        time_stamp,
                                        t_wc.x(),t_wc.y(),t_wc.z(),
                                        q_wc.x(),q_wc.y(),q_wc.z(),q_wc.w());
        map_lines.insert({time_stamp,write_line});
    }

    fin_pose.close();

    fs::path write_path = pose_path.parent_path()/(pose_path.stem().string()+"_tum.txt");
    ofstream fout_pose(write_path,std::ios::out);

    for(const auto& p : map_lines){
        fout_pose<<p.second<<endl;
    }

    fout_pose.close();

    return 0;
}

