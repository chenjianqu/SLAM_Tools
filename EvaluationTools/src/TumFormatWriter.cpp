//
// Created by cjq on 23-4-18.
//

#include <iostream>
#include <filesystem>
#include <fstream>


using namespace std;
namespace fs=std::filesystem;

/*
 * 读取文件格式为：time,tx,ty,tz,qx,qy,qz,qw的文件，转换为TUM格式
 * 如：1662063135.777000,366763.099906,3450103.800378,29.844046,-0.000768,0.014223,0.832526,0.553802
 * 转换为：
 * 1662063135.777000 366763.099906 3450103.800378 29.844046 -0.000768 0.014223 0.832526 0.553802
 */

int main(int argc,char** argv) {
    if(argc!=2){
        cerr<<"Usage:./TumFormatWriter ${pose_file}"<<endl;
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

    fs::path write_path = pose_path.parent_path()/(pose_path.stem().string()+"_tum.txt");
    ofstream fout_pose(write_path,std::ios::out);

    ///将字符串中的逗号换成空格
    string line;
    while(std::getline(fin_pose,line)){
        for(char& c:line){
            if(c==','){
                c=' ';
            }
        }
        fout_pose<<line<<endl;
    }

    fin_pose.close();
    fout_pose.close();

    return 0;
}

