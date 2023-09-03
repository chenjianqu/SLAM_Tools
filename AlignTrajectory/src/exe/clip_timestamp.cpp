//
// Created by cjq on 23-7-28.
//
#include <string>
#include <fstream>

#include <spdlog/fmt/fmt.h>

#include "def.h"

using namespace std;

int main(int argc, char *argv[]) {

    string file_path(argv[1]);
    std::ifstream infile(file_path);
    std::ofstream outfile(file_path+".txt");

    string line;
    while(std::getline(infile,line)){
        vector<string> token;
        split(line,token," ");

        double time_stamp(stod(token[0]));
        string new_time_stamp_str = fmt::format("{:.3f}",time_stamp);
        for(int i=1;i<token.size();++i){
            new_time_stamp_str += string(" ")+token[i];
        }
        outfile<<new_time_stamp_str<<endl;
    }

    infile.close();
    outfile.close();

    return 0;
}