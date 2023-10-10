//
// Created by cjq on 23-4-23.
//

#include "utils/def.h"
#include "utils/gps_pose.h"
#include "utils/file_utils.h"


int main(int argc,char** argv) {

    if(argc!=4){
        cerr<<"Usage:./select_mask_by_rgb ${rgb_dir} ${mask_dir} ${save_dir}"<<endl;
        return -1;
    }

    fs::path rgb_dir(argv[1]);
    fs::path mask_dir(argv[2]);
    fs::path save_dir(argv[3]);

    if(!fs::exists(rgb_dir)){
        cerr<<rgb_dir<<" is not exist"<<endl;
        return -1;
    }
    else if(!fs::is_directory(rgb_dir)){
        cerr<<rgb_dir<<" is not a dir"<<endl;
        return -1;
    }

    if(!fs::exists(mask_dir)){
        cerr<<mask_dir<<" is not exist"<<endl;
        return -1;
    }
    else if(!fs::is_directory(mask_dir)){
        cerr<<mask_dir<<" is not a dir"<<endl;
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

    vector<fs::path> rgb_names = GetDirectoryFileNames(rgb_dir);

    for(fs::path &rgb_name : rgb_names){
        fs::path mask_name = rgb_name.string() + string(".png");
        CopyFile(mask_dir/mask_name,save_dir/mask_name);
        cout<<mask_name<<endl;
    }

    return 0;
}



