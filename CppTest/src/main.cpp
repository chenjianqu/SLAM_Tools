#include <iostream>


std::string CutLastSubString(const std::string &s,char c='.'){
    int path_len = s.size();
    if(s.empty()){
        return s;
    }
    else if(s[path_len-1]==c){
        return s.substr(0,path_len-1);
    }
    else{
        int suffix_len=0;
        for(int i=path_len-1;i>=0;--i){
            if(s[i]==c){
                break;
            }
            suffix_len++;
        }
        if(suffix_len==0){ //无.
            return s;
        }
        else{//有点，把点也去掉
            return s.substr(0,path_len-suffix_len-1);
        }
    }
}


std::string FileNameStem(const std::string &path){
    size_t lc= path.find_last_of('/');
    if(lc!=std::string::npos){
        std::string file_name = path.substr(lc+1);
        size_t dot_end = file_name.find_last_of('.');
        return file_name.substr(0,dot_end);
    }
    else{
        size_t dot_end = path.find_last_of('.');
        return path.substr(0,dot_end);
    }
}


int main() {
    std::cout << "Hello, World!" << std::endl;

    std::string s="/chen/jianqu/img.png";
    std::cout<<FileNameStem(s)<<std::endl;

    s="img.png";
    std::cout<<FileNameStem(s)<<std::endl;

    s="/chen/";
    std::cout<<FileNameStem(s)<<std::endl;

    s="chen.";
    std::cout<<FileNameStem(s)<<std::endl;

    s="/chen.";
    std::cout<<FileNameStem(s)<<std::endl;

    return 0;
}
