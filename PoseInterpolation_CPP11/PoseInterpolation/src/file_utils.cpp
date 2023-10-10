/*******************************************************
 * Copyright (C) 2022, Chen Jianqu, Shanghai University
 *
 * This file is part of dynamic_vins.
 * Github:https://github.com/chenjianqu/dynamic_vins
 *
 * Licensed under the MIT License;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "file_utils.h"

#include <string>
#include <dirent.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <unistd.h>
#include <fcntl.h>


/**
 * 创建文件夹
 * @param dir
 * @return
 */
int CreateDir(const std::string& dir)
{
    int ret = mkdir(dir.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    if (ret && errno == EEXIST){
        printf("dir[%s] already exist.\n",dir.c_str());
    }
    else if (ret){
        printf("create dir[%s] error: %d %s\n" ,dir.c_str(),ret ,strerror(errno));
        return -1;
    }
    else{
        printf("create dir[%s] success.\n", dir.c_str());
    }
    return 0;
}


/**
 * 获取一个路径的父文件夹
 * @param dir
 * @return
 */
std::string GetParentDir(const std::string& dir)
{
    std::string pdir = dir;
    if(pdir.length() < 1 || (pdir[0] != '/')){
        return "";
    }
    while(pdir.length() > 1 && (pdir[pdir.length() -1] == '/'))
        pdir = pdir.substr(0,pdir.length() -1);

    pdir = pdir.substr(0,pdir.find_last_of('/'));
    return pdir;
}


/**
 * 递归创建多级目录
 * @param dir
 * @return
 */
int CreateDirs(const std::string& dir){
    int ret = 0;
    if(dir.empty())
        return -1;
    std::string pdir;
    if((ret = CreateDir(dir)) == -1){
        pdir = GetParentDir(dir);
        if((ret = CreateDir(pdir)) == 0){
            ret = CreateDir(dir);
        }
    }
    return ret;
}



void GetDirectoryFile(const string& path,vector<string>& filenames)
{
    DIR *pDir;
    struct dirent* ptr;
    if(!(pDir = opendir(path.c_str()))){
        cout<<"Folder doesn't Exist!"<<endl;
        return;
    }
    while((ptr = readdir(pDir))!=0) {
        if (strcmp(ptr->d_name, ".") != 0 && strcmp(ptr->d_name, "..") != 0){
            filenames.push_back(path + "/" + ptr->d_name);
        }
    }
    closedir(pDir);
}


void GetDirectoryFileNames(const string& path,vector<string>& filenames)
{
    DIR *pDir;
    struct dirent* ptr;
    if(!(pDir = opendir(path.c_str()))){
        cout<<"Folder doesn't Exist!"<<endl;
        return;
    }
    while((ptr = readdir(pDir))!=0) {
        if (strcmp(ptr->d_name, ".") != 0 && strcmp(ptr->d_name, "..") != 0){
            filenames.emplace_back(ptr->d_name);
        }
    }
    closedir(pDir);
}



/**
 * 将字符串写入到文件中
 * @param path
 * @param text
 */
void WriteTextFile(const string& path,std::string& text){
    static std::set<string> path_set;
    ///第一次,清空文件
    if(path_set.find(path)==path_set.end()){
        path_set.insert(path);
        std::ofstream fout( path.data(), std::ios::out);
        fout.close();
    }

    ///添加到文件后面
    std::ofstream fout(path.data(), std::ios::app);
    fout<<text<<endl;
    fout.close();
}



/**
 * 判断文件是否存在
 * @param path
 */
bool IsExists(const string& path){
    /**
 * int access(const char *pathname, int mode);

参数：
pathname: 需要测试的文件路径名。
mode: 需要测试的操作模式，可能值是一个或多个R_OK(可读?), W_OK(可写?), X_OK(可执行?) 或 F_OK(文件存在?)组合体。

返回说明：
成功执行时，返回0。失败返回-1，errno被设为以下的某个值
EINVAL： 模式值无效
EACCES： 文件或路径名中包含的目录不可访问
ELOOP ： 解释路径名过程中存在太多的符号连接
ENAMETOOLONG：路径名太长
ENOENT：  路径名中的目录不存在或是无效的符号连接
ENOTDIR： 路径名中当作目录的组件并非目录
EROFS： 文件系统只读
EFAULT： 路径名指向可访问的空间外
EIO：  输入输出错误
ENOMEM： 不能获取足够的内核内存
ETXTBSY：对程序写入出错
 */
    if((access(path.c_str(),F_OK))!=-1){
        return true;
    }
    else{
        return false;
    }
}


string StringStem(const string& path){
    return path.substr(0,path.find_last_of('.')+1);
}



