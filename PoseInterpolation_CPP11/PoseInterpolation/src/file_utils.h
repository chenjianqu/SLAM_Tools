/*******************************************************
 * Copyright (C) 2022, Chen Jianqu, Shanghai University
 *
 * This file is part of dynamic_vins.
 * Github:https://github.com/chenjianqu/dynamic_vins
 *
 * Licensed under the MIT License;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#ifndef DYNAMIC_VINS_FILE_UTILS_H
#define DYNAMIC_VINS_FILE_UTILS_H

#include "def.h"


int CreateDirs(const std::string& dir);

void ClearDirectory(const string &path);

void GetDirectoryFile(const string& path,vector<string>& filenames);
void GetDirectoryFileNames(const string& path,vector<string>& filenames);

void WriteTextFile(const string& path,std::string& text);

bool IsExists(const string& path);

string StringStem(const string& path);



#endif //DYNAMIC_VINS_FILE_UTILS_H
