//
// Created by anant on 08-Feb-18.
//

#pragma once

#include <vector>
#include <string>

class FileHelper {

public:
    static bool readFloats(char *path, std::vector<float> &result);

    static bool printFloats(char *path, std::vector<float> &output);

};


