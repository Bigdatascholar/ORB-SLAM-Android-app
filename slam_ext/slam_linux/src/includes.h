//
// Created by cmaster11 on 04/08/15.
//

#pragma once

#include "opencv2/opencv.hpp"
#include <iostream>
#include <string>

using namespace std;
using namespace cv;

struct Pose{
    float tx;
    float ty;
    float tz;
    float qx;
    float qy;
    float qz;
    float qw;
};

extern char *BASE_OUTPUT_PATH;
extern string BASE_OUTPUT_PATH_STRING;

#define LOGI(...) printf(__VA_ARGS__)
#define LOGE(...) printf(__VA_ARGS__)