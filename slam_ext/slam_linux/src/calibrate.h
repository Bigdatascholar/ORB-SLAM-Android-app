//
// Created by cmaster11 on 23/04/15.
//

#pragma once

#include "includes.h"

     void loadParameters();
     void saveIntrinsicParameters(Mat &intrinsic, Mat &distCoeffs);
     void loadIntrinsicParameters(Mat &intrinsic, Mat &distCoeffs);
     Mat* getIntrinsicPtr();
     Mat* getDistCoeffsPtr();



