//
// Created by cmaster11 on 23/04/15.
//

#include "calibrate.h"
#include "opencv2/core/core.hpp"

const char* CALIBRATE_FILENAME = "%s/../calibrateData/calibrate.data";

Mat _intrinsic, _distCoeffs;
bool alreadyLoaded = false;

//Mat _cameraMatrix;

void saveIntrinsicParameters(Mat &intrinsic, Mat &distCoeffs) {
    FileStorage file(CALIBRATE_FILENAME, FileStorage::WRITE);
    file << "intrinsic" << intrinsic;
    file << "distCoeffs" << distCoeffs;
    file.release();
}


void loadIntrinsicParameters(Mat &intrinsic, Mat &distCoeffs) {
    loadParameters();

    intrinsic = _intrinsic;
    distCoeffs = _distCoeffs;
}

void loadParameters() {
    if (!alreadyLoaded) {
        char fileNameBuffer[1024];
        sprintf(fileNameBuffer, CALIBRATE_FILENAME, BASE_OUTPUT_PATH);
        FileStorage file(fileNameBuffer, FileStorage::READ);
        if (!file.isOpened()) {
            cerr << "Cannot find calibrate file " << endl;
            assert(false);
        }
        Mat tmpI, tmpD;
        file["intrinsic"] >> tmpI;
        file["distCoeffs"] >> tmpD;

        _intrinsic = Mat::zeros(tmpI.size(), CV_32F);
        _distCoeffs = Mat::zeros(tmpD.size(), CV_32F);

        tmpI.convertTo(_intrinsic, CV_32F);
        tmpD.convertTo(_distCoeffs, CV_32F);
        file.release();
        alreadyLoaded = true;
    }
}

Mat *getIntrinsicPtr() {
    loadParameters();
    return &_intrinsic;
}

Mat *getDistCoeffsPtr() {
    loadParameters();
    return &_distCoeffs;
}
