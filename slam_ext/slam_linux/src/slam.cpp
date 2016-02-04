//
// Created by cmaster11 on 04/08/15.
//

#include "slam.h"


ORB_SLAM::ORBVocabulary *Vocabulary;
ORB_SLAM::KeyFrameDatabase *Database;
ORB_SLAM::Map *World;
ORB_SLAM::Tracking *Tracker;
ORB_SLAM::LocalMapping *LocalMapper;
ORB_SLAM::LoopClosing *LoopCloser;
ORB_SLAM::PoseType pose;

boost::thread *localMappingThread;
boost::thread *trackingThread;
boost::thread *loopClosingThread;


int frameCount;
bool paused;

bool poseFailed = false;

Mat mGrTmpCrop, mGrTmp2;
bool frameMatInitialized = false;
bool slamInitialized = false;
int finalWidth;
//400; //320;
int finalHeight;
// = 240;//240; //240;
Size size320;
// = Size(finalWidth,finalHeight);
float finalRatio;
// = (float)finalWidth/(float)finalHeight;
float cameraRatio;

Size cameraSize;
int cropWidth;
int cropHeight;
int cropLeft;
int cropTop = 0;
Rect cropRect;
bool hasToResize = false;

// SLAM vars
char line1[1000];
char line2[1000];

ORB_SLAM::Tracking * getTracking(){
    return Tracker;
}

struct initStructModel {

    string strSettingsFile;
    double fx;
    double fy;
    double cx;
    double cy;
} initStruct;

void resetPose(ORB_SLAM::PoseType &p) {
    p.tx = 0;
    p.ty = 0;
    p.tz = 0;
    p.qx = 0;
    p.qy = 0;
    p.qz = 0;
    p.qw = 1;
}

Pose getPose() {
    Pose p;

    p.tx = pose.tx;
    p.ty = pose.ty;
    p.tz = pose.tz;
    p.qx = pose.qx;
    p.qy = pose.qy;
    p.qz = pose.qz;
    p.qw = pose.qw;

    return p;
}

void slam(Mat &frame) {
    if (!slamInitialized) return;

    try {
        Mat mRgb = frame;
        Mat mGr; // = *(Mat*) addrGray;
        cvtColor(mRgb, mGr, COLOR_BGR2GRAY);
        if (frameCount > 1) {
            if (!frameMatInitialized) {
                cameraSize = mGr.size();

                LOGI("Frame mat size: %d, %d", cameraSize.width, cameraSize.height);

                cameraRatio = (float) cameraSize.width / (float) cameraSize.height;
                finalWidth = (int) (cameraRatio * (float) finalHeight);

                size320 = Size(finalWidth, finalHeight);
                finalRatio = (float) finalWidth / (float) finalHeight;

                if (cameraSize.height != finalHeight) {
                    hasToResize = true;

                    mGrTmp2 = Mat::zeros(size320, mGr.type());
                    LOGI("Set frame resize to %d, %d", size320.width, size320.height);
                }

                snprintf(line1, 1000, "  ");
                snprintf(line2, 1000, "    Initialization");

                frameMatInitialized = true;
                LOGI("Frame mat initialized");
            }

            if (!frameMatInitialized)
                return;

            if (hasToResize) {
                resize(mGr, mGrTmp2, size320);

            }

            double tm = std::chrono::duration_cast<std::chrono::seconds>(
                    std::chrono::system_clock::now().time_since_epoch()).count();
            if (Tracker->addFrame(hasToResize ? mGrTmp2 : mGr, tm)) {
                //  LOGI("Frame added successfully");
            } else {
                //  LOGI("Buffer Filled");
            }


            Tracker->getPosition(pose);
            poseFailed = false;




        } else {
            poseFailed = true;
            snprintf(line1, 1000, "  ");
            snprintf(line2, 1000, "    Initialization");
        }

        ++frameCount;
    } catch (std::exception &ex) {
        LOGE("Exception: %s", ex.what());

        poseFailed = false;
        return;


    }
}

void loadVocabulary(string strVocFile) {

    LOGI("Got vocabulary file %s", strVocFile.c_str());

    cv::FileStorage fsVoc(strVocFile.c_str(), cv::FileStorage::READ);

    if (!fsVoc.isOpened()) {
        LOGE("Wrong path to vocabulary.\n");
        return;
    }

    paused = true;
    slamInitialized = false;

    clearSlam();

    Vocabulary = new ORB_SLAM::ORBVocabulary;
    Vocabulary->load(fsVoc);

    LOGI("Vocabulary loaded!\n");

    Database = new ORB_SLAM::KeyFrameDatabase(*Vocabulary);
    World = new ORB_SLAM::Map;

    frameCount = 0;
    Tracker = new ORB_SLAM::Tracking(Vocabulary, World, initStruct.strSettingsFile, initStruct.fx, initStruct.fy,
                                     initStruct.cx, initStruct.cy);
    trackingThread = new boost::thread(&ORB_SLAM::Tracking::Run, Tracker);

    Tracker->SetKeyFrameDatabase(Database);

//Initialize the Local Mapping Thread and launch
    LocalMapper = new ORB_SLAM::LocalMapping(World);
    localMappingThread = new boost::thread(&ORB_SLAM::LocalMapping::Run, LocalMapper);

//Initialize the Loop Closing Thread and launch
    LoopCloser = new ORB_SLAM::LoopClosing(World, Database, Vocabulary);
    loopClosingThread = new boost::thread(&ORB_SLAM::LoopClosing::Run, LoopCloser);

//Set pointers between threads
    Tracker->SetLocalMapper(LocalMapper);
    Tracker->SetLoopClosing(LoopCloser);

    LocalMapper->SetTracker(Tracker);
    LocalMapper->SetLoopCloser(LoopCloser);

    LoopCloser->SetTracker(Tracker);
    LoopCloser->SetLocalMapper(LocalMapper);

    slamInitialized = true;
    paused = false;
    LOGI("Initialization completed");

}

void slamInitialization(string strSettingsFile, int _finalHeight, double fx, double fy, double cx, double cy) {
    finalHeight = _finalHeight;

    LOGI("Got settings file %s", strSettingsFile.c_str());

    cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);

    if (!fsSettings.isOpened()) {
        LOGE("Wrong path to settings.\n");
        return;
    }

    initStruct.strSettingsFile = strSettingsFile;
    initStruct.cx = cx;
    initStruct.cy = cy;
    initStruct.fx = fx;
    initStruct.fy = fy;

}

void clearSlam() {
//        if(Tracker!=nullptr)
//        {
//        Tracker->Reset();
//        }


}