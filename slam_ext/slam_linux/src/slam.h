//
// Created by cmaster11 on 04/08/15.
//

#pragma once

#include "includes.h"

#include <vector>
#include <fstream>
#include <unistd.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <chrono>

#include "Tracking.h"
#include "Map.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "KeyFrameDatabase.h"
#include "ORBVocabulary.h"
#include "Converter.h"

// Functions definitions
void resetPose(ORB_SLAM::PoseType &p);
void clearSlam();

Pose getPose();
ORB_SLAM::Tracking * getTracking();

void slam(Mat & frame);

void loadVocabulary(string strVocFile);
void slamInitialization(string strSettingsFile, int _finalHeight,double fx, double fy, double cx, double cy);