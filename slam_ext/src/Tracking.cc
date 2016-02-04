/**
 * This file is part of ORB-SLAM.
 *
 * Copyright (C) 2014 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
 * For more information see <http://webdiis.unizar.es/~raulmur/orbslam/>
 *
 * ORB-SLAM is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ORB-SLAM is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with ORB-SLAM. If not, see <http://www.gnu.org/licenses/>.
 */

#include "Tracking.h"
//#include<ros/ros.h>
//#include <cv_bridge/cv_bridge.h>

#include<opencv2/opencv.hpp>

#include"ORBmatcher.h"
//#include"FramePublisher.h"
#include"Converter.h"
#include"Map.h"
#include"Initializer.h"

#include"Optimizer.h"
#include"PnPsolver.h"

#include<iostream>
#include<fstream>
#include <queue>

#include "wtLog.h"

#include <chrono>
//#define NOW() std::chrono::high_resolution_clock::now()
//#define CRON(timeBegin) std::chrono::duration_cast<std::chrono::milliseconds>(NOW() - timeBegin).count()

//#define USE_CRON
#define MIN_FEATURES_FOR_INITIALIZATION 80

using namespace std;

namespace ORB_SLAM
{

auto mTrackingInitMutex = new boost::mutex;

Tracking::Tracking(ORBVocabulary* pVoc, /* FramePublisher *pFramePublisher, MapPublisher *pMapPublisher, */Map *pMap, string strSettingPath, double _fx, double _fy, double _cx, double _cy) :
    mState(NO_IMAGES_YET), mpORBVocabulary(pVoc), /*mpFramePublisher(pFramePublisher), mpMapPublisher(pMapPublisher), */mpMap(pMap),
    mnLastRelocFrameId(0), mbPublisherStopped(false), mbReseting(false), mbForceRelocalisation(false), mbMotionModel(false)
{
    // Load camera parameters from settings file
    mTrackingInitMutex->lock();



    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    double fx = _fx;//fSettings["Camera.fx"];
    double fy = _fy;//fSettings["Camera.fy"];
    double cx = _cx;//(double)width/2.0f;
    double cy = _cy;//(double)height/2.0f;

    cv::Mat K = cv::Mat::eye(3, 3, CV_64F);
    K.at<double>(0, 0) = fx;
    K.at<double>(1, 1) = fy;
    K.at<double>(0, 2) = cx;
    K.at<double>(1, 2) = cy;
    K.copyTo(mK);

    cv::Mat DistCoef = cv::Mat::zeros(4, 1, CV_64F);
    //DistCoef.at<double>(0) = fSettings["Camera.k1"];
    //DistCoef.at<double>(1) = fSettings["Camera.k2"];
    //DistCoef.at<double>(2) = fSettings["Camera.p1"];
    //DistCoef.at<double>(3) = fSettings["Camera.p2"];
    DistCoef.copyTo(mDistCoef);

    double fps = 10;//fSettings["Camera.fps"];
    if (fps == 0)
        fps = 30;

    // Max/Min Frames to insert keyframes and to check relocalisation
    mMinFrames = 0;
    mMaxFrames = 6;//8.f * fps / 30.f;


    cout << "Camera Parameters: " << endl;
    cout << "- fx: " << fx << endl;
    cout << "- fy: " << fy << endl;
    cout << "- cx: " << cx << endl;
    cout << "- cy: " << cy << endl;
    cout << "- k1: " << DistCoef.at<double>(0) << endl;
    cout << "- k2: " << DistCoef.at<double>(1) << endl;
    cout << "- p1: " << DistCoef.at<double>(2) << endl;
    cout << "- p2: " << DistCoef.at<double>(3) << endl;
    cout << "- fps: " << fps << endl;

    mbRGB = 1;
    // Load ORB parameters

    int nFeatures = fSettings["ORBextractor.nFeatures"];
     fScaleFactor = fSettings["ORBextractor.scaleFactor"];
     nLevels = fSettings["ORBextractor.nLevels"];
    int fastTh = fSettings["ORBextractor.fastTh"];
    int Score = fSettings["ORBextractor.nScoreType"];

    assert(Score == 1 || Score == 0);

#ifdef USE_FAST_FEATURES

    // FAST and BRIEF init?
    mFASTDetector = new cv::FastFeatureDetector(10);
    mBRIEFExtractor = new cv::BriefDescriptorExtractor(32);

#else
    mpORBextractor = new ORBextractor(nFeatures, fScaleFactor, nLevels, Score, fastTh);
#endif
	//	mcvORB = new cv::ORB(nFeatures, fScaleFactor, nLevels, 31, 0, 2, Score);

    cout << endl << "Extractor Parameters: " << endl;
    cout << "- Number of Features: " << nFeatures << endl;
    cout << "- Scale Levels: " << nLevels << endl;
    cout << "- Scale Factor: " << fScaleFactor << endl;
    cout << "- Fast Threshold: " << fastTh << endl;
    if (Score == 0)
        cout << "- Score: HARRIS" << endl;
    else
        cout << "- Score: FAST" << endl;


    // ORB extractor for initialization
    // Initialization uses only points from the finest scale level
    //mcvIniORB = new cv::ORB(nFeatures * 2, fScaleFactor, nLevels, 31, 0, 2, Score);

#ifdef USE_FAST_FEATURES
#else
    mpIniORBextractor = new ORBextractor(nFeatures * 2, fScaleFactor, nLevels, Score, fastTh);
#endif

    int nMotion = fSettings["UseMotionModel"];
    mbMotionModel = nMotion;

    if (mbMotionModel)
    {
        mVelocity = cv::Mat::eye(4, 4, CV_64F);
        cout << endl << "Motion Model: Enabled" << endl << endl;
    }
    else
        cout << endl << "Motion Model: Disabled (not recommended, change settings UseMotionModel: 1)" << endl << endl;


    //		tf::Transform tfT;
    //		tfT.setIdentity();
    //		mTfBr.sendTransform(tf::StampedTransform(tfT, ros::Time::now(), "/ORB_SLAM/World", "/ORB_SLAM/Camera"));

    mqueue = new std::deque<TimestampedMat>;
    mbufferMutex = new boost::mutex;
    mposeMutex = new boost::mutex;
    mcurrentPose = nullptr;
    bufferSize = 6;

    camToeng = Eigen::Affine3d::Identity();
    camToeng.prerotate(Eigen::Quaterniond(0, 1, 0, 0));
    camToeng.pretranslate(Eigen::Vector3d(0, 0, 0));
    camToWorldPrev = Eigen::Affine3d::Identity();
    camToWorldPrev.prerotate(Eigen::Quaterniond(1, 0, 0, 0));
    camToWorldPrev.pretranslate(Eigen::Vector3d(0, 0, 0));
    engToWorldPrev = Eigen::Affine3d::Identity();
    engToWorldPrev.prerotate(Eigen::Quaterniond(1, 0, 0, 0));
    engToWorldPrev.pretranslate(Eigen::Vector3d(0, 0, 0));

/*#ifdef ANDROID
    LOGI("Tracker initialization completed");
#endif*/

mTrackingInitMutex->unlock();
}

void Tracking::SetLocalMapper(LocalMapping *pLocalMapper)
{
    mpLocalMapper = pLocalMapper;
}

void Tracking::SetLoopClosing(LoopClosing *pLoopClosing)
{
    mpLoopClosing = pLoopClosing;
}

void Tracking::SetKeyFrameDatabase(KeyFrameDatabase *pKFDB)
{
    mpKeyFrameDB = pKFDB;
}

bool Tracking::addFrame(const cv::Mat& im, double timestamp)
{
    boost::mutex::scoped_lock waitLock(*mbufferMutex);
    if (mqueue->size() > bufferSize)
        return false;
    mqueue->push_back(TimestampedMat(im, timestamp));
    waitLock.unlock(); //?????????????
    return true;
}

void Tracking::Run()
{
mTrackingInitMutex->lock();
mTrackingInitMutex->unlock();
    //		ros::NodeHandle nodeHandler;
    //		ros::Subscriber sub = nodeHandler.subscribe("/camera/image_raw", 1, &Tracking::GrabImage, this);
    //
    //		ros::spin();
/*#ifdef ANDROID
    LOGI("Tracker running");
#endif*/
    while (true)
    {
        boost::mutex::scoped_lock waitLock(*mbufferMutex);
        if (mqueue->size() > 0)
        {
            TimestampedMat image = mqueue->front();
            mqueue->pop_front();
            waitLock.unlock();
            GrabImage(image);
        } else {
        waitLock.unlock();
}
        usleep(2000);
    }
}

void Tracking::GrabImage(const TimestampedMat& frame)
{
    //cout << "Image grab" << endl;

/*#if defined(USE_CRON)
   // LOGI("Image grabbing");
#endif*/
    cv::Mat im;

    //		// Copy the ros image message to cv::Mat. Convert to grayscale if it is a color image.
    //		cv_bridge::CvImageConstPtr cv_ptr;
    //		try {
    //			cv_ptr = cv_bridge::toCvShare(msg);
    //		} catch (cv_bridge::Exception& e) {
    //			ROS_ERROR("cv_bridge exception: %s", e.what());
    //			return;
    //		}

    assert(frame.im.channels() == 3 || frame.im.channels() == 1);

    if (frame.im.channels() == 3)
    {
        if (mbRGB)
            cvtColor(frame.im, im, CV_RGB2GRAY);
        else
            cvtColor(frame.im, im, CV_BGR2GRAY);
    }
    else if (frame.im.channels() == 1)
    {
        frame.im.copyTo(im);
    }

//    auto now = NOW();

#ifdef USE_FAST_FEATURES

    if (mState == WORKING || mState == LOST)
        //mCurrentFrame = Frame(im, frame.timestamp, mcvORB, fScaleFactor, nLevels, mpORBVocabulary, mK, mDistCoef);
        mCurrentFrame = Frame(im, frame.timestamp, mFASTDetector, mBRIEFExtractor, mpORBVocabulary, mK, mDistCoef);
    else
        //mCurrentFrame = Frame(im, frame.timestamp, mcvIniORB, 1.2, 8, mpORBVocabulary, mK, mDistCoef);
        mCurrentFrame = Frame(im, frame.timestamp, mFASTDetector, mBRIEFExtractor, mpORBVocabulary, mK, mDistCoef);

#else


    if (mState == WORKING || mState == LOST)
			//mCurrentFrame = Frame(im, frame.timestamp, mcvORB, fScaleFactor, nLevels, mpORBVocabulary, mK, mDistCoef);
        mCurrentFrame = Frame(im, frame.timestamp, mpORBextractor, mpORBVocabulary, mK, mDistCoef);
    else
			//mCurrentFrame = Frame(im, frame.timestamp, mcvIniORB, 1.2, 8, mpORBVocabulary, mK, mDistCoef);
        mCurrentFrame = Frame(im, frame.timestamp, mpIniORBextractor, mpORBVocabulary, mK, mDistCoef);

#endif

/*#if defined(USE_CRON)
    //LOGI("GrabImage - Frame %llu", CRON(now));
#endif*/
    // Depending on the state of the Tracker we perform different tasks

    if (mState == NO_IMAGES_YET)
    {
        mState = NOT_INITIALIZED;
/*#if defined(USE_CRON)
       // LOGI("mState = NO_IMAGES_YET");
#endif*/
    }

    mLastProcessedState = mState;

    if (mState == NOT_INITIALIZED)
    {
/*#if defined(USE_CRON)
        LOGI("mState = NOT_INITIALIZED");
#endif*/
        FirstInitialization();
    }
    else if (mState == INITIALIZING)
    {

/*#if defined(USE_CRON)
        LOGI("mState = INITIALIZING");
#endif*/
        Initialize();
    }
    else
    {
/*#if defined(USE_CRON)
      //  LOGI("mState = BHO");
#endif*/
        // System is initialized. Track Frame.
        bool bOK;

//        now = NOW();
        // Initial Camera Pose Estimation from Previous Frame (Motion Model or Coarse) or Relocalisation
        if (mState == WORKING && !RelocalisationRequested())
        {
            if (!mbMotionModel || mpMap->KeyFramesInMap() < 4 || mVelocity.empty() || mCurrentFrame.mnId < mnLastRelocFrameId + 2)
                bOK = TrackPreviousFrame();
            else
            {
                bOK = TrackWithMotionModel();
                if (!bOK)
                    bOK = TrackPreviousFrame();
            }
        }
        else
        {
            //cout << "Relocalize" << endl;
            bOK = Relocalisation();
        }
/*#if defined(USE_CRON)
        LOGI("GrabImage - Camera pose estimation or relocal. %llu", CRON(now));
#endif*/

//        now = NOW();
        // If we have an initial estimation of the camera pose and matching. Track the local map.
        if (bOK)
        {
            //cout << "Track previous movement ok" << endl;

            bOK = TrackLocalMap();
/*#if defined(USE_CRON)
          //  LOGI("bOK == true1");
#endif*/
        }
/*#if defined(USE_CRON)
        LOGI("GrabImage - TrackLocalMap %llu", CRON(now));
#endif*/

        // If tracking were good, check if we insert a keyframe
        if (bOK)
        {
            //cout << "TrackLocalMap ok" << endl;

//            now = NOW();
/*#if defined(USE_CRON)
          //  LOGI("bOK == true2, check position");
#endif*/
            //            mpMapPublisher->SetCurrentCameraPose(mCurrentFrame.mTcw); FABMOD
            ///////////////////////////////////
            KeyFrame* pKF = new KeyFrame(mCurrentFrame, mpMap, mpKeyFrameDB);
            cv::Mat R = pKF->GetRotation().t();
            vector<double> q = ORB_SLAM::Converter::toQuaternion(R);
            cv::Mat t = pKF->GetCameraCenter();
/*#if defined(USE_CRON)
           // LOGI("%f %f %f %f %f %f %f %f",pKF->mTimeStamp, t.at<double>(0), t.at<double>(1), t.at<double>(2), q[0], q[1], q[2], q[3]);
#endif*/
            //cout << setprecision(6) << pKF->mTimeStamp << setprecision(7) << " " << t.at<double>(0) << " " << t.at<double>(1) << " " << t.at<double>(2)
            //     << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
            ////////////////////////////////////

            {
                boost::mutex::scoped_lock waitLock(*mposeMutex);
                if (mcurrentPose != nullptr)
                {
                    delete mcurrentPose;
                    mcurrentPose = nullptr;
                }
                mcurrentPose = new KeyFrame(mCurrentFrame, mpMap, mpKeyFrameDB);
            }

            if (NeedNewKeyFrame())
                CreateNewKeyFrame();

            // We allow points with high innovation (considererd outliers by the Huber Function)
            // pass to the new keyframe, so that bundle adjustment will finally decide
            // if they are outliers or not. We don't want next frame to estimate its position
            // with those points so we discard them in the frame.

            int countNulledOutliers = 0;

            for (size_t i = 0; i < mCurrentFrame.mvbOutlier.size(); i++)
            {
                if (mCurrentFrame.mvpMapPoints[i] && mCurrentFrame.mvbOutlier[i]) {
                    mCurrentFrame.mvpMapPoints[i] = NULL;
                    countNulledOutliers++;
                }
            }

            //if(countNulledOutliers>0)
                //cout << "GrabImage - countNulledOutliers: " << countNulledOutliers << endl;

/*#if defined(USE_CRON)
            LOGI("GrabImage - Check position %llu", CRON(now));
#endif*/
        }

        if (bOK)
            mState = WORKING;
        else
            mState = LOST;

        // Reset if the camera get lost soon after initialization
        if (mState == LOST)
        {
            cout << "Lost" << endl;

/*#if defined(USE_CRON)
            LOGI("mState == LOST");
#endif*/
            if (mpMap->KeyFramesInMap() <= 5)
            {
                Reset();
                return;
            }
        }


//        now = NOW();
        // Update motion model
        if (mbMotionModel)
        {
            if (bOK && !mLastFrame.mTcw.empty())
            {
                cv::Mat LastRwc = mLastFrame.mTcw.rowRange(0, 3).colRange(0, 3).t();
                cv::Mat Lasttwc = -LastRwc * mLastFrame.mTcw.rowRange(0, 3).col(3);
                cv::Mat LastTwc = cv::Mat::eye(4, 4, CV_64F);
                LastRwc.copyTo(LastTwc.rowRange(0, 3).colRange(0, 3));
                Lasttwc.copyTo(LastTwc.rowRange(0, 3).col(3));
                mVelocity = mCurrentFrame.mTcw*LastTwc;
            }
            else
                mVelocity = cv::Mat();
        }

        mLastFrame = Frame(mCurrentFrame);
/*#if defined(USE_CRON)
        LOGI("GrabImage - Update motion model %llu", CRON(now));
#endif*/
    }

    // Update drawer
    //mpFramePublisher->Update(this); FABMOD

    //		if (!mCurrentFrame.mTcw.empty()) {
    //			cv::Mat Rwc = mCurrentFrame.mTcw.rowRange(0, 3).colRange(0, 3).t();
    //			cv::Mat twc = -Rwc * mCurrentFrame.mTcw.rowRange(0, 3).col(3);
    //			tf::Matrix3x3 M(Rwc.at<double>(0, 0), Rwc.at<double>(0, 1), Rwc.at<double>(0, 2),
    //					Rwc.at<double>(1, 0), Rwc.at<double>(1, 1), Rwc.at<double>(1, 2),
    //					Rwc.at<double>(2, 0), Rwc.at<double>(2, 1), Rwc.at<double>(2, 2));
    //			tf::Vector3 V(twc.at<double>(0), twc.at<double>(1), twc.at<double>(2));
    //
    //			tf::Transform tfTcw(M, V);
    //
    //			mTfBr.sendTransform(tf::StampedTransform(tfTcw, ros::Time::now(), "ORB_SLAM/World", "ORB_SLAM/Camera"));
    //		}

}

void Tracking::getPosition(PoseType& pose)
{
    boost::mutex::scoped_lock waitLock(*mposeMutex);
    if (mcurrentPose != nullptr)
    {
        cv::Mat R = mcurrentPose->GetRotation().t();
        cv::Mat t = mcurrentPose->GetCameraCenter();
        pose.timestamp = mcurrentPose->mTimeStamp;
        waitLock.unlock();
        vector<double> q = ORB_SLAM::Converter::toQuaternion(R);

        camToWorld = Eigen::Affine3d::Identity();
        camToWorld.prerotate(Eigen::Quaterniond(q[3], q[0], q[1], q[2]));
        camToWorld.pretranslate(Eigen::Vector3d(t.at<double>(0), t.at<double>(1), t.at<double>(2)));

        engToWorld = engToWorldPrev * camToeng * (camToWorldPrev.inverse()) * camToWorld*camToeng;

        Eigen::Vector3d tr = engToWorld.translation();
        Eigen::Quaterniond qd = Eigen::Quaterniond(engToWorld.rotation());

        pose.tx = tr(0);
        pose.ty = tr(1);
        pose.tz = tr(2);
        pose.qx = qd.x();
        pose.qy = qd.y();
        pose.qz = qd.z();
        pose.qw = qd.w();

        engToWorldPrev = engToWorld;
        camToWorldPrev = camToWorld;
    }
}

void Tracking::FirstInitialization()
{
    //We ensure a minimum ORB features to continue, otherwise discard frame
/*#if defined(ANDROID) && defined(DEEP_DEBUG)
    LOGI("mCurrentFrame.mvKeys.size() = %d", mCurrentFrame.mvKeys.size());
#endif*/
    if (mCurrentFrame.mvKeys.size() > MIN_FEATURES_FOR_INITIALIZATION)
    {
        mInitialFrame = Frame(mCurrentFrame);
        mLastFrame = Frame(mCurrentFrame);
        mvbPrevMatched.resize(mCurrentFrame.mvKeysUn.size());
        for (size_t i = 0; i < mCurrentFrame.mvKeysUn.size(); i++)
            mvbPrevMatched[i] = mCurrentFrame.mvKeysUn[i].pt;

        if (mpInitializer)
            delete mpInitializer;

        mpInitializer = new Initializer(mCurrentFrame, 1.0, 200);


        mState = INITIALIZING;
    }
}

void Tracking::Initialize()
{
    // Check if current frame has enough keypoints, otherwise reset initialization process
/*#if defined(ANDROID) && defined(DEEP_DEBUG)
    LOGI("mCurrentFrame.mvKeys.size() = %d", mCurrentFrame.mvKeys.size());
#endif*/
    if (mCurrentFrame.mvKeys.size() <= MIN_FEATURES_FOR_INITIALIZATION)
    {
        fill(mvIniMatches.begin(), mvIniMatches.end(), -1);
        mState = NOT_INITIALIZED;
        return;
    }

    int nmatches = 0;

#ifdef USE_FAST_FEATURES
    ORBmatcher matcher(0.6, false);
#else

    ORBmatcher matcher(0.6, true);
#endif

     nmatches = matcher.SearchForInitialization(mInitialFrame, mCurrentFrame, mvbPrevMatched, mvIniMatches, 100);

    // Find correspondences

/*#if defined(ANDROID) && defined(DEEP_DEBUG)
    LOGI("nmatches = %d", nmatches);
#endif*/
    // Check if there are enough correspondences
    if (nmatches < minFeatures)
    {
        mState = NOT_INITIALIZED;
        return;
    }

    cv::Mat Rcw; // Current Camera Rotation
    cv::Mat tcw; // Current Camera Translation
    vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)

    if (mpInitializer->Initialize(mCurrentFrame, mvIniMatches, Rcw, tcw, mvIniP3D, vbTriangulated))
    {
        for (size_t i = 0, iend = mvIniMatches.size(); i < iend; i++)
        {
            if (mvIniMatches[i] >= 0 && !vbTriangulated[i])
            {
                mvIniMatches[i] = -1;
                nmatches--;
            }
        }

        CreateInitialMap(Rcw, tcw);
    }
    else
    {
/*#ifdef ANDROID
        LOGI("Failed Initialization");
#endif*/
    }

}

void Tracking::CreateInitialMap(cv::Mat &Rcw, cv::Mat &tcw)
{
    // Set Frame Poses
    mInitialFrame.mTcw = cv::Mat::eye(4, 4, CV_64F);
    mCurrentFrame.mTcw = cv::Mat::eye(4, 4, CV_64F);
    Rcw.copyTo(mCurrentFrame.mTcw.rowRange(0, 3).colRange(0, 3));
    tcw.copyTo(mCurrentFrame.mTcw.rowRange(0, 3).col(3));

    // Create KeyFrames
    KeyFrame* pKFini = new KeyFrame(mInitialFrame, mpMap, mpKeyFrameDB);
    KeyFrame* pKFcur = new KeyFrame(mCurrentFrame, mpMap, mpKeyFrameDB);

    pKFini->ComputeBoW();
    pKFcur->ComputeBoW();

    // Insert KFs in the map
    mpMap->AddKeyFrame(pKFini);
    mpMap->AddKeyFrame(pKFcur);

    // Create MapPoints and asscoiate to keyframes
    for (size_t i = 0; i < mvIniMatches.size(); i++)
    {
        if (mvIniMatches[i] < 0)
            continue;

        //Create MapPoint.
        cv::Mat worldPos(mvIniP3D[i]);

        MapPoint* pMP = new MapPoint(worldPos, pKFcur, mpMap);

        pKFini->AddMapPoint(pMP, i);
        pKFcur->AddMapPoint(pMP, mvIniMatches[i]);

        pMP->AddObservation(pKFini, i);
        pMP->AddObservation(pKFcur, mvIniMatches[i]);

        pMP->ComputeDistinctiveDescriptors();
        pMP->UpdateNormalAndDepth();

        //Fill Current Frame structure
        mCurrentFrame.mvpMapPoints[mvIniMatches[i]] = pMP;

        //Add to Map
        mpMap->AddMapPoint(pMP);

    }

    // Update Connections
    pKFini->UpdateConnections();
    pKFcur->UpdateConnections();

    // Bundle Adjustment
    std::cout << "New Map created with " << mpMap->MapPointsInMap() << " points" << std::endl;

    Optimizer::GlobalBundleAdjustemnt(mpMap, 20);

    // Set median depth to 1
    double medianDepth = pKFini->ComputeSceneMedianDepth(2);
    double invMedianDepth = 1.0f / medianDepth;

    if (medianDepth < 0 || pKFcur->TrackedMapPoints() < minFeatures)
    {
        //std::cout << "Wrong initialization, reseting..." << std::endl;
        Reset();
        return;
    }

    // Scale initial baseline
    cv::Mat Tc2w = pKFcur->GetPose();
    Tc2w.col(3).rowRange(0, 3) = Tc2w.col(3).rowRange(0, 3) * invMedianDepth;
    pKFcur->SetPose(Tc2w);

    // Scale points
    vector<MapPoint*> vpAllMapPoints = pKFini->GetMapPointMatches();
    for (size_t iMP = 0; iMP < vpAllMapPoints.size(); iMP++)
    {
        if (vpAllMapPoints[iMP])
        {
            MapPoint* pMP = vpAllMapPoints[iMP];
            pMP->SetWorldPos(pMP->GetWorldPos() * invMedianDepth);
        }
    }

    mpLocalMapper->InsertKeyFrame(pKFini);
    mpLocalMapper->InsertKeyFrame(pKFcur);

    mCurrentFrame.mTcw = pKFcur->GetPose().clone();
    mLastFrame = Frame(mCurrentFrame);
    mnLastKeyFrameId = mCurrentFrame.mnId;
    mpLastKeyFrame = pKFcur;

    mvpLocalKeyFrames.push_back(pKFcur);
    mvpLocalKeyFrames.push_back(pKFini);
    mvpLocalMapPoints = mpMap->GetAllMapPoints();
    mpReferenceKF = pKFcur;

    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

    //    mpMapPublisher->SetCurrentCameraPose(pKFcur->GetPose()); FABMOD

    mState = WORKING;
}

bool Tracking::TrackPreviousFrame()
{
#ifdef USE_FAST_FEATURES
    ORBmatcher matcher(0.9, false);
#else

    ORBmatcher matcher(0.9,true);
#endif
    vector<MapPoint*> vpMapPointMatches;

    // Search first points at coarse scale levels to get a rough initial estimate
    int minOctave = 0;
    int maxOctave = mCurrentFrame.mvScaleFactors.size() - 1;

    if (mpMap->KeyFramesInMap() > 5)
        minOctave = maxOctave / 2 + 1;

    int nmatches = matcher.WindowSearch(mLastFrame, mCurrentFrame, 200, vpMapPointMatches, minOctave);

    // If not enough matches, search again without scale constraint
    if (nmatches < 10)
    {
        nmatches = matcher.WindowSearch(mLastFrame, mCurrentFrame, 100, vpMapPointMatches, 0);
        if (nmatches < 10)
        {
            vpMapPointMatches = vector<MapPoint*>(mCurrentFrame.mvpMapPoints.size(), static_cast<MapPoint*> (NULL));
            nmatches = 0;
            cout << "TrackPreviousFrame - no nmatches"<<endl;
        }
    }

    mLastFrame.mTcw.copyTo(mCurrentFrame.mTcw);
    mCurrentFrame.mvpMapPoints = vpMapPointMatches;

    // If enough correspondeces, optimize pose and project points from previous frame to search more correspondences
    if (nmatches >= 10)
    {
        // Optimize pose with correspondences
        Optimizer::PoseOptimization(&mCurrentFrame);

        int countClearedOutliers = 0;

        for (size_t i = 0; i < mCurrentFrame.mvbOutlier.size(); i++)
            if (mCurrentFrame.mvbOutlier[i])
            {
                mCurrentFrame.mvpMapPoints[i] = NULL;
                mCurrentFrame.mvbOutlier[i] = false;
                nmatches--;
                countClearedOutliers ++;
            }

        //if (countClearedOutliers>0)
        //    cout << "TrackPreviousFrame - countClearedOutliers: " << countClearedOutliers << endl;

        // Search by projection with the estimated pose
        nmatches += matcher.SearchByProjection(mLastFrame, mCurrentFrame, 15, vpMapPointMatches);
    }
    else   //Last opportunity
        nmatches = matcher.SearchByProjection(mLastFrame, mCurrentFrame, 50, vpMapPointMatches);


    mCurrentFrame.mvpMapPoints = vpMapPointMatches;

    if (nmatches < 10)
        return false;

    // Optimize pose again with all correspondences
    Optimizer::PoseOptimization(&mCurrentFrame);

    int countNulledOutliers = 0;

    // Discard outliers
    for (size_t i = 0; i < mCurrentFrame.mvbOutlier.size(); i++)
        if (mCurrentFrame.mvbOutlier[i])
        {
            mCurrentFrame.mvpMapPoints[i] = NULL;
            mCurrentFrame.mvbOutlier[i] = false;
            nmatches--;
            countNulledOutliers++;
        }

   // if(countNulledOutliers>0)
     //   cout << "TrackPreviousFrame - countNulledOutliers: " << countNulledOutliers << endl;

    return nmatches >= 10;
}

bool Tracking::TrackWithMotionModel()
{
#ifdef USE_FAST_FEATURES
    ORBmatcher matcher(0.9, false);
#else

    ORBmatcher matcher(0.9);
#endif

    vector<MapPoint*> vpMapPointMatches;

    // Compute current pose by motion model
    mCurrentFrame.mTcw = mVelocity * mLastFrame.mTcw;

    fill(mCurrentFrame.mvpMapPoints.begin(), mCurrentFrame.mvpMapPoints.end(), static_cast<MapPoint*> (NULL));

    // Project points seen in previous frame
    int nmatches = matcher.SearchByProjection(mCurrentFrame, mLastFrame, 15);

    if (nmatches < 20)
        return false;

    // Optimize pose with all correspondences
    Optimizer::PoseOptimization(&mCurrentFrame);

    int countClearedOutliers = 0;
    // Discard outliers
    for (size_t i = 0; i < mCurrentFrame.mvpMapPoints.size(); i++)
    {
        if (mCurrentFrame.mvpMapPoints[i])
        {
            if (mCurrentFrame.mvbOutlier[i])
            {
                mCurrentFrame.mvpMapPoints[i] = NULL;
                mCurrentFrame.mvbOutlier[i] = false;
                nmatches--;
                countClearedOutliers++;
            }
        }
    }
   // if(countClearedOutliers>0)
   //     cout << "TrackWithMotionModel - countClearedOutliers: " << countClearedOutliers<<endl;

    return nmatches >= 10;
}

int debugCountValidPoints(Frame * frame)
{
    int count = 0;
    for (size_t i = 0; i < frame->mvpMapPoints.size(); i++)
        if (frame->mvpMapPoints[i])
        {
           count ++;
        }

    return count;
}

bool Tracking::TrackLocalMap()
{
    // Tracking from previous frame or relocalisation was succesfull and we have an estimation
    // of the camera pose and some map points tracked in the frame.
    // Update Local Map and Track

    //cout << "TrackLocalMap - map points: " << debugCountValidPoints(&mCurrentFrame) << endl;

    // Update Local Map
    UpdateReference();

    //cout << "TrackLocalMap - post UpdateReference - map points: " <<debugCountValidPoints(&mCurrentFrame) << endl;

    // Search Local MapPoints
    SearchReferencePointsInFrustum();

    //cout << "TrackLocalMap - post SearchReferencePointsInFrustum - map points: " << debugCountValidPoints(&mCurrentFrame) << endl;

    // Optimize Pose
    mnMatchesInliers = Optimizer::PoseOptimization(&mCurrentFrame);

    //cout << "TrackLocalMap - post PoseOptimization - map points: " << debugCountValidPoints(&mCurrentFrame) << endl;

    // Update MapPoints Statistics
    for (size_t i = 0; i < mCurrentFrame.mvpMapPoints.size(); i++)
        if (mCurrentFrame.mvpMapPoints[i])
        {
            if (!mCurrentFrame.mvbOutlier[i])
                mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
        }

    // Decide if the tracking was succesful
    // More restrictive if there was a relocalization recently
    if (mCurrentFrame.mnId < mnLastRelocFrameId + mMaxFrames && mnMatchesInliers < 50) {
        cout << "TrackLocalMap failed with relocalization: " << mnMatchesInliers << endl;
        return false;
    }

    if (mnMatchesInliers < 30) {
        cout << "TrackLocalMap failed: " << mnMatchesInliers << endl;
        return false;
    }
    else
        return true;
}

bool Tracking::NeedNewKeyFrame()
{
    // If Local Mapping is freezed by a Loop Closure do not insert keyframes
    if (mpLocalMapper->isStopped() || mpLocalMapper->stopRequested())
        return false;

    // Not insert keyframes if not enough frames from last relocalisation have passed
    if (mCurrentFrame.mnId < mnLastRelocFrameId + mMaxFrames && mpMap->KeyFramesInMap() > mMaxFrames)
        return false;

    // Reference KeyFrame MapPoints
    int nRefMatches = mpReferenceKF->TrackedMapPoints();

    // Local Mapping accept keyframes?
    bool bLocalMappingIdle = mpLocalMapper->AcceptKeyFrames();

    // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
    const bool c1a = mCurrentFrame.mnId >= mnLastKeyFrameId + mMaxFrames;
    // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
    const bool c1b = mCurrentFrame.mnId >= mnLastKeyFrameId + mMinFrames && bLocalMappingIdle;
    // Condition 2: Less than 90% of points than reference keyframe and enough inliers
    const bool c2 = mnMatchesInliers < nRefMatches * 0.9 && mnMatchesInliers > 15;

    if ((c1a || c1b) && c2)
    {
        // If the mapping accepts keyframes insert, otherwise send a signal to interrupt BA, but not insert yet
        if (bLocalMappingIdle)
        {
            return true;
        }
        else
        {
            mpLocalMapper->InterruptBA();
            return false;
        }
    }
    else
        return false;
}

void Tracking::CreateNewKeyFrame()
{
    KeyFrame* pKF = new KeyFrame(mCurrentFrame, mpMap, mpKeyFrameDB);

    mpLocalMapper->InsertKeyFrame(pKF);

    mnLastKeyFrameId = mCurrentFrame.mnId;
    mpLastKeyFrame = pKF;
}

void Tracking::SearchReferencePointsInFrustum()
{
    // Do not search map points already matched
    for (vector<MapPoint*>::iterator vit = mCurrentFrame.mvpMapPoints.begin(), vend = mCurrentFrame.mvpMapPoints.end(); vit != vend; vit++)
    {
        MapPoint* pMP = *vit;
        if (pMP)
        {
            if (pMP->isBad())
            {
                *vit = NULL;
            }
            else
            {
                pMP->IncreaseVisible();
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                pMP->mbTrackInView = false;
            }
        }
    }

    mCurrentFrame.UpdatePoseMatrices();

    int nToMatch = 0;

    // Project points in frame and check its visibility
    for (vector<MapPoint*>::iterator vit = mvpLocalMapPoints.begin(), vend = mvpLocalMapPoints.end(); vit != vend; vit++)
    {
        MapPoint* pMP = *vit;
        if (pMP->mnLastFrameSeen == mCurrentFrame.mnId)
            continue;
        if (pMP->isBad())
            continue;
        // Project (this fills MapPoint variables for matching)
        if (mCurrentFrame.isInFrustum(pMP, 0.5))
        {
            pMP->IncreaseVisible();
            nToMatch++;
        }
    }


    if (nToMatch > 0)
    {
#ifdef USE_FAST_FEATURES
        ORBmatcher matcher(0.8, false);
#else

        ORBmatcher matcher(0.8);
#endif

        int th = 1;
        // If the camera has been relocalised recently, perform a coarser search
        if (mCurrentFrame.mnId < mnLastRelocFrameId + 2)
            th = 5;
        matcher.SearchByProjection(mCurrentFrame, mvpLocalMapPoints, th);
    }
}

void Tracking::UpdateReference()
{
    // This is for visualization
    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

    // Update
    UpdateReferenceKeyFrames();
    UpdateReferencePoints();
}

void Tracking::UpdateReferencePoints()
{
    mvpLocalMapPoints.clear();

    for (vector<KeyFrame*>::iterator itKF = mvpLocalKeyFrames.begin(), itEndKF = mvpLocalKeyFrames.end(); itKF != itEndKF; itKF++)
    {
        KeyFrame* pKF = *itKF;
        vector<MapPoint*> vpMPs = pKF->GetMapPointMatches();

        for (vector<MapPoint*>::iterator itMP = vpMPs.begin(), itEndMP = vpMPs.end(); itMP != itEndMP; itMP++)
        {
            MapPoint* pMP = *itMP;
            if (!pMP)
                continue;
            if (pMP->mnTrackReferenceForFrame == mCurrentFrame.mnId)
                continue;
            if (!pMP->isBad())
            {
                mvpLocalMapPoints.push_back(pMP);
                pMP->mnTrackReferenceForFrame = mCurrentFrame.mnId;
            }
        }
    }
}

void Tracking::UpdateReferenceKeyFrames()
{
    // Each map point vote for the keyframes in which it has been observed
    map<KeyFrame*, int> keyframeCounter;

    int countNulledPoints = 0;

    for (size_t i = 0, iend = mCurrentFrame.mvpMapPoints.size(); i < iend; i++)
    {
        if (mCurrentFrame.mvpMapPoints[i])
        {
            MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
            if (!pMP->isBad())
            {
                map<KeyFrame*, size_t> observations = pMP->GetObservations();
                for (map<KeyFrame*, size_t>::iterator it = observations.begin(), itend = observations.end(); it != itend; it++)
                    keyframeCounter[it->first]++;
            }
            else
            {
                mCurrentFrame.mvpMapPoints[i] = NULL;
                countNulledPoints++;
            }
        }
    }

    //if(countNulledPoints>0)
        //cout << "UpdateReferenceKeyFrames - countNulledPoints: " << countNulledPoints << endl;


    int max = 0;
    KeyFrame* pKFmax = NULL;

    mvpLocalKeyFrames.clear();
    mvpLocalKeyFrames.reserve(3 * keyframeCounter.size());

    // All keyframes that observe a map point are included in the local map. Also check which keyframe shares most points
    for (map<KeyFrame*, int>::iterator it = keyframeCounter.begin(), itEnd = keyframeCounter.end(); it != itEnd; it++)
    {
        KeyFrame* pKF = it->first;

        if (pKF->isBad())
            continue;

        if (it->second > max)
        {
            max = it->second;
            pKFmax = pKF;
        }

        mvpLocalKeyFrames.push_back(it->first);
        pKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
    }


    // Include also some not-already-included keyframes that are neighbors to already-included keyframes
    for (vector<KeyFrame*>::iterator itKF = mvpLocalKeyFrames.begin(), itEndKF = mvpLocalKeyFrames.end(); itKF != itEndKF; itKF++)
    {
        // Limit the number of keyframes
        if (mvpLocalKeyFrames.size() > 80)
            break;

        KeyFrame* pKF = *itKF;

        vector<KeyFrame*> vNeighs = pKF->GetBestCovisibilityKeyFrames(10);

        for (vector<KeyFrame*>::iterator itNeighKF = vNeighs.begin(), itEndNeighKF = vNeighs.end(); itNeighKF != itEndNeighKF; itNeighKF++)
        {
            KeyFrame* pNeighKF = *itNeighKF;
            if (!pNeighKF->isBad())
            {
                if (pNeighKF->mnTrackReferenceForFrame != mCurrentFrame.mnId)
                {
                    mvpLocalKeyFrames.push_back(pNeighKF);
                    pNeighKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
                    break;
                }
            }
        }

    }

    mpReferenceKF = pKFmax;
}

bool Tracking::Relocalisation()
{
    cout << "Relocalisation" << endl;

    // Compute Bag of Words Vector
    mCurrentFrame.ComputeBoW();

    // Relocalisation is performed when tracking is lost and forced at some stages during loop closing
    // Track Lost: Query KeyFrame Database for keyframe candidates for relocalisation
    vector<KeyFrame*> vpCandidateKFs;
    if (!RelocalisationRequested())
        vpCandidateKFs = mpKeyFrameDB->DetectRelocalisationCandidates(&mCurrentFrame);
    else // Forced Relocalisation: Relocate against local window around last keyframe
    {
        boost::mutex::scoped_lock lock(mMutexForceRelocalisation);
        mbForceRelocalisation = false;
        vpCandidateKFs.reserve(10);
        vpCandidateKFs = mpLastKeyFrame->GetBestCovisibilityKeyFrames(9);
        vpCandidateKFs.push_back(mpLastKeyFrame);
    }

    if (vpCandidateKFs.empty())
        return false;

    const int nKFs = vpCandidateKFs.size();

    // We perform first an ORB matching with each candidate
    // If enough matches are found we setup a PnP solver

#ifdef USE_FAST_FEATURES
    ORBmatcher matcher(0.75, false);
#else

    ORBmatcher matcher(0.75, true);
#endif

    vector<PnPsolver*> vpPnPsolvers;
    vpPnPsolvers.resize(nKFs);

    vector<vector<MapPoint*> > vvpMapPointMatches;
    vvpMapPointMatches.resize(nKFs);

    vector<bool> vbDiscarded;
    vbDiscarded.resize(nKFs);

    int nCandidates = 0;

    for (size_t i = 0; i < vpCandidateKFs.size(); i++)
    {
        KeyFrame* pKF = vpCandidateKFs[i];
        if (pKF->isBad())
            vbDiscarded[i] = true;
        else
        {
            int nmatches = matcher.SearchByBoW(pKF, mCurrentFrame, vvpMapPointMatches[i]);
            if (nmatches < 15)
            {
                vbDiscarded[i] = true;
                continue;
            }
            else
            {
                PnPsolver* pSolver = new PnPsolver(mCurrentFrame, vvpMapPointMatches[i]);
                pSolver->SetRansacParameters(0.99, 10, 300, 4, 0.5, 5.991);
                vpPnPsolvers[i] = pSolver;
                nCandidates++;
            }
        }
    }

    // Alternatively perform some iterations of P4P RANSAC
    // Until we found a camera pose supported by enough inliers
    bool bMatch = false;
#ifdef USE_FAST_FEATURES
    ORBmatcher matcher2(0.9, false);
#else

    ORBmatcher matcher2(0.9, true);
#endif


    while (nCandidates > 0 && !bMatch)
    {
        for (size_t i = 0; i < vpCandidateKFs.size(); i++)
        {
            if (vbDiscarded[i])
                continue;

            // Perform 5 Ransac Iterations
            vector<bool> vbInliers;
            int nInliers;
            bool bNoMore;

            PnPsolver* pSolver = vpPnPsolvers[i];
            cv::Mat Tcw = pSolver->iterate(5, bNoMore, vbInliers, nInliers);

            // If Ransac reachs max. iterations discard keyframe
            if (bNoMore)
            {
                vbDiscarded[i] = true;
                nCandidates--;
            }

            // If a Camera Pose is computed, optimize
            if (!Tcw.empty())
            {
                Tcw.copyTo(mCurrentFrame.mTcw);

                set<MapPoint*> sFound;

                int countNulledPoints = 0;

                for (size_t j = 0; j < vbInliers.size(); j++)
                {
                    if (vbInliers[j])
                    {
                        mCurrentFrame.mvpMapPoints[j] = vvpMapPointMatches[i][j];
                        sFound.insert(vvpMapPointMatches[i][j]);
                    }
                    else {
                        mCurrentFrame.mvpMapPoints[j] = NULL;
                        countNulledPoints++;
                    }
                }
                //if(countNulledPoints>0)
                //    cout << "Relocalization - countNulledPoints: " << countNulledPoints << endl;

                int nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                if (nGood < 10)
                    continue;

                countNulledPoints = 0;
                for (size_t io = 0, ioend = mCurrentFrame.mvbOutlier.size(); io < ioend; io++)
                    if (mCurrentFrame.mvbOutlier[io]) {
                        mCurrentFrame.mvpMapPoints[io] = NULL;
                    }
               // if(countNulledPoints>0)
                    //cout << "Relocalization 2 - countNulledPoints: " << countNulledPoints << endl;

                // If few inliers, search by projection in a coarse window and optimize again
                if (nGood < 50)
                {
                    int nadditional = matcher2.SearchByProjection(mCurrentFrame, vpCandidateKFs[i], sFound, 10, 100);

                    if (nadditional + nGood >= 50)
                    {
                        nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                        // If many inliers but still not enough, search by projection again in a narrower window
                        // the camera has been already optimized with many points
                        if (nGood > 30 && nGood < 50)
                        {
                            sFound.clear();
                            for (size_t ip = 0, ipend = mCurrentFrame.mvpMapPoints.size(); ip < ipend; ip++)
                                if (mCurrentFrame.mvpMapPoints[ip])
                                    sFound.insert(mCurrentFrame.mvpMapPoints[ip]);
                            nadditional = matcher2.SearchByProjection(mCurrentFrame, vpCandidateKFs[i], sFound, 3, 64);

                            // Final optimization
                            if (nGood + nadditional >= 50)
                            {
                                nGood = Optimizer::PoseOptimization(&mCurrentFrame);
                                countNulledPoints=0;
                                for (size_t io = 0; io < mCurrentFrame.mvbOutlier.size(); io++)
                                    if (mCurrentFrame.mvbOutlier[io]) {
                                        mCurrentFrame.mvpMapPoints[io] = NULL;
                                        countNulledPoints++;
                                    }

                                //if(countNulledPoints>0)
                                    //cout << "Relocalization 3 - countNulledPoints: " << countNulledPoints << endl;
                            }
                        }
                    }
                }


                // If the pose is supported by enough inliers stop ransacs and continue
                if (nGood >= 50)
                {
                    bMatch = true;
                    break;
                }
            }
        }
    }

    if (!bMatch)
    {
        return false;
    }
    else
    {
        mnLastRelocFrameId = mCurrentFrame.mnId;
        return true;
    }

}

void Tracking::ForceRelocalisation()
{
    boost::mutex::scoped_lock lock(mMutexForceRelocalisation);
    mbForceRelocalisation = true;
    mnLastRelocFrameId = mCurrentFrame.mnId;
}

bool Tracking::RelocalisationRequested()
{
    boost::mutex::scoped_lock lock(mMutexForceRelocalisation);
    return mbForceRelocalisation;
}

void Tracking::Reset()
{
    {
        boost::mutex::scoped_lock lock(mMutexReset);
        //mbPublisherStopped = false;
        mbReseting = true;
    }

    // Wait until publishers are stopped
//    while (1)
//    {
//        {
//            boost::mutex::scoped_lock lock(mMutexReset);
//            if (mbPublisherStopped)
//                break;
//        }
//        usleep(2000);
//    }

    // Reset Local Mapping
    mpLocalMapper->RequestReset();
    // Reset Loop Closing
    mpLoopClosing->RequestReset();
    // Clear BoW Database
    mpKeyFrameDB->clear();
    // Clear Map (this erase MapPoints and KeyFrames)
    mpMap->clear();

    KeyFrame::nNextId = 0;
    Frame::nNextId = 0;
    mState = NOT_INITIALIZED;

    {
        boost::mutex::scoped_lock lock(mMutexReset);
        mbReseting = false;
    }
}

void Tracking::CheckResetByPublishers()
{
return;

    bool bReseting = false;

    {
        boost::mutex::scoped_lock lock(mMutexReset);
        bReseting = mbReseting;
    }

    if (bReseting)
    {
        boost::mutex::scoped_lock lock(mMutexReset);
        mbPublisherStopped = true;
    }

    // Hold until reset is finished
    while (1)
    {
        {
            boost::mutex::scoped_lock lock(mMutexReset);
            if (!mbReseting)
            {
                mbPublisherStopped = false;
                break;
            }
        }
        usleep(2000);
    }
}

Map* Tracking::getMap(){
return mpMap;
}

} //namespace ORB_SLAM
