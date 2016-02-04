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

#include "Frame.h"
#include "Converter.h"

#include <chrono>

//#define NOW() std::chrono::high_resolution_clock::now()
//#define CRON(timeBegin) std::chrono::duration_cast<std::chrono::milliseconds>(NOW() - timeBegin).count()

/*#ifdef ANDROID
#include <android/log.h>
#define LOGI(...) ((void)__android_log_print(ANDROID_LOG_INFO, "ORB_SLAM", __VA_ARGS__))
#else
#define LOGI(...) printf(__VA_ARGS__)
#endif*/

//#define USE_CRON

namespace ORB_SLAM {
    long unsigned int Frame::nNextId = 0;
    bool Frame::mbInitialComputations = true;
    double Frame::cx, Frame::cy, Frame::fx, Frame::fy;
    int Frame::mnMinX, Frame::mnMinY, Frame::mnMaxX, Frame::mnMaxY;

    Frame::Frame() { }

//Copy Constructor
    Frame::Frame(const Frame &frame) // , mcvORB(frame.mcvORB)
            : mpORBvocabulary(frame.mpORBvocabulary), mpORBextractor(frame.mpORBextractor), im(frame.im.clone()),
              mTimeStamp(frame.mTimeStamp),
              mK(frame.mK.clone()), mDistCoef(frame.mDistCoef.clone()), N(frame.N), mvKeys(frame.mvKeys),
              mvKeysUn(frame.mvKeysUn),
              mBowVec(frame.mBowVec), mFeatVec(frame.mFeatVec), mDescriptors(frame.mDescriptors.clone()),
              mvpMapPoints(frame.mvpMapPoints), mvbOutlier(frame.mvbOutlier),
              mfGridElementWidthInv(frame.mfGridElementWidthInv), mfGridElementHeightInv(frame.mfGridElementHeightInv),
              mnId(frame.mnId),
              mpReferenceKF(frame.mpReferenceKF), mnScaleLevels(frame.mnScaleLevels),
              mfScaleFactor(frame.mfScaleFactor),
              mvScaleFactors(frame.mvScaleFactors), mvLevelSigma2(frame.mvLevelSigma2),
              mvInvLevelSigma2(frame.mvInvLevelSigma2) {
        for (int i = 0; i < FRAME_GRID_COLS; i++)
            for (int j = 0; j < FRAME_GRID_ROWS; j++)
                mGrid[i][j] = frame.mGrid[i][j];

        if (!frame.mTcw.empty())
            mTcw = frame.mTcw.clone();
    }

/*Frame::Frame(cv::Mat &im_, const double &timeStamp, cv::ORB* extractor, double scaleFactor, int nlevels, ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef)
    :mpORBvocabulary(voc), mcvORB(extractor), mnScaleLevels(nlevels), mfScaleFactor(scaleFactor),
		im(im_),mTimeStamp(timeStamp), mK(K.clone()),mDistCoef(distCoef.clone())
{


//    auto orbExtractionTime = NOW();


//	Timer tm;
    // Exctract ORB
	(*mcvORB)(im,cv::Mat(),mvKeys,mDescriptors);

#if defined(USE_CRON)
    LOGI("ORB - Extraction: %llu", CRON(orbExtractionTime));
#endif
//	int tmexp = tm.getTime().milli;
//	std::cout << "Extraction time =\t" << tmexp << std::endl;

    N = mvKeys.size();


	//std::cout << "N=\t\t\t" << N << std::endl;

    if(mvKeys.empty())
        return;

    mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(NULL));

    UndistortKeyPoints();


    // This is done for the first created Frame
    if(mbInitialComputations)
    {
        ComputeImageBounds();

        mfGridElementWidthInv=static_cast<double>(FRAME_GRID_COLS)/static_cast<double>(mnMaxX-mnMinX);
        mfGridElementHeightInv=static_cast<double>(FRAME_GRID_ROWS)/static_cast<double>(mnMaxY-mnMinY);

        fx = K.at<double>(0,0);
        fy = K.at<double>(1,1);
        cx = K.at<double>(0,2);
        cy = K.at<double>(1,2);

        mbInitialComputations=false;
    }


    mnId=nNextId++;

    //Scale Levels Info
//    mnScaleLevels = mcvORB.GetLevels();
//    mfScaleFactor = mpORBextractor->GetScaleFactor();

    mvScaleFactors.resize(mnScaleLevels);
    mvLevelSigma2.resize(mnScaleLevels);
    mvScaleFactors[0]=1.0f;
    mvLevelSigma2[0]=1.0f;
    for(int i=1; i<mnScaleLevels; i++)
    {
        mvScaleFactors[i]=mvScaleFactors[i-1]*mfScaleFactor;
        mvLevelSigma2[i]=mvScaleFactors[i]*mvScaleFactors[i];
    }

    mvInvLevelSigma2.resize(mvLevelSigma2.size());
    for(int i=0; i<mnScaleLevels; i++)
        mvInvLevelSigma2[i]=1/mvLevelSigma2[i];

    // Assign Features to Grid Cells
    int nReserve = 0.5*N/(FRAME_GRID_COLS*FRAME_GRID_ROWS);
    for(unsigned int i=0; i<FRAME_GRID_COLS;i++)
        for (unsigned int j=0; j<FRAME_GRID_ROWS;j++)
            mGrid[i][j].reserve(nReserve);


    for(size_t i=0;i<mvKeysUn.size();i++)
    {
        cv::KeyPoint &kp = mvKeysUn[i];

        int nGridPosX, nGridPosY;
        if(PosInGrid(kp,nGridPosX,nGridPosY))
            mGrid[nGridPosX][nGridPosY].push_back(i);
    }


    mvbOutlier = vector<bool>(N,false);

}*/

    Frame::Frame(cv::Mat &im_, const double &timeStamp, ORBextractor *extractor, ORBVocabulary *voc, cv::Mat &K,
                 cv::Mat &distCoef)
            : mpORBvocabulary(voc), mpORBextractor(extractor), im(im_), mTimeStamp(timeStamp), mK(K.clone()),
              mDistCoef(distCoef.clone()) {
//        auto orbExtractionTime = NOW();


        // Exctract ORB
        (*mpORBextractor)(im, cv::Mat(), mvKeys, mDescriptors);

/*
#if defined(USE_CRON)
        LOGI("ORB - Extraction: %llu", CRON(orbExtractionTime));
#endif*/

        N = mvKeys.size();

        if (mvKeys.empty())
            return;

        mvpMapPoints = vector<MapPoint *>(N, static_cast<MapPoint *>(NULL));

        UndistortKeyPoints();


        // This is done for the first created Frame
        if (mbInitialComputations) {
//            orbExtractionTime = NOW();

            ComputeImageBounds();

            mfGridElementWidthInv = static_cast<double>(FRAME_GRID_COLS) / static_cast<double>(mnMaxX - mnMinX);
            mfGridElementHeightInv = static_cast<double>(FRAME_GRID_ROWS) / static_cast<double>(mnMaxY - mnMinY);

            fx = K.at<double>(0, 0);
            fy = K.at<double>(1, 1);
            cx = K.at<double>(0, 2);
            cy = K.at<double>(1, 2);

            mbInitialComputations = false;
/*#if defined(USE_CRON)
            //LOGI("ORB - ComputeImageBounds: %llu", CRON(orbExtractionTime));
#endif*/
        }

//        orbExtractionTime = NOW();

        mnId = nNextId++;

        //Scale Levels Info
        mnScaleLevels = mpORBextractor->GetLevels();
        mfScaleFactor = mpORBextractor->GetScaleFactor();

        mvScaleFactors.resize(mnScaleLevels);
        mvLevelSigma2.resize(mnScaleLevels);
        mvScaleFactors[0] = 1.0f;
        mvLevelSigma2[0] = 1.0f;
        for (int i = 1; i < mnScaleLevels; i++) {
            mvScaleFactors[i] = mvScaleFactors[i - 1] * mfScaleFactor;
            mvLevelSigma2[i] = mvScaleFactors[i] * mvScaleFactors[i];
        }

        mvInvLevelSigma2.resize(mvLevelSigma2.size());
        for (int i = 0; i < mnScaleLevels; i++)
            mvInvLevelSigma2[i] = 1.f / mvLevelSigma2[i];

        // Assign Features to Grid Cells
        int nReserve = 0.5 * N / (FRAME_GRID_COLS * FRAME_GRID_ROWS);
        for (unsigned int i = 0; i < FRAME_GRID_COLS; i++)
            for (unsigned int j = 0; j < FRAME_GRID_ROWS; j++)
                mGrid[i][j].reserve(nReserve);


        for (size_t i = 0; i < mvKeysUn.size(); i++) {
            cv::KeyPoint &kp = mvKeysUn[i];

            int nGridPosX, nGridPosY;
            if (PosInGrid(kp, nGridPosX, nGridPosY))
                mGrid[nGridPosX][nGridPosY].push_back(i);
        }


        mvbOutlier = vector<bool>(N, false);

/*#if defined(USE_CRON)
        //LOGI("ORB - Final processing: %llu", CRON(orbExtractionTime));
#endif*/
    }

    Frame::Frame(cv::Mat &im_, const double &timeStamp, cv::FastFeatureDetector* detector, cv::BriefDescriptorExtractor *extractor, ORBVocabulary *voc,
                 cv::Mat &K, cv::Mat &distCoef)
            : mpORBvocabulary(voc), mFASTDetector(detector), mBRIEFExtractor(extractor), im(im_), mTimeStamp(timeStamp), mK(K.clone()),
              mDistCoef(distCoef.clone()) {
//        auto fastExtractionTime = NOW();

        mFASTDetector->detect(im, mvKeys);

        //cv::FAST(im, mvKeys, 9);

        mBRIEFExtractor->compute(im, mvKeys, mDescriptors);


/*#if defined(USE_CRON)
        LOGI("FAST - Extraction: %llu", CRON(fastExtractionTime));
#endif*/

        N = mvKeys.size();

        if (mvKeys.empty()) {
            cout << "No keys" << endl;
            return;
        }
        mvpMapPoints = vector<MapPoint *>(N, static_cast<MapPoint *>(NULL));

        UndistortKeyPoints();


        // This is done for the first created Frame
        if (mbInitialComputations) {
//            fastExtractionTime = NOW();

            ComputeImageBounds();

            mfGridElementWidthInv = static_cast<double>(FRAME_GRID_COLS) / static_cast<double>(mnMaxX - mnMinX);
            mfGridElementHeightInv = static_cast<double>(FRAME_GRID_ROWS) / static_cast<double>(mnMaxY - mnMinY);

            fx = K.at<double>(0, 0);
            fy = K.at<double>(1, 1);
            cx = K.at<double>(0, 2);
            cy = K.at<double>(1, 2);

            mbInitialComputations = false;
/*#if defined(USE_CRON)
            //LOGI("ORB - ComputeImageBounds: %llu", CRON(orbExtractionTime));
#endif*/
        }

//        fastExtractionTime = NOW();

        mnId = nNextId++;

        //Scale Levels Info
        mnScaleLevels = 1; // mpORBextractor->GetLevels();
        mfScaleFactor = 1; // mpORBextractor->GetScaleFactor();

        mvScaleFactors.resize(mnScaleLevels);
        mvLevelSigma2.resize(mnScaleLevels);
        mvScaleFactors[0] = 1.0f;
        mvLevelSigma2[0] = 1.0f;
        /*for(int i=1; i<mnScaleLevels; i++)
        {
            mvScaleFactors[i]=mvScaleFactors[i-1]*mfScaleFactor;
            mvLevelSigma2[i]=mvScaleFactors[i]*mvScaleFactors[i];
        }*/

        mvInvLevelSigma2.resize(mvLevelSigma2.size());
        for (int i = 0; i < mnScaleLevels; i++)
            mvInvLevelSigma2[i] = 1.f / mvLevelSigma2[i];

        // Assign Features to Grid Cells
        int nReserve = 0.5 * N / (FRAME_GRID_COLS * FRAME_GRID_ROWS);
        for (unsigned int i = 0; i < FRAME_GRID_COLS; i++)
            for (unsigned int j = 0; j < FRAME_GRID_ROWS; j++)
                mGrid[i][j].reserve(nReserve);


        for (size_t i = 0; i < mvKeysUn.size(); i++) {
            cv::KeyPoint &kp = mvKeysUn[i];

            int nGridPosX, nGridPosY;
            if (PosInGrid(kp, nGridPosX, nGridPosY))
                mGrid[nGridPosX][nGridPosY].push_back(i);
        }


        mvbOutlier = vector<bool>(N, false);

/*#if defined(USE_CRON)
        //LOGI("ORB - Final processing: %llu", CRON(orbExtractionTime));
#endif*/
    }

    void Frame::UpdatePoseMatrices() {
        mRcw = mTcw.rowRange(0, 3).colRange(0, 3);
        mtcw = mTcw.rowRange(0, 3).col(3);
        mOw = -mRcw.t() * mtcw;
    }

    bool Frame::isInFrustum(MapPoint *pMP, double viewingCosLimit) {
        pMP->mbTrackInView = false;

        // 3D in absolute coordinates
        cv::Mat P = pMP->GetWorldPos();

        // 3D in camera coordinates
        const cv::Mat Pc = mRcw * P + mtcw;
        const double PcX = Pc.at<double>(0);
        const double PcY = Pc.at<double>(1);
        const double PcZ = Pc.at<double>(2);

        // Check positive depth
        if (PcZ < 0.0)
            return false;

        // Project in image and check it is not outside
        const double invz = 1.0 / PcZ;
        const double u = fx * PcX * invz + cx;
        const double v = fy * PcY * invz + cy;

        if (u < mnMinX || u > mnMaxX)
            return false;
        if (v < mnMinY || v > mnMaxY)
            return false;

        // Check distance is in the scale invariance region of the MapPoint
        const double maxDistance = pMP->GetMaxDistanceInvariance();
        const double minDistance = pMP->GetMinDistanceInvariance();
        const cv::Mat PO = P - mOw;
        const double dist = cv::norm(PO);

        if (dist < minDistance || dist > maxDistance)
            return false;

        // Check viewing angle
        cv::Mat Pn = pMP->GetNormal();

        double viewCos = PO.dot(Pn) / dist;

        if (viewCos < viewingCosLimit)
            return false;

        int nPredictedLevel = 0;
#ifdef USE_FAST_FEATURES

#else
        double ratio = dist/minDistance;

        vector<double>::iterator it = lower_bound(mvScaleFactors.begin(), mvScaleFactors.end(), ratio);
        nPredictedLevel = it-mvScaleFactors.begin();

        if(nPredictedLevel>=mnScaleLevels)
            nPredictedLevel=mnScaleLevels-1;

#endif
        // Predict scale level acording to the distance


        // Data used by the tracking
        pMP->mbTrackInView = true;
        pMP->mTrackProjX = u;
        pMP->mTrackProjY = v;
        pMP->mnTrackScaleLevel = nPredictedLevel;
        pMP->mTrackViewCos = viewCos;

        return true;
    }

    vector<size_t> Frame::GetFeaturesInArea(const double &x, const double &y, const double &r, int minLevel,
                                            int maxLevel) const {
        vector<size_t> vIndices;
        vIndices.reserve(mvKeysUn.size());

        int nMinCellX = floor((x - mnMinX - r) * mfGridElementWidthInv);
        nMinCellX = max(0, nMinCellX);
        if (nMinCellX >= FRAME_GRID_COLS)
            return vIndices;

        int nMaxCellX = ceil((x - mnMinX + r) * mfGridElementWidthInv);
        nMaxCellX = min(FRAME_GRID_COLS - 1, nMaxCellX);
        if (nMaxCellX < 0)
            return vIndices;

        int nMinCellY = floor((y - mnMinY - r) * mfGridElementHeightInv);
        nMinCellY = max(0, nMinCellY);
        if (nMinCellY >= FRAME_GRID_ROWS)
            return vIndices;

        int nMaxCellY = ceil((y - mnMinY + r) * mfGridElementHeightInv);
        nMaxCellY = min(FRAME_GRID_ROWS - 1, nMaxCellY);
        if (nMaxCellY < 0)
            return vIndices;

        bool bCheckLevels = true;
        bool bSameLevel = false;
        if (minLevel == -1 && maxLevel == -1)
            bCheckLevels = false;
        else if (minLevel == maxLevel)
            bSameLevel = true;

        for (int ix = nMinCellX; ix <= nMaxCellX; ix++) {
            for (int iy = nMinCellY; iy <= nMaxCellY; iy++) {
                vector<size_t> vCell = mGrid[ix][iy];
                if (vCell.empty())
                    continue;

                for (size_t j = 0, jend = vCell.size(); j < jend; j++) {
                    const cv::KeyPoint &kpUn = mvKeysUn[vCell[j]];
                    if (bCheckLevels && !bSameLevel) {
                        if (kpUn.octave < minLevel || kpUn.octave > maxLevel) {
                            continue;
                        }
                    }
                    else if (bSameLevel) {
                        if (kpUn.octave != minLevel) {
                            //cout << "Discarded because of octave: " << kpUn.octave << endl;
                            continue;
                        }
                    }

                    if (abs(kpUn.pt.x - x) > r || abs(kpUn.pt.y - y) > r)
                        continue;

                    vIndices.push_back(vCell[j]);
                }
            }
        }

        return vIndices;

    }

    bool Frame::PosInGrid(cv::KeyPoint &kp, int &posX, int &posY) {
        posX = round((kp.pt.x - mnMinX) * mfGridElementWidthInv);
        posY = round((kp.pt.y - mnMinY) * mfGridElementHeightInv);

        //Keypoint's coordinates are undistorted, which could cause to go out of the image
        if (posX < 0 || posX >= FRAME_GRID_COLS || posY < 0 || posY >= FRAME_GRID_ROWS)
            return false;

        return true;
    }


    void Frame::ComputeBoW() {
        if (mBowVec.empty()) {
            vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(mDescriptors);
            mpORBvocabulary->transform(vCurrentDesc, mBowVec, mFeatVec, 4);
        }
    }

    void Frame::UndistortKeyPoints() {
// Disable undistortion
        mvKeysUn = mvKeys;
        return;

//    if(mDistCoef.at<double>(0)==0.0)
//    {
//        mvKeysUn=mvKeys;
//        return;
//    }
//
//    // Fill matrix with points
//    cv::Mat mat(mvKeys.size(),2,CV_64F);
//    for(unsigned int i=0; i<mvKeys.size(); i++)
//    {
//        mat.at<double>(i,0)=mvKeys[i].pt.x;
//        mat.at<double>(i,1)=mvKeys[i].pt.y;
//    }
//
//    // Undistort points
//    mat=mat.reshape(2);
//    cv::undistortPoints(mat,mat,mK,mDistCoef,cv::Mat(),mK);
//    mat=mat.reshape(1);
//
//    // Fill undistorted keypoint vector
//    mvKeysUn.resize(mvKeys.size());
//    for(unsigned int i=0; i<mvKeys.size(); i++)
//    {
//        cv::KeyPoint kp = mvKeys[i];
//        kp.pt.x=mat.at<double>(i,0);
//        kp.pt.y=mat.at<double>(i,1);
//        mvKeysUn[i]=kp;
//    }
    }

    void Frame::ComputeImageBounds() {
        mnMinX = 0;
        mnMaxX = im.cols;
        mnMinY = 0;
        mnMaxY = im.rows;

//    if(mDistCoef.at<double>(0)!=0.0)
//    {
//        cv::Mat mat(4,2,CV_64F);
//        mat.at<double>(0,0)=0.0;
//        mat.at<double>(0,1)=0.0;
//        mat.at<double>(1,0)=im.cols;
//        mat.at<double>(1,1)=0.0;
//        mat.at<double>(2,0)=0.0;
//        mat.at<double>(2,1)=im.rows;
//        mat.at<double>(3,0)=im.cols;
//        mat.at<double>(3,1)=im.rows;
//
//        // Undistort corners
//        mat=mat.reshape(2);
//        cv::undistortPoints(mat,mat,mK,mDistCoef,cv::Mat(),mK);
//        mat=mat.reshape(1);
//
//        mnMinX = min(floor(mat.at<double>(0,0)),floor(mat.at<double>(2,0)));
//        mnMaxX = max(ceil(mat.at<double>(1,0)),ceil(mat.at<double>(3,0)));
//        mnMinY = min(floor(mat.at<double>(0,1)),floor(mat.at<double>(1,1)));
//        mnMaxY = max(ceil(mat.at<double>(2,1)),ceil(mat.at<double>(3,1)));
//
//    }
//    else
//    {
//        mnMinX = 0;
//        mnMaxX = im.cols;
//        mnMinY = 0;
//        mnMaxY = im.rows;
//    }
    }

} //namespace ORB_SLAM
