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

#ifndef TRACKING_H
#define TRACKING_H


#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>
//#include<sensor_msgs/Image.h>
//#include<sensor_msgs/image_encodings.h>

//#include"FramePublisher.h"
#include"Map.h"
#include"LocalMapping.h"
#include"LoopClosing.h"
#include"Frame.h"
#include "ORBVocabulary.h"
#include"KeyFrameDatabase.h"
#include"ORBextractor.h"
#include "Initializer.h"
#include <boost/thread/mutex.hpp>
//#include "MapPublisher.h"
#include <deque>

#include <eigen3/Eigen/Geometry>

//#include<tf/transform_broadcaster.h>


namespace ORB_SLAM {

	//class FramePublisher;
	class Map;
	class LocalMapping;
	class LoopClosing;

	struct TimestampedMat {

		TimestampedMat(const cv::Mat& _im, double _time) : im(_im), timestamp(_time) {
		};
		cv::Mat im;
		double timestamp;
	};

	struct PoseType {
		double tx, ty, tz, qx, qy, qz, qw;
		double timestamp;
	};

	class Tracking {
	public:
		Tracking(ORBVocabulary* pVoc, /*FramePublisher* pFramePublisher, MapPublisher* pMapPublisher,*/ Map* pMap, string strSettingPath, double _fx, double _fy, double _cx, double _cy);

		enum eTrackingState {
			SYSTEM_NOT_READY = -1,
			NO_IMAGES_YET = 0,
			NOT_INITIALIZED = 1,
			INITIALIZING = 2,
			WORKING = 3,
			LOST = 4
		};

		void SetLocalMapper(LocalMapping* pLocalMapper);
		void SetLoopClosing(LoopClosing* pLoopClosing);
		void SetKeyFrameDatabase(KeyFrameDatabase* pKFDB);

		// This is the main function of the Tracking Thread
		void Run();

		void ForceRelocalisation();

		eTrackingState mState;
		eTrackingState mLastProcessedState;

		// Current Frame
		Frame mCurrentFrame;

		// Initialization Variables
		std::vector<int> mvIniLastMatches;
		std::vector<int> mvIniMatches;
		std::vector<cv::Point2d> mvbPrevMatched;
		std::vector<cv::Point3d> mvIniP3D;
		Frame mInitialFrame;


		void CheckResetByPublishers();

		bool addFrame(const cv::Mat& im, double timestamp);

		void getPosition(PoseType& pose);

		Map* getMap();

	protected:
		void GrabImage(const TimestampedMat& frame);

		void FirstInitialization();
		void Initialize();
		void CreateInitialMap(cv::Mat &Rcw, cv::Mat &tcw);

		void Reset();

		bool TrackPreviousFrame();
		bool TrackWithMotionModel();

		bool RelocalisationRequested();
		bool Relocalisation();

		void UpdateReference();
		void UpdateReferencePoints();
		void UpdateReferenceKeyFrames();

		bool TrackLocalMap();
		void SearchReferencePointsInFrustum();

		bool NeedNewKeyFrame();
		void CreateNewKeyFrame();


		//Other Thread Pointers
		LocalMapping* mpLocalMapper;
		LoopClosing* mpLoopClosing;

		//ORB
		ORBextractor* mpORBextractor;
		ORBextractor* mpIniORBextractor;

		// FAST + Brief

		//int FASTThreshold = fSettings["FAST.FASTThreshold"];
		cv::FastFeatureDetector *mFASTDetector;
		cv::BriefDescriptorExtractor* mBRIEFExtractor;

		//cv::ORB* mcvORB;
		//cv::ORB* mcvIniORB;

		//BoW
		ORBVocabulary* mpORBVocabulary;
		KeyFrameDatabase* mpKeyFrameDB;

		// Initalization
		Initializer* mpInitializer;

		//Local Map
		KeyFrame* mpReferenceKF;
		std::vector<KeyFrame*> mvpLocalKeyFrames;
		std::vector<MapPoint*> mvpLocalMapPoints;

		//Publishers
		//    FramePublisher* mpFramePublisher;
		//    MapPublisher* mpMapPublisher;

		//Map
		Map* mpMap;

		//Calibration matrix
		cv::Mat mK;
		cv::Mat mDistCoef;

		//New KeyFrame rules (according to fps)
		int mMinFrames;
		int mMaxFrames;

		int nLevels;
		double fScaleFactor;

		int minFeatures = 20;
		int minFeat = minFeatures;

		//Current matches in frame
		int mnMatchesInliers;

		//Last Frame, KeyFrame and Relocalisation Info
		KeyFrame* mpLastKeyFrame;
		Frame mLastFrame;
		unsigned int mnLastKeyFrameId;
		unsigned int mnLastRelocFrameId;

		//Mutex
		boost::mutex mMutexTrack;
		boost::mutex mMutexForceRelocalisation;

		//Reset
		bool mbPublisherStopped;
		bool mbReseting;
		boost::mutex mMutexReset;

		//Is relocalisation requested by an external thread? (loop closing)
		bool mbForceRelocalisation;

		//Motion Model
		bool mbMotionModel;
		cv::Mat mVelocity;

		//Color order (true RGB, false BGR, ignored if grayscale)
		bool mbRGB;

		// Transfor broadcaster (for visualization in rviz)
		//		tf::TransformBroadcaster mTfBr;

		std::deque<TimestampedMat>* mqueue;
		boost::mutex* mbufferMutex;
		boost::mutex* mposeMutex;
		KeyFrame* mcurrentPose;
		int bufferSize;

		Eigen::Affine3d camToWorld;
		Eigen::Affine3d camToWorldPrev;
		Eigen::Affine3d engToWorld;
		Eigen::Affine3d engToWorldPrev;
		Eigen::Affine3d camToeng;
	};

} //namespace ORB_SLAM

#endif // TRACKING_H
