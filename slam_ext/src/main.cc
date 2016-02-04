///**
// * This file is part of ORB-SLAM.
// *
// * Copyright (C) 2014 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
// * For more information see <http://webdiis.unizar.es/~raulmur/orbslam/>
// *
// * ORB-SLAM is free software: you can redistribute it and/or modify
// * it under the terms of the GNU General Public License as published by
// * the Free Software Foundation, either version 3 of the License, or
// * (at your option) any later version.
// *
// * ORB-SLAM is distributed in the hope that it will be useful,
// * but WITHOUT ANY WARRANTY; without even the implied warranty of
// * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// * GNU General Public License for more details.
// *
// * You should have received a copy of the GNU General Public License
// * along with ORB-SLAM. If not, see <http://www.gnu.org/licenses/>.
// */
//
//#include<iostream>
//#include<fstream>
////#include<ros/ros.h>
////#include<ros/package.h>
//#include<boost/thread.hpp>
//
//#include<opencv2/core/core.hpp>
//
//#include <chrono>
//
//#include "Tracking.h"
////#include "FramePublisher.h"
//#include "Map.h"
////#include "MapPublisher.h"
//#include "LocalMapping.h"
//#include "LoopClosing.h"
//#include "KeyFrameDatabase.h"
//#include "ORBVocabulary.h"
//
//
//#include "Converter.h"
//
//
//using namespace std;
//
//int main(int argc, char **argv) {
////	ros::init(argc, argv, "ORB_SLAM");
////	ros::start();
//
//	cout << endl << "ORB-SLAM Copyright (C) 2014 Raul Mur-Artal" << endl <<
//			"This program comes with ABSOLUTELY NO WARRANTY;" << endl <<
//			"This is free software, and you are welcome to redistribute it" << endl <<
//			"under certain conditions. See LICENSE.txt." << endl;
//
////	if (argc != 3) {
////		cerr << endl << "Usage: rosrun ORB_SLAM ORB_SLAM path_to_vocabulary path_to_settings (absolute or relative to package directory)" << endl;
////		ros::shutdown();
////		return 1;
////	}
//
//	// Load Settings and Check
//	string strSettingsFile = "/home/fab/freelance/ORB_SLAM/Data/Settings.yaml";
//
//	cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
//	if (!fsSettings.isOpened()) {
//		std::cerr << "Wrong path to settings. Path must be absolut or relative to ORB_SLAM package directory." << std::endl;
//		return 1;
//	}
//
//	bool paused = false;
//
//	//Create Frame Publisher for image_view
//	//    ORB_SLAM::FramePublisher FramePub;
//
//	//Load ORB Vocabulary
//	string strVocFile = "/home/fab/freelance/ORB_SLAM/Data/ORBvoc.yml";
//	cout << endl << "Loading ORB Vocabulary. This could take a while." << endl;
//	cv::FileStorage fsVoc(strVocFile.c_str(), cv::FileStorage::READ);
//	if (!fsVoc.isOpened()) {
//		cerr << endl << "Wrong path to vocabulary. Path must be absolut or relative to ORB_SLAM package directory." << endl;
//		return 1;
//	}
//	ORB_SLAM::ORBVocabulary Vocabulary;
//	Vocabulary.load(fsVoc);
//
//	cout << "Vocabulary loaded!" << endl << endl;
//
//	//Create KeyFrame Database
//	ORB_SLAM::KeyFrameDatabase Database(Vocabulary);
//
//	//Create the map
//	ORB_SLAM::Map World;
//	cv::VideoCapture cap("/home/fab/freelance/ORB_SLAM/BAG/frame%05d.png");
//	if (!cap.isOpened()) {
//		std::cerr << "Error opening image stream" << std::endl;
//	}
//	cv::Mat im;
//
//
//	//    FramePub.SetMap(&World);
//
//	//Create Map Publisher for Rviz
//	//    ORB_SLAM::MapPublisher MapPub(&World);
//
//	//Initialize the Tracking Thread and launch
//	ORB_SLAM::Tracking Tracker(&Vocabulary, /*&FramePub, &MapPub, */&World, strSettingsFile);
//	boost::thread trackingThread(&ORB_SLAM::Tracking::Run, &Tracker);
//
//	Tracker.SetKeyFrameDatabase(&Database);
//
//	//Initialize the Local Mapping Thread and launch
//	ORB_SLAM::LocalMapping LocalMapper(&World);
//	boost::thread localMappingThread(&ORB_SLAM::LocalMapping::Run, &LocalMapper);
//
//	//Initialize the Loop Closing Thread and launch
//	ORB_SLAM::LoopClosing LoopCloser(&World, &Database, &Vocabulary);
//	boost::thread loopClosingThread(&ORB_SLAM::LoopClosing::Run, &LoopCloser);
//
//	//Set pointers between threads
//	Tracker.SetLocalMapper(&LocalMapper);
//	Tracker.SetLoopClosing(&LoopCloser);
//
//	LocalMapper.SetTracker(&Tracker);
//	LocalMapper.SetLoopCloser(&LoopCloser);
//
//	LoopCloser.SetTracker(&Tracker);
//	LoopCloser.SetLocalMapper(&LocalMapper);
//
//	//This "main" thread will show the current processed frame and publish the map
//	double fps = fsSettings["Camera.fps"];
//	if (fps == 0)
//		fps = 30;
//
//	ORB_SLAM::PoseType pose;
//
//	while (!paused) {
//		cap.read(im);
//		if (!im.empty()) {
//			double tm = std::chrono::duration_cast< std::chrono::seconds >(std::chrono::system_clock::now().time_since_epoch()).count();
//			Tracker.addFrame(im, tm); //
//		}
//		//        FramePub.Refresh();
//		//        MapPub.Refresh();
//		Tracker.CheckResetByPublishers();
//		Tracker.getPosition(pose);
//		cout << setprecision(6) << pose.timestamp << setprecision(7) << " " << pose.tx << " " << pose.ty << " " << pose.tz
//										<< " " << pose.qx << " " << pose.qy << " " << pose.qz << " " << pose.qw << endl;
//		usleep(1000000.0/fps);
//	}
//
//
//
////	// Save keyframe poses at the end of the execution
////	ofstream f;
////
////	vector<ORB_SLAM::KeyFrame*> vpKFs = World.GetAllKeyFrames();
////	sort(vpKFs.begin(), vpKFs.end(), ORB_SLAM::KeyFrame::lId);
////
////	cout << endl << "Saving Keyframe Trajectory to KeyFrameTrajectory.txt" << endl;
////	string strFile = ros::package::getPath("ORB_SLAM") + "/" + "KeyFrameTrajectory.txt";
////	f.open(strFile.c_str());
////	f << fixed;
////
////	for (size_t i = 0; i < vpKFs.size(); i++) {
////		ORB_SLAM::KeyFrame* pKF = vpKFs[i];
////
////		if (pKF->isBad())
////			continue;
////
////		cv::Mat R = pKF->GetRotation().t();
////		vector<double> q = ORB_SLAM::Converter::toQuaternion(R);
////		cv::Mat t = pKF->GetCameraCenter();
////		f << setprecision(6) << pKF->mTimeStamp << setprecision(7) << " " << t.at<double>(0) << " " << t.at<double>(1) << " " << t.at<double>(2)
////				<< " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
////
////	}
////	f.close();
////
////	ros::shutdown();
//
//	return 0;
//}
