#include <jni.h>
#include <vector>

//#include "LiveSLAMWrapper.h"
//#include "IOWrapper/Custom/CustomImageStreamThread.h"
//#include "util/settings.h"
//#include "util/globalFuncs.h"
//#include "SlamSystem.h"

#include <thread>

#include<fstream>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <android/log.h>

#include "Tracking.h"
#include "Map.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "KeyFrameDatabase.h"
#include "ORBVocabulary.h"
#include "Converter.h"

using namespace std;
using namespace cv;
//using namespace lsd_slam;

extern "C" {

#define LOGI(...) ((void)__android_log_print(ANDROID_LOG_INFO, "ORB_SLAM", __VA_ARGS__))
#define LOGW(...) ((void)__android_log_print(ANDROID_LOG_WARN, "ORB_SLAM", __VA_ARGS__))
#define LOGE(...) ((void)__android_log_print(ANDROID_LOG_ERROR, "ORB_SLAM", __VA_ARGS__))

	ORB_SLAM::ORBVocabulary* Vocabulary;
    ORB_SLAM::KeyFrameDatabase* Database;
    ORB_SLAM::Map* World;
	ORB_SLAM::Tracking* Tracker;
	ORB_SLAM::LocalMapping* LocalMapper;
	ORB_SLAM::LoopClosing* LoopCloser;
	ORB_SLAM::PoseType pose;

	boost::thread* localMappingThread;
	boost::thread* trackingThread;
	boost::thread* loopClosingThread;

	int frameCount;
	std::thread* t;

	bool paused;

	JNIEXPORT void JNICALL Java_org_opencv_samples_tutorial2_Tutorial2Activity_ORBSlam(JNIEnv*, jobject, jlong addrGray, jlong addrRgba) {
		Mat& mGr = *(Mat*) addrGray;
		Mat& mRgb = *(Mat*) addrRgba;

		char line1[1000];
		char line2[1000];
		if (frameCount > 5) {
			resize(mGr, mGr, Size(640, 480));
			//callback per le immagini
			double tm = std::chrono::duration_cast< std::chrono::seconds >(std::chrono::system_clock::now().time_since_epoch()).count();
			if (Tracker->addFrame(mGr, tm)){
				LOGI("Frame added successfully");
			} else {
				LOGI("Buffer Filled");
			}
			//callback per la posa
			Tracker->getPosition(pose);

			snprintf(line1, 1000, "  %f %f %f %f", tm, pose.tx, pose.ty, pose.tz);
			snprintf(line2, 1000, "  %f %f %f %f", pose.qx, pose.qy, pose.qz, pose.qw);

		} else {
			snprintf(line1, 1000, "  ");
			snprintf(line2, 1000, "    Initialization");
		}

		LOGI("%s", line1);
		LOGI("%s", line2);
		++frameCount;
		rectangle(mRgb, Point(0, 530), Point(839, 600), Scalar(64, 64, 64), -1);
		putText(mRgb, line1, Point(15, 550), FONT_HERSHEY_PLAIN, 1.5, Scalar(255, 255, 255));
		putText(mRgb, line2, Point(15, 580), FONT_HERSHEY_PLAIN, 1.5, Scalar(255, 255, 255));
	}

	JNIEXPORT void JNICALL Java_org_opencv_samples_tutorial2_Tutorial2Activity_slamInitialization(JNIEnv *, jobject) {
		string strSettingsFile = "/storage/emulated/0/ORB_SLAM/Settings.yaml";
        cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
        if (!fsSettings.isOpened()) {
        	LOGE("Wrong path to settings. Path must be absolut or relative to ORB_SLAM package directory.\n");
        	return;
        }

        paused = false;

        //string strVocFile = "/storage/emulated/0/ORB_SLAM/ORBvoc.yml";
        string strVocFile = "/storage/emulated/0/ORB_SLAM/small_voc.yml";
        LOGI("Loading ORB Vocabulary. This could take a while.\n");
        cv::FileStorage fsVoc(strVocFile.c_str(), cv::FileStorage::READ);
        if (!fsVoc.isOpened()) {
        	LOGE("Wrong path to vocabulary. Path must be absolut or relative to ORB_SLAM package directory.\n");
        	return;
        }
        Vocabulary = new ORB_SLAM::ORBVocabulary;
        Vocabulary->load(fsVoc);

		LOGI("Vocabulary loaded!\n");

		Database = new ORB_SLAM::KeyFrameDatabase(*Vocabulary);
        World = new ORB_SLAM::Map;
//		cd.cx = 319.666 / 2.0;
//		cd.cy = 229.666 / 2.0;
//		cd.fx = 641.00850 / 2.0;
//		cd.fy = 641.00850 / 2.0;
//		cd.width = 640 / 2.0;
//		cd.height = 480 / 2.0;

		frameCount = 0;

		Tracker = new ORB_SLAM::Tracking(Vocabulary, World, strSettingsFile);
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

		std::cout << "Initialization completed" << std::endl;
	}
}
