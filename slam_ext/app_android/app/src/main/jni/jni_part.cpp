#include <android.h>


//#include "LiveSLAMWrapper.h"
//#include "IOWrapper/Custom/CustomImageStreamThread.h"
//#include "util/settings.h"
//#include "util/globalFuncs.h"

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

#if defined( NDK_PROFILE )
extern "C" void monstartup( char const * );
  extern "C" void moncleanup();
#endif

using namespace std;
using namespace cv;
//using namespace lsd_slam;

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

extern "C" {

    int frameCount;
    bool paused;

    bool poseFailed = false;

    Mat mGrTmpCrop,mGrTmp2;
    bool frameMatInitialized = false;
    bool slamInitialized = false;
    int finalWidth;//400; //320;
    int finalHeight;// = 240;//240; //240;
    Size size320;// = Size(finalWidth,finalHeight);
    float finalRatio;// = (float)finalWidth/(float)finalHeight;
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

    struct initStructModel
    {

        string strSettingsFile;
        double fx;
        double fy;
        double cx;
        double cy;
    } initStruct;

// Functions definitions
    void resetPose(ORB_SLAM::PoseType &p);
    void clearSlam();

    void resetPose(ORB_SLAM::PoseType &p)
    {
        p.tx = 0;
        p.ty = 0;
        p.tz = 0;
        p.qx = 0;
        p.qy  = 0;
        p.qz = 0;
        p.qw = 1;
    }


    JNIEXPORT jfloatArray JNICALL Java_com_viewtwoo_SlamConnector_getCurrentPose(JNIEnv* env, jobject)
    {

        jfloatArray result;
        result = env->NewFloatArray(8);

        if(result == NULL)
            return NULL;

        jfloat returnArray[8];
        returnArray[0] = pose.tx;
        returnArray[1] = pose.ty;
        returnArray[2] = pose.tz;
        returnArray[3] = pose.qx;
        returnArray[4] = pose.qy;
        returnArray[5] = pose.qz;
        returnArray[6] = pose.qw;
        returnArray[7] = poseFailed ? 0 : 1;

        env->SetFloatArrayRegion(result, 0, 8, returnArray);

        return result;

    }

    JNIEXPORT void JNICALL Java_com_viewtwoo_SlamConnector_Slam(JNIEnv*, jobject, jlong addrGray, jlong addrRgba)
    {
        if(!slamInitialized) return;

        try
        {
            Mat& mRgb = *(Mat*) addrRgba;
            Mat& mGr = *(Mat*) addrGray;
            if (frameCount > 1)
            {
                if(!frameMatInitialized)
                {
                    cameraSize = mGr.size();

                    LOGI("Frame mat size: %d, %d", cameraSize.width, cameraSize.height);

                    cameraRatio = (float)cameraSize.width/(float)cameraSize.height;
                    finalWidth = (int) (cameraRatio * (float)finalHeight);

                    size320 = Size(finalWidth,finalHeight);
                    finalRatio = (float)finalWidth/(float)finalHeight;

                    if(cameraSize.height != finalHeight)
                    {
                        hasToResize = true;

                        mGrTmp2 = Mat::zeros(size320, mGr.type());
                        LOGI("Set frame resize to %d, %d", size320.width, size320.height);
                    }

                    snprintf(line1, 1000, "  ");
                    snprintf(line2, 1000, "    Initialization");

                    frameMatInitialized = true;
                    LOGI("Frame mat initialized");
                }

                if(!frameMatInitialized)
                    return;

                if(hasToResize)
                {
                    resize(mGr, mGrTmp2, size320);

                }

                double tm = std::chrono::duration_cast< std::chrono::seconds >(std::chrono::system_clock::now().time_since_epoch()).count();
                if (Tracker->addFrame(hasToResize ? mGrTmp2 : mGr, tm))
                {
                    //  LOGI("Frame added successfully");
                }
                else
                {
                    //  LOGI("Buffer Filled");
                }


                Tracker->getPosition(pose);
                poseFailed = false;

		stringstream ss;
		ss << "X: " << pose.tx << " Y: " << pose.ty << " Z: " << pose.tz;

		cv::putText(mRgb,ss.str(), cv::Point(5,20), CV_FONT_HERSHEY_SIMPLEX, 0.5f, Scalar(255, 255, 255));

		ss.str("");
		ss.clear();

		// tracking state

		ORB_SLAM::Tracking::eTrackingState state = Tracker->mState;
		ss << "State: " << state;

		cv::putText(mRgb,ss.str(), cv::Point(5,40), CV_FONT_HERSHEY_SIMPLEX, 0.5f, Scalar(255, 255, 255));

		ss.str("");
		ss.clear();

		// tracking nmatches

		/*int nMatches = Tracker->lastNMatches;
		ss << "Last NMatches: " << nMatches;

		cv::putText(mRgb,ss.str(), cv::Point(5,60), CV_FONT_HERSHEY_COMPLEX, 0.5f, Scalar(255, 255, 255));*/

            }
            else
            {
                poseFailed = true;
                snprintf(line1, 1000, "  ");
                snprintf(line2, 1000, "    Initialization");
            }

            ++frameCount;
        }
        catch(std::exception &ex)
        {
            LOGE("Exception: %s", ex.what());

            poseFailed = false;
            return;


        }
    }

    JNIEXPORT void JNICALL Java_com_viewtwoo_SlamConnector_loadVocabulary(JNIEnv * jenv, jobject obj, jstring vocabularyFilename)
    {



        const char * fn = jenv->GetStringUTFChars(vocabularyFilename, NULL);
        string strVocFile = fn;
        jenv->ReleaseStringUTFChars(vocabularyFilename,fn);

        LOGI("Got vocabulary file %s", strVocFile.c_str());

        cv::FileStorage fsVoc(strVocFile.c_str(), cv::FileStorage::READ);

        if (!fsVoc.isOpened())
        {
            LOGE("Wrong path to vocabulary.\n");
            return;
        }

        paused = true;
        slamInitialized= false;

        clearSlam();

        Vocabulary = new ORB_SLAM::ORBVocabulary;
        Vocabulary->load(fsVoc);

        LOGI("Vocabulary loaded!\n");

        Database = new ORB_SLAM::KeyFrameDatabase(*Vocabulary);
        World = new ORB_SLAM::Map;

        frameCount = 0;
        Tracker = new ORB_SLAM::Tracking(Vocabulary, World,initStruct.strSettingsFile, initStruct.fx, initStruct.fy, initStruct.cx, initStruct.cy);
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

bool debugInitialized = false;

JNIEXPORT void JNICALL Java_com_viewtwoo_SlamConnector_exit(JNIEnv * jenv, jobject obj){
#if defined( NDK_PROFILE )
if(debugInitialized)
moncleanup();

#endif
}

JNIEXPORT void JNICALL Java_com_viewtwoo_SlamConnector_initDebug(JNIEnv * jenv, jobject obj){
#if defined( NDK_PROFILE )
if(!debugInitialized){
// setenv( "CPUPROFILE_FREQUENCY", "500", 1 ); // interrupts per second, default 100
monstartup( "libslamengine.so" );
debugInitialized=true;
}
#endif
}

    void clearSlam()
    {



//        if(Tracker!=nullptr)
//        {
//        Tracker->Reset();
//        }


    }

    JNIEXPORT void JNICALL Java_com_viewtwoo_SlamConnector_slamInitialization(JNIEnv * jenv, jobject obj, jstring settingsFilename, jint _finalHeight,jdouble fx, jdouble fy, jdouble cx, jdouble cy)
    {


        finalHeight = _finalHeight;

        const char * fn = jenv->GetStringUTFChars(settingsFilename, NULL);
        string strSettingsFile = fn;
        jenv->ReleaseStringUTFChars(settingsFilename,fn);

        LOGI("Got settings file %s", strSettingsFile.c_str());

        cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);

        if (!fsSettings.isOpened())
        {
            LOGE("Wrong path to settings.\n");
            return;
        }

        initStruct.strSettingsFile = strSettingsFile;
        initStruct.cx=cx;
        initStruct.cy=cy;
        initStruct.fx=fx;
        initStruct.fy=fy;

    }

}
