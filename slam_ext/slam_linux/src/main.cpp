#include "main.h"

#include "calibrate.h"
#include "slam.h"


int main(int argc, char *argv[]) {
    bool slamInitialized = false;
    int finalHeight = 200;

    loadParameters();

    VideoCapture cap(0); // open the default camera
    if(!cap.isOpened())  // check if we succeeded
       {cout << "No camera" << endl; return -1;}

    cap.set(CV_CAP_PROP_FRAME_WIDTH, 640);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, 480);

    namedWindow("SLAM",1);
    for(;;)
    {
        Mat frame;
        cap >> frame; // get a new frame from camera

        if(!slamInitialized)
        {
            Mat * intrinsics = getIntrinsicPtr();
            int frameCols = frame.cols;

            int finalWidth = (int)((float)frameCols/(float)frame.rows*(float)finalHeight);

            float originalfx =intrinsics->at<float>(0,0);
            float originalfy = intrinsics->at<float>(1,1);
            float originalcx =intrinsics->at<float>(0,2);
            float originalcy = intrinsics->at<float>(1,2);

            float fx = originalfx/(originalcx*2)*(float)finalWidth; // abs((float) finalWidth / (2 * tan(p.getHorizontalViewAngle() / 180 * Math.PI / 2)));
            float fy = originalfy/(originalcy*2)*(float)finalHeight; // abs((float) finalHeight / (2 * tan(p.getVerticalViewAngle() / 180 * Math.PI / 2)));

            float cx = (float)finalWidth/2.f;
            float cy = (float)finalHeight/2.f;

            slamInitialization(BASE_OUTPUT_PATH_STRING + "/../data/Settings.yaml",finalHeight, fx, fy, cx,cy );
            loadVocabulary(BASE_OUTPUT_PATH_STRING + "/../data/small_voc.yml");

            slamInitialized = true;
        }

        slam(frame);

        Pose p = getPose();

        stringstream ss;
        ss << "X: " << setprecision(4) << p.tx << " Y: " << setprecision(4) << p.ty << " Z: " << setprecision(4) << p.tz;

        cv::putText(frame,ss.str(), cv::Point(5,20), CV_FONT_HERSHEY_COMPLEX, 0.5f, Scalar(255, 255, 255));

        ss.str("");
        ss.clear();

        // tracking state

        ORB_SLAM::Tracking::eTrackingState state = getTracking()->mState;
        ss << "State: " << state;

        cv::putText(frame,ss.str(), cv::Point(5,40), CV_FONT_HERSHEY_COMPLEX, 0.5f, Scalar(255, 255, 255));

        imshow("SLAM", frame);
        if(waitKey(30) >= 0) break;
    }

    return 0;
}
