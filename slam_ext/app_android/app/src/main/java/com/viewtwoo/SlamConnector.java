package com.viewtwoo;

import android.app.Activity;
import android.graphics.Point;
import android.hardware.Camera;
import android.util.Log;
import android.view.Display;
import android.view.WindowManager;

import com.github.ikonst.android_dl.AndroidDl;

import org.opencv.android.CameraBridgeViewBase;
import org.opencv.core.CvType;
import org.opencv.core.Mat;

/**
 * Created by cmaster11 on 24/06/15.
 */
public class SlamConnector implements CameraBridgeViewBase.CvCameraViewListener2 {

    private static final String TAG = "SlamConnector";

    private Mat mRgba;
    //private Mat mRgbaUp;
    private Mat mGray;
    //private Mat mGrayTmp;
    private Activity context;

    private CameraBridgeViewBase mOpenCvCameraView;

    private int screenW;
    private int screenH;

    public SlamConnector(Activity context) {
        this.context = context;
        Log.i(TAG, "Instantiated new " + this.getClass());
    }

    private void loadLibraries() {
        AndroidDl.loadLibrary("opencv_java");
        AndroidDl.loadLibrary("slamengine");
    }

    private void resetCamera() {
        mOpenCvCameraView.enableView();
    }

    /**
     * Called when the activity is first created.
     */
    public void onCreate() {
        Log.i(TAG, "called onCreate");

        loadLibraries();

        context.getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        /*context.getWindow().setFlags(WindowManager.LayoutParams.FLAG_FULLSCREEN,
                WindowManager.LayoutParams.FLAG_FULLSCREEN);*/

        Display display = context.getWindowManager().getDefaultDisplay();
        Point outSize = new Point();
        display.getSize(outSize);

        screenW = outSize.x;
        screenH = outSize.y;

        mOpenCvCameraView = (CameraBridgeViewBase) context.findViewById(R.id.cameraSurface);
        mOpenCvCameraView.setCameraIndex(CameraBridgeViewBase.CAMERA_ID_BACK);
        mOpenCvCameraView.setMaxFrameSize(1024, 576);
        //mOpenCvCameraView.setMaxFrameSize(1280,720);
        mOpenCvCameraView.setCvCameraViewListener(this);


        resetCamera();
        //  reset();

        Log.i(TAG, "onCreate finished");
    }

    public void onPause() {
        Log.i(TAG, "called onPause");
        // stop();
        if (mOpenCvCameraView != null)
            mOpenCvCameraView.disableView();
    }

    public void onResume() {
        Log.i(TAG, "called onResume");
        resetCamera();
        //  reset();
        //OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_2_4_3, this, mLoaderCallback);
    }

    public void onDestroy() {
        //stop();
        Log.i(TAG, "called onDestroy");
        if (mOpenCvCameraView != null)
            mOpenCvCameraView.disableView();
    }

    public void onCameraViewStarted(int width, int height) {
        Log.i(TAG, "called onCameraViewStarted w:" + width + " - h:" + height);
        mRgba = new Mat(height, width, CvType.CV_8UC4);
        //mRgbaUp = new Mat(screenH,screenW, CvType.CV_8UC4);
        mGray = new Mat(height, width, CvType.CV_8UC1);
        //mGrayTmp = new Mat(height, width, CvType.CV_8UC1);

        // Calculate camera fields
        double fx;
        double fy;
        double cx;
        double cy;

        int finalHeight = 200; // Hardcoded!
        int finalWidth = (int) ((float) width / (float) height * (float) finalHeight);

        cx = (float) finalWidth / 2.0;
        cy = (float) finalHeight / 2.0;

        Camera.Parameters p = ((CustomJavaCameraView) mOpenCvCameraView).getCameraParameters();

        fx = Math.abs((double) finalWidth / (2 * Math.tan(p.getHorizontalViewAngle() / 180 * Math.PI / 2)));
        fy = Math.abs((double) finalHeight / (2 * Math.tan(p.getVerticalViewAngle() / 180 * Math.PI / 2)));

        Log.d(TAG, String.format("Using camera parameters: %.2f %.2f %.2f %.2f", fx, fy, cx, cy));

        String settingsFilename = ((MinkoActivity) context).getSettingsDirPath() + "/Settings.yaml";
        Log.d(TAG, "Initializing SLAM with settings: " + settingsFilename);

        slamInitialization(settingsFilename, finalHeight, fx, fy, cx, cy);

        String vocFilename = ((MinkoActivity) context).getSampleDirPath() + "/sampleVoc.yaml";
        Log.d(TAG, "Initializing SLAM with vocabulary: " + vocFilename);

        loadVocabulary(vocFilename);
        // reset();
    }

    public void onCameraViewStopped() {
        mRgba.release();
        //mRgbaUp.release();
        mGray.release();
        //  stop();
    }

    public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame) {

        mRgba = inputFrame.rgba();
        mGray = inputFrame.gray();

        Slam(mGray.getNativeObjAddr(), mRgba.getNativeObjAddr());

        return mRgba;
        // Imgproc.resize(mRgba,mRgbaUp,new Size(screenW, screenH));

        // return mRgbaUp;
    }


    public native void loadVocabulary(String settingsFilename);
    public native void slamInitialization(String settingsFilename, int finalHeight, double fx, double fy, double cx, double cy);

    public native void Slam(long addrGray, long addrRgba);
/*

    public native void stop();

    public native void reset();
*/

    public native float[] getCurrentPose();

    public native void saveMapPoints();
    public native void exit();
    public native void initDebug();

}
