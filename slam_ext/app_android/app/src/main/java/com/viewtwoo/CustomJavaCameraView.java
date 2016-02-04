package com.viewtwoo;

import android.content.Context;
import android.hardware.Camera;
import android.util.AttributeSet;
import android.util.Log;

import org.opencv.android.JavaCameraView;

import java.util.List;

/**
 * Created by cmaster11 on 25/06/15.
 */
public class CustomJavaCameraView extends JavaCameraView {
    private static final String TAG = "CustomJavaCameraView";

    public CustomJavaCameraView(Context context, int cameraId) {
        super(context, cameraId);
    }

    public CustomJavaCameraView(Context context, AttributeSet attrs) {
        super(context, attrs);
    }

    @Override
    protected boolean initializeCamera(int width, int height) {

        boolean initSuper = false;
        /*try {
             initSuper = super.initializeCamera(1280, 720);
        }
        catch (Exception ex)
        {
            Log.i(TAG, "Could not initialize to 1280");
        }*/

        if(!initSuper)
        {
            initSuper = super.initializeCamera(width, height);
        }

        if (initSuper) {
            setFixedFocus();
        }

        return initSuper;
    }

    public Camera.Parameters getCameraParameters()
    {
        return mCamera.getParameters();
    }

    public void setFixedFocus() {

        Camera.Parameters parameters = mCamera.getParameters();

        parameters.setPreviewFpsRange(5,15);


        try {
            List<String> supportedFocusModes = parameters.getSupportedFocusModes();

            if (supportedFocusModes.contains(Camera.Parameters.FOCUS_MODE_FIXED)) {
                parameters.setFocusMode(Camera.Parameters.FOCUS_MODE_FIXED);
                Log.i(TAG, "Set focus: " + Camera.Parameters.FOCUS_MODE_FIXED);
            } else if (supportedFocusModes.contains(Camera.Parameters.FOCUS_MODE_INFINITY)) {
                parameters.setFocusMode(Camera.Parameters.FOCUS_MODE_INFINITY);
                Log.i(TAG, "Set focus: " + Camera.Parameters.FOCUS_MODE_INFINITY);

            } else
                throw new Exception("No fixed focus mode available");
            mCamera.setParameters(parameters);
        } catch (Exception ex) {
            Log.e(TAG, "Set focus failed: " + ex);
        }

    }
}
