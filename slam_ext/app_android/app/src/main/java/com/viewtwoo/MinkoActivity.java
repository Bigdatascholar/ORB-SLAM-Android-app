package com.viewtwoo;

import android.content.pm.ApplicationInfo;
import android.content.res.AssetManager;
import android.os.Bundle;
import android.util.Log;
import android.view.Menu;
import android.view.MenuInflater;
import android.view.MenuItem;
import com.utils.net.rest.RequestManager;

import com.github.ikonst.android_dl.AndroidDl;
import com.viewtwoo.utils.io.Assets;
import com.viewtwoo.utils.io.Files;

import org.libsdl.app.*;

import java.io.File;
import java.util.List;

/**
 * Minko Activity, Java entry point
 */
public class MinkoActivity extends SDLActivity {
    static final String TAG = "MinkoActivity";

    EngineConnector engineConnector;
    SlamConnector slamConnector;
    RequestManager glue = new RequestManager();

    Thread poseUpdateThread = null;
    boolean runPoseUpdateThread = false;
    File filesDir = null;
    AssetManager assetManager;

    public String getSettingsDirPath() {
        return settingsDirPath;
    }

    String settingsDirPath = null;

    public String getSampleDirPath() {
        return sampleDirPath;
    }

    String sampleDirPath = null;

    Runnable poseUpdateRunnable = new Runnable() {
        @Override
        public void run() {
            Log.i(TAG, "poseUpdateThread running.");


            float[] pose;

            while (runPoseUpdateThread) {
                pose = slamConnector.getCurrentPose();

                if (pose != null) {
                    if(pose[7] == 1) {
                        engineConnector.setPose(
                                pose[0], pose[1], pose[2], pose[3], pose[4], pose[5], pose[6], pose[6]!=0
                        );
                    }
                    else
                    {
                        engineConnector.setPoseFailed();
                    }
                    /*Log.i(TAG, String.format("poseUpdateThread set new pose: %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f!",
                            pose[0], pose[1], pose[2], pose[3], pose[4], pose[5], pose[6]
                    ));*/
                }

                try {
                    Thread.sleep(50, 0);
                } catch (InterruptedException e) {
                    Log.i(TAG, "poseUpdateThread interrupted exc.");
                }
            }
        }
    };

    private void resetPoseUpdateThread() {

        if (poseUpdateThread != null && poseUpdateThread.isAlive()) {

            stopPoseUpdateThread();
            try {
                poseUpdateThread.join();
            } catch (InterruptedException e) {
                Log.i(TAG, "poseUpdateThread interrupted exc. on join");
            }
        }

        poseUpdateThread = new Thread(poseUpdateRunnable);
        runPoseUpdateThread = true;
        poseUpdateThread.start();
    }

    private void stopPoseUpdateThread() {
        Log.i(TAG, "Stopping poseUpdateThread");
        runPoseUpdateThread = false;
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        ApplicationInfo ai = getApplicationInfo();

        AndroidDl.initialize(ai.nativeLibraryDir);

        filesDir = getFilesDir();
        filesDir.mkdir();

        // Init assets
        assetManager = getAssets();

        // Initialize Settings.yaml
        //String settingsFilename = "settings/Settings.yaml";
        File settingsDir = new File(filesDir, "settings");
        settingsDir.mkdir();
        File sampleDir = new File(filesDir, "sample");
        sampleDir.mkdir();

        try{
            Assets.copyAssets(assetManager, "settings", filesDir);
            Assets.copyAssets(assetManager, "sample", filesDir);
            settingsDirPath=settingsDir.getAbsolutePath();
            sampleDirPath=sampleDir.getAbsolutePath();
        } catch (Exception e) {
            Log.e(TAG, "Could not copy assets!\n"+e);
            throw new RuntimeException(e);
        }

        // END init

        // DEBUG! List all libs
        //List<File> allLibs = Files.getListFiles(new File(ai.nativeLibraryDir));

        //AndroidDl.loadLibrary("wtengine");

        engineConnector = new EngineConnector();
        slamConnector = new SlamConnector(this);

        slamConnector.onCreate();

        resetPoseUpdateThread();

        // Initialize HTTP request connector
        glue.Init();

    }

    @Override
    protected void onPause() {
        super.onPause();

        stopPoseUpdateThread();
        slamConnector.onPause();

        //System.exit(0);
    }

    @Override
    protected void onResume() {
        super.onResume();

        slamConnector.onResume();
        resetPoseUpdateThread();
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();

        stopPoseUpdateThread();
        slamConnector.onDestroy();
    }

    private Menu menu;
    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        MenuInflater inflater = getMenuInflater();
        inflater.inflate(R.menu.main_menu, menu);
        this.menu = menu;

        return true;
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        switch (item.getItemId()) {
            case R.id.exit:
                //engineConnector.exit();
                //mCamera.release();

                slamConnector.exit();

                this.finish();

                System.exit(0);
                return true;

            case R.id.initDebug:

                slamConnector.initDebug();

                return true;
            /*case R.id.saveMapPoints:

                slamConnector.saveMapPoints();

                return true;
            case R.id.reloadScene:

                engineConnector.loadScene("http://hydex11.net/getscene/index.php");

                return true;*/
            default:
                return super.onOptionsItemSelected(item);
        }
    }
}