package com.viewtwoo.utils.io;

import android.content.res.AssetManager;
import android.util.Log;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.ArrayList;

/**
 * Created by cmaster11 on 13/05/15.
 */
public class Assets {

    private static String LOG_TAG = "Assets";

    public static boolean copyAssets(AssetManager assetManager, String path,
                                     File targetFolder) throws Exception {
        Log.i(LOG_TAG, "Copying " + path + " to " + targetFolder);
        String sources[] = assetManager.list(path);
        if (sources.length == 0) { // its not a folder, so its a file:
            copyAssetFileToFolder(assetManager, path, targetFolder);
        } else { // its a folder:
            if (path.startsWith("images") || path.startsWith("sounds")
                    || path.startsWith("webkit")) {
                Log.i(LOG_TAG, "  > Skipping " + path);
                return false;
            }
            File targetDir = new File(targetFolder, path);
            targetDir.mkdirs();
            for (String source : sources) {
                String fullSourcePath = path.equals("") ? source : (path
                        + File.separator + source);
                copyAssets(assetManager, fullSourcePath, targetFolder);
            }
        }
        return true;
    }

    private static void copyAssetFileToFolder(AssetManager assetManager,
                                              String fullAssetPath, File targetBasePath) throws IOException {
        InputStream in = assetManager.open(fullAssetPath);
        OutputStream out = new FileOutputStream(new File(targetBasePath,
                fullAssetPath));
        byte[] buffer = new byte[16 * 1024];
        int read;
        while ((read = in.read(buffer)) != -1) {
            out.write(buffer, 0, read);
        }
        in.close();
        out.flush();
        out.close();
    }

    /*public static ArrayList<String> loadAvailableBundleModels(String dataPath)
    {
        File modelsDir = new File(dataPath + "/model/bundle");

        File[] file = modelsDir.listFiles();

        ArrayList<String> availableModels = new ArrayList<>();

        for (File f : file)
        {
            if (f.isFile() && f.getPath().endsWith(".json")) {
            // Load model file

                availableModels.add(f.getAbsolutePath());
            }
        }

        return availableModels;
    }*/
}
