package com.viewtwoo.utils.io;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

/**
 * Created by cmaster11 on 27/07/15.
 */
public class Files {
    public static List<File> getListFiles(File parentDir) {
        ArrayList<File> inFiles = new ArrayList<File>();
        File[] files = parentDir.listFiles();
        for (File file : files) {
            if (file.isDirectory()) {
                inFiles.addAll(getListFiles(file));
            } else {

                    inFiles.add(file);
            }
        }
        return inFiles;
    }
}
