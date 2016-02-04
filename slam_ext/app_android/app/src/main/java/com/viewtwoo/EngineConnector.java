package com.viewtwoo;

/**
 * Created by cmaster11 on 24/06/15.
 */
public class EngineConnector {

    public native void setPose(float x, float y, float z, float dx, float dy, float dz, float dw, boolean isValid);
    public native void setPoseFailed();
    public native void loadScene(String sceneUrl);


}
