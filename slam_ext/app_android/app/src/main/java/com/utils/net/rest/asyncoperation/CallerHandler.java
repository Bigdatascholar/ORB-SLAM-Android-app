package com.utils.net.rest.asyncoperation;
import com.utils.net.rest.IResultHandler;
import com.utils.net.results.*;

import android.app.Activity;
import android.app.ProgressDialog;
import android.os.Handler;
import android.os.Message;
import android.os.Looper;

//Handler Utile x CallerThread. Si occupa di passare messaggi dal Thrad Principale a quello dell'UI.

public class CallerHandler extends Handler{

     private IResultHandler H;
     
	 public CallerHandler(IResultHandler H ){
		 super(Looper.getMainLooper());
		 this.H = H;
	 }
	
	 public void handleMessage(Message Msg) {
		 com.utils.net.results.RestResult M = (com.utils.net.results.RestResult) Msg.obj;
		 M.callHandler(H);
	 }
}
