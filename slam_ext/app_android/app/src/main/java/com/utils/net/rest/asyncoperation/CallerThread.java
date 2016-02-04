package com.utils.net.rest.asyncoperation;



import com.utils.net.rest.RequestMethod;
import com.utils.net.rest.RestClient;
import com.utils.net.results.RestResult;
import com.utils.net.results.ResultKind;

import android.os.Handler;
import android.os.Message;

//Thread che si occupa di effettuare le chiamate POST Tramite RestClient.

public class CallerThread implements Runnable{

	private Handler H;
	private RestClient rClient;
	
	public CallerThread(Handler H, RestClient C){
		
		this.H = H;
		this.rClient = C;
	
	}
			
	
	@Override
	public void run() {
		
		Message ResultMessage = new Message();
		
		try{
		
			RestResult M = rClient.Execute();
			ResultMessage.obj = M;
		
		}catch(Exception E){
			ResultMessage.obj = new RestResult(ResultKind.CALL_FAILURE,ResultKind.CALL_FAILURE.getValue(), "Call Failed: exception in worker thread: "+ E.getMessage()); 
		}
		
		H.sendMessage(ResultMessage);
		
	}
}
