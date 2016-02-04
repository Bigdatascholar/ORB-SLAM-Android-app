package com.utils.net.rest;

import com.utils.net.rest.IResultHandler;
import com.utils.net.rest.RequestMethod;
import com.utils.net.rest.ServiceCall;
import com.utils.net.results.RestResult;
import java.util.HashMap;

public class RequestManager {

	public native boolean Init();
	public native boolean Shutdown();
	public native void SendResult(int handle, RestResult result);
	
	private HashMap<Integer,ServiceCall> mCalls = new HashMap<Integer,ServiceCall>();
	private static int lastId = 0;

	public int AddRequest(String Url, String Method, String Body /*, HashMap<String,String> iHashMap*/)
	{
		synchronized(this){
			lastId++;
			ServiceCall call = new ServiceCall();
			call.init(Url,RequestMethod.valueOf(Method));
			call.setBody(Body);

			call.setResultHandler(new IResultHandler() {
				final int myId = lastId;

				@Override
				public void onResult(final RestResult result) {
					RemoveRequest(myId); 
					SendResult(myId,result);
				}
			});
		
			try{
				mCalls.put(lastId,call);
				call.call();
			}catch(Exception E){
				RemoveRequest(lastId);
				E.printStackTrace();
				return -1;
			}
						
			return lastId;
		}
	}

	public void RemoveRequest(int id){
		synchronized(this){
			mCalls.remove(id);
		}
	}

}
