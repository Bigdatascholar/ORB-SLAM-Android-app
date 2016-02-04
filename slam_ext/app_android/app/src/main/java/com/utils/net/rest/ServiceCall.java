package com.utils.net.rest;

import com.utils.net.rest.asyncoperation.CallerHandler;
import com.utils.net.rest.asyncoperation.CallerThread;
import com.utils.net.results.RestResult;
import com.utils.net.results.ResultKind;


public class ServiceCall {
	
	//Triade!
	protected CallerThread cThread;
	private CallerHandler cHandler;
	protected RestClient restClient;
	
	private IResultHandler rHandler;
	
	public ServiceCall(){}
	
	public void init(String Url, RequestMethod Method){
		restClient = new RestClient(Url,Method);
	}
	
	public void setResultHandler(IResultHandler ResHandler){
		rHandler = ResHandler;
	}
	
	public void setBody(String body){
		restClient.SetBody(body);
	}
	
	public void addHeader(String Name, Object Value){
		restClient.AddHeader(Name,Value.toString());
	}
	
	public void call() throws Exception{
		
		if (rHandler == null || restClient == null  ){
			throw new Exception("ServiceCall : Handler or RestClient are null");
		}
		
		cHandler = new CallerHandler(rHandler);
		cThread  = new CallerThread(cHandler,restClient);
	
		Thread T = new Thread(cThread);
		T.start();
	}
}
