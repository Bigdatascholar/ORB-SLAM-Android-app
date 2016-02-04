package com.utils.net.results;

import com.utils.net.rest.IResultHandler;


public class RestResult {
	
	private int httpCode;
	private String body; 
	private ResultKind result;
	
	public RestResult(){
		setResult(ResultKind.CALL_UNINITIALIZED);
		setHttpCode(ResultKind.CALL_UNINITIALIZED.getValue());
		setBody(""); 
	}
	
	public RestResult(ResultKind result, int httpCode, String body ){
		this.setResult(result);
		this.setHttpCode(httpCode);
		this.setBody(body);
	}
	
	public void callHandler(IResultHandler H){
		H.onResult(this);
	}

	public int getHttpCode() {
		return httpCode;
	}

	private void setHttpCode(int httpCode) {
		this.httpCode = httpCode;
	}

	public String getBody() {
		return body;
	}

	private void setBody(String body) {
		this.body = body;
	}

	public ResultKind getResult() {
		return result;
	}

	private void setResult(ResultKind result) {
		this.result = result;
	}
}
