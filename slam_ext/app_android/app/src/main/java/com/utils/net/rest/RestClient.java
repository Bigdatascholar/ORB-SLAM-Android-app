package com.utils.net.rest;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.util.ArrayList;

import org.apache.http.HttpEntity;
import org.apache.http.HttpResponse;
import org.apache.http.NameValuePair;
import org.apache.http.client.ClientProtocolException;
import org.apache.http.client.HttpClient;
import org.apache.http.client.methods.HttpGet;
import org.apache.http.client.methods.HttpPost;
import org.apache.http.client.methods.HttpUriRequest;
import org.apache.http.entity.StringEntity;
import org.apache.http.impl.client.DefaultHttpClient;
import org.apache.http.message.BasicNameValuePair;
import org.apache.http.protocol.HTTP;
import org.json.JSONObject;

import com.utils.net.results.RestResult;
import com.utils.net.results.ResultKind;

//Client  HTTP specializzato x le chiamate GET , usato come base x serviceCall

public class RestClient {

	private ArrayList<NameValuePair> headers;
	private String url;
	private String body;
	private int responseCode;
	private String response;
	private RequestMethod method;
	
	public RestClient(String url,RequestMethod method) {
		this.url = url;
		this.method = method;
		this.body="";
		headers = new ArrayList<NameValuePair>();
	}

	public void SetBody(String body) {
		this.body = body;
	}

	public void AddHeader(String name, String value) {
		headers.add(new BasicNameValuePair(name, value));
	}

	public RestResult Execute() {
		RestResult M;

		try {

			switch (method) {
				case POST: {
					HttpPost request = new HttpPost(url);
					
					for (NameValuePair h : headers) {
						request.addHeader(h.getName(), h.getValue());
					}

					request.setEntity(new StringEntity(body, HTTP.UTF_8));
					
					M = executeRequest(request, url);
					break;
				}
				
				case GET: {
					
					HttpGet request  = new HttpGet(url);
					
					for (NameValuePair h : headers) {
						request.addHeader(h.getName(), h.getValue());
					}

					M = executeRequest(request, url);
					break;
				}
				
				default:{
					
					M = new RestResult(ResultKind.INVALID_REQUEST_METHOD, ResultKind.INVALID_REQUEST_METHOD.getValue(),"Call Failed: Invalid Request Method");
					
				}
			}

		} catch (Exception E) {

			M = new RestResult(ResultKind.CALL_FAILURE, ResultKind.CALL_FAILURE.getValue(),"Call Failed: Exception : "+E.getMessage());
		}

		return M;
	}

	private RestResult executeRequest(HttpUriRequest request, String url) {
		HttpClient client = new DefaultHttpClient();
		
		HttpResponse httpResponse;
		RestResult M = new RestResult();

		try {
			
			httpResponse = client.execute(request);
			
			responseCode = httpResponse.getStatusLine().getStatusCode();
			

			HttpEntity entity = httpResponse.getEntity();

			if (entity != null) {

				InputStream instream = entity.getContent();
				response = convertStreamToString(instream);
				instream.close();

				M = new RestResult(ResultKind.CALL_SUCCESS,responseCode,response);
			
			} 
			else {
				
				M = new RestResult(ResultKind.SERVER_EMPTY_RESPONSE,ResultKind.SERVER_EMPTY_RESPONSE.getValue(),"Call Failed: Empty Http Response");
		
			}
			
			client.getConnectionManager().shutdown();

		} catch (ClientProtocolException e) {

			client.getConnectionManager().shutdown();
			M = new RestResult(ResultKind.HTTP_PROTOCOL_ERROR,ResultKind.HTTP_PROTOCOL_ERROR.getValue(),"Call Failded: HTTP Protocol error: "+e.getMessage());

		} catch (IOException e) {

			client.getConnectionManager().shutdown();
			M = new RestResult(ResultKind.CALL_FAILURE,ResultKind.CALL_FAILURE.getValue(),"Call Failed: "+e.getMessage());
		}

		return M;
	}

	

	private static String convertStreamToString(InputStream is) throws IOException {

		BufferedReader reader = new BufferedReader(new InputStreamReader(is));
		StringBuilder sb = new StringBuilder();

		String line = null;
		try {
			
			while ((line = reader.readLine()) != null) {
				sb.append(line + "\n");
			}
			
			is.close();
		
		} catch (IOException e) {
		
			try {
				is.close();
			} catch (IOException innerException) {
				innerException.printStackTrace();
			}
			
			throw e;
		}
		
		
		return sb.toString();
	}
}
