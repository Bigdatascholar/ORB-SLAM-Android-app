package com.utils.net.rest;

public enum RequestMethod
{
	GET(0),
	POST(1);

    private final int value;
    private RequestMethod(int value){
		this.value = value;
    }
    
	public int getValue(){
		return value;
    }
}
