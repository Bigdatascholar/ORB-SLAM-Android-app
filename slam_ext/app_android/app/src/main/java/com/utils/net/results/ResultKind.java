package com.utils.net.results;

public enum ResultKind{
	CALL_SUCCESS(0),
	SERVER_EMPTY_RESPONSE(1),
	HTTP_PROTOCOL_ERROR(2),
	INVALID_REQUEST_METHOD(3),
	CALL_FAILURE(4),
	CALL_UNINITIALIZED(5);
		
    private final int value;
    private ResultKind(int value){
        this.value = value;
    }
    public int getValue(){
        return value;
    }
}