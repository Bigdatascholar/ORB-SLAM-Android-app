package com.utils.net.rest;

import com.utils.net.results.RestResult;

//Interfaccia x Gestione Risultati chiamate remote 

public interface IResultHandler {
	
	public void onResult(RestResult Message);

}
