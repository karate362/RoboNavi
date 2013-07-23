#include "StdAfx.h"
#include "JoyStick.h"

JoyStick::JoyStick(void)
{
	m_lpDIDevice=NULL;
	m_lpDI=NULL;
	m_hWnd=NULL;
	m_hInstance=GetModuleHandle(NULL);//Get handle
}

JoyStick::~JoyStick(void)
{
	if(m_lpDIDevice)
		m_lpDIDevice->Unacquire();//Release DI and DIDevice
	
	if(m_lpDIDevice){
		m_lpDIDevice->Release();
		m_lpDIDevice=NULL; 
	}
	
	if(m_lpDI){
		m_lpDI->Release();
		m_lpDI=NULL; 
	}

}

bool JoyStick::Initialize(){

	HRESULT hr;
	
	if(NULL==m_lpDI){

		hr=DirectInput8Create(m_hInstance,DIRECTINPUT_VERSION,IID_IDirectInput8,(void**)&m_lpDI,NULL);

        if FAILED(hr){
			//OutputDebugString((LPCSTR)"Create failed-inCDIJoystick::Initialize\n");
			return false;
		}
        //Scan for devices
		hr=m_lpDI->EnumDevices(DI8DEVCLASS_GAMECTRL,DIEnumDevicesCallback,&JoystickGUID,DIEDFL_ATTACHEDONLY);
		
		if (FAILED(hr)){
			//OutputDebugString((LPCSTR)"Enumeration failed-inCDIJoystick::Initialize\n");
			return false;
		}
	}

    //Create device
	if(!m_lpDIDevice){ 
		hr=m_lpDI->CreateDevice(JoystickGUID,&m_lpDIDevice,NULL); 
		if FAILED(hr){
			//OutputDebugString((LPCSTR)"Create device failed-inCDIJoystick::Initialize\n"); 
			return false; 
		} 
	}

	//Device Setup
    hr=m_lpDIDevice->SetCooperativeLevel(m_hWnd,DISCL_BACKGROUND|DISCL_EXCLUSIVE);
	if FAILED(hr){
		//OutputDebugString((LPCSTR)"SetCooperativeLevel Failed-inCDIJoystick::Initialize\n");
		return false;
	}


	hr=m_lpDIDevice->SetDataFormat(&c_dfDIJoystick); 
	if(FAILED(hr)){
		//OutputDebugString((LPCSTR)"SetDataFormat failed-inCDIJoystick::Initialize\n");
		return false; 
	}


	hr=m_lpDIDevice->EnumObjects(EnumObjectsCallback,(VOID*)this,DIDFT_ALL);
	if(FAILED(hr)){
		//OutputDebugString((LPCSTR)"EnumObjects failed-inCDIJoystick::Initialize\n");
		return false;
	}

	return true;

}

HRESULT JoyStick::PollDevice(){
	HRESULT hr;				
	static int iCount=0;

	if(NULL==m_lpDIDevice) //Device is not acquired
		return S_OK;
	
	hr=m_lpDIDevice->Poll();//read state
	if(FAILED(hr)){//stream interrupt, we should acquirer the device again
		hr=m_lpDIDevice->Acquire();

		while(hr==DIERR_INPUTLOST){

			if(iCount>30)
				exit(-1); //30 times failure
			
			//OutputDebugString((LPCSTR)"Lost Device-inCJoystick::PollDevice\n");
			
			hr=m_lpDIDevice->Acquire();
			if(SUCCEEDED(hr))
				iCount=0; 
		} //while
		return S_OK;
	}//if

	if(FAILED(hr = m_lpDIDevice->GetDeviceState(sizeof(DIJOYSTATE),&m_diJs)))
		return hr;//

		return S_OK;

}


BOOL CALLBACK JoyStick::DIEnumDevicesCallback(const DIDEVICEINSTANCE* lpddi,VOID* pvRef){
	*(GUID*)pvRef=lpddi->guidInstance;
    return DIENUM_STOP; 
}


BOOL CALLBACK JoyStick::EnumObjectsCallback(const DIDEVICEOBJECTINSTANCE* pdidoi,VOID* pContext){

	HRESULT hr;
	JoyStick* js=(JoyStick*)pContext; //

	if(pdidoi->dwType && DIDFT_AXIS){ //If enum object is stick
		DIPROPRANGE diprg; //stick range struct

		diprg.diph.dwSize =sizeof(DIPROPRANGE);
        diprg.diph.dwHeaderSize=sizeof(DIPROPHEADER);
        diprg.diph.dwHow =DIPH_BYID;
		diprg.diph.dwObj =pdidoi->dwType;//
		diprg.lMin = 0; //min
		diprg.lMax = 65536; //max
        //Set stick range
		hr=js->m_lpDIDevice->SetProperty(DIPROP_RANGE,&diprg.diph);
		if(FAILED(hr)){
			//OutputDebugString((LPCSTR)"Set stick range failed-inCDIJoystick::EnumObjectsCallback\n");
			//return DIENUM_STOP;
		}
        /*
		DIPROPDWORD dipdw; //dead zone
		dipdw.diph.dwSize =sizeof(dipdw);
		dipdw.diph.dwHeaderSize=sizeof(dipdw.diph);
		diprg.diph.dwObj =pdidoi->dwType;//
		dipdw.diph.dwHow =DIPH_DEVICE;
		dipdw.dwData = 1000; //10%
		hr= js->m_lpDIDevice->SetProperty(DIPROP_DEADZONE,&dipdw.diph);
		if(FAILED(hr)){
			OutputDebugString((LPCSTR)"Set dead zone failed-inCDIJoystick::EnumObjectsCallback\n");
			//return DIENUM_STOP;
		}*/

	}

	return DIENUM_CONTINUE;
}