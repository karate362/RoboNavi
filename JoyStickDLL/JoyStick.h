#pragma once


#define DIRECTINPUT_VERSION 0x0800
#include <dinput.h>

class _declspec(dllexport)  JoyStick
{
public:
	JoyStick(void);
	virtual ~JoyStick(void);

    bool Initialize(void); //initialization
    static BOOL CALLBACK DIEnumDevicesCallback(const DIDEVICEINSTANCE*lpddi,VOID*pvRef);//
    static BOOL CALLBACK EnumObjectsCallback(const DIDEVICEOBJECTINSTANCE*pdidoi,VOID*pContext);//
    HRESULT PollDevice(void); //Read device state

public:
	HINSTANCE m_hInstance; //
	HWND m_hWnd; //
	LPDIRECTINPUT8 m_lpDI; //DI8 pointer
	LPDIRECTINPUTDEVICE8 m_lpDIDevice; //DIDevice8 pointer
	DIJOYSTATE m_diJs; //Joystick state 
	GUID JoystickGUID; //GUID

};


