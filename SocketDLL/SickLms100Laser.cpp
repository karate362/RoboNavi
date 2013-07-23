/**
 * Sick LMS100 Interface
 *
 * Created: December  2nd, 2008 by Shao-Wen Yang.
 */
#include "stdafx.h"
#include "SickLms100Laser.h"
#include <string.h>

using namespace Robotics;

const char SickLms100Laser::CmdStx            = 0x02;
const char SickLms100Laser::CmdEtx            = 0x03;
const char SickLms100Laser::CmdLogin[]        = "sMN SetAccessMode 03 F4724744";
const char SickLms100Laser::CmdGetConfig[]    = "sRN LMPscancfg";
const char SickLms100Laser::CmdSetConfig[]    = "sMN mLMPsetscancfg +5000 +1 +5000 -450000 +2250000";	// configurable
const char SickLms100Laser::CmdGetState[]     = "sRN LCMstate";
const char SickLms100Laser::CmdSetStart[]     = "sMN LMCstartmeas";
const char SickLms100Laser::CmdSetStop[]      = "sMN LMCstopmeas";
const char SickLms100Laser::CmdSetContent[]   = "sWN LMDscandatacfg 01 00 0 1 0 00 00 0 0 0 0 +1";		// configurable
const char SickLms100Laser::CmdSetParameter[] = "sMN mEEwriteall";
const char SickLms100Laser::CmdPollData[]     = "sRN LMDscandata";
const char SickLms100Laser::CmdStreamData[]   = "sEN LMDscandata";

SickLms100Laser::SickLms100Laser() {
	m_nBuffer    = 0;
	m_pBuffer    = new char[SOCKET_BUFFERSIZE];
	m_pIndicator = m_pBuffer;
	m_pToken     = new char[DATA_TOKENSIZE];
	m_pChannel   = new SickLms100Channel();
	m_pConfig    = new SickLms100Config();

	memset(m_pBuffer, 0, SOCKET_BUFFERSIZE);
	memset(m_pChannel, 0, sizeof(SickLms100Channel));
}


SickLms100Laser::~SickLms100Laser() {
	delete m_pToken;
	delete m_pBuffer;
	delete m_pConfig;
	delete m_pChannel;
}

bool SickLms100Laser::Open(const char *pAddr) {
	m_socket.Initialize();

	return m_socket.Open((const uint8 *)pAddr, 2111)
		&& Login()			// Login as an authorized client
//		&& SetConfig()		// Configure resolution and frequency
		&& GetConfig()		// Check configured resolution and frequency
//		&& SetStop()		// Stop scanning
		&& SetStart()		// Ensure in the measure mode
		&& GetState()		// Check the state
		&& SetContent()		// Set data content
		&& SetParameter()	// Store the parameters
		&& GetData();
}

bool SickLms100Laser::Scan(int *aScan) {
	return GetData() && memcpy(aScan, m_pChannel->Data16, m_pChannel->NumData16*sizeof(int));
}

bool SickLms100Laser::Close() {
	return m_socket.Close();
}

bool SickLms100Laser::Login() {
	return SendReceive(CmdLogin) && ParseLogin();
}

bool SickLms100Laser::GetConfig() {
	return SendReceive(CmdGetConfig) && ParseGetConfig();
}

bool SickLms100Laser::SetConfig() {
	return SendReceive(CmdSetConfig) && ParseSetConfig();
}

bool SickLms100Laser::SetStart() {
	return SendReceive(CmdSetStart) && ParseSetStart();
}

bool SickLms100Laser::SetStop() {
	return SendReceive(CmdSetStop) && ParseSetStop();
}

bool SickLms100Laser::GetState() {
	return SendReceive(CmdGetState) && ParseGetState();
}

bool SickLms100Laser::SetContent() {
	return SendReceive(CmdSetContent) && ParseSetContent();
}

bool SickLms100Laser::SetParameter() {
	return SendReceive(CmdSetParameter) && ParseSetParameter();
}

bool SickLms100Laser::GetData() {
	return SendReceive(CmdPollData) && ParseGetData();
}

bool SickLms100Laser::ParseLogin() {
	Rewind();

	// Command Type and Command
	Next();	// sAN
	Next();	// SetAcessMode

	// 0: Error
	// 1: Success
	return (bool)NextUnsignedCharacter();
}

bool SickLms100Laser::ParseGetConfig() {
	Rewind();

	// Command Type and Command
	Next();	// sAN
	Next();	// mLMPsetscancfg

	m_pConfig->FreqScan             = NextUnsignedInteger();	// scan frequency in 1/100 Hz
	m_pConfig->Reserved             = NextUnsignedCharacter();	// reserved
	m_pConfig->Resolution           = NextUnsignedInteger();	// angular resolution in 1/100 Hz
	m_pConfig->AngleStart           = NextInteger();			// start angle in 1/10 degree
	m_pConfig->AngleStop            = NextInteger();			// stop angle in 1/10 degree

	return true;
}

bool SickLms100Laser::ParseSetConfig() {
	Rewind();

	// Command Type and Command
	Next();	// sAN
	Next();	// mLMPsetscancfg

	// 0: No Error
	// 1: Frequency Error
	// 2: Resolution Error
	// 3: Resolution and Scan Area Error
	// 4: Scan Area Error
	// 5: Other Error
	m_pConfig->Status               = NextUnsignedCharacter();

	m_pConfig->FreqScan             = NextUnsignedInteger();	// scan frequency in 1/100 Hz
	m_pConfig->Reserved             = NextUnsignedCharacter();	// reserved
	m_pConfig->Resolution           = NextUnsignedInteger();	// angular resolution in 1/100 Hz
	m_pConfig->AngleStart           = NextInteger();			// start angle in 1/10 degree
	m_pConfig->AngleStop            = NextInteger();			// stop angle in 1/10 degree

	return !(bool)m_pConfig->Status;
}

bool SickLms100Laser::ParseSetStart() {
	Rewind();

	// Command Type and Command
	Next();	// sAN
	Next();	// LMCstartmeas

	// 0: No Error
	// 1: Not Allowed
	return !(bool)NextUnsignedCharacter();
}

bool SickLms100Laser::ParseSetStop() {
	Rewind();

	// Command Type and Command
	Next();	// sAN
	Next();	// LMCstopmeas

	// 0: No Error
	// 1: Not Allowed
	return !(bool)NextUnsignedCharacter();
}

bool SickLms100Laser::ParseGetState() {
	Rewind();

	// Command Type and Command
	Next();	// sAN
	Next();	// LCMstate

	// 0: No Error
	// 1: Pollution Warning
	// 2: Pollution Error
	// 3: Fatal Error
	return !(bool)NextUnsignedCharacter();
}

bool SickLms100Laser::ParseSetContent() {
	Rewind();

	// Command Type and Command
	Next();	// sWA
	Next();	// LMDscandatacfg

	return true;
}

bool SickLms100Laser::ParseSetParameter() {
	Rewind();

	// Command Type and Command
	Next();	// sAN
	Next();	// LMCstopmeas

	// 0: Error
	// 1: Success
	return (bool)NextUnsignedCharacter();
}

bool SickLms100Laser::ParseGetData() {
	Rewind();

	// Command Type and Command
	Next();	// sRA
	Next();	// LMDscandata

	// Device Inforemation
	m_pChannel->Version             = NextUnsignedShort();	
	m_pChannel->Device              = NextUnsignedShort();	
	m_pChannel->Serial              = NextUnsignedInteger();	
	m_pChannel->Status[0]           = NextUnsignedCharacter();
	m_pChannel->Status[1]           = NextUnsignedCharacter();

	// Status Information
	m_pChannel->CountTelegram       = NextUnsignedShort();
	m_pChannel->CountScan           = NextUnsignedShort();
	m_pChannel->TimeUp              = NextUnsignedInteger();
	m_pChannel->TimeTrans           = NextUnsignedInteger();
	m_pChannel->StatusInput[0]      = NextUnsignedShort();
	m_pChannel->StatusInput[1]      = NextUnsignedShort();
	m_pChannel->StatusOutput[0]     = NextUnsignedShort();
	m_pChannel->StatusOutput[1]     = NextUnsignedShort();
	m_pChannel->Reserved            = NextUnsignedCharacter();

	// Measurement Parameters
	m_pChannel->FreqScan            = NextUnsignedInteger();
	m_pChannel->FreqMeasure         = NextUnsignedInteger();

	// Encoders
	m_pChannel->NumEncoder          = NextUnsignedCharacter();
	for (int i = 0; i < m_pChannel->NumEncoder; i++) {
		Next();
		Next();
	}

	// 16-bit Data Channels
	m_pChannel->NumChannel16        = NextUnsignedCharacter();
	for (int i = 0; i < m_pChannel->NumChannel16; i++) {
		memcpy(m_pChannel->Content16, Next(), sizeof(m_pChannel->Content16));
		m_pChannel->ScaleFactor16   = NextFloat();
		m_pChannel->ScaleOffset16   = NextFloat();
		m_pChannel->AngleStart16    = NextInteger();
		m_pChannel->AngleStep16     = NextUnsignedShort();
		m_pChannel->NumData16       = NextUnsignedShort();

		for (int j = 0; j < m_pChannel->NumData16; j++) {
			m_pChannel->Data16[j]   = NextUnsignedShort();
		}
	}

	// 8-bit Data Channels
	m_pChannel->NumChannel8		    = NextUnsignedCharacter();
	for (int i = 0; i < m_pChannel->NumChannel8; i++) {
		memcpy(m_pChannel->Content8, Next(), sizeof(m_pChannel->Content8));
		m_pChannel->ScaleFactor8    = NextFloat();
		m_pChannel->ScaleOffset8    = NextFloat();
		m_pChannel->AngleStart8     = NextInteger();
		m_pChannel->AngleStep8      = NextUnsignedShort();
		m_pChannel->NumData8        = NextUnsignedShort();

		for (int j = 0; j < m_pChannel->NumData8; j++) {
			m_pChannel->Data8[j]    = NextUnsignedShort();
		}
	}

	// Others
	m_pChannel->Position            = NextUnsignedCharacter();
	m_pChannel->Name                = NextUnsignedCharacter();
	m_pChannel->Comment             = NextUnsignedCharacter();
	m_pChannel->Time                = NextUnsignedCharacter();
	m_pChannel->Event               = NextUnsignedCharacter();

	return (bool)m_pChannel->NumChannel16;
}

bool SickLms100Laser::Send(const char *pCmd) {
//	printf("Send: %s\n", pCmd);

	return m_socket.Send((const uint8 *)&CmdStx, 1)
		&& m_socket.Send((const uint8 *)pCmd, strlen(pCmd))
		&& m_socket.Send((const uint8 *)&CmdEtx, 1);
}

bool SickLms100Laser::Receive() {
	char *p = m_pBuffer;
	int   n = 0;

	if (n = m_socket.Receive() && *m_socket.GetData() == CmdStx) {	
		while ((n = m_socket.Receive(SOCKET_BLOCKSIZE)) > 0) {
			memcpy(p, m_socket.GetData(), n);

			p += n;

			if (*(p-1) == CmdEtx) {
				*(p-1) = '\0';// discard ETX

				break;
			}
		}
	}

	m_nBuffer = p - m_pBuffer - 1;// discard STX							

//	printf("Recv: %s\n", m_pBuffer);

	return (bool)m_nBuffer;
}

bool SickLms100Laser::SendReceive(const char *pCmd) {
	return Send(pCmd) && Receive();
}

void *SickLms100Laser::Rewind() {
	m_pIndicator = strtok(m_pBuffer, " ");

	return (void *)m_pIndicator;
}

void *SickLms100Laser::Next() {
	m_pToken = m_pIndicator;	

	m_pIndicator = strtok(NULL, " ");

	return (void *)m_pToken;
}

void *SickLms100Laser::NextHexadecimal() {
	sscanf(m_pIndicator, "%x", (void *)m_pToken);//把m_pIndicator轉換成16進位整數 存進m_pToken

	m_pIndicator = strtok(NULL, " ");//往後讀一個字串

	return (void *)m_pToken;
}

char SickLms100Laser::NextCharacter() {
	return *(char *)NextHexadecimal();
}

short SickLms100Laser::NextShort() {
	return *(short *)NextHexadecimal();
}

int SickLms100Laser::NextInteger() {
	return *(int *)NextHexadecimal();
}
			
float SickLms100Laser::NextFloat() {
	return *(float *)NextHexadecimal();
}

double SickLms100Laser::NextDouble() {
	return *(double *)NextHexadecimal();
}

unsigned char SickLms100Laser::NextUnsignedCharacter() {
	return *(unsigned char *)NextHexadecimal();
}

unsigned short SickLms100Laser::NextUnsignedShort() {
	return *(unsigned short *)NextHexadecimal();
}

unsigned int SickLms100Laser::NextUnsignedInteger() {
	return *(unsigned int *)NextHexadecimal();
}
