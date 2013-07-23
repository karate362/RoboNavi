/**
 * Sick LMS100 Interface
 *
 * Created: December  2nd, 2008 by Shao-Wen Yang.
 *	Polling mode data retreival is provided with up to 25Hz frequency in all resolutions.
 *  Getters and setters for all parameters are provided, including angular resolution and scan frequency.
 *  Return messages from sensor is checked to ensure robustness.
 *  Data content is set to be composed of one 16-bit data channel without any encoder attached.
 * TODO:
 *  Check the configuration often in case parameters are changed by other clients.
 *  Enable end users to change angular resolution and scan frequency silently.
 */

#ifndef __ROBOTICS_SICKLMS100LASER_H__
#define __ROBOTICS_SICKLMS100LASER_H__

#include "Laser.h"
#include "ActiveSocket.h"

#define SOCKET_BUFFERSIZE 8192
#define SOCKET_BLOCKSIZE  512
#define DATA_BUFFERSIZE   1082
#define DATA_TOKENSIZE    16

namespace Robotics {

	struct SickLms100Channel {
		// Command
		// NOTE: Skip command type and command here.

		// Device Inforemation
		unsigned short Version;
		unsigned short Device;
		unsigned int   Serial;
		unsigned char  Status[2];

		// Status Information
		unsigned short CountTelegram;
		unsigned short CountScan;
		unsigned int   TimeUp;
		unsigned int   TimeTrans;
		unsigned short StatusInput[2];
		unsigned short StatusOutput[2];
		unsigned char  Reserved;

		// Measurement Parameters
		unsigned int   FreqScan;
		unsigned int   FreqMeasure;

		// Encoders
		unsigned char  NumEncoder;
		// NOTE: Skip encoder position and speed here.

		// 16-bit Data Channels
		unsigned char  NumChannel16;
		// Only take into account the last channel.
		char           Content16[6];
		float          ScaleFactor16;
		float          ScaleOffset16;
		int            AngleStart16;
		unsigned short AngleStep16;
		unsigned short NumData16;
		int            Data16[DATA_BUFFERSIZE];	// underlying unsigned short

		// 8-bit Data Channels
		unsigned char  NumChannel8;
		// Only take into account the last channel.
		char           Content8[6];
		float          ScaleFactor8;
		float          ScaleOffset8;
		int            AngleStart8;
		unsigned short AngleStep8;
		unsigned short NumData8;
		int            Data8[DATA_BUFFERSIZE];	// underlying unsigned short

		// Position
		unsigned char  Position;
		// NOTE: Skip internal content.

		// Name
		unsigned char  Name;
		// NOTE: Skip internal content.

		// Comment
		unsigned char  Comment;
		// NOTE: Skip internal content.

		// Time
		unsigned char  Time;
		// NOTE: Skip internal content.

		// Event
		unsigned char  Event;
		// NOTE: Skip internal content.
	};

	struct SickLms100Config {
		// Measurement Parameters
		unsigned char  Status;
		unsigned int   FreqScan;
		unsigned int   Resolution;
		unsigned char  Reserved;
		int            AngleStart;
		int            AngleStop;
	};


	class _declspec(dllexport) SickLms100Laser : public Laser {
		public:
			SickLms100Laser();
			~SickLms100Laser();

			virtual bool Open(const char *pAddr);
			virtual bool Scan(int *aScan);
			virtual bool Close();

			const static char CmdStx;
			const static char CmdEtx;
			const static char CmdLogin[];
			const static char CmdGetConfig[];
			const static char CmdSetConfig[];
			const static char CmdGetState[];
			const static char CmdSetStart[];
			const static char CmdSetStop[];
			const static char CmdSetContent[];
			const static char CmdSetParameter[];
			const static char CmdPollData[];
			const static char CmdStreamData[];

		private:
			SickLms100Channel *m_pChannel;
			SickLms100Config  *m_pConfig;

			CActiveSocket m_socket;
			int           m_nBuffer;
			char         *m_pBuffer;
			char         *m_pIndicator;
			char         *m_pToken;

			inline bool Login();
			inline bool GetConfig();
			inline bool SetConfig();
			inline bool SetStart();
			inline bool SetStop();
			inline bool GetState();
			inline bool SetContent();
			inline bool SetParameter();
			inline bool GetData();

			inline bool ParseLogin();
			inline bool ParseGetConfig();
			inline bool ParseSetConfig();
			inline bool ParseSetStart();
			inline bool ParseSetStop();
			inline bool ParseGetState();
			inline bool ParseSetContent();
			inline bool ParseSetParameter();
			inline bool ParseGetData();

			inline bool Send(const char *pCmd);
			inline bool Receive();
			inline bool SendReceive(const char *pCmd);

			inline void *Rewind();
			inline void *Next();
			inline void *NextHexadecimal();

			inline          char   NextCharacter();
			inline          short  NextShort();
			inline          int    NextInteger();
			inline          float  NextFloat();
			inline          double NextDouble();
			inline unsigned char   NextUnsignedCharacter();
			inline unsigned short  NextUnsignedShort();
			inline unsigned int    NextUnsignedInteger();

	};

}

#endif