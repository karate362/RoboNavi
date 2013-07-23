/*---------------------------------------------------------------------------*/
/*                                                                           */
/* Socket.cpp - Socket Implementation										 */
/*                                                                           */
/* Author : Mark Carrier (mark@carrierlabs.com)                              */
/*                                                                           */
/*---------------------------------------------------------------------------*/
/* Copyright (c) 2007-2008 CarrierLabs, LLC.  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * 4. The name "CarrierLabs" must not be used to
 *    endorse or promote products derived from this software without
 *    prior written permission. For written permission, please contact
 *    mark@carrierlabs.com.
 *
 * THIS SOFTWARE IS PROVIDED BY MARK CARRIER ``AS IS'' AND ANY
 * EXPRESSED OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL MARK CARRIER OR
 * ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *----------------------------------------------------------------------------*/
#include "stdafx.h"
#include "SimpleSocket.h"

CSimpleSocket::CSimpleSocket(CSocketType nType) :
    m_socket(CSimpleSocket::SocketError), 
    m_socketErrno(CSimpleSocket::SocketInvalidSocket), 
    m_pBuffer(NULL), m_nSocketDomain(AF_INET), 
    m_nSocketType(SocketTypeInvalid), m_nBytesReceived(-1), 
    m_nBytesSent(-1), m_nFlags(0), 
    m_bIsBlocking(true)
{
    SetConnectTimeout(1, 0);
    memset(&m_stRecvTimeout, 0, sizeof(struct timeval));
    memset(&m_stSendTimeout, 0, sizeof(struct timeval));
    memset(&m_stLinger, 0, sizeof(struct linger));

	switch(nType)
	{
        //----------------------------------------------------------------------
		// Declare socket type stream - TCP
        //----------------------------------------------------------------------
		case CSimpleSocket::SocketTypeTcp: 
            {
                m_nSocketDomain = AF_INET;
                m_nSocketType = CSimpleSocket::SocketTypeTcp;
                break;
            }
            //----------------------------------------------------------------------
            // Declare socket type datagram - UDP
            //----------------------------------------------------------------------
		case CSimpleSocket::SocketTypeUdp:
            {
                m_nSocketDomain = AF_INET;
                m_nSocketType = CSimpleSocket::SocketTypeUdp;
                break;
            }
            //----------------------------------------------------------------------
            // Declare socket type raw Ethernet - Ethernet
            //----------------------------------------------------------------------
		case CSimpleSocket::SocketTypeRaw:
            {
#ifdef _LINUX
                m_nSocketDomain = AF_PACKET;
                m_nSocketType = CSimpleSocket::SocketTypeRaw;
#endif
#ifdef WIN32
                m_nSocketType = CSimpleSocket::SocketTypeInvalid;
#endif
                break;
            }
		default:
			m_nSocketType = CSimpleSocket::SocketTypeInvalid;
			break;
	}
}


//------------------------------------------------------------------------------
//
// Initialize() - Initialize socket class
//
//------------------------------------------------------------------------------
bool CSimpleSocket::Initialize()
{
    errno = CSimpleSocket::SocketSuccess;

#ifdef WIN32
    //-------------------------------------------------------------------------
    // Data structure containing general Windows Sockets Info                
    //-------------------------------------------------------------------------
    memset(&m_hWSAData, 0, sizeof(m_hWSAData));
    WSAStartup(MAKEWORD(2, 0), &m_hWSAData);
#endif

    //-------------------------------------------------------------------------
    // Create the basic Socket Handle										 
    //-------------------------------------------------------------------------
    m_timer.Initialize();
    m_timer.SetStartTime();
    m_socket = socket(m_nSocketDomain, m_nSocketType, 0);
    m_timer.SetEndTime();

    TranslateSocketError();

    return (IsSocketValid());
}


//------------------------------------------------------------------------------
//
// SetSocketDscp() 
//
//------------------------------------------------------------------------------
bool CSimpleSocket::SetSocketDscp(int nDscp)
{
	bool bRetVal = true;
	int nTempVal = nDscp;

	nTempVal <<= 4;
	nTempVal /= 4;

	if (IsSocketValid())
	{
		if (SETSOCKOPT(m_socket, IPPROTO_IP, IP_TOS, &nTempVal, sizeof(nTempVal)) == SocketError)
		{
			TranslateSocketError();
			bRetVal = false;
		}
	}

	return bRetVal;
}


//------------------------------------------------------------------------------
//
// GetSocketDscp() 
//
//------------------------------------------------------------------------------
int CSimpleSocket::GetSocketDscp(void)
{
	bool       bRetVal = true;
	int        nTempVal = 0;
	socklen_t  nLen = 0; 

	if (IsSocketValid())
	{
		if (GETSOCKOPT(m_socket, IPPROTO_IP, IP_TOS, &nTempVal, &nLen) == SocketError)
		{
			TranslateSocketError();
			bRetVal = false;
		}

		nTempVal *= 4;
		nTempVal >>= 4;

	}

	return nTempVal;
}


//------------------------------------------------------------------------------
//
// Send() - Send data on a valid socket
//
//------------------------------------------------------------------------------
int32 CSimpleSocket::Send(const uint8 *pBuf, size_t bytesToSend)
{
    SetSocketError(SocketSuccess);
    m_nBytesSent = 0;

    switch(m_nSocketType)
    {
        case CSimpleSocket::SocketTypeTcp:
            {
                if (IsSocketValid())
                {
                    if ((bytesToSend > 0) && (pBuf != NULL))
                    {
                        m_timer.Initialize();
                        m_timer.SetStartTime();

                        m_nBytesSent = SEND(m_socket, pBuf, bytesToSend, 0);

                        m_timer.SetEndTime();

                        if (m_nBytesSent == CSimpleSocket::SocketError)
                            TranslateSocketError();
                    }
                }
                break;
            }
        case CSimpleSocket::SocketTypeUdp:
            {
                if (IsSocketValid())
                {
                    if ((bytesToSend > 0) && (pBuf != NULL))
                    {
                        m_timer.Initialize();
                        m_timer.SetStartTime();

                        m_nBytesSent = SENDTO(m_socket, pBuf, bytesToSend, 0, (const sockaddr *)&m_stServerSockaddr, sizeof(m_stServerSockaddr));

                        m_timer.SetEndTime();

                        if (m_nBytesSent == CSimpleSocket::SocketError)
                        {
                            TranslateSocketError();
                        }
                    }
                }
                break;
            }
    }

    return m_nBytesSent;
}


//------------------------------------------------------------------------------
//
// Send() - Send data on a valid socket via a vector of buffers.
//
//------------------------------------------------------------------------------
#ifdef _LINUX
int32 CSimpleSocket::Send(const struct iovec *sendVector, int32 nNumItems)
{
	SetSocketError(SocketSuccess);
	m_nBytesSent = 0;

	if ((m_nBytesSent = writev(m_socket, sendVector, nNumItems)) == CSimpleSocket::SocketError)
    {
        TranslateSocketError();
    }

	return m_nBytesSent;
}
#endif



//------------------------------------------------------------------------------
//
// SetReceiveTimeout()
//
//------------------------------------------------------------------------------
bool CSimpleSocket::SetReceiveTimeout(int32 nRecvTimeoutSec, int32 nRecvTimeoutUsec)
{
	bool bRetVal = true;

	memset(&m_stRecvTimeout, 0, sizeof(struct timeval));

    m_stRecvTimeout.tv_sec = nRecvTimeoutSec;
    m_stRecvTimeout.tv_usec = nRecvTimeoutUsec;

	//--------------------------------------------------------------------------
	// Sanity check to make sure the options are supported!					
	//--------------------------------------------------------------------------
	if (setsockopt(m_socket, SOL_SOCKET, 
                   SO_RCVTIMEO, (const char*) &m_stRecvTimeout,
                   sizeof(struct timeval)) == CSimpleSocket::SocketError)
	{
		bRetVal = false;
		TranslateSocketError();
	}

	return bRetVal;
}


//------------------------------------------------------------------------------
//
// SetSendTimeout()
//
//------------------------------------------------------------------------------
bool CSimpleSocket::SetSendTimeout(int32 nSendTimeoutSec, int32 nSendTimeoutUsec)
{
	bool bRetVal = true;

	memset(&m_stSendTimeout, 0, sizeof(struct timeval));
    m_stSendTimeout.tv_sec = nSendTimeoutSec;
    m_stSendTimeout.tv_usec = nSendTimeoutUsec;

	//--------------------------------------------------------------------------
	// Sanity check to make sure the options are supported!					
	//--------------------------------------------------------------------------
	if (setsockopt(m_socket, SOL_SOCKET, 
                   SO_SNDTIMEO, (const char*) &m_stSendTimeout,
                   sizeof(struct timeval)) == CSimpleSocket::SocketError)
	{
		bRetVal = false;
		TranslateSocketError();
	}

	return bRetVal;
}


//------------------------------------------------------------------------------
//
// SetLinger()
//																			
//------------------------------------------------------------------------------
bool CSimpleSocket::SetOptionLinger(bool bEnable, uint16 nTime)
{
    bool bRetVal = false;

    m_stLinger.l_onoff = (bEnable == true) ? 1: 0;
    m_stLinger.l_linger = nTime;

    if (SETSOCKOPT(m_socket, SOL_SOCKET, SO_LINGER, &m_stLinger, sizeof(m_stLinger)) == 0)
    {
        bRetVal = true;
    }

    TranslateSocketError();

    return bRetVal;
}



//------------------------------------------------------------------------------
//
// Receive() - Attempts to receive a block of data on an established		
//			   connection.	Data is received in an internal buffer managed	
//			   by the class.  This buffer is only valid until the next call	
//			   to Receive(), a call to Close(), or until the object goes out
//			   of scope.													
//																			
//------------------------------------------------------------------------------
int32 CSimpleSocket::Receive(int32 nMaxBytes)
{
	m_nBytesReceived = 0;

	//--------------------------------------------------------------------------
    // If the socket is invalid then return false.
	//--------------------------------------------------------------------------
    if (IsSocketValid() == false)
    {
        return m_nBytesReceived;
    }

	//--------------------------------------------------------------------------
	// Free existing buffer and allocate a new buffer the size of
	// nMaxBytes.
	//--------------------------------------------------------------------------
	if (m_pBuffer != NULL)
	{
		delete [] m_pBuffer;
		m_pBuffer = NULL;
	}

	//--------------------------------------------------------------------------
    // Allocate a new internal buffer to receive data.
	//--------------------------------------------------------------------------
	m_pBuffer = new uint8[nMaxBytes]; 
	SetSocketError(SocketSuccess);

    switch (m_nSocketType)
    {
        m_timer.Initialize();
        m_timer.SetStartTime();

        //----------------------------------------------------------------------
        // If zero bytes are received, then return.  If SocketERROR is 
        // received, free buffer and return CSocket::SocketError (-1) to caller.	
        //----------------------------------------------------------------------
        case CSimpleSocket::SocketTypeTcp:
            m_nBytesReceived = RECV(m_socket, (m_pBuffer + m_nBytesReceived), nMaxBytes, m_nFlags);
            break;
        case CSimpleSocket::SocketTypeUdp:
            {
                uint32 srcSize;
                
                srcSize = sizeof(struct sockaddr_in);
                m_nBytesReceived = RECVFROM(m_socket, m_pBuffer, nMaxBytes, 0, &m_stClientSockaddr, &srcSize);
            
                //--------------------------------------------------------------
                // If src address and server address are not the same then a 
                // bogus message has been received, disregard the message and
                // throw an error.  Return value will be -1.
                //--------------------------------------------------------------
                if (m_stClientSockaddr.sin_addr.s_addr != m_stServerSockaddr.sin_addr.s_addr)
                {
                    m_nBytesReceived = CSimpleSocket::SocketError;
                    SetSocketError(CSimpleSocket::SocketEunknown);
                }
                break;
            }
        default:
            break;
    }
    
    m_timer.SetEndTime();
    TranslateSocketError();

    //--------------------------------------------------------------------------
    // If we encounter an error translate the error code and return.  One 
    // possible error code could be EAGAIN (EWOULDBLOCK) if the socket is
    // non-blocking.  This does not mean there is an error, but no data is
    // yet available on the socket.
    //--------------------------------------------------------------------------
    if (m_nBytesReceived == CSimpleSocket::SocketError)
    {
        if (m_pBuffer != NULL)
        {
            delete [] m_pBuffer;
            m_pBuffer = NULL;
        }
    }

    return m_nBytesReceived;
}


//------------------------------------------------------------------------------
//
// SetNonblocking()
//
//------------------------------------------------------------------------------
bool CSimpleSocket::SetNonblocking(void)
{
	int32  nCurFlags;

#if WIN32
    nCurFlags = 1;

    if (ioctlsocket(m_socket, FIONBIO, (ULONG *)&nCurFlags) != 0)
    {
        TranslateSocketError();
        return false;
    }
#else
	if ((nCurFlags = fcntl(m_socket, F_GETFL)) < 0)
    {
        TranslateSocketError();
		return false;
    }

	nCurFlags |= O_NONBLOCK;

	if (fcntl(m_socket, F_SETFL, nCurFlags) != 0)
    {
        TranslateSocketError();
		return false;
    }

#endif

	m_bIsBlocking = false;

	return true;
}


//------------------------------------------------------------------------------
//
// SetBlocking()
//
//------------------------------------------------------------------------------
bool CSimpleSocket::SetBlocking(void)
{
	int32 nCurFlags;

#if WIN32
    nCurFlags = 1;

    if (ioctlsocket(m_socket, FIONBIO, (ULONG *)&nCurFlags) != 0)
    {
        return false;
    }
#else
	if ((nCurFlags = fcntl(m_socket, F_GETFL)) < 0)
    {
        TranslateSocketError();
		return false;
    }

	nCurFlags &= (~O_NONBLOCK);

	if (fcntl(m_socket, F_SETFL, nCurFlags) != 0)
    {
        TranslateSocketError();
		return false;
    }

#endif
	m_bIsBlocking = true;

	return true;
}


//------------------------------------------------------------------------------
//
// SendFile() - stands-in for system provided sendfile				
//
//------------------------------------------------------------------------------
int32 CSimpleSocket::SendFile(int32 nOutFd, int32 nInFd, off_t *pOffset, int32 nCount)
{
    int32  nOutCount = 0;

#ifdef WIN32
	static char szData[SOCKET_SENDFILE_BLOCKSIZE];
    int32       nInCount = 0;

	if (_lseek(nInFd, *pOffset, SEEK_SET) == -1)
    {
		return -1;
    }

	while (nOutCount < nCount)
	{
		nInCount = (nCount - nOutCount) < SOCKET_SENDFILE_BLOCKSIZE ? (nCount - nOutCount) : SOCKET_SENDFILE_BLOCKSIZE;

		if ((_read(nInFd, szData, nInCount)) != (int32)nInCount)
        {
			return -1;
        }

		if ((SEND(nOutFd, szData, nInCount, 0)) != (int32)nInCount)
        {
			return -1;
        }

		nOutCount += nInCount;
	}
		
	*pOffset += nOutCount;

#else
    nOutCount = sendfile(nOutFd, nInFd, pOffset, (size_t)nCount);
#endif	  
    TranslateSocketError();

	return nOutCount;
}
 

//------------------------------------------------------------------------------
//
// TranslateSocketError() -					
//
//------------------------------------------------------------------------------
void CSimpleSocket::TranslateSocketError(void)
{
#ifdef _LINUX
	switch (errno)
	{
        case EXIT_SUCCESS:
			SetSocketError(CSimpleSocket::SocketSuccess);
            break;
        case ENOTCONN:
            SetSocketError(CSimpleSocket::SocketNotconnected);
            break;
        case ENOTSOCK:
        case EBADF:
        case EACCES:
        case EAFNOSUPPORT:
        case EMFILE:
        case ENFILE:
        case ENOBUFS:
        case ENOMEM:
        case EPROTONOSUPPORT:
			SetSocketError(CSimpleSocket::SocketInvalidSocket);
            break;
		case ECONNREFUSED :
			SetSocketError(CSimpleSocket::SocketConnectionRefused);
			break;
		case ETIMEDOUT:
			SetSocketError(CSimpleSocket::SocketConnectionTimeout);
			break;
		case EINPROGRESS:
			SetSocketError(CSimpleSocket::SocketEinprogress);
			break;
		case EWOULDBLOCK:
            //		case EAGAIN:
			SetSocketError(CSimpleSocket::SocketEwouldblock);
			break;
        case EINTR:
			SetSocketError(CSimpleSocket::SocketInterrupted);
            break;
        case ECONNABORTED:
			SetSocketError(CSimpleSocket::SocketConnectionAborted);
            break;
        case EINVAL:
        case EPROTO:
			SetSocketError(CSimpleSocket::SocketProtocolError);
            break;
        case EPERM:
			SetSocketError(CSimpleSocket::SocketFirewallError);
            break;
        case EFAULT:
			SetSocketError(CSimpleSocket::SocketInvalidSocketBuffer);
            break;
        case ECONNRESET:
            SetSocketError(CSimpleSocket::SocketConnectionReset);
            break;
        default:
            SetSocketError(CSimpleSocket::SocketEunknown);
            break;	
    }
#endif
#ifdef WIN32
    int32 nError = WSAGetLastError();
    switch (nError)
    {
        case EXIT_SUCCESS:
            SetSocketError(CSimpleSocket::SocketSuccess);
            break;
        case WSAEBADF:
        case WSAENOTCONN:
            SetSocketError(CSimpleSocket::SocketNotconnected);
            break;
        case WSAEINTR:
            SetSocketError(CSimpleSocket::SocketInterrupted);
            break;
        case WSAEACCES:
        case WSAEAFNOSUPPORT:
        case WSAEINVAL:
        case WSAEMFILE:
        case WSAENOBUFS:
        case WSAEPROTONOSUPPORT:
            SetSocketError(CSimpleSocket::SocketInvalidSocket);
            break;
        case WSAECONNREFUSED :
            SetSocketError(CSimpleSocket::SocketConnectionRefused);
            break;
        case WSAETIMEDOUT:
            SetSocketError(CSimpleSocket::SocketConnectionTimeout);
            break;
        case WSAEINPROGRESS:
            SetSocketError(CSimpleSocket::SocketEinprogress);
            break;
        case WSAECONNABORTED:
            SetSocketError(CSimpleSocket::SocketConnectionAborted);
            break;
        case WSAEWOULDBLOCK:
            SetSocketError(CSimpleSocket::SocketEwouldblock);
            break;
        case WSAENOTSOCK:
            SetSocketError(CSimpleSocket::SocketInvalidSocket);
            break;
        case WSAECONNRESET:
            SetSocketError(CSimpleSocket::SocketConnectionReset);
            break;
        case WSANO_DATA:
            SetSocketError(CSimpleSocket::SocketInvalidAddress);
            break;
        case WSAEADDRINUSE:
            SetSocketError(CSimpleSocket::SocketAddressInUse);
            break;
        default:
            SetSocketError(CSimpleSocket::SocketEunknown);
            break;	
    }
#endif
}


//------------------------------------------------------------------------------
//
// Select()
//
//------------------------------------------------------------------------------
bool CSimpleSocket::Select()
{
    bool bRetVal = false;

    //-------------------------------------------------------------------------
    // If the socket is non-blocking and the current socket error is 
    // SocketEinprogress or SOcketEwouldblock then poll connection with select 
    // for designated timeout period.
    //-------------------------------------------------------------------------
    if ((IsNonblocking()) && 
        ((GetSocketError() == SocketEwouldblock) || (GetSocketError() == SocketEinprogress)))
    {
        struct timeval timeout;
        int32          nNumDescriptors = -1;

        FD_ZERO(&m_errorFds);
        FD_ZERO(&m_readFds);
        FD_ZERO(&m_writeFds);
        FD_SET(m_socket, &m_errorFds);
        FD_SET(m_socket, &m_readFds);
        FD_SET(m_socket, &m_writeFds);

        timeout.tv_sec = GetConnectTimeoutSec();
        timeout.tv_usec = GetConnectTimeoutUSec();

        nNumDescriptors = SELECT(m_socket+1, &m_readFds, &m_writeFds, &m_errorFds, &timeout);

        //----------------------------------------------------------------------
        // Handle timeout
        //----------------------------------------------------------------------
        if (nNumDescriptors == 0) 
        {
            Close();
            SetSocketError(CSimpleSocket::SocketConnectionTimeout);
        }

        //----------------------------------------------------------------------
        // If a file descriptor (read/write) is set then check the
        // socket error (SO_ERROR) to see if there is a pending error.
        //----------------------------------------------------------------------
        else if (FD_ISSET(m_socket, &m_readFds) || FD_ISSET(m_socket, &m_writeFds))
        {
            int32 nError;
            int32 nLen = sizeof(nError);

            if (GETSOCKOPT(m_socket, SOL_SOCKET, SO_ERROR, &nError, &nLen) == 0)
            {
                if (nError == 0)
                {
                    bRetVal = true;
                }
                else
                {
                    errno = nError;
                }
            }

            TranslateSocketError();
        }
        //----------------------------------------------------------------------
        // There is a definite error, so just return.
        //----------------------------------------------------------------------
        else 
        {
            TranslateSocketError();
        }
    }
    else
    {
        bRetVal = true;
    }

    return bRetVal;
}

