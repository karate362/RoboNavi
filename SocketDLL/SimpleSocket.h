/*---------------------------------------------------------------------------*/
/*                                                                           */
/* SimpleSocket.h - Simple Socket base class decleration.                    */
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
#ifndef __SOCKET_H__
#define __SOCKET_H__

#include <sys/stat.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <errno.h>

#ifdef _LINUX
  #include <sys/socket.h>
  #include <netinet/in.h>
  #include <arpa/inet.h>
  #include <netinet/tcp.h>
  #include <netinet/ip.h>
  #include <netdb.h>
  #include <linux/if_packet.h>
  #include <linux/if_packet.h>
  #include <linux/if_ether.h>
  #include <linux/if.h>
  #include <sys/time.h>
  #include <sys/uio.h>
  #include <unistd.h>	
  #include <fcntl.h>
  #include <sys/sendfile.h>
#endif
#ifdef WIN32
  #include <io.h>
  #include <winsock2.h>

  #define IP_TOS      1 /// int; IP type of service and precedence.
#endif
#include "Host.h"
#include "StatTimer.h"


//-----------------------------------------------------------------------------
// General class macro definitions and typedefs
//-----------------------------------------------------------------------------
#define SOCKET_SENDFILE_BLOCKSIZE 8192

/// Provides a platform independent class to for socket development.
/// This class is designed to abstract socket communication development in a 
/// platform independent manner. 
/// A list of events:
/// - Socket types
///  -# CActiveSocket Class
///  -# CPassiveSocket Class
class _declspec(dllexport) CSimpleSocket {
public:
    /// Defines the three possible states for shuting down a socket.
    typedef enum 
    {
        Receives = SHUT_RD, ///< Shutdown active socket.
        Sends = SHUT_WR,    ///< Shutdown passive socket.
        Both = SHUT_RDWR    ///< Shutdown both active and passive sockets.
    } CShutdownMode; 

    /// Defines the socket types defined by CSimpleSocket class.
    typedef enum  
    {
        SocketTypeInvalid,   ///< Invalid socket type.
        SocketTypeTcp,       ///< Defines socket as TCP socket.
        SocketTypeUdp,       ///< Defines socket as UDP socket.
        SocketTypeRaw        ///< Provides raw network protocol access.
    } CSocketType;

    /// Defines all error codes handled by the CSimpleSocket class.
    typedef enum 
    {
        SocketError = -1,          ///< Generic socket error translates to error below.
        SocketSuccess = 0,         ///< No socket error.
        SocketInvalidSocket,       ///< Invalid socket handle.
        SocketInvalidAddress,      ///< Invalid destination address specified.
        SocketInvalidPort,         ///< Invalid destination port specified.
        SocketConnectionRefused,   ///< No server is listening at remote address.
        SocketConnectionTimeout,   ///< Timed out while attempting connection.
        SocketEwouldblock,         ///< Operation would block if socket were blocking.
        SocketNotconnected,        ///< Currently not connected.
        SocketEinprogress,         ///< Socket is non-blocking and the connection cannot be completed immediately
        SocketInterrupted,         ///< Call was interrupted by a signal that was caught before a valid connection arrived.
        SocketConnectionAborted,   ///< The connection has been aborted.
        SocketProtocolError,       ///< Invalid protocol for operation.
        SocketFirewallError,       ///< Firewall rules forbid connection.
        SocketInvalidSocketBuffer, ///< The receive buffer point outside the process's address space.
        SocketConnectionReset,     ///< Connection was forcibly closed by the remote host.
        SocketAddressInUse,        ///< Address already in use.
        SocketEunknown
    } CSocketError;

public:
    CSimpleSocket(CSocketType type = SocketTypeTcp);
    virtual ~CSimpleSocket() {};

    /// Initialize instance of CSocket.  This method MUST be called before an
    /// object can be used. Errors : CSocket::SocketProtocolError, 
    /// CSocket::SocketInvalidSocket,
    /// @return true if properly initialized.
    virtual bool Initialize(void);

    virtual bool Close(void) { return true;};

    /// Examine the socket descriptor sets currently owned by the instance of
    /// the socket class (the readfds, writefds, and errorfds parameters) to 
    /// see whether some of their descriptors are ready for reading, are ready 
    /// for writing, or have an exceptional condition pending, respectively.
    /// @return
    virtual bool Select(void);

    /// Does the current instance of the socket object contain a valid socket
    /// descriptor.
    ///  @return true 
    virtual bool IsSocketValid(void) { return (m_socket != SocketError); };

    /// Provides a standard error code for cross platform development by
    /// mapping the operating system error to an error defined by the CSocket
    /// class.
    void TranslateSocketError(void);

    /// Attempts to receive a block of data on an established connection.	
    /// @param nMaxBytes maximum number of bytes to receive.
    /// @return number of bytes actually received, return of zero means the
    /// connection has been shutdown on the other side, and a return of -1
    /// means that an error has occurred.
    virtual int32 Receive(int32 nMaxBytes = 1);

    /// Attempts to send a block of data on an established connection.
    /// @param pBuf block of data to be sent.
    /// @param bytesToSend size of data block to be sent.
    /// @return number of bytes actually sent, return of zero means the
    /// connection has been shutdown on the other side, and a return of -1
    /// means that an error has occurred.
    virtual int32 Send(const uint8 *pBuf, size_t bytesToSend);

    /// Copies data between one file descriptor and another.  
    /// On some systems this copying is done within the kernel, and thus is 
    /// more efficient than the combination of CSimpleSocket::Send and 
    /// CSimpleSocket::Receive, which would require transferring data to and 
    /// from user space.  
    /// <br>\b Note: This is avaialbe on all implementations, but the kernel 
    /// implementation is only avialble on Unix type systems.
	/// @param nOutFd descriptor opened for writing.
    /// @param nInFd descriptor opened for reading.
    /// @param pOffset from which to start reading data from input file.
    /// @param nCount number of bytes to copy between file descriptors.
    /// @return number of bytes written to the out file descriptor.
    int32 SendFile(int32 nOutFd, int32 nInFd, off_t *pOffset, int32 nCount);

    /// Returns blocking/non-blocking state of socket.
    /// @return true if the socket is blocking, else return false.
    bool IsNonblocking(void) { return (m_bIsBlocking == false); };

    /// Set the socket to blocking.
    /// @return true if successful set to blocking, else return false;
    bool SetBlocking(void);

    /// Set the socket as non-blocking.
    /// @return true if successful set to non-blocking, else return false;
    bool SetNonblocking(void);

    /// Get a pointer to internal receive buffer.  The user MUST not free this
    /// pointer when finished.  This memory is managed internally by the CSocket 
    /// class.
    /// @return pointer to data if valid, else returns NULL.
    uint8 *GetData(void)  { return m_pBuffer; };

    /// Returns the number of bytes received on the last call to 
    /// CSocket::Receive().
    /// @return number of bytes received.
    int32 GetBytesReceived(void) { return m_nBytesReceived; };

    /// Returns the number of bytes sent on the last call to 
    /// CSocket::Send().
    /// @return number of bytes sent.
    int32 GetBytesSent(void) { return m_nBytesSent; };

    /// Controls the actions taken when CSimpleSocket::Close is executed on a socket object that has
    /// unsent data.  The default value for this option is \b off.  
    /// - Following are the three possible scenarios.
    ///  -# \b bEnable is false, CSimpleSocket::Close returns immediately, but any unset data
    /// is transmitted (after CSimpleSocket::Close returns)
    ///  -# \b bEnable is true and \b nTime is zero, CSimpleSocket::Close return immediately and 
    /// any unsent data is discarded.
    ///  -# \b bEnable is true and \b nTime is nonzero, CSimpleSocket::Close does not return
    /// until all unsent data is transmitted (or the connection is Closed by the
    /// remote system).
    /// <br><p>
    /// @param bEnable true to enable option false to disable option.
    /// @param nTime time in seconds to linger.
    /// @return true if option successfully set
    bool SetOptionLinger(bool bEnable, uint16 nTime);

    /// Gets the timeout value that specifies the maximum number of seconds a
    /// call to CSimpleSocket::Open waits until it completes. 
    /// @return the length of time in seconds
    int32 GetConnectTimeoutSec(void) { return  m_stConnectTimeout.tv_sec; };

    /// Gets the timeout value that specifies the maximum number of microseconds
    /// a call to CSimpleSocket::Open waits until it completes. 
    /// @return the length of time in microseconds
    int32 GetConnectTimeoutUSec(void) { return  m_stConnectTimeout.tv_usec; };

    /// Sets the timeout value that specifies the maximum amount of time a call 
    /// to CSimpleSocket::Receive waits until it completes. Use the method
    /// CSimpleSocket::SetReceiveTimeout to specify the number of seconds to wait.
    /// If a call to CSimpleSocket::Receive has blocked for the specified length of
    /// time without receiving additional data, it returns with a partial count 
    /// or CSimpleSocket::GetSocketError set to CSimpleSocket::SocketEwouldblock if no data 
    /// were received. 
    /// @param nConnectTimeoutSec of timeout in seconds.
    /// @param nConnectTimeoutUsec of timeout in microseconds.
    /// @return true if socket connection timeout was successfully set.
    void SetConnectTimeout(int32 nConnectTimeoutSec, int32 nConnectTimeoutUsec = 0) 
    { 
        m_stConnectTimeout.tv_sec = nConnectTimeoutSec; 
        m_stConnectTimeout.tv_usec = nConnectTimeoutUsec; 
    };

    /// Gets the timeout value that specifies the maximum number of seconds a
    /// a call to CSimpleSocket::Receive waits until it completes. 
    /// @return the length of time in seconds
    int32 GetReceiveTimeoutSec(void) { return  m_stRecvTimeout.tv_sec; };

    /// Gets the timeout value that specifies the maximum number of microseconds
    /// a call to CSimpleSocket::Receive waits until it completes. 
    /// @return the length of time in microseconds
    int32 GetReceiveTimeoutUSec(void) { return  m_stRecvTimeout.tv_usec; };

    /// Sets the timeout value that specifies the maximum amount of time a call 
    /// to CSimpleSocket::Receive waits until it completes. Use the method
    /// CSimpleSocket::SetReceiveTimeout to specify the number of seconds to wait.
    /// If a call to CSimpleSocket::Receive has blocked for the specified length of
    /// time without receiving additional data, it returns with a partial count 
    /// or CSimpleSocket::GetSocketError set to CSimpleSocket::SocketEwouldblock if no data 
    /// were received. 
    ///  @param nRecvTimeoutSec of timeout in seconds.
    ///  @param nRecvTimeoutUsec of timeout in microseconds.
    ///  @return true if socket timeout was successfully set.
    bool SetReceiveTimeout(int32 nRecvTimeoutSec, int32 nRecvTimeoutUsec = 0);

    /// Gets the timeout value that specifies the maximum number of seconds a
    /// a call to CSimpleSocket::Send waits until it completes. 
    /// @return the length of time in seconds
    int32 GetSendTimeoutSec(void) { return  m_stSendTimeout.tv_sec; };

    /// Gets the timeout value that specifies the maximum number of microseconds
    /// a call to CSimpleSocket::Send waits until it completes. 
    /// @return the length of time in microseconds
    int32 GetSendTimeoutUSec(void) { return  m_stSendTimeout.tv_usec; };

    /// Gets the timeout value that specifies the maximum amount of time a call 
    /// to CSimpleSocket::Send waits until it completes. 
    /// @return the length of time in seconds
    bool SetSendTimeout(int32 nSendTimeoutSec, int32 nSendTimeoutUsec = 0);

    /// Returns the last error that occured for the instace of the CSimpleSocket
    /// instance.  This method should be called immediately to retrieve the 
    /// error code for the failing mehtod call.
    ///  @return last error that occured.
    CSocketError GetSocketError(void) { return m_socketErrno; };

    /// Get the total time the of the last operation in milliseconds.
    ///  @return number of milliseconds of last operation.
    uint32 GetTotalTimeMs() { return m_timer.GetMilliSeconds(); };

    /// Get the total time the of the last operation in microseconds.
    ///  @return number of microseconds or last operation.
    uint32 GetTotalTimeUsec() { return m_timer.GetMicroSeconds(); };

#ifdef _LINUX
    int32 Send(const struct iovec *sendVector, int32 nNumItems);
#endif 

    /// Return Differentiated Services Code Point (DSCP) value currently set on the socket object.
    /// @return DSCP for current socket object.
    /// <br><br> \b NOTE: Windows special notes http://support.microsoft.com/kb/248611.
	int GetSocketDscp(void);

	/// Set Differentiated Services Code Point (DSCP) for socket object.
	///  @param nDscp value of TOS setting which will be converted to DSCP
	///  @return true if DSCP value was properly set
    /// <br><br> \b NOTE: Windows special notes http://support.microsoft.com/kb/248611.
	bool SetSocketDscp(int nDscp);

	/// Return socket descriptor
	///  @return socket descriptor which is a signed 32 bit integer.
    SOCKET GetSocketDescriptor() { return m_socket; };

    /// Returns clients Internet host address as a string in standard numbers-and-dots notation.
	///  @return NULL if invalid
    uint8 *GetClientAddr() { return (uint8 *)inet_ntoa(m_stClientSockaddr.sin_addr); };

	/// Returns the port number on which the client is connected.
	///  @return client port number.
    int16  GetClientPort() { return ntohs(m_stClientSockaddr.sin_port); };

    /// Returns server Internet host address as a string in standard numbers-and-dots notation.
	///  @return NULL if invalid
    uint8 *GetServerAddr() { return (uint8 *)inet_ntoa(m_stServerSockaddr.sin_addr); };

	/// Returns the port number on which the server is connected.
	///  @return server port number.
    int16  GetServerPort() { return ntohs(m_stServerSockaddr.sin_port); };

protected:
    /// Set internal socket error to that specified error
    ///  @param error type of error
    void SetSocketError(CSimpleSocket::CSocketError error) { m_socketErrno = error; };

    /// Set object socket handle to that specified as parameter 
    ///  @param socket value of socket descriptor
    void SetSocketHandle(SOCKET socket) { m_socket = socket; };

protected:
    SOCKET              m_socket;
    CSocketError        m_socketErrno;
    uint8              *m_pBuffer;
    int32               m_nSocketDomain;
    int32               m_nSocketType;
    int32               m_nBytesReceived;
    int32               m_nBytesSent;
    uint32              m_nFlags;
    bool                m_bIsBlocking;
    struct timeval      m_stConnectTimeout;
    struct timeval      m_stRecvTimeout;
    struct timeval      m_stSendTimeout;
    struct sockaddr_in  m_stServerSockaddr;
    struct sockaddr_in  m_stClientSockaddr;
    struct linger       m_stLinger; 
    CStatTimer          m_timer;
#ifdef WIN32
    WSADATA             m_hWSAData;
#endif
    fd_set              m_writeFds;		  
    fd_set              m_readFds;		  
    fd_set              m_errorFds;		 
};


#endif /*  __SOCKET_H__  */

