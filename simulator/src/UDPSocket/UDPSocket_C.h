//=============================================================================
//UDPSocket_C.h
//=============================================================================
#ifndef _UDPSocket_C_h_DEFINED
#define _UDPSocket_C_h_DEFINED
//=============================================================================


//=============================================================================
//Includes
//=============================================================================
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <stdio.h>
#include <string.h>
//=============================================================================


//=============================================================================
//Customizations
//=============================================================================
#define TraceErrors 1
#define TraceInit   0
#define TraceSend   0
#define TraceRecv   0
//=============================================================================


//=============================================================================
//Function Prototypes
//=============================================================================
//Interface Functions
int InitSendUDP(char* ip, unsigned short port);
int SendUDP(unsigned char* send_data, int length);
int SendUDPTo(char* ip, unsigned short port, unsigned char* send_data, int length);

int InitRecvUDP(unsigned short port);
int RecvUDP(unsigned char* recv_data, int max_length);
int RecvUDPOn(unsigned short port, unsigned char* recv_data, int max_length);

int GetSendIP(char *ip, int max_length);
int GetSendPort(void);
int GetRecvPort(void);

int HaveReceivedSomething(void);
int LastPacketReceivedFrom(char *ip, int max_length);
int GetRecvHandle(void);
int GetSendHandle(void);

int PeekRecv(void);
//=============================================================================


//=============================================================================
#endif //#ifndef _UDPSocket_C_h_DEFINED
//=============================================================================
