//=============================================================================
//UDPSocket_C.c
//=============================================================================


//=============================================================================
//Includes
//=============================================================================
#include "UDPSocket_C.h"
//=============================================================================


//=============================================================================
//Globals
//=============================================================================
//Send info
static int send_handle;    //Handle to send socket
static int send_init_done; //Indicates if send socket is initialised
static int send_port;      //Current send port
static char send_ip[1024]; //String containing current remote host/IP
static struct sockaddr_in send_remote_sockaddr;  //Remote socket info (send to)
static struct hostent* send_remote_host_info;

//Receive info
static int recv_handle;    //Handle to recv socket
static int recv_init_done; //Indicates if recv socket is initialised
static int recv_port;      //Current recv port
static int have_received_something;  //Indicates if data has been received
static struct sockaddr_in recv_local_sockaddr;   //Local socket info
static struct sockaddr_in recv_remote_sockaddr;  //Remote socket info (received)
static char recv_ip[1024]; //IP of last remote sender
//=============================================================================


//=============================================================================
//Interface Functions
//=============================================================================
int InitSendUDP(char* ip, unsigned short port)
{
  #if (TraceInit || TraceSend)
  printf("UDPSocket: InitSend: Doing send init...\n");
  #endif

  send_handle = -1;
  send_init_done = 0;
  send_port = -1;
  send_ip[0] = '\0';

  //Copy ip and port to object variables
  sprintf(send_ip,"%s", ip);
  send_port = port;

  //Create socket
  send_handle = socket(AF_INET, SOCK_DGRAM, 0);
  if (send_handle < 0)
  {
    #if (TraceErrors || TraceSend || TraceInit)
    printf("UDPSocket: InitSend: Error creating socket...\n");
    #endif
    return -1;
  }

  //Initialise remote host info
  bzero(&send_remote_sockaddr,sizeof(send_remote_sockaddr));
  send_remote_sockaddr.sin_family=AF_INET;
  send_remote_sockaddr.sin_port=htons(port);
  send_remote_host_info = gethostbyname(ip);
  if (send_remote_host_info == NULL)
  {
    #if (TraceErrors || TraceSend || TraceInit)
    printf("UDPSocket: InitSend: Error finding host...\n");
    #endif
    return -1;
  }
  bcopy((char *)send_remote_host_info->h_addr,
        (char *)&send_remote_sockaddr.sin_addr,
        send_remote_host_info->h_length);

  //Indicate this is done
  send_init_done = 1;

  #if (TraceInit || TraceSend)
  printf("UDPSocket: InitSend: Done send init...\n");
  #endif

  return 0;
}
//=============================================================================
int SendUDPTo(char* ip, unsigned short port, unsigned char* send_data, int length)
{
  int num; //Number of bytes sent
  int size = sizeof(send_remote_sockaddr);

  //If send isn't initialised, do it now
  if ((!send_init_done) || (strcmp(ip, send_ip) != 0) || (send_port != port))
    InitSendUDP(ip, port);

  //Send data
  #if TraceSend
  printf("UDPSocket: Send: Sending %d bytes of data (%s)\n",
         length,send_data);
  #endif
  num = sendto(send_handle, send_data, length, 0,
               (struct sockaddr*)&send_remote_sockaddr,
               size);
  if (num < 0)
  {
    #if (TraceErrors || TraceSend)
    printf("UDPSocket: Send: Error sending data...\n");
    #endif
    return -1;
  }

  #if TraceSend
  printf("UDPSocket: Send: Sent %d bytes of data (%s)\n",
         length,send_data);
  #endif

  return num;
}
//=============================================================================
int SendUDP(unsigned char* send_data, int length)
{
  if (!send_init_done)
  {
    #if (TraceErrors || TraceSend)
    printf("UDPSocket: Send: Error - destination IP and port not set...\n");
    #endif
    return -1;
  }

  return SendUDPTo(send_ip, send_port, send_data, length);
}
//=============================================================================
int InitRecvUDP(unsigned short port)
{
  #if (TraceInit || TraceRecv)
  printf("UDPSocket: InitRecv: Doing recv init (port %d)...\n", (unsigned short) port);
  #endif

  recv_handle = -1;
  recv_init_done = 0;
  recv_port = -1;
  have_received_something = 0;
  recv_ip[0] = '\0';

  //Copy port to object variable
  recv_port = port;

  //If this socket already exists close old one - this allows
  //re-initialisation of the socket with new receive port...
  //if (recv_init_done)
  //  shutdown(recv_handle, 2);

  //Create socket
  recv_handle = socket(AF_INET, SOCK_DGRAM, 0);
  if (recv_handle < 0)
  {
    #if (TraceErrors || TraceRecv || TraceInit)
    printf("UDPSocket: InitRecv: Error creating socket...\n");
    #endif
    return -1;
  }

  //Zero structures
  bzero(&recv_local_sockaddr,sizeof(recv_local_sockaddr));

  //Set local_sockaddr
  recv_local_sockaddr.sin_family=AF_INET;
  recv_local_sockaddr.sin_addr.s_addr=INADDR_ANY;
  recv_local_sockaddr.sin_port=htons(port);

  //Bind socket
  if (bind(recv_handle,(struct sockaddr *)&recv_local_sockaddr,sizeof(recv_local_sockaddr))   < 0)
  {
    #if (TraceErrors || TraceRecv || TraceInit)
    printf("UDPSocket: InitRecv: Error binding socket...\n");
    #endif
    return -1;
  }

  //Indicate this is done
  recv_init_done = 1;

  #if (TraceInit || TraceRecv)
  printf("UDPSocket: InitRecv: Done recv init...\n");
  #endif

  return 0;
}
//=============================================================================
int RecvUDPOn(unsigned short port, unsigned char* recv_data, int max_length)
{
  int num;
  socklen_t size = sizeof(recv_remote_sockaddr);

  //If recv isn't initialised, do it now
  if ((!recv_init_done) || (recv_port != port))
  {

    #if (TraceRecv || TraceInit)
    printf("UDPSocket: Recv: Recv init not done yet - doing it now...\n");
    #endif
    if (InitRecvUDP(port) != 0)
    {
      #if (TraceRecv || TraceInit || TraceErrors)
      printf("UDPSocket: Recv: Error performing recv init...\n");
      #endif
      return -1;
    }
  }

  //Receive data
  #if TraceRecv
  printf("UDPSocket: Recv: Waiting for data on port %d...\n", port);
  #endif
  num = recvfrom(recv_handle,recv_data,max_length,0,
                 (struct sockaddr *)&recv_remote_sockaddr,
                 &size);

  if (num < 0)
  {
    #if (TraceErrors || TraceRecv)
    printf("UDPSocket: Recv: Error receiving data...\n");
    #endif
    return -1;
  }

  //Null terminate the received data, just in case
  recv_data[num] = '\0';
  have_received_something = 1;
  sprintf(recv_ip,"%s",inet_ntoa(recv_remote_sockaddr.sin_addr));

  #if TraceRecv
  printf("UDPSocket: Recv: Received %d bytes of data (%s)\n",num,recv_data);
  #endif

  return num;
}
//=============================================================================
int RecvUDP(unsigned char* recv_data, int max_length)
{
  if (!recv_init_done)
  {
    #if (TraceErrors || TraceSend)
    printf("UDPSocket: Recv: Error - receive port not set...\n");
    #endif
    return -1;
  }

  return RecvUDPOn(recv_port, recv_data, max_length);
}
//=============================================================================
int GetSendIP(char *ip, int max_length)
{
  int i;

  if (!send_init_done)
  {
    #if TraceErrors
    printf("UDPSocket: GetSendIP: Error - send IP not set...\n");
    #endif
    return -1;
  }

  for (i=0;i<max_length;i++)
  {
    ip[i]=send_ip[i];
    if (ip[i] == '\0')
      break;
  }

  if (ip[i] != '\0')
  {
    ip[i] = '\0';
    return -1; //signal incomplete copy
  }

  return 0;
}
//=============================================================================
int GetSendPort(void)
{
  if (!send_init_done)
  {
    #if TraceErrors
    printf("UDPSocket: GetSendPort: Error - send port not set...\n");
    #endif
    return -1;
  }

  return send_port;
}
//=============================================================================
int GetRecvPort(void)
{
  if (!recv_init_done)
  {
    #if TraceErrors
    printf("UDPSocket: GetRecvPort: Error - recv port not set...\n");
    #endif
    return -1;
  }

  return recv_port;
}
//=============================================================================
int HaveReceivedSomething(void)
{
  return have_received_something;
}
//=============================================================================
int LastPacketReceivedFrom(char *ip, int max_length)
{
  int i;

  if (!have_received_something)
  {
    #if TraceErrors
    printf("UDPSocket: LastPacketReceivedFrom: Error - nothing received...\n");
    #endif
    return -1;
  }

  for (i=0;i<max_length;i++)
  {
    ip[i]=recv_ip[i];
    if (ip[i] == '\0')
      break;
  }

  if (ip[i] != '\0')
  {
    ip[i] = '\0';
    return -1; //signal incomplete copy
  }

  return 0;
}
//=============================================================================
int GetRecvHandle(void)
{
  return recv_handle;
}
//=============================================================================
int GetSendHandle(void)
{
  return send_handle;
}
//=============================================================================
int PeekRecv(void)
{
  fd_set readfds;     //List of file descriptors for select to listen to
  struct timeval timeout;

  FD_ZERO(&readfds);  //Initialize readfds
  FD_SET(recv_handle,&readfds);  //Set readfds to include udp_socket
  timeout.tv_sec = 0;  //Set timeout to zero so select polls socket
  timeout.tv_usec = 0;

  //If there is data on socket, execute the message, otherwise just skip it
  if (select(FD_SETSIZE,&readfds,(fd_set *)NULL,(fd_set *)NULL,&timeout) > 0)
    return 1;

  return 0;
}
//=============================================================================
