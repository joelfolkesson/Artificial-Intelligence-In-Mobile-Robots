//=============================================================================
//kh_linux.c
//=============================================================================


//=============================================================================
//Includes
//=============================================================================
#include <sys/time.h>
#include <stdio.h>
#include <string.h>
#include "UDPSocket/UDPSocket_C.h"
//=============================================================================


//=============================================================================
//Defines
//=============================================================================
//Tracing
#define TraceErrors       1
#define TraceSocket       0

//Socket settings
#define SEND_PORT         10000
#define RECV_PORT         10001
#define SIM_HOST          "localhost" //Host where simulator is running

#define SIM_DELAY         50

#define MAX_MSG_SIZE      1023
//=============================================================================


//=============================================================================
//Function Prototypes
//=============================================================================
//Open COM port
int Open_COM(int com_num);

//Close COM port
void Close_COM(void);

//Send message to robot
int SendMsg(char* send_msg);

//Get response from robot
int RecvMsg(char* recv_msg, int max_recv_len);

//Sleep for "ms" milliseconds
void Sleep(unsigned long ms);
//=============================================================================


//=============================================================================
//Communication Functions
//=============================================================================
int Open_COM(int com_num)
{
  //Init socket properties
  if (InitSendUDP(SIM_HOST,SEND_PORT) != 0)
  {
    #if (TraceErrors || TraceSocket)
    printf("Could not initialise socket's send settings!\n");
    #endif
    return -1;
  }

  //Init socket properties
  if (InitRecvUDP(RECV_PORT) != 0)
  {
    #if (TraceErrors || TraceSocket)
    printf("Could not initialise socket's receive settings!\n");
    #endif
    return -1;
  }

  #if TraceSocket
  printf("Socket initialised!\n");
  #endif

  return 0;
}
//=============================================================================
void Close_COM(void)
{
  #if TraceSocket
  printf("Socket closed!\n");
  #endif
}
//=============================================================================
int SendMsg(char *send_msg)
{
  //String must be zero terminated
  int length = strlen(send_msg);

  //Discard empty messages
  if (length == 0)
    return 0;

  //Convert restart message into reset encoders
  if (strcmp(send_msg,"restart") == 0)
  {
    //Delay to simulate real robot
    Sleep(SIM_DELAY);
    strcpy(send_msg,"G,0,0");
  }

  #if TraceSocket
  printf("Sending %d bytes: %s\n",length,send_msg);
  #endif

  //Delay to simulate real robot
  Sleep(SIM_DELAY);

  if (SendUDP((unsigned char*)send_msg,length) == 0)
  {
    #if (TraceErrors || TraceSocket)
    printf("Error sending message!\n");
    #endif
    return 0;
  }

  return length;
}
//=============================================================================
int RecvMsg(char *recv_msg, int max_recv_len)
{
  char string[MAX_MSG_SIZE];
  int num, temp;

  if (max_recv_len > MAX_MSG_SIZE)
    max_recv_len = MAX_MSG_SIZE;

  #if TraceSocket
  printf("Receiving up to %d bytes...\n", max_recv_len);
  #endif

  num = RecvUDP((unsigned char*)string,max_recv_len);

  if (num <= 0)
  {
    #if (TraceErrors || TraceSocket)
    printf("Error receiving message!\n");
    #endif
    string[0] = '\0';
    return 0;
  }

  if (num > max_recv_len)
  {
    #if (TraceErrors || TraceSocket)
    printf("Receive string not long enough (read %d bytes, max was %d)!\n", num, max_recv_len);
    #endif
    string[0] = '\0';
    return 0;
  }

  //Delay to simulate real robot
  Sleep(SIM_DELAY);

  while(PeekRecv())
  {
    temp = RecvUDP((unsigned char*)&string[num], max_recv_len);
    num+=temp;

    if (num > max_recv_len)
    {
      #if (TraceErrors || TraceSocket)
      printf("Receive string too short (read %d bytes, max was %d)!\n", num, max_recv_len);
      #endif
      string[0] = '\0';
      return 0;
    }
  }

  string[num] = '\0';

  #if TraceSocket
  printf("Received %d byte(s): %s\n",num, string);
  #endif

  strcpy(recv_msg,string);

  return num;
}
//=============================================================================
void Sleep(unsigned long ms)
{
  struct timeval time;
  double start_time, elapsed_time, sleep_time;

  //Convert ms to s
  sleep_time = (double)(ms)/1000.0;

  //Get time of day, initialise start_time (in s)
  gettimeofday(&time,NULL);
  start_time = time.tv_sec + time.tv_usec/1000000.0;

  do
  {
    gettimeofday(&time,NULL);
    elapsed_time = (time.tv_sec + time.tv_usec/1000000.0) - start_time;
  }
  while (elapsed_time < sleep_time);
}
//=============================================================================
