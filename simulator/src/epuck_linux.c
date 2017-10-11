//=============================================================================
//kh_linux.c
//=============================================================================


//=============================================================================
//Includes
//=============================================================================
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/time.h>
#include <stdio.h>
#include <string.h>
//=============================================================================


//=============================================================================
//Defines
//=============================================================================
//Tracing
#define TraceErrors       1
#define TraceCOM          0

//COM port settings
#define IN_BUF_SIZE       200
#define BAUD_RATE         B38400
#define MAX_MSG_SIZE      1023
//=============================================================================


//=============================================================================
//Variables
//=============================================================================
struct termios  KH_PortSettings;
struct termios  KH_OldPortSettings;
struct termios  KH_TTYSettings;
struct termios  KH_OldTTYSettings;
int             KH_Port;
//=============================================================================


//=============================================================================
//Function Prototypes
//=============================================================================
//Open COM port
int KH_Open_COM(int com_num);

//Close COM port
void KH_Close_COM(void);

//Send message to robot
int KH_SendMsg(char* send_msg);

//Get response from robot
int KH_RecvMsg(char* recv_msg, int max_recv_len);

//Sleep for "ms" milliseconds
void KH_Sleep(unsigned long ms);
//=============================================================================


//=============================================================================
//Communication Functions
//=============================================================================
int KH_Open_COM(int com_num)
{
  //Opens the COM port.  Returns 0 if ok, -1 on error.

  //Misc
  int errors = 0;

  //Com port stuff
  char port_name[32];

  //Check which port should be used
  sprintf(port_name, "%s%d", "/dev/ttyS",com_num-1);

  //Open port
  KH_Port = open(port_name, O_RDWR | O_NOCTTY);

  //Check port
  if (KH_Port < 0)
  {
    #if (TraceErrors || TraceCOM)
    printf("Cannot open COM port - exiting!\n");
    //perror("COM Error");
    #endif
    return -1;
  }
  else
  {
    #if TraceCOM
    printf("COM%d successfully opened!\n",com_num);
    #endif
  }

  //Save old COM status
  if ((tcgetattr(KH_Port, &KH_OldPortSettings)) == -1)
  {
    errors++;
    #if (TraceErrors || TraceCOM)
    printf("Cannot read current COM status!\n");
    #endif
  }

  //Edit COM status
  bzero(&KH_PortSettings,sizeof(KH_PortSettings));

  KH_PortSettings.c_iflag = IGNPAR;
  //KH_PortSettings.c_iflag |= IGNBRK|~BRKINT|IGNPAR|~PARMRK|~INPCK|
  //                          ~ISTRIP|~INLCR|~IGNCR|~ICRNL|~IUCLC|~IXON|
  //                          IXANY|~IXOFF|~IMAXBEL;

  KH_PortSettings.c_oflag = 0;
  //KH_PortSettings.c_oflag |= ~OPOST;

  KH_PortSettings.c_cflag = BAUD_RATE|CS8|CSTOPB|CREAD|CLOCAL;
  //KH_PortSettings.c_cflag |= BAUD_RATE|CS8|CSTOPB|CREAD|~PARENB|
  //                           ~PARODD|~HUPCL|CLOCAL|~LOBLK|~CRTSCTS;

  KH_PortSettings.c_lflag = 0;
  //KH_PortSettings.c_lflag |= ~ISIG|~ICANON|~XCASE|~ECHO|~IEXTEN;

  KH_PortSettings.c_cc[VTIME] = 1;
  KH_PortSettings.c_cc[VMIN] = IN_BUF_SIZE;

  /*
  printf("New settings:\n");
  printf(" c_iflag = %0x\n", KH_PortSettings.c_iflag);
  printf(" c_oflag = %0x\n", KH_PortSettings.c_oflag);
  printf(" c_cflag = %0x\n", KH_PortSettings.c_cflag);
  printf(" c_lflag = %0x\n", KH_PortSettings.c_lflag);
  */

  //Purge buffers
  if (tcflush(KH_Port, TCIOFLUSH) == -1)
  {
    errors++;
    #if (TraceErrors || TraceCOM)
    printf("Could not purge buffers!\n");
    #endif
  }

  //Set new COM status
  if (tcsetattr(KH_Port, TCSANOW, &KH_PortSettings) == -1)
  {
    errors++;
    #if (TraceErrors || TraceCOM)
    printf("Cannot set COM status!\n");
    #endif
  }

  //Check for errors
  if (errors == 0)
  {
    #if TraceCOM
    printf ("\nCOM%d successfully configured!\n\n", com_num);
    #endif
    return 0;
  }
  else
  {
    #if TraceCOM
    printf ("\nWarning: There were errors configuring COM%d!\n\n", com_num);
    #endif
    return -1;
  }
}
//=============================================================================
void KH_Close_COM(void)
{

  //Purge buffers
  if (tcflush(KH_Port, TCIOFLUSH) == -1)
  {
    #if (TraceErrors || TraceCOM)
    printf("Could not purge buffers!\n");
    #endif
  }

  //Restore old COM settings
  if (tcsetattr(KH_Port,TCSANOW,&KH_OldPortSettings) == -1)
  {
    #if (TraceErrors || TraceCOM)
    printf("Could not set COM settings!\n");
    #endif
  }

  if (KH_Port > 0)
    close(KH_Port);

  #if TraceCOM
  printf ("\nCOM port closed!\n\n");
  #endif
}
//=============================================================================
int KH_SendMsg(char *send_msg)
{
  //String must be zero terminated
  int length = strlen(send_msg);

  //Discard empty messages
  if (length == 0)
    return 0;

  #if TraceCOM
  printf("Sending %d bytes: %s\n",length,send_msg);
  #endif

  if (write(KH_Port,send_msg,length) < 0)
  {
    #if (TraceErrors || TraceCOM)
    printf("Error writing to COM port!\n");
    #endif
    return 0;
  }

  return length;
}
//=============================================================================
int KH_RecvMsg(char *recv_msg, int max_recv_len)
{
  char string[MAX_MSG_SIZE];
  int num, temp;

  if (max_recv_len > MAX_MSG_SIZE)
    max_recv_len = MAX_MSG_SIZE;

  #if TraceCOM
  printf("Receiving up to %d bytes...\n", max_recv_len);
  #endif

  num = read(KH_Port,string,IN_BUF_SIZE);

  if (num <= 0)
  {
    #if (TraceErrors || TraceCOM)
    printf("Error receiving message!\n");
    #endif
    string[0] = '\0';
    return 0;
  }

  if (num > max_recv_len)
  {
    #if TraceErrors || TraceCOM
    printf("Receive string not long enough (read %d bytes, max was %d)!\n", num, max_recv_len);
    #endif
    string[0] = '\0';
    return 0;
  }

  temp = num;
  while (temp >= IN_BUF_SIZE)
  {
    //Message was longer than buffer size...  Keep reading until buffer isn't full
    //temp = read(KH_Port,&string[num],max_recv_len - num);
    temp = read(KH_Port,&string[num],IN_BUF_SIZE);
    num += temp;

    if (num > max_recv_len)
    {
      #if TraceErrors || TraceCOM
      printf("Receive string too short (read %d bytes, max was %d)!\n", num, max_recv_len);
      #endif
      string[0] = '\0';
      return 0;
    }
  }

  string[num] = '\0';

  #if TraceCOM
  printf("Received %d byte(s): %s\n",num, string);
  #endif

  strcpy(recv_msg,string);

  return num;
}
//=============================================================================
void KH_Sleep(unsigned long ms)
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
