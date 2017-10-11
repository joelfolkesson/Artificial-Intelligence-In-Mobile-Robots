//=============================================================================
//kh.c
//=============================================================================


//=============================================================================
//Includes
//=============================================================================
#include "epuck.h"
//=============================================================================


//=============================================================================
//Variables
//=============================================================================
Posture posture; // Global posture
Sensors ir;      // IR sensors
Sensors ambient; // Ambient light sensors
int     led1;    // LED1 state
int     led2;    // LED2 state
//=============================================================================


//=============================================================================
//Interface functions
//=============================================================================
void epuck(int com_num)
{
  if (Open_COM(com_num) == -1)
  {
    #if TraceErrors
    printf("Error opening COM port %d - Exiting...\n", com_num);
    #endif
  }

  Restart();

}
//=============================================================================
void Restart(void)
{
  char recv_msg[LARGE_MSG_SIZE];

  SendMsg("restart\r");
  RecvMsg(recv_msg,LARGE_MSG_SIZE);

  ClearSteps();
  ClearSensor(IR|AMBIENT);

  posture.x = 0.0;
  posture.y = 0.0;
  posture.th = 0.0;

  led1 = OFF;
  led2 = OFF;
}
//=============================================================================
void SetPosture(double x, double y, double th)
{
  posture.x = x;
  posture.y = y;
  posture.th = th;
}
//=============================================================================
void SetPosition(double x, double y)
{
  posture.x = x;
  posture.y = y;
}
//=============================================================================
Posture GetPosture(void)
{
  return posture;
}
//=============================================================================
int Talk(char* send_msg, char* recv_msg, int max_recv_len)
{
  int length;
  int tries = 0;
  char string[LARGE_MSG_SIZE];

  max_recv_len = MIN(max_recv_len, LARGE_MSG_SIZE);

  SendMsg(send_msg);
  length = RecvMsg(string, max_recv_len);

  if (length > max_recv_len)
  {
    #if TraceErrors
    printf("Receive string not long enough!\n");
    #endif
    recv_msg[0] = '\0';
    return -1;
  }

  while ((tolower(send_msg[0]) != string[0]) && (tries < RETRY_SEND))
  {
    tries++;
    SendMsg(send_msg);
    length = RecvMsg(string, max_recv_len);
  }

  if (length > max_recv_len)
  {
    #if TraceErrors
    printf("Receive string not long enough!\n");
    #endif
    recv_msg[0] = '\0';
    return -1;
  }

  if (tolower(send_msg[0]) != string[0])
  {
    #if TraceErrors
    printf("Invalid response to message!\n");
    #endif
    recv_msg[0] = '\0';
    return -1;
  }

  strcpy(recv_msg, string);

  return 1;
}
//=============================================================================
Steps GetSteps(void)
{
  char recv_msg[SMALL_MSG_SIZE],temp;
  Steps enc = {0,0};

  if (Talk("H\r", recv_msg, SMALL_MSG_SIZE) == -1)
  {
    #if TraceErrors
    printf("Invalid response to GetSteps()!\n");
    #endif
    return enc;
  }
  sscanf(recv_msg, "%c,%d,%d",&temp,&enc.l,&enc.r);

  return enc;
}

/*
Posture GetPosture2(void)
{
  char recv_msg[SMALL_MSG_SIZE],temp;
  Posture pos = {0.0,0.0,0.0};

double a,b,c;
  if (Talk("I\r", recv_msg, SMALL_MSG_SIZE) == -1)
  {
    #if TraceErrors
    printf("Invalid response to GetSteps()!\n");
    #endif
    return pos;
  }
  sscanf(recv_msg, "%c,%lf,%lf %lf",&temp,&a,&b,c);
printf("pos= %lf  %lf  %lf \n",a,b,c );


  return pos;
}
*/
//=============================================================================
Speed GetSpeed(void)
{
  char  recv_msg[SMALL_MSG_SIZE],temp;
  Speed s = {0,0};

  if (Talk("E\r", recv_msg, SMALL_MSG_SIZE) == -1)
  {
    #if TraceErrors
    printf("Invalid response to GetSpeed()!\n");
    #endif
    return s;
  }
  sscanf(recv_msg, "%c,%d,%d",&temp,&s.l,&s.r);

s.l*=100;
s.r*=100;
  return s;
}
//=============================================================================
Speed GetSpeed_mm(void)
{
  Speed s;
  s = GetSpeed();
  
  //Convert from pulses per 10ms to mm per second
  s.l = (int)rint(s.l*MM_PER_PULSE_L);
  s.r = (int)rint(s.r*MM_PER_PULSE_R);
  return s;
}
//=============================================================================
int SetSpeed(int left, int right)
{
left/=100;
right/=100;

  char send_msg[SMALL_MSG_SIZE], recv_msg[SMALL_MSG_SIZE];

  //Check for min/max pulses per 10ms
  left = MAX(MIN(left,MAX_SPEED),MIN_SPEED);
  right = MAX(MIN(right,MAX_SPEED),MIN_SPEED);


  sprintf(send_msg, "D,%d,%d\r", left, right);

  if (Talk(send_msg, recv_msg, SMALL_MSG_SIZE) == -1)
  {
    #if TraceErrors
    printf("Invalid response to SetSpeed(%d,%d)!\n",left,right);
    #endif
    return -1;
  }

  return 1;
}
//=============================================================================
int SetSpeed_mm(int left, int right)
{


  //Convert from mm per second to pulses per 10ms
  return SetSpeed((int)rint(left*PULSES_PER_MM_L),
                     (int)rint(right*PULSES_PER_MM_R));
}
//=============================================================================
int Stop(void)
{
  return SetSpeed(0,0);
}
//=============================================================================
int SetPolarSpeed(double vlin, double vrot)
{
  //Convert from mm/s and rads/s to pulses per 10ms
  return SetSpeed((int)rint(((PULSES_PER_MM_L*vlin) - 
                                     (vrot*ROBOT_RADIUS*PULSES_PER_MM_L))),
                     (int)rint(((PULSES_PER_MM_R*vlin) + 
                                     (vrot*ROBOT_RADIUS*PULSES_PER_MM_R))));
}
//=============================================================================
void GetPolarSpeed(double* vlin, double* vrot)
{
  Speed s;
  double l,r;
  s = GetSpeed();
  
  //Convert from pluses per 1s to mm/s and rads/s
  l = s.l*MM_PER_PULSE_L;
  r = s.r*MM_PER_PULSE_R;

  *vrot = (r-l)/55;
  *vlin = (r+l)*0.5;
}
//=============================================================================
int SetTargetSteps(int left, int right)
{
  //Send target encoder values to Khepera.  Robot will move until desired
  //encoder values are reached.

  char send_msg[SMALL_MSG_SIZE], recv_msg[SMALL_MSG_SIZE];

  sprintf(send_msg,"C,%d,%d\r", left, right);

if(Talk(send_msg, recv_msg, SMALL_MSG_SIZE) == -1)
  {
    #if TraceErrors
    printf("Invalid response to SetTargetSteps(%d,%d)!\n",left,right);
    #endif
    return -1;
  }

//TODO refine below lines
#define MAXSTEPS  9999999

if(left >= MAXSTEPS || right >=MAXSTEPS )
  {
    printf("Max step limit exceed in SetTargetSteps(%d,%d)!\n",left,right);
    return -1;
  }

  Steps steps;
  steps.l = steps.r=MAXSTEPS;

while (left != steps.l && right != steps.r)
  steps = GetSteps();

  return 1;
}

//=============================================================================
int ClearSteps(void)
{
  char recv_msg[SMALL_MSG_SIZE];

  if (Talk("G,0,0\r", recv_msg, SMALL_MSG_SIZE) == -1)
  {
    #if TraceErrors
    printf("Invalid response to ClearSteps()!\n");
    #endif
    return -1;
  }
  return 1;
}
//=============================================================================
int SetSteps(int left, int right)
{
  char send_msg[SMALL_MSG_SIZE], recv_msg[SMALL_MSG_SIZE];
  sprintf(send_msg,"G,%d,%d\r", left, right);

  if (Talk(send_msg, recv_msg, SMALL_MSG_SIZE) == -1)
  {
    #if TraceErrors
    printf("Invalid response to SetSteps(%d,%d)!\n",left,right);
    #endif
    return -1;
  }
  return 1;
}
//=============================================================================


//=============================================================================
//Sensors
//=============================================================================
Sensors GetIR(void)
{
  char recv_msg[SMALL_MSG_SIZE], temp;
  int num;

  if (Talk("N\r", recv_msg, SMALL_MSG_SIZE) == -1)
  {
    #if TraceErrors
    printf("Invalid response to GetIR()!\n");
    #endif
    return ir;
  }

  num = sscanf(recv_msg,"%c,%d,%d,%d,%d,%d,%d,%d,%d",&temp,
               &ir.sensor[0],&ir.sensor[1],&ir.sensor[2],
               &ir.sensor[3],&ir.sensor[4],&ir.sensor[5],
               &ir.sensor[6],&ir.sensor[7]);

  if (num != 9)
  {
    #if TraceErrors
    printf("Invalid response to GetIR()!\n");
    #endif
    return ir;
  }

  return ir;
}
//=============================================================================
Sensors GetAMB(void)
{
  char recv_msg[SMALL_MSG_SIZE], temp;
  int num;

  if (Talk("O\r", recv_msg, SMALL_MSG_SIZE) == -1)
  {
    #if TraceErrors
    printf("Invalid response to GetAMB()!\n");
    #endif
    return ambient;
  }

  num = sscanf(recv_msg,"%c,%d,%d,%d,%d,%d,%d,%d,%d",&temp,
               &ambient.sensor[0],&ambient.sensor[1],&ambient.sensor[2],
               &ambient.sensor[3],&ambient.sensor[4],&ambient.sensor[5],
               &ambient.sensor[6],&ambient.sensor[7]);

  if (num != 9)
  {
    #if TraceErrors
    printf("Invalid response to GetAMB()!\n");
    #endif
    return ambient;
  }

  return ambient;
}
//=============================================================================
void ClearSensor(int val)
{
  int i;

  for(i=0; i<8; i++)
  {
    if ((val==IR)||(val==(IR|AMBIENT)))
      ir.sensor[i] = 0;
    if ((val==AMBIENT)||(val==(IR|AMBIENT)))
      ambient.sensor[i] = 0;
  }
}
//=============================================================================


//=============================================================================
//LED control
//=============================================================================
int LedGetState(int num)
{
  switch (num)
  {
    case 0:
      return led1;
    case 1:
      return led2;
  }

	#if TraceErrors
	printf("Invalid LED number.\n");
	#endif
	return -1;
}
//=============================================================================
int LedOn(int num)
{
  char send_msg[SMALL_MSG_SIZE], recv_msg[SMALL_MSG_SIZE];

  switch (num)
  {
    case 0:
      led1 = ON;
      break;
    case 1:
      led2 = ON;
      break;
    default:
      #if TraceErrors
      printf("Invalid LED number.\n");
      #endif
      return -1;
  }

  sprintf(send_msg, "L,%d,1\r", num);

  if (Talk(send_msg, recv_msg, SMALL_MSG_SIZE) == -1)
  {
    #if TraceErrors
    printf("Invalid response to LedOn()!\n");
    #endif
    return -1;
  }

  return 1;
}
//=============================================================================
int LedOff(int num)
{
  char send_msg[SMALL_MSG_SIZE], recv_msg[SMALL_MSG_SIZE];

  switch (num)
  {
    case 0:
      led1 = OFF;
      break;
    case 1:
      led2 = OFF;
      break;
    default:
      #if TraceErrors
      printf("Invalid LED number.\n");
      #endif
      return -1;
  }

  sprintf(send_msg, "L,%d,0\r", num);
  if (Talk(send_msg, recv_msg, SMALL_MSG_SIZE) == -1)
  {
    #if TraceErrors
    printf("Invalid response to LedOff()!\n");
    #endif
    return -1;
  }

  return 1;
}
//=============================================================================
int LedToggleState(int num)
{
  char send_msg[SMALL_MSG_SIZE], recv_msg[SMALL_MSG_SIZE];

  switch (num)
  {
    case 0:
      if (led1 == OFF)
        led1 = ON;
      else
        led1 = OFF;
      break;
    case 1:
      if (led2 == OFF)
        led2 = ON;
      else
        led2 = OFF;
      break;
    default:
      #if TraceErrors
      printf("Invalid LED number.\n");
      #endif
      return -1;
  }

  sprintf(send_msg, "L,%d,2\r", num);

  if (Talk(send_msg, recv_msg, SMALL_MSG_SIZE) == -1)
  {
    #if TraceErrors
    printf("Invalid response to LedToggleState()!\n");
    #endif
    return -1;
  }

  return 1;
}
//=============================================================================


//=============================================================================
//Conversions
//=============================================================================
Steps enc2mm(Steps enc)
{
  Steps enc_mm;
  enc_mm.l = (int)rint(enc.l * MM_PER_PULSE_L);
  enc_mm.r = (int)rint(enc.r * MM_PER_PULSE_R);
  return enc_mm;
}
//=============================================================================
Steps mm2enc(Steps enc_mm)
{
  Steps enc;
  enc.l = (int)rint(enc_mm.l * PULSES_PER_MM_L);
  enc.r = (int)rint(enc_mm.r * PULSES_PER_MM_R);
  return enc;
}
//=============================================================================
double NORM_RAD(double angle)
{
  while(angle > M_PI)
    angle -= 2*M_PI;
  while(angle <= -M_PI)
    angle += 2*M_PI;
  return angle;
}
//=============================================================================
double NORM_DEG(double angle)
{
  while(angle > 180.0)
    angle -= 360.0;
  while(angle <= -180.0)
    angle += 360.0;
  return angle;
}
//=============================================================================
double DIST(double x1, double y1, double x2, double y2)
{
  return sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1));
}
//=============================================================================
