//=============================================================================
//kh.h
//=============================================================================
#ifndef kh_h_DEFINED
#define kh_h_DEFINED
//=============================================================================


//=============================================================================
//Includes
//=============================================================================
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <ctype.h>
#include <string.h>
//=============================================================================


//=============================================================================
//Defines
//=============================================================================
//Tracing
#define TraceErrors       1

//Macros to convert numbers
#define SQRT_2            1.414213562373
#define PI                3.14159265358979323846264338327950
#define RAD(x)            (x*0.01745329251994329576923690768)
#define DEG(x)            (x*57.2957795130823208767981548141)
#define MIN(x,y)          (((x) > (y)) ? (y) : (x))
#define MAX(x,y)          (((x) < (y)) ? (y) : (x))
#define ABS(x)            (((x) > 0.0) ? (x) : -(x))
//======================================================


//Message settings
#define LARGE_MSG_SIZE    1023
#define SMALL_MSG_SIZE    128
#define RETRY_SEND        10

//Speed settings
#define MAX_SPEED         1000           //pulses per 10ms
#define MIN_SPEED         (-MAX_SPEED)

//Physical settings
#define ROBOT_DIAMETER    52.55 //Length of wheel axis
#define WHEEL_DIAMETER_L  41.1
#define WHEEL_DIAMETER_R  41.1
#define RADS_PER_PULSE_L  2*PI/1000   //0.006283185  //2*PI/1000
#define RADS_PER_PULSE_R  2*PI/1000   //0.006283185  //2*PI/1000

//Derived physical settings
#define ROBOT_RADIUS      (ROBOT_DIAMETER*0.5)
#define ROBOT_CIRCUMF     (ROBOT_DIAMETER*PI)
#define WHEEL_RADIUS_L    (WHEEL_DIAMETER_L*0.5)
#define WHEEL_RADIUS_R    (WHEEL_DIAMETER_R*0.5)
#define WHEEL_CIRCUMF_L   (WHEEL_DIAMETER_L*PI)
#define WHEEL_CIRCUMF_R   (WHEEL_DIAMETER_R*PI)
#define PULSES_PER_RAD_L  (1.0/RADS_PER_PULSE_L)
#define PULSES_PER_RAD_R  (1.0/RADS_PER_PULSE_R)
#define PULSES_PER_REV_L  (PULSES_PER_RAD_L*2*PI)
#define PULSES_PER_REV_R  (PULSES_PER_RAD_R*2*PI)
#define REVS_PER_PULSE_L  (1.0/PULSES_PER_REV_L)
#define REVS_PER_PULSE_R  (1.0/PULSES_PER_REV_R)
#define MM_PER_PULSE_L    (WHEEL_RADIUS_L*RADS_PER_PULSE_L)
#define MM_PER_PULSE_R    (WHEEL_RADIUS_R*RADS_PER_PULSE_R)
#define MM_PER_PULSE_AVG  ((MM_PER_PULSE_L+MM_PER_PULSE_R)*0.5)
#define PULSES_PER_MM_L   (1.0/MM_PER_PULSE_L)
#define PULSES_PER_MM_R   (1.0/MM_PER_PULSE_R)
#define PULSES_PER_MM_AVG ((PULSES_PER_MM_L+PULSES_PER_MM_R)*0.5)

//Sensor identifiers
#define IR                1
#define AMBIENT           2

//LED states
#define ON                1
#define OFF               0

//LEDs
#define LED1              0
#define LED2              1

//Macros to convert numbers
#define SQRT_2            1.414213562373
#define PI                3.14159265358979323846264338327950
#define RAD(x)            (x*0.01745329251994329576923690768)
#define DEG(x)            (x*57.2957795130823208767981548141)
#define MIN(x,y)          (((x) > (y)) ? (y) : (x))
#define MAX(x,y)          (((x) < (y)) ? (y) : (x))
#define ABS(x)            (((x) > 0.0) ? (x) : -(x))
//=============================================================================


//=============================================================================
//Structures
//=============================================================================
//Contains robot posture
typedef struct
{
  double x;
  double y;
  double th;
} Posture;

//Position in world
typedef struct
{
  double x;
  double y;
} Position;

//Contains encoder values, and speed settings
typedef struct
{
  int l;
  int r;
} Steps, Speed;

//Contains IR data from sensors
typedef struct
{
  unsigned int sensor[8];
} Sensors;
//=============================================================================


//=============================================================================
//Functions
//=============================================================================


//-----------------------------------------------------------------------------
//Declared in kh_linux.c, kh_windows.c, kh_sim_linux.c or kh_sim_windows.c
//-----------------------------------------------------------------------------
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
//-----------------------------------------------------------------------------


//-----------------------------------------------------------------------------
//Declared in kh.c
//-----------------------------------------------------------------------------
//Open COM port and set posture to (0,0,0)
void epuck(int com_num);

//Restart robot and set posture to (0,0,0)
void Restart(void);

//Send message to robot and read response
int Talk(char* send_msg, char* recv_msg, int max_recv_len);

//Set posture to (x,y,th)
void SetPosture(double x, double y, double th);

//Set posture to (x,y), does not change th
void SetPosition(double x, double y);

//Read current posture
Posture GetPosture(void);
Posture GetPosture2(void);


//Read current encoder values
Steps GetSteps(void);

//Set target encoder values (robot moves until target values reached)
int SetTargetSteps(int left, int right);

//Set encoder values to (0,0)
int ClearSteps(void);

//Set encoder values to (left, right)
int SetSteps(int left, int right);

//Convert steps to mm
Steps enc2mm(Steps enc);

//Convert mm to steps
Steps mm2enc(Steps enc_mm);

//Read current speed, pulses per 10ms
Speed GetSpeed(void);

//Read current speed, mm per second
Speed GetSpeed_mm(void);

//Set speed to (left, right), steps per s
int SetSpeed(int left, int right);

//Set speed to (left, right), mm per second
int SetSpeed_mm(int left, int right);

//Stop robot (set speed to (0,0)
int Stop(void);

//Set and get polar speed to (vlin, vrot)
//vlin is in mm/s, vrot is in rads/s
int SetPolarSpeed(double vlin, double vrot);
void GetPolarSpeed(double* vlin, double* vrot);

//Read IR sensors
Sensors GetIR(void);

//Read ambient light sensors
Sensors GetAMB(void);

//Clear internal sensor struct
void ClearSensor(int val);

//Control LEDs (LED1 and LED2)
int LedOn(int num);
int LedOff(int num);
int LedToggleState(int num);
int LedGetState(int num);

//Normalise angles (radians and degrees), compute distances
double NORM_RAD(double angle);
double NORM_DEG(double angle);
double DIST(double x1, double y1, double x2, double y2);
//-----------------------------------------------------------------------------

//=============================================================================


//=============================================================================
#endif //#ifndef kh_h_DEFINED
//=============================================================================
