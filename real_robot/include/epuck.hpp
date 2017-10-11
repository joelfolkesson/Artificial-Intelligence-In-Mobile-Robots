#ifndef EPUCK_HPP
#define EPUCK_HPP

#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include"epuckLEDs.hpp"
#include <string.h>
#include"serialPort.hpp"
#include"epuckPosition2d.hpp"
#include"epuckIR.hpp"
#include <iostream>
#include <math.h>
#include <float.h>
#include "rfid.hpp"
#include <algorithm>
#include <unistd.h>
#include <time.h>  

#include "maps.h"
#include "lists.h"
#include "fuzzy.h"

//#define packet_size 41

//#define writing_length 36

/** \file
 * Header file of the epuck.cpp  (AIMR)
 */

  using namespace std;

/** 
 *    @name  Physical settings
 */
 //@{
#define ROBOT_DIAMETER    52.55 //Length of wheel axis

#define WHEEL_DIAMETER_L  41.1   //---
#define WHEEL_DIAMETER_R  41.1   //---

#define RADS_PER_PULSE_L  0.006283185  //2*PI/600
#define RADS_PER_PULSE_R  0.006283185  //2*PI/600
//@}

/** 
 *    @name  Derived physical settings
 */
 //@{
#define ROBOT_RADIUS      (ROBOT_DIAMETER*0.5) 
#define ROBOT_CIRCUMF     (ROBOT_DIAMETER*PI)  

#define WHEEL_RADIUS_L    (WHEEL_DIAMETER_L*0.5)   //---
#define WHEEL_RADIUS_R    (WHEEL_DIAMETER_R*0.5)   //---

#define WHEEL_CIRCUMF_L   (WHEEL_DIAMETER_L*PI)   //---
#define WHEEL_CIRCUMF_R   (WHEEL_DIAMETER_R*PI)   //---

#define PULSES_PER_RAD_L  (1.0/RADS_PER_PULSE_L)   //---
#define PULSES_PER_RAD_R  (1.0/RADS_PER_PULSE_R)   //---

#define PULSES_PER_REV_L  (PULSES_PER_RAD_L*2*PI)   //---
#define PULSES_PER_REV_R  (PULSES_PER_RAD_R*2*PI)   //---

#define REVS_PER_PULSE_L  (1.0/PULSES_PER_REV_L)   //---
#define REVS_PER_PULSE_R  (1.0/PULSES_PER_REV_R)   //---

#define MM_PER_PULSE_L    (WHEEL_RADIUS_L*RADS_PER_PULSE_L)   //---
#define MM_PER_PULSE_R    (WHEEL_RADIUS_R*RADS_PER_PULSE_R)   //---

#define MM_PER_PULSE_AVG  ((MM_PER_PULSE_L+MM_PER_PULSE_R)*0.5)   //---

#define PULSES_PER_MM_L   (1.0/MM_PER_PULSE_L)   //---
#define PULSES_PER_MM_R   (1.0/MM_PER_PULSE_R)   //---

#define PULSES_PER_MM_AVG ((PULSES_PER_MM_L+PULSES_PER_MM_R)*0.5)   //---
//@}


#define	RING_LEDS_NUM	8

/** 
 *    @name  Robot mas speeds in steps and in mm
 */
 //@{
#define MAX_SPEED_STEPS 1000
#define MAX_SPEED_MM 	160
//@}


/** 
 *    @name  Macros to convert numbers
 */
 //@{
#define SQRT_2            1.414213562373
#define PI                3.14159265358979323846264338327950
#define RAD(x)            (x*0.01745329251994329576923690768)
#define DEG(x)            (x*57.2957795130823208767981548141)
#define MIN(x,y)          (((x) > (y)) ? (y) : (x))
#define MAX(x,y)          (((x) < (y)) ? (y) : (x))
#define ABS(x)            (((x) > 0.0) ? (x) : -(x))
#define MM2M(x)			  (x/1000)
#define M2MM(x)			  (x*1000)


//@}

//=============================================================================
//Structures
//=============================================================================

/*!@struct*/ 
/*!@brief Contains robot posture*/ 
 typedef struct
{
  float x;
  float y;
  float th;
}Posture;

/*!@struct*/ 
/*!@brief Contains position in world*/ 
 typedef struct
{
  double x;
  double y;
}Position;

/*!@struct*/ 
/*!@brief Contains encoder values, and speed settings*/ 
 typedef struct
{
  int l;
  int r;
} Steps, Speed;

/*!@struct*/ 
/*!@brief Contains IR data from sensors*/  
 typedef struct
{
  unsigned int sensor[8];
} Sensors;
//=============================================================================



//=============================================================================
//Functions
//=============================================================================

/*!@brif Connect epuck with computer on a port*/ 
  void epuck(std::string port);

/*!@brief Set posture to (x,y,th)*/ 
 void SetPosture(float x, float y, float th); //==>

/*!@brief Set position to (x,y), does not change th*/ 
 void SetPosition(double x, double y);

/*!@brief Read current posture (x,y,th)*/ 
 Posture GetPosture(void); //==>

/*!@brief Set and get polar speed to (vlin, vrot)*/ 
void SetPolarSpeed(double vlin, double vrot); //==>

/*!@brief vlin is in mm/s, vrot is in rads/s*/ 
void GetPolarSpeed(double* vlin, double* vrot); //==>

/*!@brief Set encoder values to (left, right)*/ 
 void SetSteps(int left, int right); //==>

/*!@brief Read current encoder values*/ 
 Steps GetSteps(void); //==>

/*!@brief Read current speed, pulses per 10ms*/ 
 Speed GetSpeed(void); //==>

/*!@brief Set speed to (left, right), steps per sec*/ 
int SetSpeed(int left, int right); //==>

/*!@brief Set encoder values to (0,0)*/ 
 void ClearSteps(void);  //==>

/*!@brief Convert encoders to mm*/ 
 Steps enc2mm(Steps enc); //==>

/*!@brief Convert mm to encoders*/ 
 Steps mm2enc(Steps enc_mm); //==>

/*!@brief Read current speed, mm per second*/ 
 Speed GetSpeed_mm(void);   //==>

/*!@brief Set speed to (left, right), mm per second */
void SetSpeed_mm(int left, int right);

/*!@brief Set target encoder values (robot moves until target values reached)*/ 
 int SetTargetSteps(int left, int right);  //==>

//===

/*!@brief Read IR sensors*/ 
 Sensors GetIR(void);

//===

/*!@brief Set all eight ring LEDs */ 
 void SetRingLED (bool ringLED[RING_LEDS_NUM]);

/*!@brief Set single led at a time*/ 
 void SetSingleLED (unsigned id, bool state);

/*!@brief Set front led*/ 
 void SetFrontLED(bool state);

/*!@brief Compute distance between two points*/ 
 double DIST(double x1, double y1, double x2, double y2);

/*!@brief Stop robot (set speed to (0,0)*/ 
 void Stop(void);  //==>

/*!@brief Sleep for "ms" milliseconds*/ 
 void Sleep(unsigned long ms);  //==>

void getImage (void);

#endif
