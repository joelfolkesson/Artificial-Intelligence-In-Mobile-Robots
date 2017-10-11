/*****************************************************************************/
/* File:        robot.h (Khepera Simulator)                                  */
/* Author:      Olivier MICHEL <om@alto.unice.fr>                            */
/* Date:        Thu Sep 21 14:39:05 1995                                     */
/* Description: robot header file                                            */
/*                                                                           */
/* Copyright (c) 1995                                                        */
/* Olivier MICHEL                                                            */
/* MAGE team, i3S laboratory,                                                */
/* CNRS, University of Nice - Sophia Antipolis, FRANCE                       */
/*                                                                           */
/* Permission is hereby granted to copy this package for free distribution.  */
/* The author's name and this copyright notice must be included in any copy. */
/* Commercial use is forbidden.                                              */
/*****************************************************************************/

#ifndef ROBOT_H
#define ROBOT_H


#define TIME_DELAY 20000   /* number of usecs to delay on each time step */

#define ENCODER_FACTOR 1.9  //--KL added this factor to calibrate encoders

//#define ENCODER_FACTOR 3.44/2  //--KL added this factor to calibrate encoders
                             //  to match real robot's encoders

#define SPEED_FACTOR 7.49     /* affects general speed of the simulated robot
														 * speed; 15.0 recommended; smaller is faster */
														 //--KL Changed this value


#define ROTATION_FACTOR 55.84 /* affects speed of rotation;
																* 100.0 recommended; smaller is faster */
														  //--KL Changed this value
#define MAX_ACCEL 256    /* max acceleration of the robot
													* in ((pulses)/256)/10ms; 256 works ok... */

#define NormRad(x) {while(x > M_PI) x-=2*M_PI; while(x < -M_PI) x+=2*M_PI;}

#define BUMP                     1
#define REAL_ROBOT_FLAG          2
#define DISTANCE_SENSOR_FLAG     4
#define LIGHT_SENSOR_FLAG        8
#define MOTOR_VALUES_FLAG        16

#define ROBOT_DIAMETER           (double)52.55 // robot diameter in mm
#define ROBOT_WHEEL_BASE         (double)52.55 // space between two wheels in mm
                                              //--KL changed from 50
#define WHEEL_DIAMETER           (double)41.2 //--KL changed from 13 to 15

#define SPEED_MODE 0
#define POSITION_MODE 1
#define PWM_MODE 2

#define DEG(x)            (x*57.2957795130823208767981548141)

struct Motor
{
  double    X,Y,Alpha;
	double Value;
  short TargetValue, Max, Acc, mode;
	int Pos, TargetPos;
};

struct IRSensor
{
  double    X,Y,Alpha;
  short int DistanceValue;
  short int LightValue;
};

struct Robot
{
  u_char                State;
  char                  Name[16];
  double                X,Y,Alpha; /* X and Y (millimeter), Alpha (rad) */
  double                Diameter;  /* (millimeter) */
  struct Motor          Motor[2];
  struct IRSensor       IRSensor[8];
};

#endif
