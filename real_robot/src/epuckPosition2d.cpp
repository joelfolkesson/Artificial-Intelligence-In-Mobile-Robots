/* Copyright 2008 Renato Florentino Garcia <fgar.renato@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 3, as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "epuckPosition2d.hpp"
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <iostream>
#include "epuck.hpp"
//#define PI       3.141592654
#define DOIS_PI  6.283185308
double xx;
double yy;
double tt;
double left;
double right;

EpuckPosition2d::EpuckPosition2d(const SerialPort* const serialPort)
  :EpuckInterface(serialPort)
{
   this->geometry.width  = EpuckInterface::EPUCK_DIAMETER;
   this->geometry.height = EpuckInterface::EPUCK_DIAMETER;
}

void
EpuckPosition2d::SetVel(float px, float pa) const
{
  // Angular speed for each wheel [rad/s]
  float angSpeedRw = ( 2*px + TRACK*pa )/( WHEEL_DIAMETER );
  float angSpeedLw = ( 2*px - TRACK*pa )/( WHEEL_DIAMETER );

  // Speed for each motor [steps/s]
  int stepsR = (int)( ( 1000*angSpeedRw )/ DOIS_PI );
  int stepsL = (int)( ( 1000*angSpeedLw )/ DOIS_PI );

  if (stepsR > 1000)
  {
    stepsR = 1000;
//     PLAYER_WARN("Rotational speed from epuck motor saturated in maximum value");

  }else if (stepsR < -1000)
  {
    stepsR = -1000;
//     PLAYER_WARN("Rotational speed from epuck motor saturated in maximum value");
  }

  if (stepsL > 1000)
  {
    stepsL = 1000;
//     PLAYER_WARN("Rotational speed from epuck motor saturated in maximum value");

  }else if (stepsL < -1000)
  {
    stepsL = -1000;
//     PLAYER_WARN("Rotational speed from epuck motor saturated in maximum value");
  }

  this->SendRequest(EpuckInterface::SET_VEL);

  this->serialPort->sendInt(stepsR);
  this->serialPort->sendInt(stepsL);

  this->serialPort->recvChar(); // Wait for e-puck to send an end of task signal.
}

void
EpuckPosition2d::SetOdometry(Triple odometry)
{
  // Send this message for side effect of reset the step counters on e-puck.
  this->SendRequest(EpuckInterface::GET_STEPS);
  this->serialPort->recvInt();
  this->serialPort->recvInt();

  // Overwrite the current odometric pose.
  this->odometryState.pose.x     = odometry.x;
  this->odometryState.pose.y     = odometry.y;
  this->odometryState.pose.theta = odometry.theta;

  xx    = odometry.x;
  yy     = odometry.y;
  tt = odometry.theta;

}


void
EpuckPosition2d::ResetOdometry()
{
  memset(&this->odometryState, 0, sizeof(DynamicConfiguration));
}

void
EpuckPosition2d::StopMotors() const
{
  this->SendRequest(EpuckInterface::STOP_MOTORS);

  this->serialPort->recvChar(); // Wait for e-puck to send an end of task signal.
}

EpuckPosition2d::DynamicConfiguration
EpuckPosition2d::UpdateOdometry()
{

static int lastRight= 0;
static int lastLeft = 0;

  this->SendRequest(EpuckInterface::GET_STEPS);
  int steps_right = this->serialPort->recvInt();
  int steps_left = this->serialPort->recvInt();

  int steps_right_new = (steps_right-lastRight);
  int steps_left_new = (steps_left - lastLeft);

  double delta_theta = r_DIV_L * STEP_ANG_DISP * (steps_right_new - steps_left_new);

  // Linear displacement, in direction of last theta
  double delta_l = r_DIV_2 * STEP_ANG_DISP * (steps_right_new + steps_left_new);

  // delta_l components on global system coordinates
  double delta_x = delta_l * cos(tt);
  double delta_y = delta_l * sin(tt);

  double deltaTime = this->timer.intervalDelay();



  this->timer.resetInterval();
  this->odometryState.velocity.x     += delta_x/deltaTime;
  this->odometryState.velocity.y     += delta_y/deltaTime;
  this->odometryState.velocity.theta += delta_theta/deltaTime;

  xx     += delta_x;
  yy     += delta_y;
  tt     += delta_theta;

  this->odometryState.pose.x     =xx;
  this->odometryState.pose.y     =yy;
  this->odometryState.pose.theta =tt;

  getsteps.right = steps_right_new;
  getsteps.left = steps_left_new;

  if(tt > PI)
  {
    tt -= DOIS_PI;
this->odometryState.pose.theta =tt;
  }
  if(tt < -PI)
  {
    tt += DOIS_PI;
this->odometryState.pose.theta =tt;
  }

lastRight =  steps_right;
 lastLeft = steps_left;

  return this->odometryState;
}

//======================================================================================//
//                                  new functions                                       //
//======================================================================================//



EpuckPosition2d::Steps EpuckPosition2d::GetSpeed(void)
{
 EpuckPosition2d::Steps speed;

 double deltaTime = ElapsedTime();
 StartInterval();

  this->SendRequest(EpuckInterface::GET_STEPS);
  int steps_right = this->serialPort->recvInt();
  int steps_left = this->serialPort->recvInt();


 if(deltaTime>0)
  {
  speed.right = (int) (((double)steps_right-(double)last_r)/deltaTime);
  speed.left = (int) (((double)steps_left -(double)last_l)/deltaTime);
  }
 else
  {
  speed.right =0.0;// (int) (((double)steps_right-(double)last_r)/1.0);
  speed.left =0.0;// (int) (((double)steps_left -(double)last_l)/1.0);
  }

 last_r=steps_right;
 last_l=steps_left;

 return speed;
}


EpuckPosition2d::Steps EpuckPosition2d::GetSteps(void)
{
  this->SendRequest(EpuckInterface::GET_STEPS);
  steps.right = this->serialPort->recvInt();
  steps.left = this->serialPort->recvInt();

  return steps;
}



void EpuckPosition2d::ClearSteps(void)
{
  this->SendRequest(EpuckInterface::RESET_STEPS);
}


 void EpuckPosition2d::SetSteps(int left, int right)
{
  this->SendRequest(EpuckInterface::SET_STEPS);

  this->serialPort->sendInt(left);
  this->serialPort->sendInt(right);
}

 int EpuckPosition2d::SetSpeed(int stepsL, int stepsR)
{

  this->SendRequest(EpuckInterface::SET_VEL);

  this->serialPort->sendInt(stepsR);
  this->serialPort->sendInt(stepsL);

  return  this->serialPort->recvChar(); // Wait for e-puck to send an end of task signal.
}

 int EpuckPosition2d::SetTargetSteps(int stepsL, int stepsR)
{

  this->SendRequest(EpuckInterface::SET_TARGET_STEPS);

  this->serialPort->sendInt(stepsR);
  this->serialPort->sendInt(stepsL);

  return  this->serialPort->recvChar(); // Wait for e-puck to send an end of task signal.
}





//======private:

//==================================================
  void EpuckPosition2d::StartInterval()
//==================================================
 {
  //struct timeval time;
  //double start_time;

    intervalRunning = true;
  //Get time of day, initialise start_time (in s)
  gettimeofday(&time,NULL);
  start_time = time.tv_sec + time.tv_usec/1000000.0;
}


//==================================================
  double EpuckPosition2d::ElapsedTime()
//==================================================
 {
double elapsed_time;
 gettimeofday(&time,NULL);

    if(intervalRunning == false)
    {
      return 0;
    }

    return elapsed_time = (time.tv_sec + time.tv_usec/1000000.0) - start_time;
}

