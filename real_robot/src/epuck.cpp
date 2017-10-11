#include "epuck.hpp"
#include "rfid.hpp"


// Globals
 SerialPort* SP;
 EpuckPosition2d::Steps lastStp;
 Posture posture; 
//==================================================
  void epuck(std::string port)
//==================================================
  {	

   std::string path="/dev/rfcomm";
   path.append(port);
   std::string error;
   SP = new SerialPort(path);
   int signal= SP->initialize();
	
   error=SP->getError();

   if(signal<0)
    {
   cout<<"BLUETOOTH ERROR: Please check the bluetooth connection"<<endl;
   cout<<error<<endl;
   exit(1);
   }

  }

//==================================================
  void Sleep(unsigned long ms)
//==================================================
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

//printf("time= %lf\n", elapsed_time);
  }
  while (elapsed_time < sleep_time);

 }

//==================================================
void SetPosture(float x, float y, float th)
//==================================================
 {
  posture.x = x;
  posture.y = y;
  posture.th = th;
/*
 EpuckPosition2d move(SP);
 EpuckInterface::Triple odometry;

 odometry.x = MM2M(x);
 odometry.y = MM2M(y);
 odometry.theta = th;

 move.SetOdometry(odometry);*/
 }

//==================================================
void SetPosition(double x, double y)
//==================================================
 {
 EpuckPosition2d move(SP);

 EpuckPosition2d::DynamicConfiguration getOdometry;
 getOdometry = move.UpdateOdometry ();

 EpuckInterface::Triple setOdometry;

 setOdometry.x = MM2M(x);
 setOdometry.y = MM2M(y);
 setOdometry.theta = getOdometry.pose.theta;

 move.SetOdometry(setOdometry);
 }


//==================================================
Posture GetPosture(void)
//==================================================
 {




/*
 EpuckPosition2d move(SP);
 EpuckPosition2d::DynamicConfiguration odometry;
 odometry = move.UpdateOdometry ();
 Posture pos;

 pos.x = M2MM(odometry.pose.x);
 pos.y = M2MM(odometry.pose.y);
 pos.th = odometry.pose.theta;
*/
 return posture;
 }

//==================================================
Steps GetSteps(void)
//==================================================
 {
 EpuckPosition2d move(SP);

 EpuckPosition2d::Steps getSteps;
 Steps returnSteps;

 getSteps= move.GetSteps();

 returnSteps.r = getSteps.right;
 returnSteps.l = getSteps.left;

return returnSteps;
 }

//==================================================
void ClearSteps(void)
//==================================================
{
 EpuckPosition2d move(SP);
 move.ClearSteps();
lastStp.left=0;
lastStp.right=0;
}

//==================================================
void SetSteps(int left, int right)
//==================================================
{
 EpuckPosition2d move(SP);

 move.SetSteps(left,right);

}

//==================================================
Steps enc2mm(Steps enc)
//==================================================
 {
  Steps enc_mm;
  enc_mm.l = (int)rint(enc.l * MM_PER_PULSE_L);
  enc_mm.r = (int)rint(enc.r * MM_PER_PULSE_R);

  return enc_mm;
 }

//==================================================
Steps mm2enc(Steps enc_mm)
//==================================================
 {
  Steps enc;
  enc.l = (int)rint(enc_mm.l * PULSES_PER_MM_L);
  enc.r = (int)rint(enc_mm.r * PULSES_PER_MM_R);

  return enc;
 }

//==================================================
Speed GetSpeed(void)
//==================================================
 {
 EpuckPosition2d move(SP);
 EpuckPosition2d::Steps Getspeed;
 Speed returnSpeed;

 Getspeed = move.GetSpeed();

 returnSpeed.r = Getspeed.right;
 returnSpeed.l = Getspeed.left;

 return returnSpeed;
 }

//==================================================
Speed GetSpeed_mm(void)
//==================================================
 {
 Speed s;
 EpuckPosition2d::Steps Getspeed;
 EpuckPosition2d move(SP);

 Getspeed = move.GetSpeed();

  //Convert from pulses per 10ms to mm per second
  s.l = (int)rint(Getspeed.left*MM_PER_PULSE_L);
  s.r = (int)rint(Getspeed.right*MM_PER_PULSE_R);

  return s;

 }

void SetSpeed_mm(int left, int right)
{
  //Convert from mm per second to pulses per 10ms
  SetSpeed((int)rint(left*PULSES_PER_MM_L),
                     (int)rint(right*PULSES_PER_MM_R));

}

//==================================================
int SetSpeed(int left, int right)
//==================================================
 {

 EpuckPosition2d move(SP);
return move.SetSpeed(left, right);
 }

//==================================================
void Stop(void)
//==================================================
 {
 EpuckPosition2d move(SP);
 move.StopMotors();
 }

//==================================================
void SetPolarSpeed(double vlin, double vrot)
//==================================================
 {
 EpuckPosition2d move(SP);

 move.SetVel((float)vlin/1000,(float)vrot);

 }

//==================================================
void GetPolarSpeed(double* vlin, double* vrot)
//==================================================
{
  EpuckPosition2d move(SP);
  EpuckPosition2d::Steps s;
  double l,r;
  s = move.GetSpeed();
  

  l = s.left*MM_PER_PULSE_L;
  r = s.right*MM_PER_PULSE_R;
  *vrot = (r-l)/ROBOT_DIAMETER;
  *vlin = (r+l)*0.5;
}

//==================================================
int SetTargetSteps(int left, int right)
//==================================================
{
 EpuckPosition2d move(SP);

/*
 int leftSpeed=350;
 int rightSpeed=350;
 EpuckPosition2d::Steps steps;
 Posture posture;

steps = move.GetSteps();

   lastStp.left = steps.left;
   lastStp.right = steps.right;


 while(1)
  {
  steps = move.GetSteps();

  if(abs(steps.left - lastStp.left)>= abs(left)-(abs(left)/20)) //reduce speed near goal
		leftSpeed =80;

  if(abs(steps.right - lastStp.right) >= abs(right)-(abs(left)/10))
		rightSpeed=80;

  if(abs(steps.left - lastStp.left)>= abs(left))
		leftSpeed =0;

  if(abs(steps.right - lastStp.right) >= abs(right))
		rightSpeed=0;

  move.SetSpeed(leftSpeed*(left/abs(left)),rightSpeed*(right/abs(right)));


  if(leftSpeed==0 && rightSpeed == 0)
     {
    // lastStp.left =steps.left;
    // lastStp.right =steps.right;
	 return(1);
     }

  }
*/
move.SetTargetSteps(left, right);

}

//==================================================
Sensors GetIR(void)
//==================================================
{

vector<float> getIr;
Sensors sendIr;

  EpuckIR IR(SP);
  EpuckIR::IRData irSensor;
  irSensor=IR.GetIRData();
  getIr=irSensor.voltages;

  for(int i=0;i<8;i++)
    {
    sendIr.sensor[i] = (unsigned int)getIr[i];
    }

 return sendIr;
}

//==================================================
void SetRingLED (bool ringLED[RING_LEDS_NUM])
//==================================================

 {
  EpuckLEDs led(SP);
  led.SetRingLED(ringLED);
  led.ClearInternal();
 }

//==================================================
void SetSingleLED (unsigned id, bool state)
//==================================================
 {
 
  EpuckLEDs led(SP);
  led.SetRingLED(id,state);
  led.ClearInternal();
 }

//==================================================
void SetFrontLED(bool state)
//==================================================
 {
  EpuckLEDs led(SP);
  led.SetFrontLED (state);
  led.ClearInternal();
 }

//==================================================
double DIST(double x1, double y1, double x2, double y2)
//==================================================
{
  return 0;//sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1));
}


void getImage (void)
{
unsigned sensor_x1 = 10;
unsigned sensor_y1 = 10;
unsigned sensor_width = 10;
unsigned sensor_height =10;
unsigned zoom_fact_width= 1;
unsigned zoom_fact_height = 1;


/*EpuckCamera cam(SP,  sensor_x1,
               sensor_y1,  sensor_width,
               sensor_height,  zoom_fact_width,
               zoom_fact_height, YUV_MODE);

*/
}



//================================EOF================================//

