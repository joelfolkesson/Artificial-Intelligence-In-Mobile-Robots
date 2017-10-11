#include "interface.h"

#define ROBOT_NUMBER "7"

//==============================================================================//
//                                  main                                        //
//==============================================================================//
int main(int argc, char *argv[])
 {

//epuck connection
epuck(ROBOT_NUMBER);


Posture posture;     //get posture; float(x,y,th)

Steps steps;  // get steps: int (l,r)

Steps en_mm;    // steps to mm int (l,r)

Steps mm_en;    // mm to encoder int (l,r)

Speed speed;       // get speed int (l,r)

Speed speed_mm;       // get speed in mm int (l,r)

Sensors ir;       // get sensors vlotage unsigned int (sensor)



double vlin, vrot;
int i;


  printf("Starting...\n");

  //Example 0 (only works with real robot)
  //bool led[8] = {1,1,1,1,1,1,1,1};  // eight leds
  //SetRingLED (led);


//SetTargetSteps(1000, 1000);




  //Example 1
  printf("\n Moving wheels for 1000 step counts\n");
  SetTargetSteps(1000, 1000);
  Sleep(3000);


  //Example 2
  printf("\n Moving wheels for -1000 step counts\n");
  SetTargetSteps(-1000, -1000);
  Sleep(3000);

  //Example 2
  printf("\n Moving wheels for 0 step counts\n");
  SetTargetSteps(0, 0);
  Sleep(3000);

  //Example 2
  printf("\n Moving right wheel for -1000 and left for 1000 step counts\n");
  SetTargetSteps(-1000, 1000);
  Sleep(3000);


  //Example 3
  printf("\n setting speed\n");
  SetSpeed(500, 500);
  Sleep(4000);
  Stop();


  //Example 4
  printf("\n Setting linear and angular velocities\n");
  SetPolarSpeed(50, RAD(-28));
  for (i=0;i<10;i++)
  {
    Sleep(100);
    GetPolarSpeed(&vlin, &vrot);
    printf("(vlin,vrot)=(%.2lf,%.2lf)\n", vlin, DEG(vrot));
  }
  Stop();


  //Example 5
  printf("\n Getting IR values\n");
  for (i=0;i<1000;i++)
  {
    ir = GetIR();
    printf("IR values: %4d, %4d, %4d, %4d, %4d, %4d, %4d, %4d\n",
           ir.sensor[0], ir.sensor[1], ir.sensor[2], ir.sensor[3],
           ir.sensor[4], ir.sensor[5], ir.sensor[6], ir.sensor[7]);
  }



  printf("Done...\n");


return (0);
 }

//==============================================================================//
//                                  end main                                    //
//==============================================================================//










