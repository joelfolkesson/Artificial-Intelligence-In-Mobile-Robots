#include "interface.h"
#include <math.h>
#define ROBOT_NUMBER "0"

//==============================================================================//
//                                  1B                                          //
//==============================================================================//
void Move(double dist){
	int steps      = dist * (1/0.129);
	Steps s        = GetSteps();
	printf           ("\nSteps walked: %d\n",steps);
	int movestepsl = s.l + steps;
	int movestepsr = s.r + steps;
	SetTargetSteps   (movestepsl,movestepsr);
	Sleep(500);
	Stop();
}


void Turn          (float degrees){
	float radians  = ((degrees*PI)/180.0);
	float steps    = ((26.75 * radians) /0.129);
	Steps s        = GetSteps();
	SetTargetSteps   (steps+s.l,-steps+s.r);
	printf           ("\n Done Turning %f Steps \n",steps);
	Sleep            (1000);
}

void update_pos     (Steps *prev){
	Steps s         = GetSteps();
	printf            ("\nPrev -> r %d\n",prev->r);
	printf            ("\nPrev -> l %d\n",prev->l);

	float rWheelDist= ((float)s.r - (float)prev->r) * 0.129; 							 //calc moved dist per wheel
	float lWheelDist= ((float)s.l - (float)prev->l) * 0.129; 							 // -||-
	float delta     = (((float)s.r * 0.129) - ((float)s.l * 0.129)) /52.5; //moved dist unit
	float dist      = ((rWheelDist + lWheelDist ) / 2.0); 								 //travelled dist
	ClearSteps();
	float dx        = dist * (cos(delta/2.0)); 														 //displacement x
	float dy        = dist * (sin(delta/2.0));														 //displacement y
	Posture p       = GetPosture(); 														 					 //get the current readings of robot pos

	float x         = p.x + (dx * cos(p.th)) - (dy * sin(p.th)); 					 //calc new x coords
	float y         = p.y + (dx * sin(p.th)) + (dy * cos(p.th)); 					 //calc new y coords
	float th        = p.th + delta; 														 					 // calc new th value
        th        = fmod(th,PI*2); 														 					 // conversion into mod

	printf            ("\nupdate_pos p.x = %.4f, p.y = %.4f, p.th = %.4f DEG \n",p.x,p.y,DEG(th));
	printf            ("\nDistance = %f\n",dist);
	*prev           = GetSteps();
	SetPosture        (x,y,th); 																					 //send away new calculated values for exec
	p               = GetPosture();
}
Sensors ir;
void sensorsIR(){

	for (int i=0;i<1000;i++)
	{

		//printf("IR values: %4d, %4d, %4d, %4d, %4d, %4d, %4d, %4d\n",
	 //ir.sensor[0], ir.sensor[1], ir.sensor[2], ir.sensor[3],
	 //ir.sensor[4], ir.sensor[5], ir.sensor[6], ir.sensor[7]);
		ir = GetIR();
		if(ir.sensor[0] >500 || ir.sensor[1] > 500){
			Turn(90);
			Move(100);
		}
	}
}


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

  //Example 0 (Note: LEDs only works with real robot)
 // bool led[8] = {1,1,1,1,1,1,1,1};  // eight leds
 // SetRingLED (led);


//SetTargetSteps(1000, 1000);


/* remove this line

  //Example 1
  //printf("\n Moving wheels for 1000 step counts\n");
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
  for (i=0;i<1000;i++)
  {
    ir = GetIR();
    printf("IR values: %4d, %4d, %4d, %4d, %4d, %4d, %4d, %4d\n",
           ir.sensor[0], ir.sensor[1], ir.sensor[2], ir.sensor[3],
           ir.sensor[4], ir.sensor[5], ir.sensor[6], ir.sensor[7]);
  }

remove this line*/

Posture now;
Steps prev = GetSteps();
now.x=0;
now.y=0;
now.th=0;

int input;
while(1){
	sensorsIR();
	printf("\n////////////////////////////\n/ 0. Display Current Coords/\n/ 1. Move 100              /\n/ 2. Turn 90 Deg           /\n/ 3. Move -100             /\n/ 4. Turn 90 Deg           /\n////////////////////////////\n");
	printf("input> ");
	input = getchar();
	getchar();
	Steps s = GetSteps();
	now     = GetPosture();
	//SetSpeed(100,100);
	update_pos(&prev);
	switch(input){
		ZERO:case '0':
			now     = GetPosture();
			printf("\n now.x:  %f\n",now.x);
			printf("\n now.y:  %f\n",now.y);
			printf("\n now.th: %f\n",now.th);
			break;
		case '1':
			Move(100);
			goto ZERO;

		case '2':
			Turn(90);
			goto ZERO;

		case '3':
			Move(-100);
			goto ZERO;

		case '4':
			Turn(-90);
			goto ZERO;

		default:
			break;
		}



	// printf("\n now.x:  %f\n",now.x);
	// printf("\n now.y:  %f\n",now.y);
	// printf("\n now.th: %f\n",now.th);
	//
	//
	// if(now.x >= 50 && now.x <= 52){
	// 	Turn    (90.0);
	// 	SetSpeed(50,50);
	// 	Sleep   (1000);
	// }
	// if(now.x <= 52){
	// 	printf("\nWE ARE DONE!\n");
	// 	Stop();
	// 	break;
	// }
	//if(now.th >= 50 || now.x <= -50){
	//	printf("Snurrat klart!");
	//	Stop();
	//	break;
	//}

}





printf("\nDone...\n");
return (0);
 }

//==============================================================================//
//                                  end main                                    //
//==============================================================================//
main