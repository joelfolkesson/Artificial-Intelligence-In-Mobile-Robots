#include "include.h"
#include "user_info.h"
#include "user.h"



//==============================================================================
//Added by KL

//Socket stuff
#include "../UDPSocket/UDPSocket_C.h"
#define RECV_PORT 10000
#define SEND_PORT 10001

//Needed to access the MessageRobotDeal function in robot.c
extern void MessageRobotDeal(struct Context *context, char *q, char *a);
//==============================================================================




void UserInit(struct Robot *robot) {

  //--KL Put robot in middle to start
	robot->X = 300;
	robot->Y = 300;
	robot->Alpha = 0.0;

	//--KL Init sockets
	InitRecvUDP(RECV_PORT);
}

void UserClose(struct Robot *robot)
{
}

void NewRobot(struct Robot *robot)
{
}

void LoadRobot(struct Robot *robot,FILE *file)
{
}

void SaveRobot(struct Robot *robot,FILE *file)
{
}

void RunRobotStart(struct Robot *robot)
{
}

void RunRobotStop(struct Robot *robot)
{
}

boolean StepRobot(struct Robot *robot)
{

  //==============================================================================
  //Added by KL

  //Do these things at every step...
  char remote_ip[32];     //IP of last person to send something
  char msg_from_ctrl[32]; //Message received from controlling program
  char reply_to_ctrl[127];//Reply to send to controlling program
  struct Context context; //Context containing this robot
  context.Robot = robot;  //Set context to this robot

  if (PeekRecv())
  {
    //Receive message
    RecvUDP((unsigned char*)msg_from_ctrl,32);

    //Deal with message
    MessageRobotDeal(&context, msg_from_ctrl, reply_to_ctrl);

    //Get IP of remote host
    LastPacketReceivedFrom(remote_ip,127);

    //Send reply
    SendUDPTo(remote_ip, SEND_PORT, (unsigned char*)reply_to_ctrl,strlen(reply_to_ctrl));
  }
  //==============================================================================

  return TRUE;
}

void FastStepRobot(struct Robot *robot)
{
}

void ResetRobot(struct Robot *robot)
{
  char temp[127];

  struct Context context; //Context containing this robot
  context.Robot = robot;            //Set context to this robot

	robot->X = 300;
	robot->Y = 300;
	robot->Alpha = 0.0;

  //Stop
  MessageRobotDeal(&context, "D,0,0", temp);

  //Reset Encoders
  MessageRobotDeal(&context, "G,0,0", temp);
}

void UserCommand(struct Robot *robot,char *text)
{
}

void DrawUserInfo(struct Robot *robot,u_char info,u_char page)
{
}



