#include "interface.h"
#include <math.h>
#define ROBOT_NUMBER "0"
#define V_MAX 50
#define V_MIN 10
#define MM_PER_PULSE 0.145
#define NO_DANGER 300
#define FULL_DANGER 400
#define FORCE_FULL_DANGER 700
#define EXPAND_ERR 0.25

//problemet just nu är att när replann anropas, så hoppar den till kanten av cellen i den interna kartan,
//säg att vi står i mitten, då hoppar den till kanten av cellen (som är positionen).
// måste lagra undan nuvarande pos

void update_pos    (Steps *prev){
    Steps s         = GetSteps();
    float rWheelDist= (s.r - prev->r) * MM_PER_PULSE;
    float lWheelDist= (s.l - prev->l) * MM_PER_PULSE;
    float delta     = ((s.r * MM_PER_PULSE) - (s.l * MM_PER_PULSE)) /52.55;
    float dist      = ((rWheelDist + lWheelDist ) / 2.0);
  	ClearSteps();
    float dx        = dist * (cos(delta/2.0));
    float dy        = dist * (sin(delta/2.0));
    Posture p       = GetPosture();

    float x         = p.x + (dx * cos(p.th)) - (dy * sin(p.th));
    float y         = p.y + (dx * sin(p.th)) + (dy * cos(p.th));
    float th        = p.th + delta;
          th        = fmod(th,PI*2);

    //printf            ("\nupdate_pos p.x = %.4f, p.y = %.4f, p.th = %.4f DEG \n",p.x,p.y,DEG(th));
    //printf            ("\nDistance = %f\n",dist);
    *prev           = GetSteps();
    SetPosture        (x,y,th);
    p               = GetPosture();
}



//==============================================================================//
//                                  LAB5                                        //
//==============================================================================//




int GoTo2(Steps *prevSteps, Posture *goal){


	float dx_dist       ,
				dy_dist       ,
				th_dist       ,
				th_err        ,
				vl, vr        ,
				p_vel, p_th   ,
				pos_err       ,
	      kpos      = 1.5,
				kth       = -3,
				delta_pos = 25;
  FPred Pos_Left, Pos_Right, Pos_Ahead, Pos_Here, DANGER, Obs_Left,Obs_Right,Obs_Ahead;
	//update_pos(prevSteps);
	Posture currentPos = GetPosture();

	dx_dist     = goal->x - currentPos.x;
	dy_dist     = goal->y - currentPos.y;

	pos_err     = sqrt(pow((dx_dist),2) + pow((dy_dist),2));
	th_err      = atan2(dy_dist,dx_dist) - currentPos.th;

	if(th_err > PI){
		th_err    = th_err - 2 * PI;
	}
	if(th_err < -PI){
		th_err    = th_err + 2 * PI;
	}
	th_err    = DEG(th_err);
	Pos_Left  = RampUp(th_err, 0, 60);
	Pos_Right = RampDown(th_err, -60, 0);
	Pos_Ahead = min(RampUp(th_err, -30, 0), //MIN eller min ??
							RampDown(th_err, 0, 30));
	Pos_Here  = RampDown(pos_err, 10, 50);


  Sensors ir = GetIR();
  Obs_Left   = RampUp(MIN(ir.sensor[5],ir.sensor[6]),NO_DANGER,FULL_DANGER);
  Obs_Ahead  = RampUp(MIN(ir.sensor[0],ir.sensor[7]),NO_DANGER,FULL_DANGER);
  Obs_Right  = RampUp(MIN(ir.sensor[1],ir.sensor[2]),NO_DANGER,FULL_DANGER);
  DANGER     = MAX(MAX(Obs_Left,Obs_Right),Obs_Ahead);

  RULESET;
  //no danger
  IF(AND (NOT(DANGER),(AND(Pos_Left, NOT(Pos_Here)))));   ROT(LEFT);
  IF(AND (NOT(DANGER),(AND(Pos_Right, NOT(Pos_Here)))));  ROT(RIGHT);
  IF(AND (NOT(DANGER),(OR (Pos_Here, Pos_Ahead))));       ROT(AHEAD);

  IF(AND (NOT(DANGER),(Pos_Ahead, NOT(Pos_Here))));       VEL(FAST);
  //danger

  IF(AND(DANGER,(AND(Obs_Right,NOT(Obs_Left)))));         ROT(LEFT);
  IF(AND(DANGER,(AND(Obs_Right, Obs_Left))));             ROT(AHEAD);
  IF(AND(DANGER,Obs_Ahead));                              ROT(RIGHT);

  IF(AND(DANGER,(OR(Obs_Right,Obs_Left))));               VEL(SLOW);
  IF(AND(DANGER,Obs_Ahead));                              VEL(NONE);


  //here
  IF (OR(Pos_Here, NOT(Pos_Ahead)));                      VEL(NONE);


  RULEEND;
	if(pos_err < delta_pos){
		return 1;
  }
	return 0;
}


void FuzzyAvoid(){
	Sensors ir = GetIR();
	FPred Obs_Left,Obs_Right,Obs_Ahead, DANGER;
  /* Force Rules */
  if (ir.sensor[5] > FULL_DANGER){
    DANGER = 1;
  }
  if (ir.sensor[2] > FULL_DANGER){
    DANGER = 1;
  }
  if (ir.sensor[0] > FULL_DANGER || ir.sensor[7] > FULL_DANGER){
    DANGER = 1;
  }
  DANGER    = RampUp(MIN(ir.sensor[5],ir.sensor[6]),NO_DANGER,FULL_DANGER);
	Obs_Left  = RampUp(MIN(ir.sensor[5],ir.sensor[6]),NO_DANGER,FULL_DANGER);
	Obs_Ahead = RampUp(MIN(ir.sensor[0],ir.sensor[7]),NO_DANGER,FULL_DANGER);
  Obs_Right = RampUp(MIN(ir.sensor[1],ir.sensor[2]),NO_DANGER,FULL_DANGER);


	//printf("\nLeft: %f,\t Right: %f,\t Ahead: %f", Obs_Left, Obs_Right, Obs_Ahead);
	//printf("\n %d, %d, %d, %d, %d, %d, %d, %d",ir.sensor[0],ir.sensor[1],ir.sensor[2],ir.sensor[3],ir.sensor[4],ir.sensor[5],ir.sensor[6],ir.sensor[7]);

	RULESET;
	IF (AND(Obs_Left,  NOT(Obs_Right)));               ROT(RIGHT);
  /*IF (AND(Obs_Ahead,(Obs_Left,Obs_Right)));            VEL(BACK);*/
	IF (AND(Obs_Right, NOT(Obs_Left)));                ROT(LEFT);
	IF (AND(Obs_Right, Obs_Left));                     ROT(AHEAD);
  IF (Obs_Ahead);                                   VEL(BACK);
	IF (AND((OR(Obs_Right, Obs_Left)), NOT(Obs_Ahead))); VEL(SLOW);
	IF (NOT(OR(OR(Obs_Right,Obs_Left), Obs_Ahead)));   VEL(FAST);
	RULEEND;
}


double ResponseToVel (double response){
		return (V_MIN + response *(V_MAX-V_MIN));
}


double ResponseToRot (double response){
	  return (PI - response * (2 * PI));
}


void GoToLab6(){
  double x,y,th,vel,rot;
  x = 0; y = 0; th = 0;
  SetPosture(x,y,th); //set startpos
  Posture currentPos = GetPosture();
  Steps prevSteps = GetSteps();
  Posture goal;

  Sensors ir;
  goal.x = 100;
  goal.y = 0;
  goal.th = 0;
	float dx_dist       ,
				dy_dist       ,
				th_dist       ,
				th_err        ,
				vl, vr        ,
				p_vel, p_th   ,
				pos_err       ,
	      kpos      =1,
				kth       = -3,
				delta_pos = 15 ;
    FPred Pos_Left, Pos_Right, Pos_Ahead, Pos_Here, Danger, Obs_Left,Obs_Right,Obs_Ahead;
  	//update_pos(&prevSteps); //KOMMENTERA OM SAKER FUCKAR

    dx_dist     = goal.x - currentPos.x;
    dy_dist     = goal.y - currentPos.y;

    pos_err     = sqrt(pow((dx_dist),2) + pow((dy_dist),2));
    th_err      = atan2(dy_dist,dx_dist) - currentPos.th;

    if(th_err > PI){
      th_err    = th_err - 2 * PI;
    }
    if(th_err < -PI){
      th_err    = th_err + 2 * PI;
    }


  while(pos_err > delta_pos){
    ClearFSet(f_set_vlin);
    ClearFSet(f_set_vrot);
    ir= GetIR();
    update_pos(&prevSteps);
    currentPos = GetPosture();



  	dx_dist     = goal.x - currentPos.x;
  	dy_dist     = goal.y - currentPos.y;

  	pos_err     = sqrt(pow((dx_dist),2) + pow((dy_dist),2));
  	th_err      = atan2(dy_dist,dx_dist) - currentPos.th;

  	if(th_err > PI){
  		th_err    = th_err - 2 * PI;
  	}
  	if(th_err < -PI){
  		th_err    = th_err + 2 * PI;
  	}

  	th_err    = DEG(th_err);
  	Pos_Left  = RampUp(th_err, 0, 60);
  	Pos_Right = RampDown(th_err, -60, 0);
  	Pos_Ahead = MIN(RampUp(th_err, -30, 0), //MIN eller min ??
  							RampDown(th_err, 0, 30));
  	Pos_Here  = RampDown(pos_err, 10, 50);



    Obs_Left  = RampUp(MIN(ir.sensor[5],ir.sensor[6]),NO_DANGER,FULL_DANGER);
    Obs_Ahead = RampUp(MIN(ir.sensor[0],ir.sensor[7]),NO_DANGER,FULL_DANGER);
    Obs_Right = RampUp(MIN(ir.sensor[1],ir.sensor[2]),NO_DANGER,FULL_DANGER);
    Danger    = MAX(MAX(Obs_Left,Obs_Right),Obs_Ahead);

    Obs_Left  = RampUp(MIN(ir.sensor[5], ir.sensor[6]), NO_DANGER, FULL_DANGER);
    Obs_Right = RampUp(MIN(ir.sensor[1], ir.sensor[2]), NO_DANGER, FULL_DANGER);
    Obs_Ahead = RampUp(MIN(ir.sensor[0], ir.sensor[7]), NO_DANGER, FULL_DANGER);
    //printf("\nObs_Left = %f \t Obs_Right = %f \t Obs_Ahead = %f\n", Obs_Left, Obs_Right, 			Obs_Ahead);
    Danger = MAX(MAX(Obs_Left, Obs_Right), Obs_Ahead);

    printf("DANGER = %f\n", Danger);

    RULESET;
    //No Danger
    IF (AND(NOT(Danger), (AND(Pos_Left, NOT(Pos_Here)))));		ROT(LEFT);
    IF (AND(NOT(Danger), (AND(Pos_Right, NOT(Pos_Here)))));		ROT(RIGHT);
    IF (AND(NOT(Danger), (OR(Pos_Here, Pos_Ahead))));			    ROT(AHEAD);

    IF (AND(NOT(Danger), (AND(Pos_Ahead, NOT(Pos_Here)))));		VEL(FAST);

    //Danger
    IF (AND(Danger, (AND(Obs_Left, NOT(Obs_Right)))));			  ROT(RIGHT);
    IF (AND(Danger, (AND(Obs_Right, NOT(Obs_Left)))));			  ROT(LEFT);
    IF (AND(Danger, (AND(Obs_Right, Obs_Left))));				      ROT(AHEAD);
    //IF (AND(Danger, Obs_Ahead));								              ROT(RIGHT);

    IF (AND(Danger, (OR(Obs_Right, Obs_Left))));				      VEL(SLOW);
    IF (AND(Danger, Obs_Ahead));								              VEL(NONE);

    // //Here
    IF (OR(Pos_Here, NOT(Pos_Ahead)));							          VEL(NONE);

    RULEEND;

    DeFuzzify(f_set_vrot, 3, &rot);
    DeFuzzify(f_set_vlin, 4, &vel);

    f_vlin = ResponseToVel(vel);
    f_vrot = ResponseToRot(rot);

    SetPolarSpeed(f_vlin, f_vrot);
    Sleep(100);
  }
  currentPos = GetPosture();
  Stop();
}


//==============================================================================//
//                                FINAL EXAM                                    //
//==============================================================================//
Posture GlobalPos = GetPosture();
void SetGlobalPos(int x, int y){
  GlobalPos.x  = x;
  GlobalPos.y  = y;
  GlobalPos.th = 0;
  SetPosture(GlobalPos.x,GlobalPos.y,GlobalPos.th);
}

char mapname[50] = "../real_robot/src/SampleMaps/custom.txt";
Map* grid        = CreateMapFromFile(0,mapname);
int RePlan(Cell startCell, Cell goalCell);
int Err_counter  = 0;
void MarkCell(int i, int j, List frontier[], int value){
  Cell               explore;
  explore.i        = i;
  explore.j        = j;
  int check        = GetCellState(grid,i,j);
  switch(check){
    case -1:
      Push           (frontier,explore);
      break;
    case -2:
      ChangeCellState(grid,explore.i,explore.j,value);
      Push           (frontier,explore);
      break;
    default:
      break;
  }
}

bool Search(Cell goalCell){
    int dist        = 0;
    Cell              popCell;
    popCell.h_value = 0;
    List *frontier  = CreateList(2);
    ClearList         (frontier, 2);
    Push              (frontier, goalCell);                    //put the goalCell in the frontier
    while(!IsListEmpty(frontier)){
      popCell       = Pop(frontier);
      if(GetCellState (grid,popCell.i,popCell.j) == MAP_START){ //take a cell from the queue
        return true;
      }
      dist          = GetCellState(grid, popCell.i, popCell.j)+1;
      MarkCell        (popCell.i,popCell.j-1,frontier,dist);
      MarkCell        (popCell.i,popCell.j+1,frontier,dist);
      MarkCell        (popCell.i-1,popCell.j,frontier,dist);
      MarkCell        (popCell.i+1,popCell.j,frontier,dist);
    }
    return false;
}



void FuzzyPlanSearch(double arrayX[], double arrayY[], int counter){
  //printf("\n=== ExecPlan ===\n");
  double x,y,th,vel,rot;
  x = 0; y = 0; th = 0;
  Posture goal;
  int condition   = 0,
      count       = 0;
  Steps prevSteps = GetSteps();
  Cell              storeNewStart,storeGoal;

  size_t arrsizeX = sizeof(arrayX)/sizeof(arrayX[0]); //storing last-goal so i can use it in replan
  size_t arrsizeY = sizeof(arrayY)/sizeof(arrayY[0]);
  for(int i=0;i < arrsizeX;++i){
    storeGoal.i   = arrayX[i];
  }
  for(int i=0;i < arrsizeY;++i){
    storeGoal.j   = arrayY[i];
  }
  SetPosture(arrayX[counter],arrayY[counter],th); //set startpos
  for(int i=counter-1; i>=0; i--){  //start from the last pos in array(start) -> goto -> goal
    condition=0; //set to 0 each time.
    printf("\ni = %d,size=%d\n",i,counter);
    goal.x        = arrayX[i];
    goal.y        = arrayY[i];
    goal.th       = 0;
    printf("\n=== Moving Towards: (%f,%f) === \n",goal.x,goal.y);
    do{
        update_pos        (&prevSteps);
        ClearFSet         (f_set_vlin);
        ClearFSet         (f_set_vrot);
        FuzzyAvoid        ();
        Posture now     = GetPosture();
        storeNewStart.i = (int)now.x/25;
        storeNewStart.j = (int)now.y/25;
        RePlan            (storeNewStart,storeGoal);
        condition       = GoTo2(&prevSteps,&goal);
        DeFuzzify         (f_set_vrot, 3, &rot);
        DeFuzzify         (f_set_vlin, 4, &vel);

        f_vlin          = ResponseToVel(vel);
        f_vrot          = ResponseToRot(rot);
        SetPolarSpeed     (f_vlin,f_vrot);
        //        printf("\n\n\nvlin: %f vrot:%f\n", f_vlin, f_vrot);

        Sleep(100);
    }while(condition != 1);
  }
  Stop();
}

void GoTo2AndAvoid(){
  double x,y,th,vel,rot;
  x = 0; y = 0; th = 0;
  Posture goal;
  int condition=0;
  Steps prevSteps = GetSteps();
  SetPosture(x,y,th); //set startpos
  condition=0; //set to 0 each time.
  goal.x = 200;
  goal.y = 0;
  goal.th = 0;
  printf("\n=== Moving towards: (%f,%f) === \n",goal.x,goal.y);
  do{
      update_pos(&prevSteps);
      ClearFSet(f_set_vlin);
      ClearFSet(f_set_vrot);
      //FuzzyAvoid();
      condition = GoTo2(&prevSteps,&goal);
      DeFuzzify(f_set_vrot, 3, &rot);
      DeFuzzify(f_set_vlin, 4, &vel);

      f_vlin = ResponseToVel(vel);
      f_vrot = ResponseToRot(rot);
      SetPolarSpeed(f_vlin,f_vrot);
      //        printf("\n\n\nvlin: %f vrot:%f\n", f_vlin, f_vrot);

      Sleep(100);
    }while(condition != 1);
    Stop();
}

void Plan(Cell startCell, Cell goalCell){
  ClearMap(grid);
  ChangeCellState(grid,startCell.i,startCell.j,-1);
  ChangeCellState(grid,goalCell.i,goalCell.j,0);
  int maxValue    = 0,counter=1;
  int n1,n2,n3,n4; //neighbours
  List * Path     = CreateList(1);
  Cell              bestCell;
  Cell              nextCell;
  ClearList         (Path,1);
  if                (Search(goalCell)){
    Push            (Path,startCell);
    maxValue      = GetMaxValue(grid); //assigning max-value
    bestCell.i    = startCell.i;
    bestCell.j    = startCell.j;


    while(maxValue != MAP_GOAL){ //while we are not at the goal. -1 for output
      n1     = GetCellState(grid,bestCell.i,bestCell.j-1);
      n2     = GetCellState(grid,bestCell.i,bestCell.j+1);
      n3     = GetCellState(grid,bestCell.i-1,bestCell.j);
      n4     = GetCellState(grid,bestCell.i+1,bestCell.j);
      if((n1 < maxValue) && (n1 >= 0)){
          maxValue   = n1;
          nextCell.i = bestCell.i;
          nextCell.j = bestCell.j-1;
      }
      if((n2 < maxValue) && (n2 >= 0)){
          maxValue   = n2;
          nextCell.i = bestCell.i;
          nextCell.j = bestCell.j+1;
      }
      if((n3 < maxValue) && (n3 >= 0)){
          maxValue   = n3;
          nextCell.i = bestCell.i-1;
          nextCell.j = bestCell.j;
      }
      if((n4 < maxValue) && (n4 >= 0)){
          maxValue   = n4;
          nextCell.i = bestCell.i+1;
          nextCell.j = bestCell.j;

      }
      //if(GetCellState(grid,nextCell.i,nextCell.j) != MAP_OBSTACLE){ //if path-node is in an obstacle, dont add.
        Push(Path,nextCell);
        bestCell.i = nextCell.i;
        bestCell.j = nextCell.j;
        bestCell.h_value = 0;
        ChangeCellState(grid,bestCell.i,bestCell.j,-5);
        counter++;
    //  }
    } //end while
  }else{
    printf("\nPath was NOT found");
  } // end if-else

  printf("\nPath FOUND!\n");
  //expanding the cells
  double arrayX[counter];
  double arrayY[counter];
  Cell temp;
  for(int i=0; i < counter; i++){
    temp      = Pop(Path);
    arrayX[i] = temp.i * 25;
    arrayY[i] = temp.j * 25;
  }
  for(int i=0;i < (sizeof(arrayX) / sizeof(arrayX[0])); i++){
    printf("(X: %f, Y: %f) \n",arrayX[i], arrayY[i]);
  }

  PrintMap(grid);
  FuzzyPlanSearch(arrayX,arrayY,counter-1);
}


int RePlan(Cell startCell, Cell goalCell){
  //printf("\n === RePlan === \n");
  Posture now     = GlobalPos; // store the global pos
  now = GetPosture(); // update the 'global' pos.
  Steps prevSteps = GetSteps();
  update_pos(&prevSteps);
  Sensors ir      = GetIR();
  Cell              Revisited_pos, newStart;
  newStart        = startCell;
  Revisited_pos.i = now.x; //  här, bör jag kolla globalpos.x och globalpos.y? så revisited = global;
  Revisited_pos.j = now.y; // eftersom revisited är nu lika med now redan.

  //update_pos(&prevSteps);
  float currentRot = now.th; //current rotation.
  if(currentRot > 0){
    currentRot     = currentRot - 2 * PI;
  }
  if(currentRot < -0){
    currentRot     = currentRot + 2 * PI;
  }

  currentRot       = fmod(currentRot,PI*2);
  currentRot       = DEG(currentRot);
  if(((Revisited_pos.i <= now.x+EXPAND_ERR) && (Revisited_pos.i >= now.x-EXPAND_ERR)) &&
    ((Revisited_pos.j <= now.y+EXPAND_ERR) && (Revisited_pos.j >= now.y-EXPAND_ERR))){ //increase err-path
      Err_counter++;
      printf("\nErr_count: %d,\tth_: %f",Err_counter,currentRot);
  }
  //place an obstacle if and only when error counter AND danger is triggered. If for some reason, we stutter, this is a failsafe
  if((Err_counter >= 4 && ((ir.sensor[5] > FULL_DANGER) || (ir.sensor[2] > FULL_DANGER) || ((ir.sensor[0] > FULL_DANGER || ir.sensor[7] > FULL_DANGER))))) {
    Stop(); // stop roboot momentarily to improve accuracy in placing of obs
    newStart.i = (int)(now.x/25); //Change to Cell values.
    newStart.j = (int)(now.y/25); //Scale up
    printf("\nERROR FUNC TRIGGERED, Replanning from: %d %d", newStart.i, newStart.j);
    //setting the obs according to robot-rotation in grid
   if(currentRot >= 0 && currentRot <= 45){ //KOLLA PÅ DESSA EN GÅNG TILL!!
      printf("\n0-90");
        if (ir.sensor[5] > FULL_DANGER){ //left
              if(GetCellState(grid,newStart.i+1,newStart.j) != MAP_BORDER){ChangeCellState(grid,newStart.i+1,newStart.j,-3);printf("\n1,1");goto SUCCESS;} //add obs in map
        }
        if (ir.sensor[2] > FULL_DANGER){ //right
              if(GetCellState(grid,newStart.i-1,newStart.j) != MAP_BORDER){ChangeCellState(grid,newStart.i-1,newStart.j,-3);printf("\n1,2");goto SUCCESS;}
        }
        if (ir.sensor[0] > FULL_DANGER || ir.sensor[7] > FULL_DANGER){ //ahead
              if(GetCellState(grid,newStart.i,newStart.j+1) != MAP_BORDER){ChangeCellState(grid,newStart.i,newStart.j+1,-3);printf("\n1,3");goto SUCCESS;}
        }
    }//klar
    if(currentRot >= 90 && currentRot <= 135){
      printf("\n90-180");
        if (ir.sensor[5] > FULL_DANGER){ //left
              if(GetCellState(grid,newStart.i,newStart.j-1) != MAP_BORDER){ChangeCellState(grid,newStart.i,newStart.j-1,-3);printf("\n2,1");goto SUCCESS;} //add obs in map
        }
        if (ir.sensor[2] > FULL_DANGER){ //right
              if(GetCellState(grid,newStart.i,newStart.j+1) != MAP_BORDER){ChangeCellState(grid,newStart.i,newStart.j+1,-3);printf("\n2,2");goto SUCCESS;}
        }
        if (ir.sensor[0] > FULL_DANGER || ir.sensor[7] > FULL_DANGER){ //ahead
              if(GetCellState(grid,newStart.i,newStart.j-1) != MAP_BORDER){ChangeCellState(grid,newStart.i,newStart.j-1,-3);printf("\n2,3");goto SUCCESS;}
        }
    }//klar
    if(currentRot >= 180 && currentRot <= 240){
      printf("\n180-270");
        if (ir.sensor[5] > FULL_DANGER){ //left
              if(GetCellState(grid,newStart.i+1,newStart.j) != MAP_BORDER){ChangeCellState(grid,newStart.i+1,newStart.j,-3);printf("\n3,1");goto SUCCESS;} //add obs in map
        }
        if (ir.sensor[2] > FULL_DANGER){ //right
              if(GetCellState(grid,newStart.i-1,newStart.j) != MAP_BORDER){ChangeCellState(grid,newStart.i-1,newStart.j,-3);printf("\n3,2");goto SUCCESS;}
        }
        if (ir.sensor[0] > FULL_DANGER || ir.sensor[7] > FULL_DANGER){ //ahead
              if(GetCellState(grid,newStart.i,newStart.j-1) != MAP_BORDER){ChangeCellState(grid,newStart.i,newStart.j-1,-3);printf("\n3,3");goto SUCCESS;}
        }
    } //klar
    if(currentRot >= 270 && currentRot <= 330){
      printf("\n270-360");
        if (ir.sensor[5] > FULL_DANGER){ //left
              if(GetCellState(grid,newStart.i,newStart.j+1) != MAP_BORDER){ChangeCellState(grid,newStart.i,newStart.j+1,-3);printf("\n4,1");goto SUCCESS;} //add obs in map
        }
        if (ir.sensor[2] > FULL_DANGER){ //right
              if(GetCellState(grid,newStart.i,newStart.j-1) != MAP_BORDER){ChangeCellState(grid,newStart.i,newStart.j-1,-3);printf("\n4,2");goto SUCCESS;}
        }
        if (ir.sensor[0] > FULL_DANGER || ir.sensor[7] > FULL_DANGER){ //ahead
              if(GetCellState(grid,newStart.i-1,newStart.j) != MAP_BORDER){ChangeCellState(grid,newStart.i-1,newStart.j,-3);printf("\n4,3");goto SUCCESS;}

        }

    } //klar

    if(currentRot >= 45 && currentRot <= 90){
      Err_counter--;
      return 0;
    }
    if(currentRot >= 135 && currentRot <= 180){
      Err_counter--;
      return 0;
    }
    if(currentRot >= 240 && currentRot <= 270){
      Err_counter--;
      return 0;
    }
    if(currentRot >= 330 && currentRot <= 360){
      Err_counter--;
      return 0;
    }
    // if(newStart.i ==0){newStart.i = newStart.i+1;} //error-handling for a problem i had before.
    // if(newStart.j ==0){newStart.j = newStart.j+1;} //if for some reason the robot thinks it's in a 0 pos.
    //ChangeCellState(grid, newStart.i,newStart.j,-1); //set new start where we are
    SUCCESS:
      startCell.i = newStart.i; //convert back
      startCell.j = newStart.j;
      goalCell.i  = goalCell.i/25;
      goalCell.j  = goalCell.j/25;

      //ChangeCellState(grid, goalCell.i,goalCell.j,0); //set new goal where we are going

      Err_counter = 0; //reset err-counter as new map is about to be loaded
      printf("\n   === NEW UPDATED MAP! ===");
      PrintMap(grid); //print map and where new start + obstacle is located.
      printf("\nNEW START: %d,%d, GOAL: %d,%d\n",startCell.i,startCell.j,goalCell.i,goalCell.j);
      Plan(startCell,goalCell);
      return 1;
  }
  return 0;
}


Cell menu(Cell *startCell,Cell *goalCell){
  printf("\nStart i: ");
  scanf("%d",&startCell->i);
  printf("\nStart j: ");
  scanf("%d",&startCell->j);
  printf("\nGoal i: ");
  scanf("%d",&goalCell->i);
  printf("\nGoal j: ");
  scanf("%d",&goalCell->j);
  return *goalCell,*startCell;
}


int main(int argc, char *argv[]){

//epuck connection
epuck(ROBOT_NUMBER);

double vel,rot;
int i;
  printf             ("Starting...\n");
  Cell               goalCell,startCell,Wall;
  goalCell.i       = 10;
  goalCell.j       = 11;
  ChangeCellState(grid,goalCell.i,goalCell.j,0);
  startCell.i      = 5;
  startCell.j      = 11;
  ChangeCellState(grid,startCell.i,startCell.j,-1);

  SetGlobalPos(startCell.i*25,startCell.j*25);
  //menu(startCell,goalCell);
  PrintMap(grid);
  Plan(startCell,goalCell);


	// double x,y,th;
	// x = 0; y = 0; th = 0;
	// Posture goal,currPos;
	// int condition=0;
	// Steps prevSteps = GetSteps();
	// SetPosture(x,y,th);
	// goal.x = 200;
	// goal.y = 200;
	// goal.th = 0;
  // do{
  //   update_pos(&prevSteps);
  //   currPos = GetPosture();
	// 	ClearFSet(f_set_vlin);
	// 	ClearFSet(f_set_vrot);
  //   FuzzyAvoid();
	// 	condition = GoTo2(&prevSteps,&goal);
	// 	DeFuzzify(f_set_vrot, 3, &rot);
	// 	DeFuzzify(f_set_vlin, 4, &vel);
  //
	// 	f_vlin = ResponseToVel(vel);
	// 	f_vrot = ResponseToRot(rot);
	// 	SetPolarSpeed(f_vlin,f_vrot);
  //   printf("\n%f, \t %f",currPos.x,currPos.y);
	// 	Sleep(100);
	// }while(condition != 1);
	// Stop();
  printf("\nDone...\n");
  return(0);
 }
//==============================================================================//
//                                  end main                                    //
//==============================================================================//
