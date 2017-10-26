#include "interface.h"
#include <math.h>
#define ROBOT_NUMBER "0"
#define V_MAX 50
#define V_MIN 10
#define MM_PER_PULSE 0.129
#define NO_DANGER 300
#define FULL_DANGER 400
#define FORCE_FULL_DANGER 700
#define EXPAND_ERR 0.35
#define CELLSIZE 25


//==============================================================================//
//                                FINAL EXAM                                    //
//==============================================================================//

int rWheelGlobal = 0;
int lWheelGlobal = 0;
Posture GlobalPos = GetPosture();
void TurnOffLights();
void SetGlobalPos(int x, int y, float th){
  GlobalPos.x  = x;
  GlobalPos.y  = y;
  GlobalPos.th = th;
  SetPosture(GlobalPos.x,GlobalPos.y,GlobalPos.th);
  printf("GLOBAL TH: %f",GlobalPos.th);
}

void UpdatePos    (Steps *prev){
    Steps s         = GetSteps();
    int rWheelDist  = (s.r - rWheelGlobal);
    int lWheelDist  = (s.l - lWheelGlobal);
    rWheelGlobal    = s.r; //setting global
    lWheelGlobal    = s.l;
    float dRight    = rWheelDist * MM_PER_PULSE;
    float dLeft     = lWheelDist * MM_PER_PULSE;
    float delta     = (dRight - dLeft) / ROBOT_DIAMETER;
    float dist      = ((dRight + dLeft ) / 2.0);
    float dx        = dist * (cos(delta/2.0));
    float dy        = dist * (sin(delta/2.0));
    Posture p       = GetPosture();
    float x         = p.x + (dx * cos(p.th)) - (dy * sin(p.th));
    float y         = p.y + (dx * sin(p.th)) + (dy * cos(p.th));
    float th        = p.th + delta;
          th        = fmod(th,PI*2);
    //*prev           = GetSteps();
    SetPosture        (x,y,th);
    p               = GetPosture();
}

int GoTo2(Steps *prevSteps, Posture *goal){

	float dx_dist       ,
	      dy_dist       ,
	      th_dist       ,
	      th_err        ,
	      vl, vr        ,
	      p_vel, p_th   ,
	      pos_err       ,
	      kpos      = 1 ,
	      kth       = -3,
	      delta_pos = 35;
  FPred Pos_Left, Pos_Right, Pos_Ahead, Pos_Here, DANGER, Obs_Left,Obs_Right,Obs_Ahead;
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
	th_err      = DEG(th_err);
	Pos_Left    = RampUp(th_err, 0, 60);
	Pos_Right   = RampDown(th_err, -60, 0);
	Pos_Ahead   = MIN(RampUp(th_err, -30, 0),
	      	      RampDown(th_err, 0, 30));
	Pos_Here    = RampDown(pos_err, 10, 50); //default 10,50


  Sensors ir  = GetIR();
  Obs_Left    = RampUp(MIN(ir.sensor[5],ir.sensor[6]),NO_DANGER,FULL_DANGER);
  Obs_Ahead   = RampUp(MIN(ir.sensor[0],ir.sensor[7]),NO_DANGER,FULL_DANGER);
  Obs_Right   = RampUp(MIN(ir.sensor[1],ir.sensor[2]),NO_DANGER,FULL_DANGER);
  DANGER      = MAX(MAX(Obs_Left,Obs_Right),Obs_Ahead);

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
  if(Obs_Left > 0.5){
    SetSingleLED(5,1);
  }
  if(Obs_Ahead > 0.5){
    SetSingleLED(0,1);
    SetSingleLED(7,1);
  }
  if(Obs_Right > 0.5){
    SetSingleLED(2,1);
  }
  TurnOffLights();
	if(pos_err < delta_pos){
		return 1;
  }
	return 0;
}

void FuzzyAvoid(){
  Sensors ir = GetIR();
  FPred Obs_Left,Obs_Right,Obs_Ahead, DANGER;
  /* Force Rules */
  if (ir.sensor[5] > FULL_DANGER){                              DANGER = 1;}
  if (ir.sensor[2] > FULL_DANGER){                              DANGER = 1;}
  if (ir.sensor[0] > FULL_DANGER || ir.sensor[7] > FULL_DANGER){DANGER = 1;}
  DANGER    = RampUp(MIN(ir.sensor[5],ir.sensor[6]),NO_DANGER,FULL_DANGER);
  Obs_Left  = RampUp(MIN(ir.sensor[5],ir.sensor[6]),NO_DANGER,FULL_DANGER);
  Obs_Ahead = RampUp(MIN(ir.sensor[0],ir.sensor[7]),NO_DANGER,FULL_DANGER);
  Obs_Right = RampUp(MIN(ir.sensor[1],ir.sensor[2]),NO_DANGER,FULL_DANGER);

  RULESET;
  IF (AND(Obs_Left,  NOT(Obs_Right)));                 ROT(RIGHT);
  IF (AND(Obs_Right, NOT(Obs_Left)));                  ROT(LEFT);
  IF (AND(Obs_Right, Obs_Left));                       ROT(AHEAD);
  IF (Obs_Ahead);                                      VEL(BACK);
  IF (AND((OR(Obs_Right, Obs_Left)), NOT(Obs_Ahead))); VEL(SLOW);
  IF (NOT(OR(OR(Obs_Right,Obs_Left), Obs_Ahead)));     VEL(FAST);
  RULEEND;
}

double ResponseToVel (double response){
  return (V_MIN + response *(V_MAX-V_MIN));
}

double ResponseToRot (double response){
  return (PI - response * (2 * PI));
}

char mapname[50] = "../real_robot/src/SampleMaps/custom.txt";
Map* grid        = CreateMapFromFile(0,mapname);
int Err_counter  = 0; //global Err-counter

int RePlan(Cell startCell, Cell goalCell); //constructor
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
void PaddObs(int i, int j){
  Cell Mark;
  Mark.i = i;
  Mark.j = j;
  int CellState = GetCellState(grid,i,j);
  switch(CellState){
    case 0:
      printf("Obs margin detected on goal cell!");
      Stop();
      break;
    case -1:
      printf("Obs detected on start cell");
      break;
    case -3:
      break;
    case -4:
      break;
    case -5:
      break;
    default:
      ChangeCellState(grid,i,j,-8);
      break;
  }
}
void PaddBorder(int i, int j){
  Cell Mark;
  Mark.i = i;
  Mark.j = j;
  int CellState = GetCellState(grid,i,j);
  switch(CellState){
    case 0:
      printf("Obs margin detected on goal cell!");
      Stop();
      break;
    case -1:
      printf("Obs detected on start cell");
      break;
    case -3:
      break;
    case -5:
      break;
    default:
      ChangeCellState(grid,i,j,-8);
      break;
  }
}
void addPaddingObstacles(){
      //add padding around obstacles to avoid disruption of robots-path.
      int h = grid->height;
      int w = grid->width;
      for(int i=0;i<h;i++){
        for(int j=0;j<w;j++){
          int check = GetCellState(grid,i,j);
          if(check == -3){
            for(int a = -1; a<=1;a++){
              for(int b= -1; b<=1;b++){
                PaddObs(i+a,j+b);
              }
            }
          }
        }
      }
}
void addPaddingBorderTop(){
      //add padding around border to avoid disruption of robots-path.
  int h = grid->height-3;
  int w = grid->width-3;
  for(int i=0;i<h;i++){
    for(int j=0;j<w;j++){
      int check = GetCellState(grid,i,j);
      if(check == -4){
        for(int a = 1; a<=2;a++){
          for(int b=1; b<=2;b++){
            PaddBorder(i+a,j+b);
          }
        }
      }
    }
  }
}
void addPaddingBorderBot(){
  //add padding around border to avoid disruption of robots-path.
  int h = grid->height-1;
  int w = grid->width-1;
  for(int i=h;i>2;i--){
    for(int j=w;j>2;j--){
      int check = GetCellState(grid,i,j);
      if(check == -4){
        for(int a = 1; a<=2;a++){
          for(int b=1; b<=2;b++){
            PaddBorder(i-a,j-b);
          }
        }
      }
    }
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
  double x,y,th,vel,rot;
  x = 0; y = 0; th= 0;
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

  SetPosture(arrayX[counter],arrayY[counter],th);     //set startpos
  for(int i=counter-1; i>=0; i--){                    //start from the last pos in array(start) -> goto -> goal
    condition=0;                                      //set to 0 each time.
    printf("\nNodes Until Goal = %d,SizeOf Arr=%d\n",i,counter);
    goal.x        = arrayX[i];
    goal.y        = arrayY[i];
    goal.th       = 0;
    printf("\n=== Moving Towards: (%f,%f) === \n",goal.x,goal.y);
    Err_counter = 0;                                  //reset err counter.
    do{
        UpdatePos        (&prevSteps);
        ClearFSet         (f_set_vlin);
        ClearFSet         (f_set_vrot);
        FuzzyAvoid        ();
        Posture now     = GetPosture();
        storeNewStart.i = (int)now.x/CELLSIZE;
        storeNewStart.j = (int)now.y/CELLSIZE;
        RePlan            (storeNewStart,storeGoal);
        condition       = GoTo2(&prevSteps,&goal);
        DeFuzzify         (f_set_vrot, 3, &rot);
        DeFuzzify         (f_set_vlin, 4, &vel);

        f_vlin          = ResponseToVel(vel);
        f_vrot          = ResponseToRot(rot);
        SetPolarSpeed     (f_vlin,f_vrot);
        Sleep(100);
    }while(condition != 1);
  }
  Stop();
}
void TurnOffLights(){
  SetSingleLED(5,0);
  SetSingleLED(2,0);
  SetSingleLED(7,0);
  SetSingleLED(0,0);
}
void Plan(Cell startCell, Cell goalCell){
  ClearMap             (grid);                         //for replanning purposes
  addPaddingObstacles  ();                             //add padding to obstacles
  addPaddingBorderTop  ();                             //add padding to grid-border
  addPaddingBorderBot  ();                             // -||-
  TurnOffLights        ();
  ChangeCellState      (grid,startCell.i,startCell.j,-1);
  ChangeCellState      (grid,goalCell.i,goalCell.j,0);
  int maxValue       = 0,
      counter        = 1,
      n1,n2,n3,n4       ;                              //neighbours

  List * Path        = CreateList(1);
  Cell                 bestCell;
  Cell                 nextCell;
  ClearList            (Path,1);
  if                   (Search(goalCell)){
    Push               (Path,startCell);
    maxValue         = GetMaxValue(grid);              //assigning max-value
    bestCell.i       = startCell.i;
    bestCell.j       = startCell.j;


    while(maxValue  != MAP_GOAL){                      //while we are not at the goal
      n1             = GetCellState(grid,bestCell.i,bestCell.j-1);
      n2             = GetCellState(grid,bestCell.i,bestCell.j+1);
      n3             = GetCellState(grid,bestCell.i-1,bestCell.j);
      n4             = GetCellState(grid,bestCell.i+1,bestCell.j);
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
        Push           (Path,nextCell);
        bestCell.i   = nextCell.i;
        bestCell.j   = nextCell.j;
        bestCell.h_value = 0;
        ChangeCellState(grid,bestCell.i,bestCell.j,-9); // mark path with custom sign
        counter++;

    }                                                   //end while
  }else{
    printf("\nPath was NOT found, terminating...");
    exit(1);
  }                                                    // end if-else

  printf("\nPath FOUND!\n");
  //expanding the cells
  double arrayX[counter];
  double arrayY[counter];
  Cell temp;
  for(int i=0; i < counter; i++){
    temp      = Pop(Path);
    arrayX[i] = temp.i * CELLSIZE;
    arrayY[i] = temp.j * CELLSIZE;
  }

  for(int i=0;i < (sizeof(arrayX) / sizeof(arrayX[0])); i++){
    printf("(X: %f, Y: %f) \n",arrayX[i], arrayY[i]);
  }

  PrintMap(grid);
  FuzzyPlanSearch(arrayX,arrayY,counter-1);
}

int PlaceObs(Cell startCell, Cell goalCell){
    Posture now      = GlobalPos; // store the global pos
    now              = GetPosture(); // update the 'global' pos.
    Sensors ir       = GetIR();
    Cell               Revisited_pos, newStart;
    newStart         = startCell;
    float currentRot = now.th; //current rotation.
    if(currentRot >  0){currentRot     = currentRot - 2 * PI;}
    if(currentRot < -0){currentRot     = currentRot + 2 * PI;}
    currentRot       = fmod(currentRot,PI*2);
    currentRot       = DEG(currentRot);
    Stop(); // stop robot momentarily to improve accuracy in placing of obs
    newStart.i       = (int)(now.x/CELLSIZE); //Change to Cell values.
    newStart.j       = (int)(now.y/CELLSIZE); //Scale down
    printf("\nReplanning from: %d %d", newStart.i, newStart.j);
    //setting the obs according to robot-rotation in grid
    if(currentRot >= 0 && currentRot <= 45){
        printf("\n0-45");
        if (ir.sensor[5] > FULL_DANGER){ //left
              if(GetCellState(grid,newStart.i,newStart.j+2) != MAP_BORDER){ChangeCellState(grid,newStart.i,newStart.j+2,-3);SetSingleLED(5,1); } //add obs in map
        }
        if (ir.sensor[2] > FULL_DANGER){ //right
              if(GetCellState(grid,newStart.i,newStart.j-2) != MAP_BORDER){ChangeCellState(grid,newStart.i,newStart.j-2,-3);SetSingleLED(2,1); }
        }
        if (ir.sensor[0] > FULL_DANGER || ir.sensor[7] > FULL_DANGER){ //ahead
              if(GetCellState(grid,newStart.i+2,newStart.j) != MAP_BORDER){ChangeCellState(grid,newStart.i+2,newStart.j,-3);SetSingleLED(0,1);SetSingleLED(7,1); }
        }
    }
    if(currentRot > 45 && currentRot <= 135){
       printf("\n45-90-135");
         if (ir.sensor[5] > FULL_DANGER){ //left
               if(GetCellState(grid,newStart.i-2,newStart.j) != MAP_BORDER){ChangeCellState(grid,newStart.i-2,newStart.j,-3);SetSingleLED(5,1); } //add obs in map
         }
         if (ir.sensor[2] > FULL_DANGER){ //right
               if(GetCellState(grid,newStart.i+2,newStart.j) != MAP_BORDER){ChangeCellState(grid,newStart.i+2,newStart.j,-3);SetSingleLED(2,1); }
         }
         if (ir.sensor[0] > FULL_DANGER || ir.sensor[7] > FULL_DANGER){ //ahead
               if(GetCellState(grid,newStart.i,newStart.j+2) != MAP_BORDER){ChangeCellState(grid,newStart.i,newStart.j+2,-3);SetSingleLED(0,1);SetSingleLED(7,1); }
         }
     }
    if(currentRot > 135 && currentRot <= 225){
      printf("\n135-180-225");
        if (ir.sensor[5] > FULL_DANGER){ //left
              if(GetCellState(grid,newStart.i,newStart.j-2) != MAP_BORDER){ChangeCellState(grid,newStart.i,newStart.j-2,-3);SetSingleLED(5,1); } //add obs in map
        }
        if (ir.sensor[2] > FULL_DANGER){ //right
              if(GetCellState(grid,newStart.i,newStart.j+2) != MAP_BORDER){ChangeCellState(grid,newStart.i,newStart.j+2,-3);SetSingleLED(2,1); }
        }
        if (ir.sensor[0] > FULL_DANGER || ir.sensor[7] > FULL_DANGER){ //ahead
              if(GetCellState(grid,newStart.i-2,newStart.j) != MAP_BORDER){ChangeCellState(grid,newStart.i-2,newStart.j,-3);SetSingleLED(0,1);SetSingleLED(7,1); }
        }

    }//klar // KOLLA PÅ DESSA IGEN!!! KORRIGERA.
    if(currentRot > 225 && currentRot <= 315){
      printf("\n225-270-315");
        if (ir.sensor[5] > FULL_DANGER){ //left
              if(GetCellState(grid,newStart.i+2,newStart.j) != MAP_BORDER){ChangeCellState(grid,newStart.i+2,newStart.j,-3);SetSingleLED(5,1); } //add obs in map
        }
        if (ir.sensor[2] > FULL_DANGER){ //right
              if(GetCellState(grid,newStart.i-2,newStart.j) != MAP_BORDER){ChangeCellState(grid,newStart.i-2,newStart.j,-3);SetSingleLED(2,1); }
        }
        if (ir.sensor[0] > FULL_DANGER || ir.sensor[7] > FULL_DANGER){ //ahead
              if(GetCellState(grid,newStart.i,newStart.j-2) != MAP_BORDER){ChangeCellState(grid,newStart.i,newStart.j-2,-3);SetSingleLED(0,1);SetSingleLED(7,1); }
        }
    } //klar
    if(currentRot > 315 && currentRot < 360){
      printf("\n315-360");
        if (ir.sensor[5] > FULL_DANGER){ //left
              if(GetCellState(grid,newStart.i,newStart.j+2) != MAP_BORDER){ChangeCellState(grid,newStart.i,newStart.j+2,-3);SetSingleLED(5,1); } //add obs in map
        }
        if (ir.sensor[2] > FULL_DANGER){ //right
              if(GetCellState(grid,newStart.i,newStart.j-2) != MAP_BORDER){ChangeCellState(grid,newStart.i,newStart.j-2,-3);SetSingleLED(2,1); }
        }
        if (ir.sensor[0] > FULL_DANGER || ir.sensor[7] > FULL_DANGER){ //ahead
              if(GetCellState(grid,newStart.i+2,newStart.j) != MAP_BORDER){ChangeCellState(grid,newStart.i+2,newStart.j,-3);SetSingleLED(0,1);SetSingleLED(7,1); }

        }

    } //klar

    //convert back to cells
    startCell.i = newStart.i;
    startCell.j = newStart.j;
    goalCell.i  = goalCell.i/CELLSIZE;
    goalCell.j  = goalCell.j/CELLSIZE;
    Plan(startCell,goalCell);
    return 1;
}

int CheckErr(Cell startCell, Cell goalCell){
  Posture now     = GlobalPos; // store the global pos
  now             = GetPosture(); // update the 'global' pos.
  Cell              Revisited_pos;
  Sensors ir      = GetIR();
  Revisited_pos.i = now.x;
  Revisited_pos.j = now.y;
  float currentRot = now.th; //current rotation.
  if(currentRot >  0){ currentRot     = currentRot - 2 * PI;} //normalize
  if(currentRot < -0){ currentRot     = currentRot + 2 * PI;} //normalize
  currentRot       = fmod(currentRot,PI*2);
  currentRot       = DEG(currentRot);
  if(((Revisited_pos.i <= now.x+EXPAND_ERR) && (Revisited_pos.i >= now.x-EXPAND_ERR)) &&
    ((Revisited_pos.j <= now.y+EXPAND_ERR) && (Revisited_pos.j >= now.y-EXPAND_ERR))){ //increase err-path
      Err_counter++;
      printf("\nErr_count: %d,\tth_: %f",Err_counter,currentRot);
  }
  if((Err_counter >= 4 && ((ir.sensor[5] > FULL_DANGER) || (ir.sensor[2] > FULL_DANGER) || ((ir.sensor[0] > FULL_DANGER || ir.sensor[7] > FULL_DANGER))))){
      if(PlaceObs(startCell,goalCell) == 1){ //if placing of obs went OK.
        Err_counter = 0;
        printf("\n\nPLACEOBS COMPLETE!\n\n");
        return 1;
    }
  }
  return 0;
}

int RePlan(Cell startCell, Cell goalCell){
  Steps prevSteps = GetSteps();
  UpdatePos         (&prevSteps);
  if(CheckErr       (startCell, goalCell) == 1){ //checking for errors.
    printf          ("\n  \t\t\t\t === NEW UPDATED MAP! ===");
    goalCell.i    = goalCell.i/CELLSIZE; //convert to cells
    goalCell.j    = goalCell.j/CELLSIZE;
    PrintMap        (grid);
    printf          ("\nNEW START: %d,%d, GOAL: %d,%d\n",startCell.i,startCell.j,goalCell.i,goalCell.j);
    return 1;
  }
}

Cell menu(Cell *startCell,Cell *goalCell){
  ClearMap(grid);
  printf("Start i: ");
  scanf("%d",&startCell->i);
  printf("Start j: ");
  scanf("%d",&startCell->j);
  printf("Goal i : ");
  scanf("%d",&goalCell->i);
  printf("Goal j : ");
  scanf("%d",&goalCell->j);
  return *goalCell,*startCell;
}


int main(int argc, char *argv[]){
  epuck(ROBOT_NUMBER);
  Posture GlobalPos = GetPosture();
  int i, cellSize,choice;

  do{
    printf             ("Starting...\n");
    GlobalPos = GetPosture();
    printf("Global: x:%f y:%f th:%f",GlobalPos.x,GlobalPos.y,DEG(GlobalPos.th));
    printf("\nMenu: \n1.Set New Coordinates\n2.Exit\n>> ");
    Cell goalCell,startCell,Wall;
    ClearSteps();
    scanf("%d",&choice);
    switch(choice){
      case 1:
        menu(&startCell,&goalCell);
        goto EXEC;
        break;
      case 2:
        Stop();
        exit(1);
        break;
      default:
        break;
    }

  EXEC:
    GlobalPos = GetPosture();
    cellSize         = CELLSIZE;
    //manual tries...
    // startCell.i      = 10;
    // startCell.j      = 3;
    ChangeCellState(grid,startCell.i,startCell.j,-1);
    // goalCell.i       = 10;
    // goalCell.j       = 15;
    ChangeCellState(grid,goalCell.i,goalCell.j,0);
    SetGlobalPos(startCell.i*cellSize,startCell.j*cellSize,GlobalPos.th);
    addPaddingObstacles();
    addPaddingBorderTop();
    addPaddingBorderBot();
    PrintMap(grid);
    Plan(startCell,goalCell);

    printf("\nDone...\n");
    PrintMap(grid);
    }while(choice != 2);

    return(0);
 }
//==============================================================================//
//                                  end main                                    //
//==============================================================================//
