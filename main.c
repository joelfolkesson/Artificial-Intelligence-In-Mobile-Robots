#include "interface.h"
#include <math.h>
#define ROBOT_NUMBER "7"
#define V_MAX 90
#define V_MIN 40
#define MM_PER_PULSE 0.135
#define NO_DANGER 400
#define FULL_DANGER 600
#define FORCE_FULL_DANGER 700
#define EXPAND_ERR 0.35
#define CELLSIZE 25


//==============================================================================//
//                                FINAL EXAM                                    //
//==============================================================================//
char mapname[50]  = "../real_robot/src/SampleMaps/empty.txt";
Map* grid         = CreateMapFromFile(0,mapname);
int rWheelGlobal  = 0;
int lWheelGlobal  = 0;
Posture GlobalPos = GetPosture();
Cell                GlobalGoalCell;

void SetGlobalPos(int x, int y, float th){
  GlobalPos.x     = x;
  GlobalPos.y     = y;
  GlobalPos.th    = th;
  SetPosture(GlobalPos.x,GlobalPos.y,GlobalPos.th);
}

void UpdatePos(){
  Posture pos;
	Steps s   = GetSteps();
  float distLeft  = s.l - lWheelGlobal;
  float distRight = s.r - rWheelGlobal;

  rWheelGlobal    = s.r;
  lWheelGlobal    = s.l;

  float lWheelDist= distLeft*MM_PER_PULSE;
  float rWheelDist= distRight*MM_PER_PULSE;

  float dist      = (lWheelDist + rWheelDist)/2;
  float delta     = (rWheelDist - lWheelDist)/ROBOT_DIAMETER;
  float dx        = dist* cos(delta/2);
  float dy        = dist* sin(delta/2);

  pos             = GetPosture();

  pos.x           = pos.x + dx*cos(pos.th) - dy*sin(pos.th);
  pos.y           = pos.y + dx*sin(pos.th) + dy*cos(pos.th);
  pos.th          = pos.th + delta;
  float th        = pos.th;
        th        = fmodf(th,2*PI);
  SetPosture((float)pos.x, (float)pos.y,(float)pos.th);
}

int GoTo2(Posture goal){

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
  Posture currentPos;


  currentPos  = GetPosture();
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

  th_err      = DEG(th_err);
  Sensors ir  = GetIR();
  Pos_Left    = RampUp(th_err, 0, 60);
  Pos_Right   = RampDown(th_err, -60, 0);
  Pos_Ahead   = min(RampUp(th_err, -30, 0), RampDown(th_err, 0, 30));
  Pos_Here    = RampDown(pos_err, 20, 50); //def 10,50

  Obs_Left    = RampUp(min(ir.sensor[5],ir.sensor[6]),NO_DANGER,FULL_DANGER);
  Obs_Ahead   = RampUp(min(ir.sensor[0],ir.sensor[7]),NO_DANGER,FULL_DANGER);
  Obs_Right   = RampUp(min(ir.sensor[1],((ir.sensor[2]))),NO_DANGER,FULL_DANGER);
  DANGER      = max(max(Obs_Left, Obs_Right), Obs_Ahead);


  RULESET;
    //no danger
    IF (AND(NOT(DANGER), AND(Pos_Left, NOT(Pos_Here)))); 			   ROT(LEFT);
    IF (AND(NOT(DANGER), AND(Pos_Right, NOT(Pos_Here)))); 			   ROT(RIGHT);
    IF (AND(NOT(DANGER), AND(Pos_Ahead, NOT(Pos_Here)))); 			   ROT(AHEAD);
    IF (AND(NOT(DANGER), AND(Pos_Ahead, NOT(Pos_Here)))); 			   VEL(FAST);
    IF (OR(Pos_Here, NOT(Pos_Ahead)));		 			           VEL(NONE);

    //danger
    IF (AND(DANGER, AND(Obs_Right, NOT(Obs_Left)))); 			           ROT(LEFT);
    IF (AND(DANGER, AND(Obs_Left, NOT(Obs_Right)))); 			           ROT(RIGHT);
    IF (AND(DANGER, AND(Obs_Right, Obs_Left))); 				   ROT(AHEAD);

    IF (AND(DANGER, AND(AND(Obs_Ahead, Obs_Left), Obs_Right)));		           VEL(BACK);
    IF (AND(DANGER, (OR(Obs_Right, Obs_Left), NOT(Obs_Ahead)))); 	           VEL(SLOW);
    IF (NOT(OR(DANGER, (OR(OR(Obs_Right,Obs_Left), Obs_Ahead)))));                 VEL(FAST);



    IF (AND(Obs_Left,  NOT(Obs_Right)));                                           ROT(RIGHT);
    IF (AND(Obs_Right, NOT(Obs_Left)));                                            ROT(LEFT);
    IF (AND(Obs_Right, Obs_Left));                                                 ROT(AHEAD);
    IF (Obs_Ahead);                                                                VEL(RIGHT);
    IF (AND((OR(Obs_Right, Obs_Left)), NOT(Obs_Ahead)));                           VEL(SLOW);
    IF (NOT(OR(OR(Obs_Right,Obs_Left), Obs_Ahead)));                               VEL(FAST);
    RULEEND;
    if(pos_err < delta_pos){ //Acceptable to look for next node within this range (could use Pos_Here == 1)
       return 1;
    }
    return 0;
}

double ResponseToVel (double response){
  return (V_MIN + response *(V_MAX-V_MIN));
}

double ResponseToRot (double response){
  return (PI - response * (2 * PI));
}

int RePlan(Cell goalCell); //constructor
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
  int h                  = grid->height-1;
  int w                  = grid->width-1;
  for(int i = 0; i < h; i++){
    for(int j = 0; j < w; j++){
      int check          = GetCellState(grid,i,j);
      if(check == -3){
        for(int a = -1; a<=1;a++){
          for(int b=-1; b<=1;b++){
             PaddObs(i+a,j+b);
          }
        }
      }
    }
  }
}
void addPaddingBorderTop(){
      //add padding around border to avoid disruption of robots-path.
  int h                 = grid->height-3;
  int w                 = grid->width-3;
  for(int i = h; i > 2; i--){
    for(int j = w; j > 2 ; j--){
      int check         = GetCellState(grid,i,j);
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
  int h                 = grid->height-1;
  int w                 = grid->width-1;
  for(int i = h; i > 2; i--){
    for(int j = w; j > 2 ; j--){
      int check         = GetCellState(grid,i,j);
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
  double            vel,
                    rot,
      x           = 0,
      y           = 0,
      th          = 0;
  int condition   = 0,
      count       = 0;
  Posture goal    = GetPosture();
  Cell              storeNewStart,storeGoal;
  size_t arrsize  = (sizeof(arrayX)/sizeof(arrayY[0])); //storing last-goal so i can use it in replan
  for(int i=0;i < arrsize;++i){
    storeGoal.i   = arrayX[i];
    storeGoal.j   = arrayY[i];

  }
  SetPosture(arrayX[counter],arrayY[counter],goal.th);  //set posture
  for(int i=counter-1; i>=0; i--){                      //start from the last pos in array(start) -> goto -> goal
    condition=0;                                        //set to 0 each time.
  //  printf("\nNodes Until Goal = %d,SizeOf Arr=%d\n",i,counter);
    goal.x        = arrayX[i];
    goal.y        = arrayY[i];
    printf("\n=== Moving Towards: (%f,%f) === \n",goal.x,goal.y);
    do{
        UpdatePos         ();
        ClearFSet         (f_set_vlin);
        ClearFSet         (f_set_vrot);
        Posture now     = GetPosture();
        SetGlobalPos      (now.x,now.y,now.th);
        storeNewStart.i = (int)now.x/CELLSIZE;
        storeNewStart.j = (int)now.y/CELLSIZE;
        condition       = GoTo2(goal);
        DeFuzzify         (f_set_vrot, 3, &rot);
        DeFuzzify         (f_set_vlin, 4, &vel);
        f_vlin          = ResponseToVel(vel);
        f_vrot          = ResponseToRot(rot);
        SetPolarSpeed     (f_vlin,f_vrot);
        RePlan            (storeGoal);
        Sleep(50);
    }while(condition != 1 && storeNewStart.i != storeGoal.i && storeNewStart.j != storeGoal.j); //enhanced check.
  }
  Stop();
}

void Plan(Cell startCell, Cell goalCell){
  ClearMap             (grid);                         //for replanning purposes
  addPaddingObstacles  ();                             //add padding to obstacles
  addPaddingBorderTop  ();                             //add padding to grid-border
  addPaddingBorderBot  ();                             // -||-
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
    PrintMap(grid);
    printf("\nPath was NOT found, terminating...");
    exit(1);
  }                                                    // end if-else

  printf("\nPath FOUND!\n");
  //expanding the cells
  double arrayX[counter];
  double arrayY[counter];
  Cell   temp;
  for(int i=0; i < counter; i++){ //populating arrays
    temp      = Pop(Path);
    arrayX[i] = temp.i * CELLSIZE; //scale up
    arrayY[i] = temp.j * CELLSIZE;
  }

  //reducing arrays, i'm so happy with this method of doing it. It took me ages to accomplish. (and lots of abandoned attempts)
  int n = counter;                 //storing previous counter
  int z = 0;                       // new Counter
  for (int i = 0; i < n; i++) {
    if (i == n - 1 || z == 0 || (arrayX[i] != arrayX[i + 1] || arrayX[i] != arrayX[z - 1]) && (arrayY[i] != arrayY[i + 1] || arrayY[i] != arrayY[z - 1])) {
                                   //checking turns on a node, if infront and behind us is the same, if it is, move current node in the array.
                                   //z = 0, means goal, we want to keep that.
                                   //i = n-1 means start, we want to keep that.
      arrayX[z] = arrayX[i];
      arrayY[z] = arrayY[i];
      z++;
    }
  }
  double newArrayX[z];  //create new arrays to store the final result in based on new counter for new result.
  double newArrayY[z];
  for(int i=0; i< z; i++){
    newArrayX[i] = arrayX[i];
    newArrayY[i] = arrayY[i];
  }
  for(int i=0;i < (sizeof(newArrayX) / sizeof(newArrayX[0])); i++){ //array list
    printf("(X: %f, Y: %f) \n",newArrayX[i], newArrayY[i]);
  }



  PrintMap(grid);
  FuzzyPlanSearch(newArrayX,newArrayY,z-1); //sending along the formatted array-path with the new counter.
}

int PlaceObs(Cell goalCell){
     Posture now      = GlobalPos;                               // store the global pos
     now              = GetPosture();                            // update the 'global' pos.
     Sensors ir       = GetIR();
     Cell               newStart;
     float currentRot = now.th;                                  //current rotation.
     if(currentRot >  0){currentRot     = currentRot - 2 * PI;}
     if(currentRot < -0){currentRot     = currentRot + 2 * PI;}
     currentRot       = fmod(currentRot,PI*2);
     currentRot       = DEG(currentRot);
     newStart.i       = (int)(now.x/CELLSIZE);                   //Change to Cell values.
     newStart.j       = (int)(now.y/CELLSIZE);                   //Scale down

     //setting the obs according to robot-rotation in grid
    if(currentRot >= 0 && currentRot <= 45){
        printf("\n0-45");
        if (ir.sensor[5] > FULL_DANGER){ //left
              if(GetCellState(grid,newStart.i,newStart.j+3) != MAP_BORDER){ChangeCellState(grid,newStart.i,newStart.j+3,-3);  } //add obs in map
        }
        if (ir.sensor[2] > FULL_DANGER){ //right
              if(GetCellState(grid,newStart.i,newStart.j-3) != MAP_BORDER){ChangeCellState(grid,newStart.i,newStart.j-3,-3);  }
        }
        if (ir.sensor[0] > FULL_DANGER || ir.sensor[7] > FULL_DANGER){ //ahead
              if(GetCellState(grid,newStart.i+3,newStart.j) != MAP_BORDER){ChangeCellState(grid,newStart.i+3,newStart.j,-3);   }
        }
    }
    if(currentRot > 45 && currentRot <= 135){
       printf("\n45-90-135");
         if (ir.sensor[5] > FULL_DANGER){ //left
               if(GetCellState(grid,newStart.i-3,newStart.j) != MAP_BORDER){ChangeCellState(grid,newStart.i-3,newStart.j,-3);  } //add obs in map
         }
         if (ir.sensor[2] > FULL_DANGER){ //right
               if(GetCellState(grid,newStart.i+3,newStart.j) != MAP_BORDER){ChangeCellState(grid,newStart.i+3,newStart.j,-3);  }
         }
         if (ir.sensor[0] > FULL_DANGER || ir.sensor[7] > FULL_DANGER){ //ahead
               if(GetCellState(grid,newStart.i,newStart.j+3) != MAP_BORDER){ChangeCellState(grid,newStart.i,newStart.j+3,-3);   }
         }
     }
    if(currentRot > 135 && currentRot <= 225){
      printf("\n135-180-225");
        if (ir.sensor[5] > FULL_DANGER){ //left
              if(GetCellState(grid,newStart.i,newStart.j-3) != MAP_BORDER){ChangeCellState(grid,newStart.i,newStart.j-3,-3);  } //add obs in map
        }
        if (ir.sensor[2] > FULL_DANGER){ //right
              if(GetCellState(grid,newStart.i,newStart.j+3) != MAP_BORDER){ChangeCellState(grid,newStart.i,newStart.j+3,-3);  }
        }
        if (ir.sensor[0] > FULL_DANGER || ir.sensor[7] > FULL_DANGER){ //ahead
              if(GetCellState(grid,newStart.i-3,newStart.j) != MAP_BORDER){ChangeCellState(grid,newStart.i-3,newStart.j,-3);   }
        }

    }//klar // KOLLA PÅ DESSA IGEN!!! KORRIGERA.
    if(currentRot > 225 && currentRot <= 315){
      printf("\n225-270-315");
        if (ir.sensor[5] > FULL_DANGER){ //left
              if(GetCellState(grid,newStart.i+3,newStart.j) != MAP_BORDER){ChangeCellState(grid,newStart.i+3,newStart.j,-3);  } //add obs in map
        }
        if (ir.sensor[2] > FULL_DANGER){ //right
              if(GetCellState(grid,newStart.i-3,newStart.j) != MAP_BORDER){ChangeCellState(grid,newStart.i-3,newStart.j,-3);  }
        }
        if (ir.sensor[0] > FULL_DANGER || ir.sensor[7] > FULL_DANGER){ //ahead
              if(GetCellState(grid,newStart.i,newStart.j-3) != MAP_BORDER){ChangeCellState(grid,newStart.i,newStart.j-3,-3);   }
        }
    } //klar
    if(currentRot > 315 && currentRot < 360){
      printf("\n315-360");
        if (ir.sensor[5] > FULL_DANGER){ //left
              if(GetCellState(grid,newStart.i,newStart.j+3) != MAP_BORDER){ChangeCellState(grid,newStart.i,newStart.j+3,-3);  } //add obs in map
        }
        if (ir.sensor[2] > FULL_DANGER){ //right
              if(GetCellState(grid,newStart.i,newStart.j-3) != MAP_BORDER){ChangeCellState(grid,newStart.i,newStart.j-3,-3);  }
        }
        if (ir.sensor[0] > FULL_DANGER || ir.sensor[7] > FULL_DANGER){ //ahead
              if(GetCellState(grid,newStart.i+3,newStart.j) != MAP_BORDER){ChangeCellState(grid,newStart.i+3,newStart.j,-3);   }

        }

    } //klar

    //convert back to cells
    goalCell.i  = goalCell.i/CELLSIZE;
    goalCell.j  = goalCell.j/CELLSIZE;
    Plan(newStart,goalCell);

}

int CheckErr(Cell goalCell){
  Sensors ir = GetIR();
  if((((ir.sensor[5] > FULL_DANGER) || (ir.sensor[2] > FULL_DANGER) || ((ir.sensor[0] > FULL_DANGER || ir.sensor[7] > FULL_DANGER))))){
        PlaceObs(goalCell);
        printf("\n\nPLACEOBS COMPLETE!\n\n");
        return 1;
    }
  return 0;
}

int RePlan(Cell goalCell){
  UpdatePos();
  if(CheckErr       (goalCell) == 1){ //checking for errors.
    printf          ("\n  \t\t\t\t === NEW UPDATED MAP! ===");
    goalCell.i    = goalCell.i/CELLSIZE; //convert to cells
    goalCell.j    = goalCell.j/CELLSIZE;
    Posture now = GetPosture();
    now.x /= (int)CELLSIZE;
    now.y /= (int)CELLSIZE;
    PrintMap        (grid);
    printf          ("\nNEW START: %f,%f, GOAL: %d,%d\n",now.x,now.y,goalCell.i,goalCell.j);
    if(goalCell.i == GlobalGoalCell.i && goalCell.j == GlobalGoalCell.j ){
      GlobalPos = GetPosture();
      printf("Global: x:%f y:%f th:%f\n",GlobalPos.x,GlobalPos.y,DEG(GlobalPos.th));
      printf("WE'VE REACHED THE FINAL GOAL!\n");
      Stop();
      exit(1);
    }
    return 1;
  }
  return 0;
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
  GlobalGoalCell.i = goalCell->i;
  GlobalGoalCell.j = goalCell->j;
  return *goalCell,*startCell;
}


int main(int argc, char *argv[]){
  epuck(ROBOT_NUMBER);
  Posture GlobalPos = GetPosture();
  int                 i, cellSize,choice;

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
