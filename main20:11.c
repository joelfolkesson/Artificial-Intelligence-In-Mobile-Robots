#include "interface.h"
#include <math.h>
#define ROBOT_NUMBER "7"
#define V_MAX 90
#define V_MIN 40
#define MM_PER_PULSE 0.135
#define NO_DANGER 300
#define FULL_DANGER 800
#define FORCE_FULL_DANGER 700
#define EXPAND_ERR 0.35
#define CELLSIZE 25


//==============================================================================//
//                                FINAL EXAM                                    //
//==============================================================================//

  printf("\nPath FOUND!\n");
  //expanding the cells
  double arrayX[counter];
  double arrayY[counter];
  Cell temp;
  for(int i=0; i < counter; i++){ //populating arrays
    temp      = Pop(Path);
    arrayX[i] = temp.i * CELLSIZE;
    arrayY[i] = temp.j * CELLSIZE;
  }

  //reducing arrays in C
  int n = counter; //storing previous counter
  int z = 0; // new Counter
  for (int i = 0; i < n; i++) {
    if (i == n - 1 || z == 0 || (arrayX[i] != arrayX[i + 1] || arrayX[i] != arrayX[z - 1]) && (arrayY[i] != arrayY[i + 1] || arrayY[i] != arrayY[z - 1])) {
      //checking if infront and behind us is the same, if it is, move it in the array.
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
  for(int i=0;i < (sizeof(newArrayX) / sizeof(newArrayX[0])); i++){
    printf("(X: %f, Y: %f) \n",newArrayX[i], newArrayY[i]);
  }



  PrintMap(grid);
  FuzzyPlanSearch(newArrayX,newArrayY,z-1); //sending along the formatted array-path with the new counter.
}
//==============================================================================//
//                                  end main                                    //
//==============================================================================//
