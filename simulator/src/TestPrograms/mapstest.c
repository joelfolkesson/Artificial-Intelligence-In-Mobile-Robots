//=============================================================================
//mapstest.c
//=============================================================================


//=============================================================================
//Includes
//=============================================================================
#include <stdio.h>
#include <stdlib.h>
#include "maps.h"
//=============================================================================


//=============================================================================
//Main
//=============================================================================
int main(void)
{
  char str[MAX_FILENAME_SIZE];
  int i,j,value,type,width,height,max_value,exit;
  Coordinates c;
  Map* map;
  map = NULL;
  exit = 0;


  while (exit == 0)
  {
    printf("\n\nWhat would you like to do (0 to quit):\n");
    printf(" 1. CreateMapFromDefaultFile\n");
    printf(" 2. CreateMapFromFile\n");
    printf(" 3. SaveMapAsDefaultFile\n");
    printf(" 4. SaveMapAsFile\n");
    printf(" 5. CreateEmptyMap\n");
    printf(" 6. CreateBorderedMap\n");
    printf(" 7. ClearMap\n");
    printf(" 8. PrintMap\n");
    printf(" 9. ChangeCellState\n");
    printf("10. GetCellState\n");
    printf("11. FindStart\n");
    printf("12. FindGoal\n");
    printf("13. FindMax\n");
    printf("14. FindMin\n");
    printf("15. FindValue\n");
    printf("16. GetMaxValue\n");
    printf("17. GetMinValue\n");
    printf("18. FreeMap\n");
    printf("\nChoice: ");

    scanf("%d",&i);

    if ( ((i >= 7) && (i <= 17)) ||
         (i == 3) || (i == 4))
    {
    	if (map == NULL)
    	{
    		printf("Error - map needs to be initialised!\n");
    		continue;
      }
    }

    if ((i >= 1) && (i <=4))
    {
    	printf("Choose a type of file: \n");
    	printf(" 1. Map file (*.map)\n");
    	printf(" 2. Image file (*.ppm)\n");
    	printf(" 3. Symbol file (*.txt)\n");
    	printf("Choice: ");

    	scanf("%d",&j);

    	switch(j)
    	{
    		case 1:
    		  type = MAP;
    		  break;
    		case 2:
    		  type = IMG;
    		  break;
    		case 3:
    		  type = SYM;
    		  break;
    		default:
    		  type = MAP;
    		  printf("Invalid choice - using '.map' file...\n");
      }
    }

    switch(i)
    {
      case 0:
        exit = 1;
        break;
      case 1:
        map = CreateMapFromDefaultFile(type);
        if (map != NULL)
          printf("Map created!\n");
        else
        	printf("Error creating map!\n");
        break;
      case 2:
        printf("Enter file name from which to load the map: ");
        scanf("%s",str);
        map = CreateMapFromFile(type, str);
        if (map != NULL)
          printf("Map created!\n");
        else
        	printf("Error creating map!\n");
        break;
      case 3:
        SaveMapToDefaultFile(map, type);
        printf("Map saved!\n");
        break;
      case 4:
        printf("Enter file name to save: ");
	      scanf("%s",str);
	      SaveMapToFile(map, type, str);
        printf("Map saved!\n");
        break;
      case 5:
        printf("Enter width, height, and max value (-1 for default max value) for map: ");
        scanf("%d %d %d", &width,&height,&max_value);
        map = CreateEmptyMap(width,height,max_value);
        if (map != NULL)
          printf("Map created!\n");
        else
        	printf("Error creating map!\n");
        break;
      case 6:
        printf("Enter width, height, and max value (-1 for default) for map: ");
        scanf("%d %d %d", &width,&height,&max_value);
        map = CreateBorderedMap(width,height,max_value);
        if (map != NULL)
          printf("Map created!\n");
        else
        	printf("Error creating map!\n");
        break;
      case 7:
        ClearMap(map);
        printf("Map cleared!\n");
        break;
      case 8:
        PrintMap(map);
        printf("Done printing!\n");
        break;
      case 9:
        printf("Enter cell coordinates to change: ");
        scanf("%d %d", &i,&j);
        printf("Enter new state: \n");
        printf(" 0 MAP_GOAL\n");
        printf("-1 MAP_START\n");
        printf("-2 MAP_UNKNOWN\n");
        printf("-3 MAP_OBSTACLE\n");
        printf("-4 MAP_BORDER\n");
        printf("-5 MAP_ERROR\n");
        printf(">0 OTHER\n");
        scanf("%d", &value);
        ChangeCellState(map, i,j,value);
        printf("Value changed!\n");
        break;
      case 10:
        printf("Enter cell coordinates to read: ");
        scanf("%d %d", &i,&j);
        printf("Value is: %d\n", GetCellState(map, i,j));
        break;
      case 11:
        c = FindStart(map);
        printf("Start at: (%d,%d)\n", c.i,c.j);
        break;
      case 12:
        c = FindGoal(map);
        printf("Goal at: (%d,%d)\n", c.i,c.j);
        break;
      case 13:
        c = FindMax(map);
        printf("Max at: (%d,%d)\n", c.i,c.j);
        break;
      case 14:
        c = FindMin(map);
        printf("Min at: (%d,%d)\n", c.i,c.j);
        break;
      case 15:
        printf("Enter value to find: ");
        scanf("%d", &value);
        c = FindValue(map, value);
        printf("Value at: (%d,%d)\n", c.i,c.j);
        break;
      case 16:
        printf("Max is: %d\n", GetMaxValue(map));
        break;
      case 17:
        printf("Min is: %d\n", GetMinValue(map));
        break;
      case 18:
        FreeMap(map);
        printf("Map free!\n");
        map = NULL;
        break;
      default:
        printf("Invalid choice!\n");
    }
  }
  return 0;
}
//=============================================================================
