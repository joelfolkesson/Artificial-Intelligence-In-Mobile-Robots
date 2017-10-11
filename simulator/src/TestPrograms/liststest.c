//=============================================================================
//liststest.c
//=============================================================================


//=============================================================================
//Includes
//=============================================================================
#include <stdio.h>
#include <stdlib.h>
#include "lists.h"
//=============================================================================


//=============================================================================
//Main
//=============================================================================
int main(void)
{
  List* list = NULL;
  int i,j,exit;
  Cell c;
  exit = 0;

  while (exit == 0)
  {
    printf("\n\nWhat would you like to do (0 to quit):\n");
    printf(" 1. CreateList\n");
    printf(" 2. FreeList\n");
    printf(" 3. ClearList\n");
    printf(" 4. GetListType\n");
    printf(" 5. IsListEmpty\n");
    printf(" 6. Push\n");
    printf(" 7. Pop\n");
    printf(" 8. PrintList\n");
    printf("\nChoice: ");

    scanf("%d",&i);

    if ((i > 1) && (list == NULL))
    {
   		printf("Error - list needs to be initialised!\n");
  		continue;
    }


    switch(i)
    {
      case 0:
        exit = 1;
        break;
      case 1:
	    	if (list != NULL)
	    	{
	    	  FreeList(list);
	    	  list = NULL;
	    	}
	    	printf("Choose a type of list: \n");
	    	printf(" 1. FIFO\n");
	    	printf(" 2. LIFO\n");
	    	printf(" 3. SORTED\n");
	    	printf("Choice: ");
	    	scanf("%d",&j);
	    	switch(j)
	    	{
	    		case 2:
            list = CreateList(LIFO);
	    		  break;
	    		case 3:
            list = CreateList(SORTED);
	    		  break;
          case 1:
            list = CreateList(FIFO);
	    		  break;
	    		default:
            list = CreateList(FIFO);
	    		  printf("Invalid choice - using FIFO...\n");
	      }
	      printf("List created...\n");
	      printf("List has %d items...\n", list->count);
	      break;
      case 2:
        FreeList(list);
        list = NULL;
        printf("List free!\n");
        break;
      case 3:
	    	printf("Choose a type of list: \n");
	    	printf(" 1. FIFO\n");
	    	printf(" 2. LIFO\n");
	    	printf(" 3. SORTED\n");
	    	printf("Choice: ");
	    	scanf("%d",&j);
	    	switch(j)
	    	{
	    		case 2:
            ClearList(list, LIFO);
	    		  break;
	    		case 3:
            ClearList(list, SORTED);
	    		  break;
          case 1:
            ClearList(list, FIFO);
	    		  break;
	    		default:
            ClearList(list, FIFO);
	    		  printf("Invalid choice - using FIFO...\n");
	      }
	      printf("List cleared...\n");
	      printf("List has %d items...\n", list->count);
	      break;
      case 4:
        printf("List type: %s\n", LIST_TYPE(list->type));
	      printf("List has %d items...\n", list->count);
        break;
      case 5:
        if (IsListEmpty(list))
          printf("List is empty!\n");
        else
          printf("List is not empty!\n");
	      printf("List has %d items...\n", list->count);
        break;
      case 6:
        printf("Enter i, j, and h values for cell to push: ");
        scanf("%d %d %d", &c.i,&c.j,&c.h_value);
        Push(list, c);
        printf("Pushed (%d,%d,%d) into list...\n", c.i, c.j, c.h_value);
	      printf("List has %d items...\n", list->count);
        break;
      case 7:
        if (!IsListEmpty(list))
        {
          c = Pop(list);
          printf("Popped (%d,%d,%d) from list...\n", c.i, c.j, c.h_value);
        }
        else
          printf("List is empty!\n");
	      printf("List has %d items...\n", list->count);
        break;
      case 8:
        PrintList(list);
	      printf("List has %d items...\n", list->count);
        break;
      default:
        printf("Invalid choice!\n");
    }
  }
  return 0;
}
//=============================================================================
