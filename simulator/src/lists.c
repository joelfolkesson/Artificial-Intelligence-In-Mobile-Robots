//=============================================================================
//lists.c
//=============================================================================


//=============================================================================
//Includes
//=============================================================================
#include "lists.h"
//=============================================================================


//=============================================================================
//Functions
//=============================================================================
List* CreateList(int type)
{
  List* list = (List*)malloc(sizeof(List));
  if (list == NULL)
  {
    #if TraceErrors
    printf("CreateList: Cannot create list (out of memory?)...\n");
    #endif
    return NULL;
  }

  list->head = NULL;
  list->tail = NULL;
  list->type = type;
  list->count = 0;
  return list;
}
//=============================================================================
void ClearList(List* list, int type)
{
  if (list == NULL)
  {
    #if TraceErrors
    printf("List is not initialised...\n");
    #endif
    return;
  }

	while (list->count > 0)
    Pop(list);

  list->type = type;
}
//=============================================================================
void FreeList(List* list)
{
  if (list == NULL)
  {
    #if TraceErrors
    printf("List is not initialised...\n");
    #endif
    return;
  }

  ClearList(list, list->type);
  free(list);
  list = NULL;
}
//=============================================================================
Cell Pop(List* list)
{
  ListElement* temp;
  Cell c = { 0,0,0 };

  if (list == NULL)
  {
    #if TraceErrors
    printf("List is not initialised...\n");
    #endif
    return c;
  }

  if(list->count == 0)
  {
    #if TraceErrors
    printf("Pop: Cannot pop, list is empty!\n");
    #endif
    return c;
  }

  //get the data from the head element.
  c = list->head->data;

  //get the adress to the head element.
  temp = list->head;

  //move the head pointer of the list.
  list->head = list->head->next;

  //give back the element to the mem_space.
  free(temp);

  //decrement the list->count counter.
  list->count--;

  return c;
}
//=============================================================================
void Push(List* list, Cell c)
{
  ListElement* before;
  ListElement* after;

  //temporary pointer to a element.
  ListElement* temp;
  if (list == NULL)
  {
    #if TraceErrors
    printf("List is not initialised...\n");
    #endif
    return;
  }

  temp = (ListElement*) malloc(sizeof(ListElement));
  if (temp == NULL)
  {
    #if TraceErrors
    printf("Push: Out of memory - list is full!\n");
    #endif
    return;
  }

  temp->data = c;

  //check if it's the head element.
  if(list->count == 0)
  {
    //Set the head and tail pointers to the new element
    list->head = temp;
    list->tail = list->head;

    //Set the next pointer in the tail element to NULL.
    list->tail->next = NULL;
  }
  else
  {
    //Here is where the sorting is done.
    if (list->type == LIFO)
    {
      //Insert the new element in the head place of the list.
      temp->next = list->head;
      list->head = temp;
    }
    else if (list->type == FIFO)
    {
      //Insert the new element in the tail place of the list.
      temp->next = NULL;
      list->tail->next = temp;
      list->tail = temp;
    }
    else if (list->type == SORTED)
    {
      //First check if new item will be first
      if (list->head->data.h_value > c.h_value)
      {
        temp->next = list->head;
        list->head = temp;
      }
      //Then check if new item will be last
      else if (list->tail->data.h_value <= c.h_value)
      {
        temp->next = NULL;
        list->tail->next = temp;
        list->tail = temp;
      }
      //else find correct location for new item
      else
      {
        before = list->head;
        after = list->head->next;

        while (after->data.h_value <= c.h_value)
        {
          before = before->next;
          after = after->next;
        }

        before->next = temp;
        temp->next = after;
      }
    }
    else
    {
      #if TraceErrors
      printf("Invalid list type!\n");
      #endif
      return;
    }
  }

  //increment the list->count counter.
  list->count++;
}
//=============================================================================
int IsListEmpty(List* list)
{
  if (list == NULL)
  {
    #if TraceErrors
    printf("List is not initialised...\n");
    #endif
    return -1;
  }

  if (list->count == 0)
    return 1;
  else
    return 0;
}
//=============================================================================
void PrintList(List* list)
{
  int i;
	ListElement* temp = list->head;

  for (i=0;i<list->count;i++)
  {
  	printf("(%d,%d,%.2lf)\n", temp->data.i, temp->data.j, temp->data.h_value);
  	temp = temp->next;
  }
}
//=============================================================================

