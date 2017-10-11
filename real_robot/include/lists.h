//=============================================================================
//lists.h
//=============================================================================
#ifdef __cplusplus
 extern "C" {
 #endif

#ifndef lists_h_DEFINED
#define lists_h_DEFINED
//=============================================================================


//=============================================================================
//Includes
//=============================================================================
#include <stdlib.h>
#include <stdio.h>
/** \file
 * Header file of the lists.c (AIMR)
 */
//=============================================================================


//=============================================================================
//Defines
//=============================================================================

/** 
 *    @name  Types of lists
 */
 //@{
#define SORTED 0
#define LIFO   1
#define FIFO   2
//@}

#define LIST_TYPE(x) \
 (((x)==SORTED) ? "SORTED" :\
  ((x)==LIFO)   ? "LIFO"   :\
  ((x)==FIFO)   ? "FIFO"   : "UNKNOWN")

/*!@brief Tracing */
#define TraceErrors 1
//=============================================================================


//=============================================================================
//Structures
//=============================================================================
/*!@struct*/ 
typedef struct
{
    int i; /**< row */
    int j; /**< col */
    double h_value; /**< heuristic: if you don't use this, should be set to zero */
}Cell;

typedef struct ListElementType
{
  Cell data;
  struct ListElementType* next;
}ListElement;

/*!@struct*/
typedef struct
{
  ListElement *head;      /**< pointer to first element in list */
  ListElement *tail;      /**< pointer to last element in list */
  int count;              /**< number of elements in list */
  int type;               /**< type of list being used (SORTED, LIFO, FIFO) */
}List;
//=============================================================================


//=============================================================================
//Function prototypes
//=============================================================================

/*!@brief Create new list (allocates memory)*/
List* CreateList(int type);     
     
/*!@brief Empties list, sets type to type*/
void ClearList(List* list, int type);

/*!@brief Free list memory*/
void FreeList(List* list); 
          
/*!@brief Pop cell*/
Cell Pop(List* list);
                
/*!@brief Push cell*/
void Push(List* list, Cell c); 
     
/*!@brief Returns 1 if list is empty, 0 otherwise*/
int  IsListEmpty(List* list);     
   
/*!@brief Print list contents (does not pop)*/
void PrintList(List* list);          

//To do:
//-Should have separate data type for positions!?  Somehow?
// Have two list types - one for cells, one for positions?
//-Struct names should be all lowercase?  Or change kh.h/kh.c?
//-Cell Peek(List* list);             //Preview next cell in list
//-int Remove(List* list, Cell c);    //Remove first occurence of cell in list
//=============================================================================


//=============================================================================
#endif //#ifndef lists_h_DEFINED

#ifdef __cplusplus
 }
 #endif
//=============================================================================
